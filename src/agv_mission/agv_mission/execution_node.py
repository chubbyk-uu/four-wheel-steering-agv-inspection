"""One rectangle per instance; fused navigation, explicit terminal and fault states."""
import json
import time
from pathlib import Path
from scipy.spatial.transform import Rotation
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .planner import plan,Vehicle
from .execution import compile_steps,segment_arguments,check_position,scan_end_reached
from .tracking import SegmentTracker
from .capture import CaptureGate
from .callback_trace import TracedExecutor


class Executor(Node):
    def __init__(self, **node_kwargs):
        super().__init__('rectangle_executor', **node_kwargs)
        share=Path(get_package_share_directory('agv_mission'))
        desc=Path(get_package_share_directory('agv_description'))/'config'
        self.platform=yaml.safe_load((desc/'platform.yaml').read_text())
        self.camera=yaml.safe_load((desc/'linescan.yaml').read_text())
        self.cfg=yaml.safe_load((share/'config/tracking.yaml').read_text())
        request=Path(self.declare_parameter('request','').value)
        self.plan=plan(yaml.safe_load(request.read_text()),Vehicle.from_configs(self.platform,self.camera))
        if self.plan['frame_id']!='map':raise ValueError('request road.frame_id must be map for execution')
        self.output=Path(self.declare_parameter('output_dir','').value)
        if str(self.output)=='.':raise ValueError('explicit new output_dir required')
        self.output.mkdir(parents=True,exist_ok=False)
        self.execution_id=self.output.parent.name
        (self.output/'plan.json').write_text(json.dumps(self.plan,indent=2)+'\n')
        self.log=(self.output/'execution.jsonl').open('x')
        self.auto=self.declare_parameter('autostart',False).value
        self.capture=CaptureGate(self,self.output,self.declare_parameter('capture',False).value,self.cfg['capture_state_timeout_s'])
        self.odom=None;self.mode='';self.health={};self.arrivals={};self.last_command=[0.,0.,0.];self.motion_reason=''
        self.state='READY';self.reason='';self.ready_since=None;self.steps=[];self.index=0;self.core=None
        self.last_sim=None;self.last_clock=time.monotonic();self.step_attempts=0;self.stopped_since=None
        self.pause_pose=None;self.pause_started=None;self.pause_events=[]
        self.cmd=self.create_publisher(TwistStamped,'/cmd_vel',10)
        # Telemetry is periodically refreshed and fully archived locally. A slow
        # display must not back-pressure the control loop through reliable DDS.
        self.status=self.create_publisher(String,'/mission/status',qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odometry/global',self.on_odom,20)
        self.create_subscription(String,'/motion_state',self.on_mode,20)
        self.create_subscription(String,'/motion_transition_reason',lambda m:setattr(self,'motion_reason',m.data),20)
        self.create_subscription(String,'/localization/status',self.on_health,20)
        self.create_subscription(String,'/mission/external_fault',self.external_fault,10)
        self.create_service(Trigger,'/mission/start',self.start)
        self.create_service(Trigger,'/mission/cancel',self.cancel)
        self.create_service(Trigger,'/mission/pause',self.pause)
        self.create_service(Trigger,'/mission/resume',self.resume)
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self,m):self.odom=m;self.arrivals['odom']=time.monotonic()
    def on_mode(self,m):self.mode=m.data;self.arrivals['mode']=time.monotonic()
    def on_health(self,m):self.health=json.loads(m.data);self.arrivals['health']=time.monotonic()
    def pose(self):
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
        return np.array([p.x,p.y,p.z]),np.array([q.x,q.y,q.z,q.w])
    def valid(self,now):
        if self.odom is None or self.health.get('state')!='READY':return False
        if self.odom.header.frame_id!='map' or self.odom.child_frame_id!='base_link':return False
        stamp=self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9
        return -.02<=now-stamp<self.cfg['feedback_age_limit_s'] and all(time.monotonic()-self.arrivals.get(k,0)<self.cfg['wall_timeout_s'] for k in ('odom','mode','health'))
    def command(self,v):
        self.last_command=list(map(float,v))
        m=TwistStamped();m.header.frame_id='base_link';m.header.stamp=self.get_clock().now().to_msg()
        m.twist.linear.x,m.twist.linear.y,m.twist.angular.z=map(float,v);self.cmd.publish(m)
    def start(self,request,response):
        if self.state!='READY' or self.ready_since is None or time.monotonic()-self.ready_since<self.cfg['initial_ready_hold_s']:
            response.success=False;response.message='continuous READY localization and HOLD dwell required';return response
        if not self.capture.request(False,reason='initial_disable'):
            response.success=False;response.message='waiting for camera disabled acknowledgement';return response
        try:self.steps=compile_steps(self.plan,*self.pose())
        except ValueError as exc:
            response.success=False;response.message=str(exc);return response
        (self.output/'steps.json').write_text(json.dumps(self.steps,indent=2)+'\n')
        self.state='RUNNING';response.success=True;response.message='started';return response
    def pause_record(self,event):
        p,q=self.pose()
        self.pause_events.append(dict(event=event,time_s=self.get_clock().now().nanoseconds*1e-9,
            step_index=self.index,position_m=p.tolist(),orientation_xyzw=q.tolist(),
            capture_active=self.capture.active))
        (self.output/'pause_events.json').write_text(json.dumps(self.pause_events,indent=2)+'\n')
    def stopped(self):
        if self.odom is None or self.mode!='HOLD':return False
        v=self.odom.twist.twist
        return np.hypot(v.linear.x,v.linear.y)<self.cfg['stopped_speed_m_s'] and abs(v.angular.z)<self.cfg['stopped_yaw_rate_rad_s']
    def pause(self,request,response):
        if self.state in ('PAUSING','PAUSED'):
            response.success=True;response.message=self.state;return response
        if self.state!='RUNNING':
            response.success=False;response.message='only a running mission can pause';return response
        self.state='PAUSING';self.pause_started=self.get_clock().now().nanoseconds*1e-9
        self.stopped_since=None;self.pause_record('pause_requested');self.command([0,0,0])
        response.success=True;response.message='braking; partial camera frame remains armed';return response
    def resume(self,request,response):
        now=self.get_clock().now().nanoseconds*1e-9
        if self.state!='PAUSED' or not self.valid(now) or not self.stopped() or self.capture.error or self.capture.future is not None:
            response.success=False;response.message='PAUSED, healthy feedback, HOLD and camera acknowledgement required';return response
        p,q=self.pose()
        try:
            check_position(self.plan,p)
            before_p,before_q=self.pause_pose
            if np.linalg.norm(p-before_p)>self.cfg['pause_max_displacement_m'] or (Rotation.from_quat(before_q).inv()*Rotation.from_quat(q)).magnitude()>self.cfg['pause_max_rotation_rad']:
                raise ValueError('vehicle moved while paused; cancel and replan')
            if self.index<len(self.steps):
                args=segment_arguments(self.steps[self.index],p,q)
                if self.capture.active and args and args['kind']=='rotate':
                    raise ValueError('scan heading changed; cannot rotate with a retained partial frame')
        except ValueError as exc:
            response.success=False;response.message=str(exc);return response
        self.pause_record('resumed');self.core=None;self.step_attempts=0
        self.state='RUNNING';self.stopped_since=None
        response.success=True;response.message='continuing to original endpoint with a new rest-to-rest profile';return response
    def pause_tick(self,now,valid,dt,wall):
        self.command([0,0,0])
        if not valid:self.fault('STALE_OR_UNREADY_LOCALIZATION');return
        if wall-self.last_clock>self.cfg['wall_timeout_s']:self.fault('SIM_CLOCK_STALLED');return
        if dt<0 or dt>.1:self.fault('INVALID_CONTROL_TIMESTEP');return
        if self.state=='PAUSED':return
        if now-self.pause_started>self.cfg['pause_stop_timeout_s']:
            self.fault('PAUSE_STOP_TIMEOUT');return
        if self.stopped() and self.capture.future is None:
            if self.stopped_since is None:self.stopped_since=now
            if now-self.stopped_since>=self.cfg['settle_time_s']:
                self.pause_pose=self.pose();self.state='PAUSED';self.pause_record('stopped')
        else:self.stopped_since=None
    def cancel(self,request,response):
        if self.state not in ('COMPLETED','ACQUIRED','FAULT','CANCELED'):self.state='CANCELING';self.reason='CANCELED'
        self.command([0,0,0]);response.success=True;response.message='braking requested';return response
    def fault(self,reason):
        if self.state not in ('COMPLETED','ACQUIRED','FAULT'):
            self.state='FAULT';self.reason=reason
        self.command([0,0,0])
    def external_fault(self,msg):
        try:
            value=json.loads(msg.data)
            if value.get('execution_id')==self.execution_id and value.get('reason') in ('COMPETING_COMMAND_PUBLISHER','OPERATOR_SERVICE_TIMEOUT'):
                self.fault(value['reason'])
        except (ValueError,TypeError):pass
    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9;wall=time.monotonic()
        dt=0 if self.last_sim is None else now-self.last_sim;self.last_sim=now
        if dt>0:self.last_clock=wall
        valid=self.valid(now)
        self.capture.poll()
        if self.capture.error:self.fault(self.capture.error)
        if self.state=='READY':
            if self.capture.enabled and self.capture.active is None and self.capture.future is None:
                if self.capture.client.service_is_ready():self.capture.request(False,reason='prepare')
            if valid and self.mode=='HOLD':
                if self.ready_since is None:self.ready_since=wall
                if self.auto and wall-self.ready_since>=self.cfg['initial_ready_hold_s']:self.start(None,Trigger.Response())
            else:self.ready_since=None
        elif self.state in ('PAUSING','PAUSED'):
            self.pause_tick(now,valid,dt,wall)
        elif self.state=='RUNNING':
            if self.count_publishers('/cmd_vel')!=1:self.fault('COMPETING_COMMAND_PUBLISHER')
            elif dt<0 or dt>.1:self.fault('INVALID_CONTROL_TIMESTEP')
            elif not valid:self.fault('STALE_OR_UNREADY_LOCALIZATION')
            elif wall-self.last_clock>self.cfg['wall_timeout_s']:self.fault('SIM_CLOCK_STALLED')
            elif dt>0:self.run_step(dt)
        else:
            self.command([0,0,0])
            self.capture.request(False,reason=self.reason or self.state.lower())
            if self.state=='CANCELING' and self.mode=='HOLD' and self.capture.active is False and self.capture.future is None:self.state='CANCELED'
        record={'time_s':now,'state':self.state,'reason':self.reason,'motion_state':self.mode,
                'step_index':self.index,'step_count':len(self.steps),'capture_integrated':self.capture.enabled,'capture_active':self.capture.active,'capture_sensor_enabled':self.capture.heartbeat.get('enabled') if self.capture.heartbeat else None,'capture_close_failed':self.capture.close_failed,'command_body':self.last_command,'motion_reason':self.motion_reason}
        record.update(execution_id=self.execution_id,capture_pending=self.capture.future is not None,
            ready_to_start=self.state=='READY' and self.ready_since is not None and wall-self.ready_since>=self.cfg['initial_ready_hold_s'])
        if self.steps and self.index<len(self.steps):record.update(kind=self.steps[self.index]['kind'],track_id=self.steps[self.index]['track_id'])
        if self.odom is not None:record['position_m']=self.pose()[0].tolist()
        if self.core:record.update(tracker_state=self.core.state,profile_time_s=self.core.clock,
            profile_duration_s=self.core.profile.duration,terminal_trims=self.core.trims,
            reference_position_m=self.core.reference.tolist(),**self.core.diagnostic)
        if self.state=='FAULT':
            record['health']=self.health
            record['feedback_wall_age_s']={k:time.monotonic()-v for k,v in self.arrivals.items()}
            record['odom_age_s']=now-(self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9) if self.odom else None
            record['control_dt_s']=dt
            record['operator_callbacks']=getattr(self,'operator_callback_stats',{})
            executor=getattr(self,'executor',None)
            if hasattr(executor,'trace'):record['callback_trace']=executor.trace.snapshot()
        self.log.write(json.dumps(record)+'\n');self.log.flush();self.status.publish(String(data=json.dumps(record)))
    def run_step(self,dt):
        p,q=self.pose()
        try:check_position(self.plan,p)
        except ValueError as exc:self.fault(str(exc));return
        if self.index>=len(self.steps):
            self.command([0,0,0])
            if self.capture.request(False,reason='mission_end') and self.capture.future is None:
                self.state='ACQUIRED' if self.capture.enabled else 'COMPLETED'
            return
        step=self.steps[self.index]
        if self.capture.future is not None and (self.capture.target or self.core is None):
            self.command([0,0,0]);return
        if self.core is None:
            self.command([0,0,0])
            if self.mode!='HOLD':return
            try:args=segment_arguments(step,p,q)
            except ValueError as exc:self.fault(str(exc));return
            if args is None:
                if not self.capture.request(False,reason='step_end'):return
                self.index+=1;self.step_attempts=0;return
            if step['kind']=='PASS' and args['kind']=='translate' and not scan_end_reached(self.plan,self.camera,step,p,q):
                if not self.capture.request(True,step['track_id']):return
            self.step_attempts+=1
            if self.step_attempts>5:self.fault('STEP_NOT_CONVERGED');return
            self.active_kind=args['kind']
            self.core=SegmentTracker(p,q,platform=self.platform,config=self.cfg,**args)
            self.core.forward_only=step['kind']=='PASS' and args['kind']=='translate'
        v=self.odom.twist.twist
        cmd=self.core.update(p,q,[v.linear.x,v.linear.y,v.angular.z],self.mode,dt)
        self.command(cmd)
        if step['kind']=='PASS':
            if scan_end_reached(self.plan,self.camera,step,p,q) or self.core.state=='STOPPING':
                self.capture.request(False,reason='track_end')
        if self.core.state=='FAULT':self.fault(self.core.reason)
        elif self.core.state=='COMPLETED':
            # A time-profile translation has reached its endpoint within the tracker
            # tolerance. Do not repeatedly chase fresh GNSS noise after completion.
            if self.active_kind=='translate' or step['kind']=='ROTATE_180':
                self.index+=1;self.step_attempts=0
            self.core=None
    def destroy_node(self):
        if rclpy.ok():self.command([0,0,0])
        self.capture.close();self.log.close();return super().destroy_node()


def main():
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Executor();executor=TracedExecutor();executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        import signal
        # The launch supervisor and owning broker may both send SIGINT.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if rclpy.ok():
            node.cancel(None,Trigger.Response());deadline=time.monotonic()+8
            while time.monotonic()<deadline and not(node.mode=='HOLD' and node.capture.active is False and node.capture.future is None):
                executor.spin_once(timeout_sec=.02)
        (node.output/'callback_trace.json').write_text(json.dumps(executor.trace.snapshot(),indent=2)+'\n')
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():rclpy.shutdown()
