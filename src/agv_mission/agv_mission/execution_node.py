"""One rectangle per instance; fused navigation, explicit terminal and fault states."""
import json
import time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from .planner import plan,Vehicle
from .execution import compile_steps,segment_arguments,check_position
from .tracking import SegmentTracker


class Executor(Node):
    def __init__(self):
        super().__init__('rectangle_executor')
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
        (self.output/'plan.json').write_text(json.dumps(self.plan,indent=2)+'\n')
        self.log=(self.output/'execution.jsonl').open('x')
        self.auto=self.declare_parameter('autostart',False).value
        self.odom=None;self.mode='';self.health={};self.arrivals={}
        self.state='READY';self.reason='';self.ready_since=None;self.steps=[];self.index=0;self.core=None
        self.last_sim=None;self.last_clock=time.monotonic();self.step_attempts=0;self.stopped_since=None
        self.cmd=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.status=self.create_publisher(String,'/mission/status',20)
        self.create_subscription(Odometry,'/odometry/global',self.on_odom,20)
        self.create_subscription(String,'/motion_state',self.on_mode,20)
        self.create_subscription(String,'/localization/status',self.on_health,20)
        self.create_service(Trigger,'/mission/start',self.start)
        self.create_service(Trigger,'/mission/cancel',self.cancel)
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
        m=TwistStamped();m.header.frame_id='base_link';m.header.stamp=self.get_clock().now().to_msg()
        m.twist.linear.x,m.twist.linear.y,m.twist.angular.z=map(float,v);self.cmd.publish(m)
    def start(self,request,response):
        if self.state!='READY' or self.ready_since is None or time.monotonic()-self.ready_since<self.cfg['initial_ready_hold_s']:
            response.success=False;response.message='continuous READY localization and HOLD dwell required';return response
        try:self.steps=compile_steps(self.plan,*self.pose())
        except ValueError as exc:
            response.success=False;response.message=str(exc);return response
        (self.output/'steps.json').write_text(json.dumps(self.steps,indent=2)+'\n')
        self.state='RUNNING';response.success=True;response.message='started';return response
    def cancel(self,request,response):
        if self.state not in ('COMPLETED','FAULT'):self.state='CANCELING';self.reason='CANCELED'
        self.command([0,0,0]);response.success=True;response.message='braking requested';return response
    def fault(self,reason):
        if self.state not in ('COMPLETED','FAULT'):
            self.state='FAULT';self.reason=reason
        self.command([0,0,0])
    def tick(self):
        now=self.get_clock().now().nanoseconds*1e-9;wall=time.monotonic()
        dt=0 if self.last_sim is None else now-self.last_sim;self.last_sim=now
        if dt>0:self.last_clock=wall
        valid=self.valid(now)
        if self.state=='READY':
            if valid and self.mode=='HOLD':
                if self.ready_since is None:self.ready_since=wall
                if self.auto and wall-self.ready_since>=self.cfg['initial_ready_hold_s']:self.start(None,Trigger.Response())
            else:self.ready_since=None
        elif self.state=='RUNNING':
            if not valid:self.fault('STALE_OR_UNREADY_LOCALIZATION')
            elif wall-self.last_clock>self.cfg['wall_timeout_s']:self.fault('SIM_CLOCK_STALLED')
            elif dt<0 or dt>.1:self.fault('INVALID_CONTROL_TIMESTEP')
            elif dt>0:self.run_step(dt)
        else:
            self.command([0,0,0])
            if self.state=='CANCELING' and self.mode=='HOLD':self.state='CANCELED'
        record={'time_s':now,'state':self.state,'reason':self.reason,'motion_state':self.mode,
                'step_index':self.index,'step_count':len(self.steps),'capture_integrated':False}
        if self.steps and self.index<len(self.steps):record.update(kind=self.steps[self.index]['kind'],track_id=self.steps[self.index]['track_id'])
        if self.odom is not None:record['position_m']=self.pose()[0].tolist()
        if self.core:record.update(tracker_state=self.core.state,profile_time_s=self.core.clock,
            profile_duration_s=self.core.profile.duration,terminal_trims=self.core.trims,
            reference_position_m=self.core.reference.tolist(),**self.core.diagnostic)
        if self.state=='FAULT':record['health']=self.health
        self.log.write(json.dumps(record)+'\n');self.log.flush();self.status.publish(String(data=json.dumps(record)))
    def run_step(self,dt):
        p,q=self.pose()
        try:check_position(self.plan,p)
        except ValueError as exc:self.fault(str(exc));return
        if self.index>=len(self.steps):self.state='COMPLETED';self.command([0,0,0]);return
        step=self.steps[self.index]
        if self.core is None:
            self.command([0,0,0])
            if self.mode!='HOLD':return
            args=segment_arguments(step,p,q)
            if args is None:
                self.index+=1;self.step_attempts=0;return
            self.step_attempts+=1
            if self.step_attempts>5:self.fault('STEP_NOT_CONVERGED');return
            self.active_kind=args['kind']
            self.core=SegmentTracker(p,q,platform=self.platform,config=self.cfg,**args)
        v=self.odom.twist.twist
        cmd=self.core.update(p,q,[v.linear.x,v.linear.y,v.angular.z],self.mode,dt)
        self.command(cmd)
        if self.core.state=='FAULT':self.fault(self.core.reason)
        elif self.core.state=='COMPLETED':
            # A time-profile translation has reached its endpoint within the tracker
            # tolerance. Do not repeatedly chase fresh GNSS noise after completion.
            if self.active_kind=='translate' or step['kind']=='ROTATE_180':
                self.index+=1;self.step_attempts=0
            self.core=None
    def destroy_node(self):
        self.command([0,0,0]);self.log.close();return super().destroy_node()


def main():
    rclpy.init();node=Executor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
