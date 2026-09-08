"""One-shot relative segment trial using fused odometry only."""
import json
import time
from pathlib import Path
import tempfile
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory
from .tracking import SegmentTracker


class TrackingNode(Node):
    def __init__(self):
        super().__init__('segment_tracker')
        share=Path(get_package_share_directory('agv_mission'))
        description=Path(get_package_share_directory('agv_description'))/'config'
        self.config=yaml.safe_load(Path(self.declare_parameter('config',str(share/'config/tracking.yaml')).value).read_text())
        self.platform=yaml.safe_load(Path(self.declare_parameter('platform',str(description/'platform.yaml')).value).read_text())
        self.kind=self.declare_parameter('kind','translate').value
        self.displacement=self.declare_parameter('displacement_xy_m',[4.,0.]).value
        self.angle=self.declare_parameter('angle_rad',3.141592653589793).value
        self.speed=self.declare_parameter('speed',.5).value
        self.start_offset=self.declare_parameter('start_offset_xy_m',[0.,0.]).value
        self.heading_offset=self.declare_parameter('heading_offset_rad',0.).value
        self.auto=self.declare_parameter('autostart',False).value
        self.output=Path(self.declare_parameter('output_dir','').value or tempfile.mkdtemp(prefix='agv_tracking_'))
        self.output.mkdir(parents=True,exist_ok=True);self.log=(self.output/'tracking.jsonl').open('x')
        self.odom=None;self.mode='';self.health={};self.arrivals={};self.core=None
        self.last_sim=None;self.last_clock_wall=time.monotonic();self.last_tick=time.monotonic();self.ready_since=None
        self.pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
        self.status=self.create_publisher(String,'/mission/tracking_status',20)
        self.create_subscription(Odometry,'/odometry/global',self.on_odom,20)
        self.create_subscription(String,'/motion_state',self.on_mode,20)
        self.create_subscription(String,'/localization/status',self.on_health,20)
        self.create_service(Trigger,'/mission/start_segment',self.start)
        self.create_service(Trigger,'/mission/cancel_segment',self.cancel)
        self.timer=self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self,m):self.odom=m;self.arrivals['odom']=time.monotonic()
    def on_mode(self,m):self.mode=m.data;self.arrivals['mode']=time.monotonic()
    def on_health(self,m):
        self.health=json.loads(m.data);self.arrivals['health']=time.monotonic()

    def start(self,request,response):
        if self.core is not None:
            response.success=False;response.message='one-shot node already started; use a fresh instance';return response
        if not self.valid() or self.mode!='HOLD' or self.ready_since is None or time.monotonic()-self.ready_since<self.config['initial_ready_hold_s']:
            response.success=False;response.message='continuous fresh READY localization and HOLD dwell required';return response
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
        from scipy.spatial.transform import Rotation
        rotation=Rotation.from_quat([q.x,q.y,q.z,q.w])*Rotation.from_euler('z',self.heading_offset)
        position=np.array([p.x,p.y,p.z])+rotation.apply([*self.start_offset,0.])
        try:
            self.core=SegmentTracker(position,rotation.as_quat(),self.kind,self.displacement,self.angle,self.speed,self.platform,self.config)
        except ValueError as exc:
            response.success=False;response.message=str(exc);return response
        (self.output/'request.json').write_text(json.dumps({'kind':self.kind,'start_position_m':position.tolist(),
            'start_orientation_xyzw':rotation.as_quat().tolist(),'goal_position_m':self.core.goal.tolist(),
            'goal_orientation_xyzw':self.core.goal_rotation.as_quat().tolist(),'speed':self.speed,
            'profile_duration_s':self.core.profile.duration,'config':self.config},indent=2)+'\n')
        response.success=True;response.message='started';return response

    def cancel(self,request,response):
        if self.core:self.core.fault('CANCELED')
        self.command([0.,0.,0.]);response.success=True;response.message='zero command requested';return response

    def valid(self):
        if self.odom is None or self.health.get('state')!='READY':return False
        wall=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        if any(wall-self.arrivals.get(k,-1e9)>self.config['wall_timeout_s'] for k in ('odom','mode','health')):return False
        stamp=self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9
        return self.odom.header.frame_id=='map' and self.odom.child_frame_id=='base_link' and -.02<=now-stamp<self.config['feedback_age_limit_s']

    def command(self,values):
        msg=TwistStamped();msg.header.frame_id='base_link';msg.header.stamp=self.get_clock().now().to_msg()
        msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z=map(float,values);self.pub.publish(msg)

    def tick(self):
        wall=time.monotonic();now=self.get_clock().now().nanoseconds*1e-9
        dt=now-self.last_sim if self.last_sim is not None else 0.
        if dt>0:self.last_clock_wall=wall
        self.last_sim=now
        valid=self.valid()
        if self.core is None:
            if valid and self.mode=='HOLD':
                if self.ready_since is None:self.ready_since=wall
                if self.auto and wall-self.ready_since>=self.config['initial_ready_hold_s']:self.start(None,Trigger.Response())
            else:self.ready_since=None
            return
        if wall-self.last_clock_wall>self.config['wall_timeout_s']:
            self.core.fault('SIM_CLOCK_STALLED')
        if not valid:self.core.fault('STALE_OR_UNREADY_LOCALIZATION')
        if dt<=0:
            if self.core.state=='FAULT':self.command([0.,0.,0.])
            return
        p=self.odom.pose.pose.position;q=self.odom.pose.pose.orientation
        v=self.odom.twist.twist
        command=self.core.update([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],
                                [v.linear.x,v.linear.y,v.angular.z],self.mode,dt,valid)
        self.command(command)
        from scipy.spatial.transform import Rotation
        record={'time_s':now,'state':self.core.state,'reason':self.core.reason,'motion_state':self.mode,
            'profile_time_s':self.core.clock,'profile_duration_s':self.core.profile.duration,
            'terminal_trims':self.core.trims,'reconfigurations':self.core.reconfigurations,'position_m':[p.x,p.y,p.z],
            'orientation_xyzw':[q.x,q.y,q.z,q.w],
            'rpy_rad':Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz').tolist(),
            'reference_position_m':self.core.reference.tolist(),
            'reference_orientation_xyzw':self.core.reference_rotation.as_quat().tolist(),
            'command':command.tolist(),**self.core.diagnostic}
        if self.core.state=='FAULT':
            record['health']=self.health
            record['feedback_wall_age_s']={k:wall-v for k,v in self.arrivals.items()}
            record['odom_age_s']=now-(self.odom.header.stamp.sec+self.odom.header.stamp.nanosec*1e-9)
        self.log.write(json.dumps(record,allow_nan=False)+'\n');self.log.flush()
        self.status.publish(String(data=json.dumps(record)))

    def destroy_node(self):
        self.command([0.,0.,0.]);self.log.close();return super().destroy_node()


def main():
    rclpy.init();node=TrackingNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
