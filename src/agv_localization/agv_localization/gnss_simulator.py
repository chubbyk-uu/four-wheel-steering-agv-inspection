"""Simulator-only GNSS source. This node, never the EKFs, reads ground truth."""
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from .common import load
from .core import nominal_antennas, gnss_covariance, stamp_seconds, DeliveryQueue, delay_sample


class GnssSimulator(Node):
    def __init__(self):
        super().__init__('gnss_simulator')
        self.config,self.platform,_=load(self)
        self.mounts=nominal_antennas(self.platform)
        directory=self.declare_parameter('output_dir','').value
        if directory:
            evaluation=Path(directory)/'evaluation';evaluation.mkdir(parents=True,exist_ok=True)
            (evaluation/'simulation_truth.json').open('x').write(json.dumps({'gnss_true_phase_centers_m':self.mounts.tolist(),
                'configuration':self.config,'convention':'physical model unchanged; calibration residuals affect T_hat only'},indent=2)+'\n')
        self.covariance=gnss_covariance(self.config)
        self.rng=np.random.default_rng(self.config['seed']+10)
        self.timing=np.random.default_rng(self.config['seed']+11)
        self.queue=DeliveryQueue()
        self.first=None;self.last=None;self.slot=-1
        self.pubs={side:self.create_publisher(PoseWithCovarianceStamped,'/sensors/gnss/fixed/'+side,20)
                   for side in ('left','right')}
        self.status=self.create_publisher(String,'/sensors/gnss/status',20)
        self.create_subscription(Odometry,'/ground_truth/odom',self.sample,20)
        self.create_timer(.002,self.deliver)

    def sample(self,msg):
        t=stamp_seconds(msg.header.stamp)
        if self.last is not None and t<self.last:
            raise RuntimeError('simulation clock rewind: restart localization session')
        self.last=t
        if self.first is None:self.first=t
        slot=int((t-self.first+1e-7)*self.config['gnss_frequency_hz'])
        if slot<=self.slot:return
        self.slot=slot
        relative=t-self.first
        start=self.config['gnss_outage_start_s']
        if start>=0 and start<=relative<start+self.config['gnss_outage_duration_s']:
            self.status.publish(String(data=json.dumps({'fix':'NO_FIX','measurement_time_s':t})))
            return
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        rot=Rotation.from_quat([q.x,q.y,q.z,q.w])
        points=self.mounts@rot.as_matrix().T+np.array([p.x,p.y,p.z])
        if self.config['noise_enabled']:
            points+=self.rng.multivariate_normal(np.zeros(6),self.covariance).reshape(2,3)
        due=t+delay_sample(self.config,'gnss',self.timing)
        for i,side in enumerate(('left','right')):
            out=PoseWithCovarianceStamped();out.header.stamp=msg.header.stamp;out.header.frame_id='map'
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=points[i].tolist()
            out.pose.pose.orientation.w=1.
            covariance=np.eye(6)*1e6
            covariance[:3,:3]=self.covariance[i*3:i*3+3,i*3:i*3+3] if self.config['noise_enabled'] else np.eye(3)*1e-10
            out.pose.covariance=covariance.ravel().tolist()
            self.queue.push(due,side,out)

    def deliver(self):
        now=self.get_clock().now().nanoseconds*1e-9
        for side,msg in self.queue.pop(now):
            self.pubs[side].publish(msg)
            if side=='left':
                self.status.publish(String(data=json.dumps({'fix':'RTK_FIXED',
                    'measurement_time_s':stamp_seconds(msg.header.stamp),'delivery_time_s':now,
                    'representation':'local ENU antenna position; orientation not measured'})))


def main():
    rclpy.init();node=GnssSimulator()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
