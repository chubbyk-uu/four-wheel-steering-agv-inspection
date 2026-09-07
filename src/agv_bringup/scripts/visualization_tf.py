#!/usr/bin/env python3
"""Coherent display-only TF snapshots; never used by the motion controller.

RViz's latest-time lookup otherwise mixes a fresh world pose with older joint
transforms. Interpolate every joint at the odometry timestamp, then publish the
world parent LAST so all branches advance together. No pose extrapolation.
"""
from collections import deque
import json
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class VisualizationTF(Node):
    def __init__(self):
        super().__init__('visualization_tf')
        self.buffer=Buffer()
        self.listener=TransformListener(self.buffer,self)
        self.pending=deque()
        self.frames=0;self.dropped=0;self.max_age=0.
        self.links=[f'{corner}_{part}_link' for corner in ('fl','fr','rl','rr') for part in ('steer','wheel')]
        self.pub=self.create_publisher(TFMessage,'/visualization/tf',10)
        self.status=self.create_publisher(String,'/visualization/status',10)
        self.create_subscription(Odometry,'/ground_truth/odom',self.odom,10)
        self.create_timer(.005,self.flush)
        self.create_timer(1,self.report)

    def odom(self,message):
        if message.header.frame_id!='world' or message.child_frame_id!='base_link':
            self.get_logger().error('Unexpected visualization odometry frames')
            return
        if len(self.pending)>=16:
            self.pending.popleft();self.dropped+=1
        self.pending.append(message)
        self.flush()

    def flush(self):
        while self.pending:
            odom=self.pending[0];stamp=Time.from_msg(odom.header.stamp)
            age=(self.get_clock().now()-stamp).nanoseconds*1e-9
            if age>.25 or age<-.1:
                self.pending.popleft();self.dropped+=1;continue
            try:
                transforms=[self.buffer.lookup_transform('base_link',link,stamp) for link in self.links]
            except TransformException:
                return  # Wait for both sides of the interpolation bracket.
            parent=TransformStamped();parent.header=odom.header;parent.child_frame_id='base_link'
            p=odom.pose.pose.position
            parent.transform.translation.x=p.x;parent.transform.translation.y=p.y;parent.transform.translation.z=p.z
            parent.transform.rotation=odom.pose.pose.orientation
            self.pub.publish(TFMessage(transforms=transforms+[parent]))
            self.pending.popleft();self.frames+=1;self.max_age=max(self.max_age,age)

    def report(self):
        self.status.publish(String(data=json.dumps(dict(frames=self.frames,dropped_pending=self.dropped,
            pending=len(self.pending),max_sim_age_s=self.max_age,mode='same_stamp_interpolated_display_only'))))


def main():
    rclpy.init();node=VisualizationTF()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
