#!/usr/bin/env python3
"""Send timestamped body-frame velocity commands; Ctrl-C sends a zero command."""
import argparse
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped

p=argparse.ArgumentParser()
p.add_argument('--vx', type=float, default=0)
p.add_argument('--vy', type=float, default=0)
p.add_argument('--wz', type=float, default=0)
p.add_argument('--seconds', type=float, default=5)
a=p.parse_args()
rclpy.init()
n=Node('velocity_command',parameter_overrides=[Parameter('use_sim_time',value=True)])
pub=n.create_publisher(TwistStamped,'/cmd_vel',10)
try:
    deadline=time.monotonic()+10
    while n.get_clock().now().nanoseconds==0 or pub.get_subscription_count()==0:
        if time.monotonic()>deadline:raise RuntimeError('No clock or controller subscriber')
        rclpy.spin_once(n,timeout_sec=.05)
    start=n.get_clock().now().nanoseconds
    while (n.get_clock().now().nanoseconds-start)/1e9<a.seconds:
        m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='base_link'
        m.twist.linear.x=a.vx;m.twist.linear.y=a.vy;m.twist.angular.z=a.wz
        pub.publish(m);rclpy.spin_once(n,timeout_sec=.05)
except KeyboardInterrupt:
    pass
finally:
    m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='base_link'
    pub.publish(m);n.destroy_node();rclpy.shutdown()
