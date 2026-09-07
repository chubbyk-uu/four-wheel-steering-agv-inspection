#!/usr/bin/env python3
"""Independent stage-1 evaluator. Ground truth is used only here, never by control."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class Evaluator(Node):
    def __init__(self):
        super().__init__('motion_evaluator', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.odom = None
        self.joints = None
        self.state = ''
        self.states = set()
        self.max_angle = 0.0
        self.record_angles = False
        self.recorded_max_angle = 0.0
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, qos_profile_sensor_data)
        self.create_subscription(String, '/motion_state', self.on_state, 10)

    def on_odom(self, m):
        self.odom = m

    def on_state(self, m):
        self.state = m.data
        self.states.add(m.data)

    def on_joints(self, m):
        self.joints = m
        for n, p in zip(m.name, m.position):
            if n.endswith('_steer_joint'):
                self.max_angle = max(self.max_angle, abs(p))
                if self.record_angles:
                    self.recorded_max_angle = max(self.recorded_max_angle, abs(p))

    def command(self, xyz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = map(float, xyz)
        self.pub.publish(m)

    def run_for(self, seconds, cmd=None):
        start = self.get_clock().now().nanoseconds / 1e9
        deadline = time.monotonic() + max(30, seconds * 8)
        while self.get_clock().now().nanoseconds / 1e9 - start < seconds:
            if time.monotonic() > deadline:
                raise RuntimeError('simulation clock stalled or too slow')
            if cmd is not None:
                self.command(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)

    def pose(self):
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return p.x, p.y, yaw

    def evaluate(self):
        deadline = time.monotonic() + 60
        while not (self.odom and self.joints and self.state):
            if time.monotonic() > deadline:
                raise RuntimeError('missing odometry, joints or controller state')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.run_for(1, (0, 0, 0))
        results = []
        cases = [('forward', (.5, 0, 0)), ('lateral', (0, .5, 0)),
                 ('diagonal', (.3, .3, 0)), ('rotation', (0, 0, .3)),
                 ('combined', (.3, .1, .2)), ('reverse', (-.5, 0, 0))]
        for name, cmd in cases:
            self.states.clear()
            self.run_for(4, cmd)
            assert self.state == 'DRIVE', (name, self.state)
            x0, y0, a0 = self.pose()
            t0 = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec / 1e9
            self.run_for(2, cmd)
            x, y, a = self.pose()
            duration = self.odom.header.stamp.sec + self.odom.header.stamp.nanosec / 1e9 - t0
            dx, dy = x-x0, y-y0
            bx = math.cos(a0)*dx + math.sin(a0)*dy
            by = -math.sin(a0)*dx + math.cos(a0)*dy
            angle = math.atan2(math.sin(a-a0), math.cos(a-a0))
            vx, vy, w = cmd
            if w:
                ex = (vx*math.sin(w*duration)+vy*(math.cos(w*duration)-1))/w
                ey = (vx*(1-math.cos(w*duration))+vy*math.sin(w*duration))/w
            else:
                ex, ey = vx*duration, vy*duration
            error = math.hypot(bx-ex, by-ey)
            yaw_error = abs(angle-w*duration)
            row = dict(case=name, duration_s=duration, displacement_body=[bx,by,angle], expected=[ex,ey,w*duration],
                       position_error_m=error, yaw_error_rad=yaw_error, states=sorted(self.states))
            results.append(row)
            assert error < .15 and yaw_error < .12, row
        # Small direction change should remain continuous, tested from a forward run.
        self.run_for(4, (.5,0,0))
        self.record_angles = True
        self.recorded_max_angle = 0.0
        self.run_for(3, (-.5,0,0))
        assert self.recorded_max_angle < .035, self.recorded_max_angle
        assert self.odom.twist.twist.linear.x < -.4, self.odom.twist.twist.linear.x
        self.record_angles = False
        self.run_for(3, (.5,0,0))
        self.states.clear()
        self.run_for(1, (.5,.02,0))
        assert self.states == {'DRIVE'}, self.states
        # A lateral command while moving requires brake and align before driving.
        self.states.clear()
        self.run_for(5, (0,.5,0))
        assert {'BRAKE','ALIGN','DRIVE'} <= self.states, self.states
        # Dropping commands must stop without a fresh zero command.
        self.run_for(3)
        velocities = [v for n,v in zip(self.joints.name,self.joints.velocity) if n.endswith('_drive_joint')]
        assert len(velocities)==4 and max(map(abs,velocities)) < .1, velocities
        assert self.state == 'HOLD', self.state
        # Straight-line maximum-speed feasibility under the ideal servo model.
        self.run_for(8, (10/3.6,0,0))
        assert self.state == 'DRIVE', self.state
        measured = self.odom.twist.twist.linear.x
        assert abs(measured-10/3.6)<.15, measured
        self.run_for(8, (0,0,0))
        assert self.state == 'HOLD', self.state
        assert self.max_angle <= math.radians(190)+.01, self.max_angle
        return dict(passed=True, cases=results, continuous_transition=True,
                    stop_align_transition=True, command_timeout_stop=True,
                    direct_reverse_max_steering_rad=self.recorded_max_angle,
                    max_speed_measured_m_s=measured,
                    max_mechanical_angle_rad=self.max_angle)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args=parser.parse_args()
    rclpy.init()
    node=Evaluator()
    try:
        result=node.evaluate()
        Path(args.output).write_text(json.dumps(result, indent=2)+'\n')
        print('PASS: six motions, continuous transition, stop-align transition, timeout stop.')
    finally:
        node.command((0,0,0))
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
