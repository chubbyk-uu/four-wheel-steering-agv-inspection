#!/usr/bin/env python3
"""ROS boundary regression with synthetic joint feedback; no simulator or truth."""
import os
import signal
import subprocess
import time
import json
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String


def main():
    os.environ['ROS_DOMAIN_ID']='75'
    os.environ['ROS_LOG_DIR']='/tmp/agv_safety_ros'
    log=open('/tmp/agv_command_safety_node.log','w')
    proc=subprocess.Popen(['ros2','run','agv_control','swerve_controller'],stdout=log,stderr=log,start_new_session=True)
    rclpy.init();n=Node('command_boundary_test')
    cmd=n.create_publisher(TwistStamped,'/cmd_vel',10)
    feedback=n.create_publisher(JointState,'/joint_states',10)
    data={'drive':[0.0]*4,'angle':[0.0]*4,'state':''}
    n.create_subscription(Float64MultiArray,'/drive_controller/commands',lambda m:data.update(drive=list(m.data)),10)
    n.create_subscription(Float64MultiArray,'/steering_controller/commands',lambda m:data.update(angle=list(m.data)),10)
    n.create_subscription(String,'/motion_state',lambda m:data.update(state=m.data),10)
    def run(seconds, kind=None, send_feedback=True):
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            if send_feedback:
                f=JointState();f.header.stamp=n.get_clock().now().to_msg()
                f.name=[s+'_steer_joint' for s in ['fl','fr','rl','rr']]+[s+'_drive_joint' for s in ['fl','fr','rl','rr']]
                f.position=data['angle']+[0.0]*4;f.velocity=[0.0]*4+data['drive']
                feedback.publish(f)
            if kind:
                m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='base_link';m.twist.linear.x=.4
                if kind=='stale':m.header.stamp.sec-=5
                if kind=='future':m.header.stamp.sec+=5
                if kind=='frame':m.header.frame_id='map'
                if kind=='nan':m.twist.linear.x=float('nan')
                if kind=='unsupported':m.twist.linear.z=1.0
                cmd.publish(m)
            rclpy.spin_once(n,timeout_sec=.01)
    try:
        deadline=time.monotonic()+10
        while cmd.get_subscription_count()==0:
            assert time.monotonic()<deadline,'controller did not start'
            rclpy.spin_once(n,timeout_sec=.05)
        run(.4)
        for invalid in ['stale','future','frame','nan','unsupported']:
            run(1.2,'valid');assert max(data['drive'])>3, data
            run(1,invalid);assert max(map(abs,data['drive']))<1e-6,(invalid,data)
        run(1.2,'valid');assert max(data['drive'])>3,data
        run(.35,send_feedback=False)
        assert data['state']=='FEEDBACK_HOLD' and max(map(abs,data['drive']))==0,data
        run(.5)
        assert data['state']=='HOLD' and max(map(abs,data['drive']))==0,data
        run(1.2,'valid');assert max(data['drive'])>3,data
        run(1.2)
        assert max(map(abs,data['drive']))==0 and data['state']=='HOLD',data
        result={'passed':True,'rejected':['stale','future','wrong_frame','nonfinite','unsupported_axis'],
                'feedback_loss_stop':True,'no_old_command_after_recovery':True,'timeout_stop':True}
        Path('results/stage1_command_safety.json').write_text(json.dumps(result,indent=2)+'\n')
        print('PASS: invalid commands, feedback loss, recovery and watchdog.')
    finally:
        n.destroy_node();rclpy.shutdown()
        if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        log.close()
if __name__=='__main__':main()
