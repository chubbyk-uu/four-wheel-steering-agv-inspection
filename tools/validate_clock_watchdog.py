#!/usr/bin/env python3
"""ROS integration test: motor outputs must stop even without new /clock data."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64MultiArray,String


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']='96'
    log=a.output.with_suffix('.log').open('w')
    process=subprocess.Popen(['ros2','run','agv_control','swerve_controller','--ros-args',
        '-p','use_sim_time:=true','-p','wheel_radius:=0.2'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('clock_loss_evaluator')
    clock=node.create_publisher(Clock,'/clock',10)
    joints=node.create_publisher(JointState,'/joint_states',10)
    command=node.create_publisher(TwistStamped,'/cmd_vel',10)
    feedback={'speed':[0.,0.,0.,0.],'state':''}
    node.create_subscription(Float64MultiArray,'/drive_controller/commands',lambda m:feedback.update(speed=list(m.data)),10)
    node.create_subscription(String,'/motion_state',lambda m:feedback.update(state=m.data),10)
    simulation_time=1.;clock_time=1.;freeze=None;reached=None;last_tick=0.
    deadline=time.monotonic()+12
    try:
        while time.monotonic()<deadline:
            wall=time.monotonic()
            if wall-last_tick>=.01:
                last_tick=wall;simulation_time+=.01
                c=Clock();c.clock.sec=int(simulation_time);c.clock.nanosec=int((simulation_time%1)*1e9)
                if freeze is None:clock.publish(c);clock_time=simulation_time
                m=JointState();m.header.stamp=c.clock
                for i,name in enumerate(('fl','fr','rl','rr')):
                    m.name += [name+'_steer_joint',name+'_drive_joint']
                    m.position.extend([0.,0.]);m.velocity.extend([0.,feedback['speed'][i]])
                joints.publish(m)
                cmd=TwistStamped();cmd.header.frame_id='base_link'
                cmd.header.stamp.sec=int(clock_time);cmd.header.stamp.nanosec=int((clock_time%1)*1e9)
                cmd.twist.linear.x=.3;command.publish(cmd)
            rclpy.spin_once(node,timeout_sec=.002)
            if freeze is None and feedback['state']=='DRIVE' and min(feedback['speed'])>1.4:freeze=time.monotonic()
            if freeze is not None and feedback['state']=='FEEDBACK_HOLD' and max(map(abs,feedback['speed']))<1e-8:
                if reached is None:reached=time.monotonic()
                if time.monotonic()-reached>.2:break
        assert freeze is not None and reached is not None,feedback
        latency=reached-freeze;assert .25<latency<.8,latency
        result={'passed':True,'clock_stall_to_zero_wall_s':latency,'final_state':feedback['state'],
                'final_motor_commands_rad_s':feedback['speed'],
                'scope':'standalone controller with live mocked feedback/commands and frozen clock; not physical braking'}
        a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        node.destroy_node();rclpy.shutdown()
        os.killpg(process.pid,signal.SIGINT);process.wait(timeout=8);log.close()


if __name__=='__main__':main()
