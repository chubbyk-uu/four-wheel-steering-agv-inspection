#!/usr/bin/env python3
"""Two parallel encoder-controlled passes; no GNSS fusion or autonomous coverage claim."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import yaml
from PIL import Image
from wheel_dead_reckoning import WheelOdometry

ROOT=Path(__file__).resolve().parents[1]


def evaluate(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TwistStamped
    from sensor_msgs.msg import JointState,Image as RosImage
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    config=yaml.safe_load((ROOT/'src/agv_description/config/platform.yaml').read_text())
    camera=yaml.safe_load((ROOT/'src/agv_description/config/linescan.yaml').read_text())

    class Trial(Node):
        def __init__(self):
            super().__init__('adjacent_scan_trial',parameter_overrides=[Parameter('use_sim_time',value=True)])
            self.estimate=WheelOdometry(config['wheel_radius'],config['wheelbase'],config['track'])
            self.steer=np.zeros(4);self.state='';self.phase='startup';self.events=[]
            self.trace=[];self.last_trace=-1.;self.error=None;self.received={};self.metadata={}
            self.truth_evaluation=None;self.track_intervals=[];self.positions=[]
            self.command_pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
            self.client=self.create_client(SetBool,'/linescan/set_enabled')
            self.create_subscription(JointState,'/joint_states',self.joints,qos_profile_sensor_data)
            self.create_subscription(String,'/motion_state',self.state_message,10)
            self.create_subscription(RosImage,'/linescan/image_raw',self.image_message,2)
            self.create_subscription(String,'/linescan/block_metadata',self.metadata_message,10)
            self.create_subscription(Odometry,'/ground_truth/odom',lambda m:setattr(self,'truth_evaluation',m),10)

        def now_s(self):return self.get_clock().now().nanoseconds/1e9

        def joints(self,m):
            try:
                indices={name:i for i,name in enumerate(m.name)}
                drive=np.array([m.position[indices[n+'_drive_joint']] for n in ('fl','fr','rl','rr')])
                steer=np.array([m.position[indices[n+'_steer_joint']] for n in ('fl','fr','rl','rr')])
                stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
                self.estimate.update(stamp,drive,steer);self.steer=steer
                if stamp-self.last_trace>=.1:
                    self.trace.append(dict(time_s=stamp,pose=self.estimate.pose.tolist(),phase=self.phase,state=self.state,
                                           wheel_residual_m_s=self.estimate.residual_speed))
                    self.last_trace=stamp
            except Exception as e:self.error=str(e)

        def state_message(self,m):
            if m.data!=self.state:self.events.append(dict(time_s=self.now_s(),state=m.data,phase=self.phase))
            self.state=m.data

        def image_message(self,m):
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            if stamp in self.received:self.error='duplicate image timestamp'
            self.received[stamp]=(hashlib.sha256(m.data).hexdigest(),m.width,m.height,m.encoding)

        def metadata_message(self,m):
            meta=json.loads(m.data);self.metadata[meta['block_id']]=meta

        def command(self,vx=0,vy=0):
            message=TwistStamped();message.header.stamp=self.get_clock().now().to_msg();message.header.frame_id='base_link'
            message.twist.linear.x=float(vx);message.twist.linear.y=float(vy)
            self.command_pub.publish(message)

        def tick(self,vx=0,vy=0):
            self.command(vx,vy);rclpy.spin_once(self,timeout_sec=.01)
            if self.error:raise RuntimeError(self.error)

        def hold(self,seconds=1):
            deadline=time.monotonic()+30;settled=None
            while True:
                self.tick()
                if self.state=='HOLD':
                    if settled is None:settled=self.now_s()
                    if self.now_s()-settled>=seconds:return
                else:settled=None
                if time.monotonic()>deadline:raise RuntimeError('controller did not settle to HOLD')

        def enabled(self,value):
            req=SetBool.Request();req.data=value;future=self.client.call_async(req)
            deadline=time.monotonic()+90
            while not future.done():
                self.tick()
                if time.monotonic()>deadline:raise RuntimeError('capture service timed out')
            if not future.result().success:raise RuntimeError(future.result().message)

        def align(self,direction):
            self.phase='align';deadline=time.monotonic()+30
            while self.state!='DRIVE' or np.max(np.abs(self.steer))>=.015:
                self.tick(direction*.04,0)
                if time.monotonic()>deadline:raise RuntimeError('scan wheels did not align')
            self.hold(.3)

        def move(self,axis,target,speed):
            deadline=time.monotonic()+max(60,3*abs(target-self.estimate.pose[axis])/speed+30)
            while abs(target-self.estimate.pose[axis])>.002:
                remaining=target-self.estimate.pose[axis]
                velocity=math.copysign(min(speed,max(.035,1.5*abs(remaining))),remaining)
                self.tick(velocity if axis==0 else 0,velocity if axis==1 else 0)
                if time.monotonic()>deadline:raise RuntimeError('encoder-controlled move timed out')
            self.hold(.5)

        def scan(self,index,direction,target):
            self.align(direction)
            self.phase='scan_'+str(index)
            self.enabled(True);begin=self.now_s();start=self.estimate.pose.copy()
            self.move(0,target,args.speed)
            end=self.now_s();self.enabled(False);self.hold(.5)
            self.track_intervals.append(dict(track=index,direction=direction,start_s=begin,end_s=end,
                odom_start=start.tolist(),odom_end=self.estimate.pose.tolist()))
            if self.truth_evaluation:
                p=self.truth_evaluation.pose.pose.position
                self.positions.append(dict(track=index,estimated_local=self.estimate.pose.tolist(),
                    truth_world_evaluation_only=[p.x,p.y,p.z]))

    rclpy.init();node=Trial()
    try:
        assert node.client.wait_for_service(timeout_sec=90),'camera service missing'
        deadline=time.monotonic()+60
        while node.estimate.previous is None or not node.state or node.now_s()==0:
            node.tick()
            if time.monotonic()>deadline:raise RuntimeError('missing feedback')
        node.phase='settle';node.hold(2);node.enabled(False)
        origin=node.estimate.pose.copy()
        node.scan(0,1,origin[0]+args.distance)
        node.phase='lateral_shift';node.move(1,origin[1]+args.track_step,min(args.speed,.3))
        node.scan(1,-1,origin[0])
        node.phase='done';node.hold(2)
        sessions=list(Path(args.archive).glob('session_cpp_*'));assert len(sessions)==1
        session=sessions[0];metas=[json.loads(p.read_text()) for p in sorted(session.glob('block_*.json'))]
        tracks=[[],[]]
        for m in metas:
            assert node.metadata.get(m['block_id'])==m,'ROS metadata differs from archive'
            stamp=round(m['last']['time_s']*1e9)
            pixels=np.array(Image.open(session/f'block_{m["block_id"]:06d}.pgm'))
            assert node.received[stamp]==(hashlib.sha256(pixels.tobytes()).hexdigest(),4096,m['rows'],'mono8')
            matching=[t for t in node.track_intervals if t['start_s']<=m['first']['time_s']<=m['last']['time_s']<=t['end_s']]
            assert len(matching)==1,'image outside declared straight scan interval'
            t=matching[0];assert m['first']['scan_direction']==t['direction'] and m['last']['scan_direction']==t['direction']
            tracks[t['track']].append(m)
        assert len(metas)==len(node.received)>0
        summary=[]
        for i,blocks in enumerate(tracks):
            assert blocks and any(m['rows']==4096 for m in blocks)
            assert len({m['segment_id'] for m in blocks})==1
            for left,right in zip(blocks,blocks[1:]):
                assert left['last']['global_line']+1==right['first']['global_line']
                assert abs(abs(right['first']['encoder_distance_m']-left['last']['encoder_distance_m'])-camera['line_spacing_m'])<1e-8
            summary.append(dict(track=i,block_ids=[m['block_id'] for m in blocks],rows=sum(m['rows'] for m in blocks),
                segment_id=blocks[0]['segment_id'],camera_y_evaluation_m=float(np.mean([m['first']['camera_position_world_m'][1] for m in blocks]))))
        actual_step=abs(summary[1]['camera_y_evaluation_m']-summary[0]['camera_y_evaluation_m'])
        assert abs(actual_step-args.track_step)<.02
        assert all(any(e['state']=='ALIGN' and e['phase']==phase for e in node.events) for phase in ('lateral_shift','align'))
        report=dict(passed=True,scope='two encoder-controlled parallel trial passes, no autonomous coverage / fusion localization / stitching claim',
            ground_truth_used_for_control=False,pose_source='wheel joint encoders, local origin only',
            tracks=summary,track_intervals=node.track_intervals,nominal_track_step_m=args.track_step,
            actual_track_step_evaluation_m=actual_step,nominal_side_overlap_m=camera['nominal_width_m']-args.track_step,
            measured_side_overlap_evaluation_m=camera['nominal_width_m']-actual_step,within_track_overlap_lines=0,
            no_images_during_lateral_shift=True,all_ros_images_equal_archive=True,all_ros_metadata_equal_archive=True,
            final_motion_state=node.state,final_encoder_pose=node.estimate.pose.tolist(),
            max_wheel_residual_m_s=node.estimate.max_residual_speed,events=node.events,
            pose_evaluation=node.positions,gui=args.gui,rviz=args.rviz,archive=str(session))
        (Path(args.archive)/'adjacent_summary.json').write_text(json.dumps(report,indent=2)+'\n')
        (Path(args.archive)/'encoder_trace.json').write_text(json.dumps(node.trace)+'\n')
        print(json.dumps(report),flush=True)
    finally:
        try:node.command();node.hold(.3)
        finally:node.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--archive')
    p.add_argument('--distance',type=float,default=10);p.add_argument('--track-step',type=float,default=1.0)
    p.add_argument('--speed',type=float,default=.5);p.add_argument('--spawn-x',type=float,default=2)
    p.add_argument('--spawn-y',type=float,default=2.05);p.add_argument('--gui',action='store_true');p.add_argument('--rviz',action='store_true')
    p.add_argument('--domain',type=int,default=86);a=p.parse_args()
    if not (0<a.speed<=10/3.6 and 0<a.track_step<1.5 and a.distance>1.5):raise ValueError('invalid trial settings')
    if a.archive:evaluate(a);return
    archive=Path('/tmp')/('agv_adjacent_'+str(os.getpid()));tag=archive.name
    env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),GZ_PARTITION=tag,ROS_LOG_DIR=str(archive)+'_ros')
    with Path(str(archive)+'_sim.log').open('w') as log:
        sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','headless:='+str(not a.gui).lower(),
            'rviz:='+str(a.rviz).lower(),'linescan:=true','linescan_backend:=optix',
            'scene_manifest:='+str(Path(a.scene).resolve()),'capture_dir:='+str(archive),
            'spawn_x:='+str(a.spawn_x),'spawn_y:='+str(a.spawn_y),'scan_speed_limit:='+str(a.speed*1.1)],
            env=env,stdout=log,stderr=log,start_new_session=True)
        try:
            command=[sys.executable,__file__,*sys.argv[1:],'--archive',str(archive)]
            result=subprocess.run(command,env=env,timeout=600)
            print('Simulation log: '+str(archive)+'_sim.log',flush=True)
            if result.returncode:raise SystemExit(result.returncode)
        finally:
            if sim.poll() is None:
                os.killpg(sim.pid,signal.SIGINT)
                try:sim.wait(timeout=25)
                except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()


if __name__=='__main__':main()
