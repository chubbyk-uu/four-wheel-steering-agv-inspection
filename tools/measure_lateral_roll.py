#!/usr/bin/env python3
"""Compare passive body roll after the same controller-driven lateral move."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def evaluate(output, platform):
    import numpy as np
    import rclpy
    from scipy.signal import find_peaks, savgol_filter
    from scipy.spatial.transform import Rotation
    from validate_motion import Evaluator

    class Recorder(Evaluator):
        def __init__(self):
            self.rows=[];self.joint_rows=[];self.phase='startup';self.run_id=-1
            super().__init__()
        def on_odom(self,m):
            super().on_odom(m)
            q=m.pose.pose.orientation
            roll,pitch,yaw=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_euler('xyz')
            self.rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z,
                m.twist.twist.linear.x,m.twist.twist.linear.y,roll,pitch,yaw,
                time.monotonic(),self.run_id,self.phase,self.state])
        def on_joints(self,m):
            super().on_joints(m);positions=dict(zip(m.name,m.position))
            if all(k+'_suspension_joint' in positions for k in ('fl','fr','rl','rr')):
                self.joint_rows.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                    *[positions[k+'_suspension_joint'] for k in ('fl','fr','rl','rr')]])
        def hold(self,seconds):
            self.phase='rest'
            self.run_for(seconds,(0,0,0))
        def shift(self,direction,index):
            self.run_id=index;self.phase='move';start_y=self.odom.pose.pose.position.y
            deadline=time.monotonic()+60
            while abs(self.odom.pose.pose.position.y-start_y)<.55:
                assert time.monotonic()<deadline,'lateral move timed out'
                self.command((0,direction,0));rclpy.spin_once(self,timeout_sec=.01)
            brake_time=self.get_clock().now().nanoseconds*1e-9
            self.phase='settle';self.run_for(6,(0,0,0))
            assert self.state=='HOLD' and abs(self.odom.twist.twist.linear.y)<.001
            return dict(index=index,direction=direction,brake_time_s=brake_time,
                        start_y_m=start_y,end_y_m=self.odom.pose.pose.position.y)

    rclpy.init();n=Recorder()
    try:
        deadline=time.monotonic()+100
        while not(n.odom and n.joints and n.state):
            assert time.monotonic()<deadline,'feedback missing'
            rclpy.spin_once(n,timeout_sec=.1)
        n.hold(5);runs=[]
        for i,direction in enumerate((1.,-1.)):
            runs.append(n.shift(direction,i));n.hold(2)
        n.command((0,0,0))
        config=__import__('yaml').safe_load(platform.read_text())
        result=dict(schema='agv.lateral_roll_damping.v1',passed=True,
            suspension_stiffness_n_m=config['suspension_stiffness'],
            suspension_damping_ns_m=config['suspension_damping'],
            scope='Flat-road headless controller-driven 0.55 m brake trigger, two opposite lateral moves; six-second post-brake observation; no camera capture.',runs=[])
        rows=n.rows
        for run in runs:
            numeric=np.asarray([r[:11] for r in rows if r[10]==run['index']],float)
            phases=np.asarray([r[11] for r in rows if r[10]==run['index']])
            states=np.asarray([r[12] for r in rows if r[10]==run['index']])
            t=numeric[:,0];roll=np.degrees(numeric[:,6]);brake=run['brake_time_s']
            baseline_rows=np.asarray([r[:11] for r in rows if r[10]<run['index'] and r[11]=='rest'],float)
            baseline=float(np.mean(np.degrees(baseline_rows[-50:,6])))
            relative=roll-baseline;move=phases=='move';settle=phases=='settle'
            hold_indices=np.flatnonzero(settle&(states=='HOLD'))
            assert len(hold_indices),'controller did not reach HOLD during observation'
            hold=hold_indices[0];post=relative[hold:];post_t=t[hold:]-t[hold]
            smooth=savgol_filter(post,11,3)
            extrema=np.sort(np.r_[find_peaks(smooth,prominence=.02)[0],find_peaks(-smooth,prominence=.02)[0]])
            tolerance=.05
            outside=np.flatnonzero(np.abs(post)>tolerance)
            settle_time=0. if not len(outside) else (float(post_t[outside[-1]+1]) if outside[-1]+1<len(post_t) else None)
            suspension=[]
            for sample in n.joint_rows:
                if brake-8<=sample[0]<=brake+6:suspension.extend(sample[1:5])
            item=dict(**run,hold_time_s=float(t[hold]),move_roll_min_deg=float(relative[move].min()),
                move_roll_max_deg=float(relative[move].max()),move_roll_peak_to_peak_deg=float(np.ptp(relative[move])),
                post_hold_peak_abs_roll_deg=float(np.max(np.abs(post))),post_hold_extrema_count=int(len(extrema)),
                settle_within_0_05deg_s=settle_time,final_roll_offset_deg=float(np.mean(post[-25:])),
                max_suspension_abs_m=float(np.max(np.abs(suspension))))
            assert item['max_suspension_abs_m']<config['suspension_travel']-.005
            result['runs'].append(item)
        result['mean_move_roll_peak_to_peak_deg']=float(np.mean([r['move_roll_peak_to_peak_deg'] for r in result['runs']]))
        result['mean_post_hold_peak_abs_roll_deg']=float(np.mean([r['post_hold_peak_abs_roll_deg'] for r in result['runs']]))
        result['mean_post_hold_extrema_count']=float(np.mean([r['post_hold_extrema_count'] for r in result['runs']]))
        settled=[r['settle_within_0_05deg_s'] for r in result['runs']]
        result['mean_settle_within_0_05deg_s']=float(np.mean(settled)) if all(v is not None for v in settled) else None
        output.mkdir(parents=True,exist_ok=True)
        (output/'samples.json').write_text(json.dumps(dict(columns=['time','x','y','z','vx','vy','roll','pitch','yaw','wall','run','phase','state'],rows=rows)))
        (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result))
    finally:
        n.command((0,0,0));n.destroy_node();rclpy.shutdown()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--platform',type=Path,required=True);p.add_argument('--evaluate',action='store_true');a=p.parse_args()
    if a.evaluate:evaluate(a.output,a.platform);return
    a.output.mkdir(parents=True,exist_ok=False)
    env=dict(os.environ,ROS_DOMAIN_ID=str(120+os.getpid()%50),GZ_PARTITION='lateral_roll_'+str(os.getpid()))
    with (a.output/'sim.log').open('w') as log:
        sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','headless:=true','rviz:=false','linescan:=false',
            'platform:='+str(a.platform.resolve())],env=env,stdout=log,stderr=log,start_new_session=True)
        try:
            subprocess.run([sys.executable,__file__,'--evaluate','--output',str(a.output),'--platform',str(a.platform)],env=env,check=True,timeout=240)
        finally:
            if sim.poll() is None:
                os.killpg(sim.pid,signal.SIGINT)
                try:sim.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()


if __name__=='__main__':main()
