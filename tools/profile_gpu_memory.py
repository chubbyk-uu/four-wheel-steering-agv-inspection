#!/usr/bin/env python3
"""Stationary whole-device VRAM comparison, with owned-process teardown.

Run profiles sequentially under the normal ROS/OptiX runtime environment.
GPU medians include the desktop and driver; differences are not per-process attribution.
"""
import argparse,os,time,json,subprocess,statistics
from pathlib import Path
from validate_rectangle_execution import stop_tree
from process_resources import ResourceMonitor
import rclpy
from nav_msgs.msg import Odometry
parser=argparse.ArgumentParser()
parser.add_argument('--profile',choices=['server','optix','gui_optix','full'],required=True)
parser.add_argument('--scene',type=Path,required=True)
parser.add_argument('--output',type=Path,required=True)
a=parser.parse_args();case=a.profile;out=a.output;out.mkdir(parents=True,exist_ok=False)
os.environ.update(ROS_DOMAIN_ID=str(100+os.getpid()%70),GZ_PARTITION='vram_'+str(os.getpid()))
common=['scene_manifest:='+str(a.scene.resolve()),'spawn_x:=-3']
if case=='full':args=['inspection.launch.py','session_dir:='+str((out/'session').resolve())]+common
else:args=['sim.launch.py','headless:='+('false' if case=='gui_optix' else 'true'),'rviz:=false','linescan:='+('false' if case=='server' else 'true'),'linescan_backend:=optix','capture_dir:='+str((out/'raw').resolve())]+common
rclpy.init();n=rclpy.create_node('vram_probe');odom=[];speed=[]
def receive(m):
 odom.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
 speed.append((m.twist.twist.linear.x**2+m.twist.twist.linear.y**2)**.5)
n.create_subscription(Odometry,'/ground_truth/odom',receive,10)
log=(out/'launch.log').open('w');p=subprocess.Popen(['ros2','launch','agv_bringup',*args],stdout=log,stderr=log,start_new_session=True);monitor=ResourceMonitor(p.pid,out/'resources.jsonl');start=time.monotonic();samples=[]
try:
 while time.monotonic()-start<180:
  assert p.poll() is None,'launch exited'
  rclpy.spin_once(n,timeout_sec=.1)
  if odom and odom[-1]>=10 and time.monotonic()-start>=50:break
 else:raise RuntimeError('startup timeout')
 for i in range(10):
  rclpy.spin_once(n,timeout_sec=.1)
  samples.append(int(subprocess.check_output(['nvidia-smi','--query-gpu=memory.used','--format=csv,noheader,nounits'],text=True).strip()))
  time.sleep(1)
 assert speed[-1]<.01,'robot is not stationary'
 result=dict(case=case,stationary=True,samples_mib=samples,median_mib=statistics.median(samples),sim_time=odom[-1],wall_s=time.monotonic()-start)
 (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:
 monitor.close();stop_tree(p,known_children=list(monitor.owned.values()));n.destroy_node();rclpy.shutdown();log.close()
