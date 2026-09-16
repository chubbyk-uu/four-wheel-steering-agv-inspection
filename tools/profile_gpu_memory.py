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
parser.add_argument('--settle-wall-s',type=float,default=50.0)
parser.add_argument('--sample-count',type=int,default=10)
parser.add_argument('--spawn-x',type=float,default=-3.0)
parser.add_argument('--spawn-y',type=float,default=0.0)
parser.add_argument('--camera-config',type=Path,help='optional line-scan config override')
a=parser.parse_args();case=a.profile;out=a.output;out.mkdir(parents=True,exist_ok=False)
if a.settle_wall_s<1 or a.sample_count<1:parser.error('settle-wall-s and sample-count must be positive')
os.environ.update(ROS_DOMAIN_ID=str(100+os.getpid()%70),GZ_PARTITION='vram_'+str(os.getpid()))
common=['scene_manifest:='+str(a.scene.resolve()),'spawn_x:='+str(a.spawn_x),'spawn_y:='+str(a.spawn_y)]
if a.camera_config:common.append('camera_config:='+str(a.camera_config.resolve()))
if case=='full':args=['inspection.launch.py','session_dir:='+str((out/'session').resolve())]+common
else:args=['sim.launch.py','headless:='+('false' if case=='gui_optix' else 'true'),'rviz:=false','linescan:='+('false' if case=='server' else 'true'),'linescan_backend:=optix','capture_dir:='+str((out/'raw').resolve())]+common
rclpy.init();n=rclpy.create_node('vram_probe');odom=[];odom_wall=[];speed=[]
def receive(m):
 odom.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
 odom_wall.append(time.monotonic())
 speed.append((m.twist.twist.linear.x**2+m.twist.twist.linear.y**2)**.5)
n.create_subscription(Odometry,'/ground_truth/odom',receive,10)
log=(out/'launch.log').open('w');p=subprocess.Popen(['ros2','launch','agv_bringup',*args],stdout=log,stderr=log,start_new_session=True);monitor=ResourceMonitor(p.pid,out/'resources.jsonl');start=time.monotonic();samples=[]
try:
 while time.monotonic()-start<max(180,a.settle_wall_s+60):
  assert p.poll() is None,'launch exited'
  rclpy.spin_once(n,timeout_sec=.1)
  if odom and odom[-1]>=10 and time.monotonic()-start>=a.settle_wall_s:break
 else:raise RuntimeError('startup timeout')
 steady_wall=time.monotonic();steady_sim=odom[-1]
 for i in range(a.sample_count):
  samples.append(int(subprocess.check_output(['nvidia-smi','--query-gpu=memory.used','--format=csv,noheader,nounits'],text=True).strip()))
  deadline=time.monotonic()+1
  while time.monotonic()<deadline:
   rclpy.spin_once(n,timeout_sec=min(.05,deadline-time.monotonic()))
 assert speed[-1]<.01,'robot is not stationary'
 end=time.monotonic()
 result=dict(case=case,stationary=True,samples_mib=samples,median_mib=statistics.median(samples),
             first_odometry_wall_s=odom_wall[0]-start,steady_rtf=(odom[-1]-steady_sim)/(end-steady_wall),
             sim_time=odom[-1],wall_s=end-start)
 (out/'summary.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
finally:
 monitor.close();stop_tree(p,known_children=list(monitor.owned.values()));n.destroy_node();rclpy.shutdown();log.close()
