#!/usr/bin/env python3
"""Large AGV motion, passive suspension, coherent TF and dual eight-line scan smoke test."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path


def evaluate(a):
 import rclpy, math
 from sensor_msgs.msg import PointCloud2
 from tf2_msgs.msg import TFMessage
 from validate_motion import Evaluator
 rclpy.init();n=Evaluator();clouds={};tf_bad=[];susp=[]
 display=dict(max_attachment_error_m=0.,max_relative_center_step_m=0.,max_snapshot_gap_s=0.,nonmonotonic_stamps=0)
 previous={};last_stamp=None
 def check_display(m):
  nonlocal previous,last_stamp
  stamps={(t.header.stamp.sec,t.header.stamp.nanosec) for t in m.transforms}
  tf_bad.append(len(m.transforms)!=13 or len(stamps)!=1)
  if tf_bad[-1]:return
  transforms={t.child_frame_id:t for t in m.transforms};centres={}
  stamp=next(iter(stamps));stamp=stamp[0]+stamp[1]*1e-9
  if last_stamp is not None:
   display['max_snapshot_gap_s']=max(display['max_snapshot_gap_s'],stamp-last_stamp)
   display['nonmonotonic_stamps']+=int(stamp<=last_stamp)
  last_stamp=stamp
  for corner in ('fl','fr','rl','rr'):
   for part in ('suspension','steer','wheel'):
    name=corner+'_'+part+'_link';t=transforms[name]
    assert t.header.frame_id=='base_link'
    v=t.transform.translation;centres[name]=(v.x,v.y,v.z)
    if name in previous:
     display['max_relative_center_step_m']=max(display['max_relative_center_step_m'],math.dist(centres[name],previous[name]))
   for parent,child,dz in [('suspension','steer',-.10),('steer','wheel',-.20)]:
    a0=centres[corner+'_'+parent+'_link'];b0=centres[corner+'_'+child+'_link']
    error=math.dist((a0[0],a0[1],a0[2]+dz),b0)
    display['max_attachment_error_m']=max(display['max_attachment_error_m'],error)
  previous=centres
 for side in ('left','right'):
  n.create_subscription(PointCloud2,'/lidar/'+side+'/points',lambda m,k=side:clouds.update({k:dict(width=m.width,height=m.height,frame=m.header.frame_id,bytes=len(m.data))}),10)
 if a.gui:
  n.create_subscription(TFMessage,'/visualization/tf',check_display,10)
 try:
  deadline=time.monotonic()+90
  while not (n.odom and n.joints and n.state):
   rclpy.spin_once(n,timeout_sec=.1)
   assert time.monotonic()<deadline,'feedback missing'
  n.run_for(5,(0,0,0));height=n.odom.pose.pose.position.z
  assert .60<height<.70,('suspension equilibrium',height)
  # Lowest battery corner after suspension settles, including actual body tilt.
  import numpy as np
  from scipy.spatial.transform import Rotation
  q=n.odom.pose.pose.orientation
  rotation=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
  corners=np.array([[x,y,-.3984] for x in (-.36,.36) for y in (-.55,.55)])
  battery_clearance=float((corners@rotation.T)[:,2].min()+height)
  assert abs(battery_clearance-.25)<.005,('loaded battery clearance',battery_clearance)
  for name,pos in zip(n.joints.name,n.joints.position):
   if name.endswith('_suspension_joint'):susp.append(pos)
  assert len(susp)==4 and max(map(abs,susp))<.04,susp
  # Use current platform's slower steering profile; preserve geometric error limits.
  results=[]
  for name,cmd in [('forward',(.5,0,0)),('lateral',(0,.5,0)),('diagonal',(.3,.3,0)),('rotate',(0,0,.3)),('reverse',(-.5,0,0))]:
   n.run_for(6,cmd);assert n.state=='DRIVE',(name,n.state)
   x,y,yaw=n.pose();t=n.odom.header.stamp.sec+n.odom.header.stamp.nanosec*1e-9
   n.run_for(2,cmd);xx,yy,aa=n.pose();dt=n.odom.header.stamp.sec+n.odom.header.stamp.nanosec*1e-9-t
   bx=math.cos(yaw)*(xx-x)+math.sin(yaw)*(yy-y);by=-math.sin(yaw)*(xx-x)+math.cos(yaw)*(yy-y)
   pe=math.hypot(bx-cmd[0]*dt,by-cmd[1]*dt);ae=abs(math.atan2(math.sin(aa-yaw),math.cos(aa-yaw))-cmd[2]*dt)
   assert pe<.15 and ae<.12,(name,pe,ae)
   results.append(dict(case=name,position_error_m=pe,yaw_error_rad=ae))
   n.run_for(3,(0,0,0));assert n.state=='HOLD'
  n.run_for(10,(10/3.6,0,0));speed=n.odom.twist.twist.linear.x
  assert abs(speed-10/3.6)<.15,speed
  n.run_for(8,(0,0,0));assert n.state=='HOLD'
  assert set(clouds)=={'left','right'},clouds
  assert all(c['width']*c['height']==8000 and c['bytes']>0 for c in clouds.values()),clouds
  if a.gui:
   assert tf_bad and not any(tf_bad),sum(tf_bad)
   assert display['max_attachment_error_m']<1e-8,display
   assert display['max_relative_center_step_m']<.01,display
   assert display['nonmonotonic_stamps']==0 and display['max_snapshot_gap_s']<.10,display
  report=dict(passed=True,gui_rviz=a.gui,equilibrium_base_height_m=height,suspension_positions_m=susp,loaded_battery_clearance_m=battery_clearance,cases=results,max_speed_measured_m_s=speed,clouds=clouds,tf_snapshots=len(tf_bad),bad_tf_snapshots=sum(tf_bad),display_geometry=display,final_state=n.state,scope='ideal velocity/position servos, no claim of motor torque saturation or autonomous obstacle avoidance')
  Path(a.output).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
 finally:
  n.command((0,0,0));n.destroy_node();rclpy.shutdown()


def main():
 p=argparse.ArgumentParser();p.add_argument('--gui',action='store_true');p.add_argument('--evaluate',action='store_true');p.add_argument('--output',required=True);a=p.parse_args()
 if a.evaluate:evaluate(a);return
 env=dict(os.environ,ROS_DOMAIN_ID='89',GZ_PARTITION='agv_large_'+str(os.getpid()))
 with open(a.output+'.sim.log','w') as log:
  sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','headless:='+str(not a.gui).lower(),'rviz:='+str(a.gui).lower()],env=env,stdout=log,stderr=log,start_new_session=True)
  try:
   r=subprocess.run([sys.executable,__file__,*sys.argv[1:],'--evaluate'],env=env,timeout=300)
   if r.returncode:raise SystemExit(r.returncode)
  finally:
   if sim.poll() is None:
    os.killpg(sim.pid,signal.SIGINT)
    try:sim.wait(timeout=20)
    except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
if __name__=='__main__':main()
