#!/usr/bin/env python3
"""GUI integration check; run against an already loaded project Gazebo scene."""
import argparse
from pathlib import Path
import sys,time,json,subprocess,math,os,signal
sys.path.insert(0,str(Path(__file__).resolve().parent))
from validate_motion import Evaluator
import rclpy

def message(topic):
    return json.loads(subprocess.check_output(['gz','topic','-t',topic,'-e','-n','1','--json-output'],text=True,timeout=8))

def angle(a,b):
    def q(p):return [float(p['orientation'].get(k,0)) for k in ('x','y','z','w')]
    x,y=q(a),q(b);norm=math.sqrt(sum(v*v for v in x)*sum(v*v for v in y))
    return 2*math.acos(min(1,abs(sum(a*b for a,b in zip(x,y)))/norm))

parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
rclpy.init();n=Evaluator();report={}
try:
    deadline=time.monotonic()+60
    while not(n.odom and n.state):
        rclpy.spin_once(n,timeout_sec=.1)
        assert time.monotonic()<deadline
    n.run_for(3,(0,0,0));before=message('/gui/camera/pose');state=message('/gui/currently_tracked')
    assert state.get('trackMode')=='FOLLOW_FREE_LOOK' and state.get('followTarget',{}).get('name')=='agv',state
    report['initial_tracking']=state
    win=subprocess.check_output(['xdotool','search','--onlyvisible','--name','^Gazebo Sim$'],text=True).strip().splitlines()[0]
    subprocess.run(['xdotool','mousemove','--window',win,'280','400'],check=True)
    subprocess.run(['xdotool','mousedown','2'],check=True)
    for i in range(1,21):
        subprocess.run(['xdotool','mousemove','--window',win,str(280+i*5),str(400-i*2)],check=True);time.sleep(.02)
    subprocess.run(['xdotool','mouseup','2'],check=True)
    n.run_for(1,(0,0,0));rotated=message('/gui/camera/pose')
    n.run_for(2,(0,0,0));settled=message('/gui/camera/pose')
    def distance():
        st=message('/gui/currently_tracked');p=st['followOffset'];return math.sqrt(sum(p.get(k,0)**2 for k in ('x','y','z')))
    baseline=distance()
    subprocess.run(['xdotool','mousemove','--window',win,'300','400','click','--repeat','3','--delay','200','4'],check=True)
    n.run_for(2,(0,0,0));zoom_in=distance();zoom_pose=message('/gui/camera/pose')
    n.run_for(2,(0,0,0));zoom_hold=distance()
    subprocess.run(['xdotool','click','--repeat','2','--delay','200','5'],check=True)
    n.run_for(2,(0,0,0));zoom_out=distance();settled=message('/gui/camera/pose')
    report['zoom_in_pose']=zoom_pose
    assert abs(sum((float(zoom_pose['position'].get(k,0))-float(before['position'].get(k,0)))**2 for k in ('x','y','z')))>0.1
    report.update(original_distance_m=baseline,zoom_in_distance_m=zoom_in,zoom_held_distance_m=zoom_hold,zoom_out_distance_m=zoom_out)
    assert zoom_in<baseline*.9 and zoom_out>zoom_in*1.1 and abs(zoom_hold-zoom_in)<.001,report
    start=n.odom.pose.pose.position.x
    n.run_for(5,(.5,0,0));n.run_for(2,(0,0,0))
    final=message('/gui/camera/pose');end=n.odom.pose.pose.position.x
    report.update(initial_pose=before,rotated_pose=rotated,settled_pose=settled,final_pose=final,
      mouse_rotation_rad=angle(before,rotated),released_rotation_drift_rad=angle(rotated,settled),
      moving_rotation_drift_rad=angle(settled,final),robot_displacement_m=end-start,
      camera_displacement_m=float(final['position']['x'])-float(settled['position']['x']),final_motion_state=n.state,
      final_tracking=message('/gui/currently_tracked'))
    assert report['mouse_rotation_rad']>.05,report
    assert report['released_rotation_drift_rad']<.01,report
    assert report['moving_rotation_drift_rad']<.01,report
    assert end-start>1.5 and abs(report['camera_displacement_m']-(end-start))<.05,report
    assert n.state=='HOLD'
    report['distance_after_drive_m']=distance()
    assert abs(report['distance_after_drive_m']-zoom_out)<.001
    report['passed']=True
finally:
    subprocess.run(['xdotool','mouseup','2'],check=False)
    try:
        n.run_for(2,(0,0,0))
    finally:
        n.destroy_node();rclpy.shutdown()
        Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
