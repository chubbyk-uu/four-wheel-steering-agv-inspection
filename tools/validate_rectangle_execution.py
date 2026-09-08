#!/usr/bin/env python3
"""Full rectangle motion audit; truth observed only by evaluator."""
import argparse,json,os,subprocess,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from validate_tracking import stop


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gui',action='store_true');parser.add_argument('--profile',default='zero',choices=['zero','normal'])
    parser.add_argument('--length',type=float,default=3.);parser.add_argument('--width',type=float,default=2.)
    a=parser.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    req=yaml.safe_load(Path('src/agv_mission/config/rectangle_demo.yaml').read_text())
    req['road']['frame_id']='map';req['region'].update(length_m=a.length,width_m=a.width)
    req['region']['start_xy_m']=[6.,-a.width/2];req['mission_id']='rectangle_execution_trial'
    path=a.output/'request.yaml';path.write_text(yaml.safe_dump(req))
    os.environ.update(ROS_DOMAIN_ID='98',GZ_PARTITION='agv_rectangle_'+str(os.getpid()))
    log=(a.output/'simulation.log').open('w')
    sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','localization:=true',
        'localization_profile:='+a.profile,'localization_output_dir:='+str(a.output/'navigation'),
        'headless:='+str(not a.gui).lower(),'rviz:='+str(a.gui).lower(),'follow_camera:=false','spawn_x:=3'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('rectangle_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};records=[];truth=[];run=None;runlog=None
    def status(m):
        v=json.loads(m.data);latest.update(status=v);records.append(v)
    def actual(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        truth.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    node.create_subscription(String,'/localization/status',lambda m:latest.update(health=json.loads(m.data)),20)
    node.create_subscription(String,'/mission/status',status,100)
    node.create_subscription(Odometry,'/ground_truth/odom',actual,100)
    try:
        deadline=time.monotonic()+80
        while latest.get('health',{}).get('state')!='READY':
            assert time.monotonic()<deadline and sim.poll() is None,'startup failed'
            rclpy.spin_once(node,timeout_sec=.05)
        runlog=(a.output/'executor.log').open('w')
        run=subprocess.Popen(['ros2','run','agv_mission','execute_rectangle','--ros-args',
            '-p','use_sim_time:=true','-p','autostart:=true','-p','request:='+str(path),
            '-p','output_dir:='+str(a.output/'mission')],stdout=runlog,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+700
        while latest.get('status',{}).get('state') not in ('COMPLETED','FAULT','CANCELED'):
            assert run.poll() is None,'executor exited'
            assert time.monotonic()<deadline,'mission timeout'
            rclpy.spin_once(node,timeout_sec=.02)
        end=latest['status'];np.save(a.output/'truth_evaluation.npy',np.asarray(truth))
        plan=json.loads((a.output/'mission/plan.json').read_text())
        passes=[]
        for track in plan['tracks']:
            rows=[r for r in records if r.get('kind')=='PASS' and r.get('track_id')==track['id'] and r.get('tracker_state')=='RUNNING']
            passes.append({'track_id':track['id'],'running_samples':len(rows),
                'reference_error_max_m':max((r.get('reference_position_error_m',0) for r in rows),default=None)})
        result={'passed':end['state']=='COMPLETED' and end['motion_state']=='HOLD',
            'scope':'rectangle motion only; capture integration pending','profile':a.profile,'gui_rviz':a.gui,
            'region_m':[a.length,a.width],'track_spacing_m':plan['actual_track_spacing_m'],'tracks':passes,'final':end}
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
        assert result['passed'],end
        assert all(t['running_samples']>0 for t in passes),'missing pass execution'
    finally:
        if run:stop(run)
        if runlog:runlog.close()
        node.destroy_node();rclpy.shutdown();stop(sim);log.close()


if __name__=='__main__':main()
