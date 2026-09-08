#!/usr/bin/env python3
"""Full rectangle motion audit; truth observed only by evaluator."""
import argparse,json,os,subprocess,time,hashlib
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from label_mission_capture import label
from prepare_mission_camera import prepare
from validate_tracking import stop


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gui',action='store_true');parser.add_argument('--profile',default='zero',choices=['zero','normal'])
    parser.add_argument('--follow-camera',action=argparse.BooleanOptionalAction,default=True,
                        help='follow AGV position in GUI while allowing free rotation and wheel zoom')
    parser.add_argument('--scene',type=Path)
    parser.add_argument('--length',type=float,default=3.);parser.add_argument('--width',type=float,default=2.)
    a=parser.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    req=yaml.safe_load(Path('src/agv_mission/config/rectangle_demo.yaml').read_text())
    req['road']['frame_id']='map';req['region'].update(length_m=a.length,width_m=a.width)
    req['region']['start_xy_m']=[6.,-a.width/2];req['mission_id']='rectangle_execution_trial'
    path=a.output/'request.yaml';path.write_text(yaml.safe_dump(req))
    os.environ.update(ROS_DOMAIN_ID='98',GZ_PARTITION='agv_rectangle_'+str(os.getpid()))
    extra=[]
    if a.scene:
        camera_path=prepare(Path('src/agv_description/config/linescan.yaml'),a.output/'camera.yaml')
        extra=['linescan:=true','linescan_backend:=optix','scene_manifest:='+str(a.scene.resolve()),
               'camera_config:='+str(camera_path.resolve()),'scan_speed_limit:=0.8','capture_dir:='+str(a.output/'raw')]
    log=(a.output/'simulation.log').open('w')
    sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','localization:=true',
        'localization_profile:='+a.profile,'localization_output_dir:='+str(a.output/'navigation'),
        'headless:='+str(not a.gui).lower(),'rviz:='+str(a.gui).lower(),
        'follow_camera:='+str(a.follow_camera).lower(),'spawn_x:=3']+extra,
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('rectangle_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};records=[];truth=[];run=None;runlog=None;images={}
    def status(m):
        v=json.loads(m.data);latest.update(status=v);records.append(v)
    def actual(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        truth.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    def image(m):images[m.header.stamp.sec*10**9+m.header.stamp.nanosec]=(hashlib.sha256(m.data).hexdigest(),m.width,m.height)
    node.create_subscription(Image,'/linescan/image_raw',image,2)
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
            '-p','output_dir:='+str(a.output/'mission'),'-p','capture:='+str(bool(a.scene)).lower()],stdout=runlog,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+700
        while latest.get('status',{}).get('state') not in ('COMPLETED','ACQUIRED','FAULT','CANCELED'):
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
        result={'passed':end['state'] in ('COMPLETED','ACQUIRED') and end['motion_state']=='HOLD',
            'scope':'rectangle motion only; capture integration pending','profile':a.profile,'gui_rviz':a.gui,
            'follow_camera':a.gui and a.follow_camera,
            'region_m':[a.length,a.width],'track_spacing_m':plan['actual_track_spacing_m'],'tracks':passes,'final':end}
        if a.scene and result['passed']:
            # Drain reliable metadata/image delivery after storage close acknowledgement.
            until=time.monotonic()+1
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            manifest=label(a.output/'mission',a.output/'navigation')
            from PIL import Image as PilImage
            captured=[]
            for track in plan['tracks']:
                blocks=sorted([b for b in manifest['blocks'] if b['track_id']==track['id']],key=lambda b:b['first_global_line'])
                assert blocks,'track has no captured blocks'
                raw=[]
                for b in blocks:
                    path=Path(b['image']);meta=json.loads(path.with_suffix('.json').read_text());raw.append(meta)
                    stamp=round(meta['last']['time_s']*1e9)
                    pixels=np.asarray(PilImage.open(path)).tobytes()
                    assert images.get(stamp)==(hashlib.sha256(pixels).hexdigest(),b['width'],b['rows']),'ROS/archive mismatch'
                    assert b['last_global_line']-b['first_global_line']+1==b['rows']
                start=np.array(track['scan_start_xyz_m']);finish=np.array(track['scan_end_xyz_m']);axis=(finish-start)/np.linalg.norm(finish-start)
                positions=[]
                for meta in raw:
                    for tag in meta['pose_tags']:
                        cp=np.array(tag['camera_position_world_m']);rot=np.array(tag['camera_rotation_world'])
                        ground=cp+rot[:,2]*(start[2]-cp[2])/rot[2,2]
                        positions.append(float((ground-start)@axis))
                assert min(positions)<=0 and max(positions)>=a.length,('ROI endpoints not acquired',positions[0],positions[-1])
                # A single sensor segment must span the ROI; no unreported line break.
                groups={}
                for meta in raw:groups.setdefault(meta['segment_id'],[]).append(meta)
                spanning=False
                for group in groups.values():
                    first=np.array(group[0]['first']['camera_position_world_m']);last=np.array(group[-1]['last']['camera_position_world_m'])
                    if (first-start)@axis<=0 and (last-start)@axis>=a.length:
                        spanning=True
                        for left,right in zip(group,group[1:]):assert right['first']['global_line']==left['last']['global_line']+1
                assert spanning,'no uninterrupted sensor segment spans ROI'
                captured.append({'track_id':track['id'],'blocks':len(blocks),'rows':sum(b['rows'] for b in blocks),
                    'truth_along_extent_m':[min(positions),max(positions)],'continuous_roi':True})
            result.update(scope='rectangle motion + OptiX raw capture + sparse fused labels; no correction/stitching',capture=captured)
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
        assert result['passed'],end
        assert all(t['running_samples']>0 for t in passes),'missing pass execution'
    finally:
        if run:stop(run)
        if runlog:runlog.close()
        node.destroy_node();rclpy.shutdown();stop(sim);log.close()


if __name__=='__main__':main()
