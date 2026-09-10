#!/usr/bin/env python3
"""Full rectangle motion audit; truth observed only by evaluator."""
import argparse,json,os,subprocess,time,hashlib,signal
import psutil
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger,SetBool
from sensor_msgs.msg import Image,JointState
from label_mission_capture import label
from prepare_mission_camera import prepare
from validate_tracking import stop


def stop_tree(process,known_children=()):
    # Gazebo launch may leave server/GUI children after its own exit.
    owned={p.pid:p for p in known_children}
    try:
        parent=psutil.Process(process.pid)
        owned.update({p.pid:p for p in [parent,*parent.children(recursive=True)]})
    except psutil.NoSuchProcess:pass
    owned=list(owned.values())
    for sig,timeout in ((signal.SIGINT,8),(signal.SIGTERM,4),(signal.SIGKILL,2)):
        for p in owned:
            try:p.send_signal(sig)
            except psutil.NoSuchProcess:pass
        _,owned=psutil.wait_procs(owned,timeout=timeout)
        if not owned:break


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--gui',action='store_true');parser.add_argument('--profile',default='zero',choices=['zero','normal'])
    parser.add_argument('--follow-camera',action=argparse.BooleanOptionalAction,default=True,
                        help='keep AGV in view in GUI while allowing wheel zoom')
    parser.add_argument('--scene',type=Path)
    parser.add_argument('--request',type=Path,help='validate an existing rectangle request, including a generated rescan')
    parser.add_argument('--pause-once',action='store_true',help='pause early in first PASS, retain camera frame, then resume')
    parser.add_argument('--stop-after-pause',choices=['cancel','localization_timeout'],help='instead of resuming, validate partial-frame termination')
    parser.add_argument('--moving-fault',choices=['localization_timeout','camera_disabled'])
    parser.add_argument('--length',type=float,default=3.);parser.add_argument('--width',type=float,default=2.)
    a=parser.parse_args();a.pause_once=a.pause_once or bool(a.stop_after_pause);a.output.mkdir(parents=True,exist_ok=False)
    if a.moving_fault and (a.pause_once or not a.scene):parser.error('moving fault requires a scene and cannot combine with pause probes')
    req=yaml.safe_load((a.request or Path('src/agv_mission/config/rectangle_demo.yaml')).read_text())
    if a.request:
        a.length=req['region']['length_m'];a.width=req['region']['width_m']
    else:
        req['road']['frame_id']='map';req['region'].update(length_m=a.length,width_m=a.width)
        req['region']['start_xy_m']=[6.,-a.width/2];req['mission_id']='rectangle_execution_trial'
    path=a.output/'request.yaml';path.write_text(yaml.safe_dump(req))
    os.environ.update(ROS_DOMAIN_ID='98',GZ_PARTITION='agv_rectangle_'+str(os.getpid()))
    extra=[]
    if a.scene:
        camera_path=prepare(Path('src/agv_description/config/linescan.yaml'),a.output/'camera.yaml')
        extra=['linescan:=true','linescan_backend:=optix','scene_manifest:='+str(a.scene.resolve()),
               'camera_config:='+str(camera_path.resolve()),'scan_speed_limit:='+str(yaml.safe_load(camera_path.read_text())['max_scan_speed_m_s']),'capture_dir:='+str(a.output/'raw')]
    log=(a.output/'simulation.log').open('w')
    sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','localization:=true',
        'localization_profile:='+a.profile,'localization_output_dir:='+str(a.output/'navigation'),
        'headless:='+str(not a.gui).lower(),'rviz:='+str(a.gui).lower(),
        'follow_camera:='+str(a.follow_camera).lower(),'spawn_x:=3']+extra,
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('rectangle_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};records=[];truth=[];run=None;runlog=None;images={};joint_samples=[]
    pause_client=node.create_client(Trigger,'/mission/pause');resume_client=node.create_client(Trigger,'/mission/resume')
    cancel_client=node.create_client(Trigger,'/mission/cancel');suspended=None;stop_injected=False
    pause_future=None;resume_future=None;paused_at=None;image_count_at_pause=None;pause_report={}
    camera_client=node.create_client(SetBool,'/linescan/set_enabled');moving_injection=None;camera_future=None
    def status(m):
        v=json.loads(m.data);latest.update(status=v);records.append(v)
    def actual(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        latest['truth_position']=[p.x,p.y,p.z];v=m.twist.twist
        latest['truth_speed']=float(np.hypot(v.linear.x,v.linear.y))
        truth.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z,q.x,q.y,q.z,q.w])
    def image(m):images[m.header.stamp.sec*10**9+m.header.stamp.nanosec]=(hashlib.sha256(m.data).hexdigest(),m.width,m.height)
    def joints(m):
        names={name:i for i,name in enumerate(m.name)}
        if all(k+'_steer_joint' in names for k in ('fl','fr','rl','rr')):
            joint_samples.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9]+[m.position[names[k+'_steer_joint']] for k in ('fl','fr','rl','rr')])
    node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Image,'/linescan/image_raw',image,2)
    node.create_subscription(String,'/localization/status',lambda m:latest.update(health=json.loads(m.data)),20)
    node.create_subscription(String,'/mission/status',status,qos_profile_sensor_data)
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
        while not (latest.get('status',{}).get('state') in ('COMPLETED','ACQUIRED','FAULT','CANCELED') and latest['status'].get('motion_state')=='HOLD' and latest['status'].get('capture_active') is False):
            assert run.poll() is None,'executor exited'
            assert time.monotonic()<deadline,'mission timeout'
            rclpy.spin_once(node,timeout_sec=.02)
            current=latest.get('status',{})
            if a.moving_fault and moving_injection is None and current.get('kind')=='PASS' and current.get('track_id')==0 and current.get('tracker_state')=='RUNNING' and current.get('profile_time_s',0)>2.5:
                assert latest.get('truth_speed',0)>.4,'fault must be injected while moving'
                moving_injection=dict(time_s=current['time_s'],position_m=latest['truth_position'],speed_m_s=latest['truth_speed'])
                if a.moving_fault=='localization_timeout':
                    matches=[p for p in psutil.Process(sim.pid).children(recursive=True) if any(v.endswith('/measurement_adapter') for v in p.cmdline())]
                    assert len(matches)==1
                    suspended=matches[0];suspended.send_signal(signal.SIGSTOP)
                else:
                    assert camera_client.service_is_ready()
                    camera_future=camera_client.call_async(SetBool.Request(data=False))
            if a.pause_once:
                if pause_future is None and current.get('kind')=='PASS' and current.get('track_id')==0 and current.get('tracker_state')=='RUNNING' and current.get('profile_time_s',0)>1.2:
                    assert pause_client.service_is_ready()
                    pause_future=pause_client.call_async(Trigger.Request())
                    pause_report['requested_time_s']=current['time_s']
                if pause_future is not None and pause_future.done():assert pause_future.result().success,pause_future.result().message
                if current.get('state')=='PAUSED' and paused_at is None:
                    paused_at=current['time_s'];image_count_at_pause=len(images)
                    pause_report['stopped_time_s']=paused_at
                if paused_at is not None and resume_future is None and not stop_injected and current.get('state')=='PAUSED':
                    assert len(images)==image_count_at_pause,'partial image emitted during ordinary pause'
                    assert current['motion_state']=='HOLD' and current['command_body']==[0.,0.,0.]
                    if current['time_s']-paused_at>=3:
                        if a.stop_after_pause=='cancel':
                            assert cancel_client.service_is_ready()
                            resume_future=cancel_client.call_async(Trigger.Request());stop_injected=True
                        elif a.stop_after_pause=='localization_timeout':
                            # Suspend only this launched simulator's adapter, never an unrelated process.
                            matches=[p for p in psutil.Process(sim.pid).children(recursive=True) if any(v.endswith('/measurement_adapter') for v in p.cmdline())]
                            assert len(matches)==1,'ambiguous owned localization adapter'
                            suspended=matches[0];suspended.send_signal(signal.SIGSTOP);stop_injected=True
                        else:
                            assert resume_client.service_is_ready()
                            resume_future=resume_client.call_async(Trigger.Request())
                            pause_report['resume_request_time_s']=current['time_s']
                if resume_future is not None and resume_future.done():assert resume_future.result().success,resume_future.result().message
        if suspended:
            suspended.send_signal(signal.SIGCONT);suspended=None
            until=time.monotonic()+1
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            assert latest['status']['state']=='FAULT','localization recovery restarted faulted mission'
        end=latest['status'];np.save(a.output/'truth_evaluation.npy',np.asarray(truth))
        np.save(a.output/'joint_evaluation.npy',np.asarray(joint_samples))
        assert joint_samples,'no actual wheel steering samples'
        plan=json.loads((a.output/'mission/plan.json').read_text())
        passes=[]
        for track in plan['tracks']:
            rows=[r for r in records if r.get('kind')=='PASS' and r.get('track_id')==track['id'] and r.get('tracker_state')=='RUNNING']
            passes.append({'track_id':track['id'],'running_samples':len(rows),
                'reference_error_max_m':max((r.get('reference_position_error_m',0) for r in rows),default=None)})
        result={'passed':end['state'] in ('COMPLETED','ACQUIRED') and end['motion_state']=='HOLD',
            'scope':'rectangle motion only; capture integration pending','profile':a.profile,'gui_rviz':a.gui,
            'follow_camera':a.gui and a.follow_camera,
            'wheel_steering_range_rad':{k:[float(np.min(np.asarray(joint_samples)[:,i+1])),float(np.max(np.asarray(joint_samples)[:,i+1]))] for i,k in enumerate(('fl','fr','rl','rr'))},
            'region_m':[a.length,a.width],'track_spacing_m':plan['actual_track_spacing_m'],'tracks':passes,'final':end}
        if a.pause_once and not a.stop_after_pause:
            assert resume_future is not None and resume_future.done() and resume_future.result().success,'pause/resume not exercised'
            assert all(r['command_body'][0]>=-1e-9 for r in records if r.get('kind')=='PASS'),'reverse PASS command'
            result['pause_resume']=pause_report
        if a.stop_after_pause:
            expected='CANCELED' if a.stop_after_pause=='cancel' else 'FAULT'
            assert stop_injected and end['state']==expected,end
            if expected=='FAULT':assert end['reason']=='STALE_OR_UNREADY_LOCALIZATION',end
            assert end['motion_state']=='HOLD' and end['capture_active'] is False
            until=time.monotonic()+1
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            from PIL import Image as PilImage
            raw=list((a.output/'raw').glob('*/block_*.json'))
            assert raw and len(raw)==1,'expected one unfinished first frame at pause'
            m=json.loads(raw[0].read_text());assert 0<m['rows']<4096 and m['end_reason']!='full'
            stamp=round(m['last']['time_s']*1e9);pixels=np.asarray(PilImage.open(raw[0].with_suffix('.pgm'))).tobytes()
            assert images.get(stamp)==(hashlib.sha256(pixels).hexdigest(),m['width'],m['rows'])
            result.update(passed=True,scope='paused partial frame terminated explicitly; incomplete ROI, no coverage success',
                termination_probe=dict(case=a.stop_after_pause,tail_rows=m['rows'],sensor_end_reason=m['end_reason'],ros_archive_identical=True),pause_resume=pause_report)
        if a.moving_fault:
            assert moving_injection is not None and end['state']=='FAULT' and end['motion_state']=='HOLD' and end['capture_active'] is False,end
            expected='STALE_OR_UNREADY_LOCALIZATION' if a.moving_fault=='localization_timeout' else 'CAMERA_DISABLED_UNEXPECTEDLY'
            assert end['reason']==expected,end
            if camera_future:assert camera_future.done() and camera_future.result().success
            first_fault=next(r for r in records if r['state']=='FAULT')
            latency=first_fault['time_s']-moving_injection['time_s']
            distance=float(np.linalg.norm(np.array(latest['truth_position'])-moving_injection['position_m']))
            cfg=yaml.safe_load(Path('src/agv_mission/config/tracking.yaml').read_text())
            platform=yaml.safe_load(Path('src/agv_description/config/platform.yaml').read_text())
            reaction_budget=max(cfg['wall_timeout_s'],cfg['capture_state_timeout_s'])+.15
            assert first_fault['command_body']==[0.,0.,0.] and 0<=latency<reaction_budget
            budget=moving_injection['speed_m_s']*reaction_budget+moving_injection['speed_m_s']**2/(2*platform['drive_decel'])+.10
            assert distance<budget and latest['truth_speed']<.025
            from PIL import Image as PilImage
            blocks=list((a.output/'raw').glob('*/block_*.json'));assert blocks
            until=time.monotonic()+1
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            for path in blocks:
                m=json.loads(path.read_text());pixels=np.asarray(PilImage.open(path.with_suffix('.pgm'))).tobytes()
                assert images.get(round(m['last']['time_s']*1e9))==(hashlib.sha256(pixels).hexdigest(),m['width'],m['rows'])
            result.update(passed=True,scope='moving fault braking and valid tail archive; incomplete ROI',moving_fault=dict(case=a.moving_fault,injection=moving_injection,reaction_sim_s=latency,stopping_distance_m=distance,stopping_budget_m=budget,blocks=len(blocks),ros_archive_identical=True))
        if a.scene and result['passed'] and not a.stop_after_pause and not a.moving_fault:
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
                    assert meta['end_reason'] in ('full','capture_toggle'),'unexpected sensor interruption'
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
            if a.pause_once:
                crossing=[]
                for b in manifest['blocks']:
                    tags=b['pose_tags']
                    if tags[0]['time_s']<pause_report['stopped_time_s'] and tags[-1]['time_s']>pause_report['resume_request_time_s']:
                        assert b['rows']==4096,'pause split a full frame'
                        adjacent=[(l,r) for l,r in zip(tags,tags[1:]) if r['global_line']==l['global_line']+1 and r['time_s']-l['time_s']>2]
                        assert adjacent,'missing tags directly across stopped time'
                        crossing.append(dict(block_id=b['block_id'],rows=b['rows'],pause_gap_s=adjacent[0][1]['time_s']-adjacent[0][0]['time_s']))
                assert crossing,'no complete image straddled pause'
                result['pause_resume']['crossing_blocks']=crossing
                result['pause_resume']['no_partial_frame_during_pause']=True
            result.update(scope='rectangle motion + OptiX raw capture + sparse fused labels; no correction/stitching',capture=captured)
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
        assert result['passed'],end
        if not a.stop_after_pause and not a.moving_fault:assert all(t['running_samples']>0 for t in passes),'missing pass execution'
    finally:
        if suspended:
            try:suspended.send_signal(signal.SIGCONT)
            except psutil.NoSuchProcess:pass
        if run:stop(run)
        if runlog:runlog.close()
        node.destroy_node();rclpy.shutdown();stop_tree(sim);log.close()


if __name__=='__main__':main()
