#!/usr/bin/env python3
"""C++ GZ rendering acceptance: full-width capture, continuity, ROS delivery, timings."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


# Steady real-time factor a joint capture acceptance must reach. Lowered from
# 0.95 to 0.92 on 2026-09-12 by the project owner. The simulation clock is
# authoritative and the encoder triggers by distance, so this bounds how long a
# run takes, not whether its data is valid; the measured control loop peaked at
# 6 ms against its 50 ms stall threshold at 0.944, so it is not the binding
# constraint either. Measure it from a freshly restarted WSL and record how many
# simulations have run in that instance: the figure decays with the instance, not
# with the code -- six runs in 75 minutes fell from 0.9588 to 0.8669, and a
# wsl --shutdown restored the identical arm from 0.8669 to 0.9467. A figure
# without that context is not comparable to another.
REALTIME_FLOOR=.92


def evaluate(args):
    import rclpy
    import numpy as np
    import yaml
    from PIL import Image
    from sensor_msgs.msg import Image as RosImage
    from std_srvs.srv import SetBool
    from std_msgs.msg import String
    from validate_motion import Evaluator
    rclpy.init()
    node=Evaluator()
    source_config=yaml.safe_load(Path(args.camera_config or "src/agv_description/config/linescan.yaml").read_text())
    from agv_linescan.encoder import line_spacing
    spacing=line_spacing(source_config,.2)
    platform_config=yaml.safe_load(Path("src/agv_description/config/platform.yaml").read_text())
    braking_wait=abs(args.speed)/platform_config["drive_decel"]+1
    def run_for(seconds, command):
        start=node.get_clock().now().nanoseconds*1e-9
        # Explicitly allow slower-than-real-time rendering benchmarks. Bound
        # total work by expected scan lines, and separately detect a dead clock.
        deadline=time.monotonic()+max(60,seconds+seconds*abs(args.speed)/spacing*.12)
        previous=start; advanced=time.monotonic()
        while node.get_clock().now().nanoseconds*1e-9-start<seconds:
            node.command(command); rclpy.spin_once(node,timeout_sec=.01)
            current=node.get_clock().now().nanoseconds*1e-9
            if current>previous: previous=current; advanced=time.monotonic()
            assert time.monotonic()<deadline and time.monotonic()-advanced<30,'render simulation stalled'
    client=node.create_client(SetBool,'/linescan/set_enabled')
    received=[]
    received_at=[]
    received_payloads={}
    duplicate_stamps=[]
    statuses=[]
    world_transform=[]
    display_status={}
    correction_status={}
    corrected_payloads={}
    if args.correction_profile:
        def corrected_receive(m):
            stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
            assert stamp not in corrected_payloads, "duplicate corrected image"
            corrected_payloads[stamp]=hashlib.sha256(m.data).hexdigest()
        node.create_subscription(RosImage,"/linescan/image_corrected",corrected_receive,2)
        node.create_subscription(String,"/linescan/correction_status",lambda m:correction_status.update(json.loads(m.data)),10)
    display_frames=0
    display_bad_stamps=0
    if args.rviz:
        from tf2_msgs.msg import TFMessage
        def receive_tf(message):
            nonlocal display_frames, display_bad_stamps
            display_frames+=1
            stamps={(t.header.stamp.sec,t.header.stamp.nanosec) for t in message.transforms}
            if len(message.transforms)!=13 or len(stamps)!=1: display_bad_stamps+=1
            for transform in message.transforms:
                if transform.header.frame_id=='world' and transform.child_frame_id=='base_link':
                    p=transform.transform.translation
                    world_transform[:]=[p.x,p.y,p.z]
        node.create_subscription(TFMessage,'/visualization/tf',receive_tf,10)
        node.create_subscription(String,'/visualization/status',lambda m: display_status.update(json.loads(m.data)),10)
    node.create_subscription(String,'/linescan/status',lambda m: statuses.append(m.data),10)
    def receive(m):
        received.append((m.width,m.height,m.header.stamp.sec+m.header.stamp.nanosec*1e-9,len(m.data)))
        received_at.append(time.monotonic())
        stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        if stamp in received_payloads: duplicate_stamps.append(stamp)
        received_payloads[stamp]=(hashlib.sha256(m.data).hexdigest(),m.encoding,m.step,m.header.frame_id)
    node.create_subscription(RosImage,'/linescan/image_raw',receive,2)
    def enable(value, expect_failure=None):
        request=SetBool.Request(); request.data=value
        future=client.call_async(request)
        deadline=time.monotonic()+90
        while not future.done():
            node.command((args.speed,0,0) if value else (0,0,0))
            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()>deadline: raise RuntimeError('capture service timeout')
        if (args.expect_terrain_failure if expect_failure is None else expect_failure) and value:
            assert not future.result().success, 'bad terrain must not enable capture'
        else:
            assert future.result().success
        return future.result()
    try:
        assert client.wait_for_service(timeout_sec=90), 'render plugin missing'
        deadline=time.monotonic()+90
        while not (node.odom and node.joints and node.state):
            rclpy.spin_once(node,timeout_sec=.1)
            assert time.monotonic()<deadline,'simulation feedback missing'
        if args.backend == 'optix':
            run_for(2,(0,0,0))  # Let the imported robot settle on the actual collision mesh.
        run_for(args.warmup,(args.speed,0,0))
        if args.rviz:
            deadline=time.monotonic()+30
            while not any(i.node_name=='agv_rviz' for i in node.get_subscriptions_info_by_topic('/linescan/image_preview')):
                run_for(.1,(args.speed,0,0))
                assert time.monotonic()<deadline, 'RViz image subscription missing'
        if args.correction_profile:
            deadline=time.monotonic()+30
            while not correction_status.get("ready"):
                assert not correction_status.get("error"),correction_status
                run_for(.1,(args.speed,0,0))
                assert time.monotonic()<deadline,"correction node not ready"
        enable(True)
        if args.expect_terrain_failure:
            before=node.get_clock().now().nanoseconds
            run_for(max(2,braking_wait),(0,0,0))
            assert node.state=='HOLD' and node.get_clock().now().nanoseconds>before
            assert not received
            assert any('terrain_sampling_failure' in m for m in statuses), statuses
            enable(True)  # Recovery attempt with missing data must still fail.
            repaired=False
            if args.repair_missing_tile:
                import shutil
                source,destination=map(Path,args.repair_missing_tile)
                assert source.is_file() and not destination.exists()
                shutil.copyfile(source,destination)
                enable(True,expect_failure=False)
                run_for(args.distance/abs(args.speed),(args.speed,0,0))
                run_for(max(2,braking_wait),(0,0,0))
                enable(False)
                run_for(.5,(0,0,0))
                assert received and node.state=='HOLD'
                repaired=True
            run_for(.2,(0,0,0))
            report=dict(passed=True,test='missing_terrain_file',capture_disabled=True,
                        simulation_clock_continues=True,recovered_without_restart=repaired,received_blocks=len(received),final_motion_state=node.state)
            (Path(args.archive)/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
            print(json.dumps(report),flush=True)
            return
        if args.rviz: assert world_transform, 'RViz world -> base_link transform missing'
        tf_start=list(world_transform)
        odom_start=node.odom
        wall_start=time.monotonic()
        sim_start=node.get_clock().now().nanoseconds*1e-9
        pause_interval=None
        if args.pause_once:
            run_for(args.distance/abs(args.speed)/2,(args.speed,0,0))
            run_for(max(2,braking_wait),(0,0,0))
            assert node.state=='HOLD'
            pause_start=node.get_clock().now().nanoseconds*1e-9
            run_for(1,(0,0,0))
            pause_interval=[pause_start,node.get_clock().now().nanoseconds*1e-9]
            run_for(args.distance/abs(args.speed)/2,(args.speed,0,0))
        else:
            run_for(args.distance/abs(args.speed),(args.speed,0,0))
        wall_end=time.monotonic()
        elapsed=wall_end-wall_start
        simulated=node.get_clock().now().nanoseconds*1e-9-sim_start
        tf_end=list(world_transform)
        odom_end=node.odom
        if args.stop_capture_before_brake: enable(False)
        run_for(max(2,braking_wait),(0,0,0))
        if not args.stop_capture_before_brake: enable(False)
        run_for(.5,(0,0,0))
        assert node.state=='HOLD'
        if args.correction_profile:
            deadline=time.monotonic()+15
            while len(corrected_payloads)<len(received) or correction_status.get('blocks',0)<len(received):
                assert not correction_status.get('error'),correction_status
                run_for(.1,(0,0,0))
                assert time.monotonic()<deadline,'corrected blocks missing'
        if args.gui:
            windows=subprocess.run(['xwininfo','-root','-tree'],capture_output=True,text=True,timeout=5)
            (Path(args.archive)/'windows.txt').write_text(windows.stdout+windows.stderr)
            assert windows.returncode==0 and 'Gazebo' in windows.stdout, 'Gazebo GUI window missing'
            if args.rviz: assert 'RViz' in windows.stdout, 'RViz window missing'
        session=next(Path(args.archive).glob('session_cpp_*'))
        blocks=[json.loads(p.read_text()) for p in sorted(session.glob('block_*.json'))]
        config=yaml.safe_load((session/'calibration.yaml').read_text())
        if pause_interval:
            assert any(b['first']['time_s']<pause_interval[0] and b['last']['time_s']>pause_interval[1]
                       and b['rows']==args.block_rows for b in blocks), 'pause did not preserve partial frame'

        geometry_checks=[]
        if args.backend == 'optix':
            from collections import Counter
            events=[json.loads(line) for line in (session/'events.jsonl').read_text().splitlines()]
            motion=[e['motion_condition'] for e in events if 'motion_condition' in e]
            diagnostics=dict(speed_m_s=args.speed,simulation_seconds=simulated,wall_seconds=elapsed,
                real_time_factor=simulated/elapsed,final_motion_state=node.state,
                archived_blocks=len(blocks),archived_lines=sum(b['rows'] for b in blocks),
                expected_command_distance_lines=args.distance/spacing,received_blocks=len(received),
                event_reasons=dict(Counter(e['reason'] for e in events)),
                motion_rejection_examples=motion[:8],
                final_metrics=blocks[-1]['cumulative_sampling_metrics'] if blocks else {},
                queue_peak_lines=max((b.get('sampling_queue_high_water_lines',0) for b in blocks),default=0),
                queue_max_wait_s=max((b.get('sampling_queue_observed_wait_max_s',0) for b in blocks),default=0),
                full_acceptance_passed=False,
                ros_missing_block_ids=[b['block_id'] for b in blocks
                    if not any(abs(t-b['last']['time_s'])<1e-8 for _,_,t,_ in received)])
            if motion:
                diagnostics['motion_rejection_maxima']={
                    'wheel_speed_spread_m_s':max(m['wheel_speed_spread_m_s'] for m in motion),
                    'max_abs_steer_rad':max(m['max_abs_steer_rad'] for m in motion),
                    'abs_encoder_yaw_rad_s':max(abs(m['encoder_yaw_estimate_rad_s']) for m in motion)}
            (Path(args.archive)/'diagnostics.json').write_text(json.dumps(diagnostics,indent=2)+'\n')
        assert blocks and sum(b['rows'] for b in blocks)>args.distance/spacing*.97
        assert any(b['rows']==args.block_rows for b in blocks)
        assert len({b['segment_id'] for b in blocks})==1,[(b['rows'],b['end_reason']) for b in blocks]
        for a,b in zip(blocks,blocks[1:]):
            assert b['first']['global_line']==a['last']['global_line']+1
            assert abs(abs(b['first']['encoder_distance_m']-a['last']['encoder_distance_m'])-spacing)<1e-8
        for b in blocks:
            assert 1 <= len(b['pose_tags']) <= b['rows']
            if args.rviz: assert b.get('preview_subscribers',0)>=1, 'RViz preview subscriber disappeared'
            if args.backend != 'render':
                assert b['invalid_pixels'] == 0
            if args.backend == 'cuda_tiles':
                assert b['tile_statistics']['required_tile_misses'] == 0
            if args.backend == 'optix' and b.get('tile_statistics'):
                assert b['tile_statistics']['required_tile_misses_after_warm'] == 0, b['tile_statistics']
                assert b['tile_statistics']['slot_pins'] == 0
            if args.backend in ('cuda_tiles', 'optix'):
                assert b['sampling_queue_high_water_lines'] <= b['sampling_queue_capacity_lines']
                assert b['sampling_queue_observed_wait_max_s'] <= b['sampling_queue_max_wait_s']
            assert b['first']['scan_direction']==(1 if args.speed>0 else -1)
            assert b['last']['scan_direction']==b['first']['scan_direction']
            path=session/f'block_{b["block_id"]:06d}.pgm'
            with Image.open(path) as image:
                assert image.size==(4096,b['rows'])
                pixels=np.asarray(image)
                stamp=round(b['last']['time_s']*1e9)
                assert stamp in received_payloads, ('ROS missing archived block',b['block_id'])
                assert received_payloads[stamp]==(hashlib.sha256(pixels.tobytes()).hexdigest(),'mono8',4096,'camera_optical_frame'), ('ROS/archive payload or format mismatch',b['block_id'])
                if not args.reference_target: assert np.ptp(pixels)>30,'blank or failed render'
                if b['rows']==args.block_rows:
                    image.resize((1024,max(1,round(args.block_rows*1024/config['width'])))).save(session/'render_preview.png')
                    if args.backend != 'optix':
                        # Use early rows before the optional raised marker. At
                        # least one row is between horizontal grid marks.
                        row=pixels[:200][np.argmax(pixels[:200].std(axis=1))].astype(float)
                        contrast=abs(row-np.median(row))
                        marked=np.flatnonzero(contrast>np.max(contrast)/2)
                        groups=np.split(marked,np.flatnonzero(np.diff(marked)>1)+1)
                        centers=np.array([g.mean() for g in groups if len(g)>1])
                        assert len(centers)>=11, 'missing grid or blocked scan plane'
                        assert max(map(len,groups))<20, 'persistent wide occlusion'
                        q=np.linspace(-1,1,32769)
                        rays=np.polynomial.polynomial.polyval(q,config['ray_polynomial'])
                        height=b['first']['camera_position_world_m'][2]
                        scale=4096*config['pixel_pitch_m']/(2*config['focal_length_m'])
                        # The visual grid is a thin painted mesh at z=.0003.
                        expected=(np.interp(np.arange(-5,6)*.1/((height-(.0003 if args.backend == "render" else 0))*scale),rays,q)*2048+2047.5)
                        errors=[min(abs(centers-x)) for x in expected]
                        assert max(errors)<2, ('grid projection error',max(errors))
                        geometry_checks.append(dict(max_grid_center_error_px=max(errors)))
                        if args.probe:
                            # A raised red plate at x=1.2, y=.5 must cover the
                            # ground grid. Sampling geometry accounts for its z=.06.
                            center_row=round((1.2-b['first']['camera_position_world_m'][0])/spacing)
                            source_q=np.interp(.5/((height-.06)*scale),rays,q)
                            center_col=round(source_q*2048+2047.5)
                            roi=pixels[center_row-30:center_row+31,center_col-30:center_col+31]
                            assert roi.shape==(61,61)
                            assert np.std(roi)<5 and np.mean(roi)<np.max(row)*.8, 'raised scene object not rendered'
        assert any(w==4096 and h==args.block_rows and size==w*h for w,h,_,size in received), received
        assert any(abs(t-blocks[0]['last']['time_s'])<1e-8 for _,_,t,_ in received)
        assert len(received)==len(blocks) and not duplicate_stamps, 'ROS block count/duplicate mismatch'
        metrics=blocks[-1]['cumulative_sampling_metrics']
        if args.backend=='cuda_tiles':
            # Prototype timing budgets: nominal physics step is 1 ms.
            assert metrics['physics_step_wall_interval_max_seconds'] <= .010
            assert metrics['physics_step_wall_interval_p99_upper_seconds'] <= .002
            assert metrics['sampling_batch_max_seconds'] <= blocks[-1]['cuda_batch_rows']*spacing/abs(args.speed)
            if len(received_at)>1:
                assert max(np.diff(received_at)) <= 1.5*args.block_rows*spacing/abs(args.speed)
        if args.rviz:
            assert display_frames>0 and display_bad_stamps==0, 'Incoherent visualization snapshots'
        if args.correction_profile:
            assert len(corrected_payloads)==len(blocks) and not correction_status.get('error')
            from agv_linescan.calibration import Correction
            correction=Correction(json.loads(Path(args.correction_profile).read_text()))
            correction.check_capture(config)
            for b in blocks:
                name=f'block_{b["block_id"]:06d}'
                corrected_dir=Path(args.archive)/'corrected'
                data=np.array(Image.open(corrected_dir/(name+'.pgm')))
                raw=np.array(Image.open(session/(name+'.pgm')))
                assert np.array_equal(data,correction.apply(raw)[0]),'live/offline correction mismatch'
                assert corrected_payloads[round(b['last']['time_s']*1e9)]==hashlib.sha256(data).hexdigest()
                cm=json.loads((corrected_dir/(name+'.json')).read_text())
                assert all(cm[k]==b[k] for k in ('first','last','pose_tags','rows','reference'))
        sample_seconds=sum(metrics[k] for k in ('render_seconds','readback_seconds','mapping_seconds'))
        def odom_stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        odom_dt=odom_stamp(odom_end)-odom_stamp(odom_start)
        assert odom_dt>0
        def position(m):
            p=m.pose.pose.position;return np.array([p.x,p.y,p.z])
        displacement=position(odom_end)-position(odom_start)
        motion_evaluation=dict(scope='ground-truth evaluation only, never command input; each pose uses its own timestamp',
            duration_s=odom_dt,displacement_m=displacement.tolist(),average_world_velocity_m_s=(displacement/odom_dt).tolist())
        report=dict(passed=True,pause_interval=pause_interval,motion_evaluation=motion_evaluation,correction=correction_status,corrected_received_blocks=len(corrected_payloads),gazebo_gui=args.gui,rviz=args.rviz,rviz_world_start=tf_start,rviz_world_end=tf_end,display_sync=display_status,backend=blocks[0]['scene_backend'],archive=str(session),
                    rows=[b['rows'] for b in blocks],speed_m_s=args.speed,
                    steady_wall_start_s=wall_start,steady_wall_end_s=wall_end,display_frames=display_frames,display_bad_stamps=display_bad_stamps,
                    simulation_seconds=simulated,wall_seconds=elapsed,real_time_factor=simulated/elapsed,
                    sampling_line_rate=metrics['lines']/sample_seconds,
                    wheel_motion_diagnostics=blocks[-1].get('wheel_motion_diagnostics'),
                    metrics=metrics,write_seconds=sum(b['write_seconds'] for b in blocks),
                    publish_call_seconds=sum(b['publish_call_seconds'] for b in blocks),
                    received_blocks=len(received),
                    ros_all_blocks_byte_identical=True,
                    nominal_encoder_line_rate_hz=abs(args.speed)/config['line_spacing_m'],
                    receive_interval_max_seconds=max(np.diff(received_at)) if len(received_at)>1 else None,final_motion_state=node.state,
                    grid_geometry=geometry_checks,occlusion_probe=args.probe,
                    tile_statistics=blocks[-1].get('tile_statistics'),
                    sampling_queue_high_water_lines=max(b.get('sampling_queue_high_water_lines',0) for b in blocks),
                    sampling_queue_observed_wait_max_s=max(b.get('sampling_queue_observed_wait_max_s',0) for b in blocks),
                    sampling_queue_high_water_jobs=max(b.get('sampling_queue_high_water_jobs',0) for b in blocks),
                    notes=('CUDA timings include pose packing and batch upload/sampling/readback. Static unobstructed plane; configured CUDA radiometry is recorded in the archive.' if args.backend!='render' else 'Scene held at physics-step time; camera interpolated per exposure.')+' Sampling rate excludes physics/ROS/archive, not end-to-end throughput.')
        if args.backend == 'optix':
            assert all(b['scene_backend']=='optix_shared_mesh_dynamic_robot' and b['calibration_id'].endswith('-optix-strip-v1') for b in blocks)
            assert all(len(b['robot_contract']['link_names'])==13 for b in blocks)
            full_received=[t for (w,h,stamp,n),t in zip(received,received_at) if h==args.block_rows]
            report['full_block_delivery_lines_per_wall_second']=(len(full_received)-1)*args.block_rows/(full_received[-1]-full_received[0]) if len(full_received)>1 else None
            report['full_acceptance_passed']=False
            report['notes']='OptiX shared static mesh plus thirteen robot links with per-exposure transforms and configured LED shadow samples; ROS images and PGM archive. Sampling rate excludes physics/ROS/archive. Not the independent durable-write throughput acceptance.'
        report['realtime_target_met']=report['real_time_factor']>=REALTIME_FLOOR
        report['realtime_required']=args.require_realtime;report['realtime_floor']=REALTIME_FLOOR
        report['passed']=report['passed'] and (not args.require_realtime or report['realtime_target_met'])
        (Path(args.archive)/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)
        assert report['passed'], 'required realtime factor >=%g was not met'%REALTIME_FLOOR
        if args.view_hold: run_for(args.view_hold,(0,0,0))
    finally:
        node.command((0,0,0)); node.destroy_node(); rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--archive')
    parser.add_argument('--pause-once',action='store_true')
    parser.add_argument('--actual-wheel-diameter',type=float,default=.4)
    parser.add_argument('--camera-config',default='')
    parser.add_argument('--correction-profile',default='')
    parser.add_argument('--reference-target',action='store_true')
    parser.add_argument('--require-realtime',action='store_true',help='fail capture acceptance when the steady real-time factor is below the floor')
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--rviz',action='store_true')
    parser.add_argument('--stop-capture-before-brake',action='store_true')
    parser.add_argument('--view-hold',type=float,default=0)
    parser.add_argument('--speed',type=float,default=.05)
    parser.add_argument('--warmup',type=float,default=3)
    parser.add_argument('--distance',type=float,default=1.25)
    parser.add_argument('--terrain',default='')
    parser.add_argument('--scene',default='')
    parser.add_argument('--spawn-y',type=float,default=0)
    parser.add_argument('--spawn-x',type=float,default=0)
    parser.add_argument('--domain',type=int,default=81)
    parser.add_argument('--expect-terrain-failure',action='store_true')
    parser.add_argument('--repair-missing-tile',nargs=2,metavar=('SOURCE','DESTINATION'))
    parser.add_argument('--probe',action='store_true')
    parser.add_argument('--backend',choices=['render','cuda_grid','cuda_tiles','optix'],default='render')
    parser.add_argument('--block-rows',type=int,default=4096)
    args=parser.parse_args()
    if args.archive:
        evaluate(args); return
    tag=f'agv_cpp_scan_{os.getpid()}'
    archive='/tmp/'+tag
    env=dict(os.environ,ROS_DOMAIN_ID=str(args.domain),GZ_PARTITION=tag,ROS_LOG_DIR=archive+'_ros')
    with open(archive+'_sim.log','w') as log:
        sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','headless:='+str(not args.gui).lower(),'rviz:='+str(args.rviz).lower(),
            'actual_wheel_diameter:='+str(args.actual_wheel_diameter),'linescan:=true','linescan_backend:='+args.backend,'capture_dir:='+archive,
            *(['correction_profile:='+args.correction_profile] if args.correction_profile else []),*(['camera_config:='+args.camera_config] if args.camera_config else []),'scan_probe:='+str(args.probe).lower(),'spawn_x:='+str(args.spawn_x),
            'scan_block_rows:='+str(args.block_rows),'spawn_y:='+str(args.spawn_y),
            'scan_speed_limit:='+str(max(.25,abs(args.speed)*1.1))]+(['terrain_manifest:='+args.terrain] if args.terrain else [])+(['scene_manifest:='+args.scene] if args.scene else []),env=env,stdout=log,stderr=log,start_new_session=True)
        try:
            command=[sys.executable,__file__,'--archive',archive,'--speed',str(args.speed),
                     '--distance',str(args.distance),'--block-rows',str(args.block_rows),'--backend',args.backend,'--warmup',str(args.warmup),'--terrain',args.terrain,'--spawn-x',str(args.spawn_x)]
            command.extend(['--view-hold',str(args.view_hold)])
            if args.correction_profile: command.extend(['--correction-profile',args.correction_profile])
            if args.pause_once: command.append('--pause-once')
            if args.reference_target: command.append('--reference-target')
            if args.stop_capture_before_brake: command.append('--stop-capture-before-brake')
            if args.camera_config: command.extend(['--camera-config',args.camera_config])
            if args.require_realtime: command.append('--require-realtime')
            if args.gui: command.append('--gui')
            if args.rviz: command.append('--rviz')
            if args.repair_missing_tile: command.extend(['--repair-missing-tile',*args.repair_missing_tile])
            if args.probe: command.append('--probe')
            if args.expect_terrain_failure: command.append('--expect-terrain-failure')
            result=subprocess.run(command,env=env,timeout=480)
            print('Simulator log: '+archive+'_sim.log',flush=True)
            raise SystemExit(result.returncode)
        finally:
            if sim.poll() is None:
                os.killpg(sim.pid,signal.SIGINT)
                try: sim.wait(timeout=25)
                except subprocess.TimeoutExpired:
                    os.killpg(sim.pid,signal.SIGKILL); sim.wait()


if __name__=='__main__': main()
