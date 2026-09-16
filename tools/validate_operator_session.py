#!/usr/bin/env python3
"""Exercise the RViz broker with real GUI, localization, wheels and OptiX archives."""
import argparse,hashlib,json,os,subprocess,time,uuid,signal
import psutil
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image,JointState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from PIL import Image as PilImage
from validate_rectangle_execution import stop_tree
from process_resources import ResourceMonitor


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--inspect-seconds',type=float,default=0)
    p.add_argument('--speed',type=float,default=.5);p.add_argument('--length',type=float,default=3.);p.add_argument('--width',type=float,default=2.)
    p.add_argument('--start-x',type=float,default=6.);p.add_argument('--start-y',type=float);p.add_argument('--spawn-x',type=float,default=3.)
    p.add_argument('--scene',type=Path,default=Path('assets/road/runtime_fullwidth_20m_v1/manifest.json'));p.add_argument('--startup-timeout',type=float,default=300.);p.add_argument('--run-timeout',type=float,default=400.)
    p.add_argument('--fault-probe',action='store_true');p.add_argument('--no-pause',action='store_true');p.add_argument('--short-tail-probe',action='store_true');p.add_argument('--no-cancel-probe',action='store_true')
    p.add_argument('--broker-stall',action='store_true',help='freeze only the GUI broker for 0.6 seconds during capture')
    p.add_argument('--executor-exit-probe',action='store_true',help='kill only the owned mission worker during a second task')
    p.add_argument('--localization-config',type=Path,help='measurement config overriding the shipped one, to run a named noise seed')
    p.add_argument('--rounds',type=int,default=1,
                   help='repeat load/prepare/execute/audit this many times in one session, to show the broker carries nothing between tasks')
    a=p.parse_args();process_start=time.monotonic()
    if a.short_tail_probe:a.no_pause=True;a.no_cancel_probe=True
    a.output.mkdir(parents=True,exist_ok=False);session=a.output/'session'
    os.environ.update(ROS_DOMAIN_ID=str(100+os.getpid()%80),GZ_PARTITION='agv_operator_'+str(os.getpid()))
    log=(a.output/'simulation.log').open('w');sim=subprocess.Popen(['ros2','launch','agv_bringup','inspection.launch.py','session_dir:='+str(session.resolve()),'spawn_x:='+str(a.spawn_x),'scene_manifest:='+str(a.scene.resolve())]
        +(['localization_config:='+str(a.localization_config.resolve())] if a.localization_config else []),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    resources=ResourceMonitor(sim.pid,a.output/'resources.jsonl')
    rclpy.init();n=Node('operator_evaluator');latest={};images={};joints=[];states=[];actual=[]
    health_history=deque(maxlen=512);motion_ready={}
    def motion_status(m):
        motion_ready.update(mode=m.data,wall=time.monotonic())
    n.create_subscription(String,'/motion_state',motion_status,10)
    n.create_subscription(String,'/localization/status',lambda m:health_history.append(dict(observed_sim_time=actual[-1][1] if actual else None,health=json.loads(m.data))),20)
    def status(m):
        latest.clear();latest.update(json.loads(m.data));states.append(latest.get('status',{}))
        (a.output/'latest.json').write_text(m.data)
    n.create_subscription(String,'/mission/operator/state',status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(Image,'/linescan/image_raw',lambda m:images.update({m.header.stamp.sec*10**9+m.header.stamp.nanosec:(hashlib.sha256(m.data).hexdigest(),m.width,m.height)}),2)
    def joint(m):
        indices={v:i for i,v in enumerate(m.name)}
        if all(k+'_steer_joint' in indices for k in ('fl','fr','rl','rr')):joints.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9]+[m.position[indices[k+'_steer_joint']] for k in ('fl','fr','rl','rr')])
    n.create_subscription(JointState,'/joint_states',joint,qos_profile_sensor_data)
    n.create_subscription(Odometry,'/ground_truth/odom',lambda m:actual.append([time.monotonic(),m.header.stamp.sec+m.header.stamp.nanosec*1e-9,float(np.hypot(m.twist.twist.linear.x,m.twist.twist.linear.y))]),20)
    pub=n.create_publisher(String,'/mission/operator/request',10)
    def wait(predicate,timeout=90):
        deadline=time.monotonic()+timeout
        while not predicate():
            assert sim.poll() is None,'simulation exited'
            assert time.monotonic()<deadline,('timeout',latest.get('message'),latest.get('status'))
            rclpy.spin_once(n,timeout_sec=.02)
    def delay(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.02)
    def send(action,expected=True,**kwargs):
        identity=uuid.uuid4().hex;pub.publish(String(data=json.dumps(dict(id=identity,action=action,**kwargs))))
        wait(lambda:latest.get('response_id')==identity and not latest.get('busy'))
        assert latest['ok']==expected,latest['message']
    try:
        wait(lambda:latest.get('editable') and pub.get_subscription_count()>0,a.startup_timeout)
        send('preview',expected=False,fields={'length':100.})
        send('preview',fields={'length':a.length,'width':a.width,'start_x':a.start_x,'start_y':-a.width/2 if a.start_y is None else a.start_y,'speed':a.speed})
        send('save',path=str((a.output/'saved_request.yaml').resolve()))
        send('load',path=str((a.output/'saved_request.yaml').resolve()))
        send('preview')
        # Joint feedback may arrive while GUI loading still gates the controller.
        wait(lambda:motion_ready.get('mode')=='HOLD' and time.monotonic()-motion_ready['wall']<.5,a.startup_timeout)
        # The evaluator and mission broker subscribe independently. The first HOLD
        # can reach this process one scheduling turn before it reaches the broker;
        # require it to remain fresh after a short settling interval before prepare.
        delay(1.0)
        wait(lambda:motion_ready.get('mode')=='HOLD' and time.monotonic()-motion_ready['wall']<.5,a.startup_timeout)
        chassis_ready_wall_s=time.monotonic()-process_start
        # The broker judges HOLD freshness when it processes the request, which
        # this process cannot observe: one stale-feedback tick flips the
        # controller to FEEDBACK_HOLD and the prepare is refused although the
        # chassis is stopped. Settling here cannot close that gap, so retry the
        # one rejection that means it, as an operator would click again. Any
        # other refusal, and a chassis that never stops, still fail.
        prepare_attempts=0
        while True:
            prepare_attempts+=1
            identity=uuid.uuid4().hex
            pub.publish(String(data=json.dumps(dict(id=identity,action='prepare'))))
            wait(lambda:latest.get('response_id')==identity and not latest.get('busy'))
            if latest['ok']:break
            assert latest['message']=='等待底盘HOLD',latest['message']
            assert prepare_attempts<8,'prepare refused for chassis HOLD %d times'%prepare_attempts
            delay(1.)
        wait(lambda:latest.get('status',{}).get('ready_to_start'))
        # Join after publication, as an RViz display enabled later would do.
        retained={}
        for key,topic in (('road','/mission/road/markers'),('plan','/mission/preview/markers')):
            n.create_subscription(MarkerArray,topic,lambda m,k=key:retained.update({k:len(m.markers)}),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        wait(lambda:retained.get('road',0)>0 and retained.get('plan',0)>0,10)
        mission_start_time=latest['status']['time_s']
        (a.output/'startup_timing.json').write_text(json.dumps(dict(chassis_ready_wall_s=chassis_ready_wall_s,task_ready_wall_s=time.monotonic()-process_start),indent=2)+'\n')
        (a.output/'ready_for_ui').touch()
        until=time.monotonic()+a.inspect_seconds
        while time.monotonic()<until and latest['status']['state']=='READY':rclpy.spin_once(n,timeout_sec=.02)
        if latest['status']['state']=='READY':send('start')
        if a.broker_stall:
            wait(lambda:latest.get('status',{}).get('kind')=='PASS' and latest['status'].get('profile_time_s',0)>2.5,180)
            brokers=[p for p in psutil.Process(sim.pid).children(recursive=True) if any(Path(x).name=='mission_operator' for x in p.cmdline())]
            assert len(brokers)==1,[(p.pid,p.cmdline()) for p in brokers]
            try:
                brokers[0].send_signal(signal.SIGSTOP);time.sleep(.6)
            finally:brokers[0].send_signal(signal.SIGCONT)
        if not a.no_pause:
            wait(lambda:latest.get('status',{}).get('kind')=='PASS' and latest['status'].get('profile_time_s',0)>2.5,180)
            send('preview',expected=False,fields={'length':4.})
            send('pause');wait(lambda:latest['status']['state']=='PAUSED');before=len(images);delay(3);assert len(images)==before
            send('resume')
        wait(lambda:latest['status']['state'] in ('ACQUIRED','FAULT'),a.run_timeout)
        assert latest['status']['state']=='ACQUIRED',latest['status']
        wait(lambda:latest.get('editable'));send('audit');assert latest.get('coverage')
        delay(2);blocks=sorted(session.glob('raw/*/block_*.json'))
        events=[json.loads(line) for path in session.glob('raw/*/events.jsonl') for line in path.read_text().splitlines()]
        discarded=[e for e in events if e['reason']=='tail_discarded']
        if a.short_tail_probe:
            assert not blocks and not images and discarded and all(0<e['rows']<1000 for e in discarded)
        else:assert blocks
        assert not any(e['reason']=='unsupported_scan_motion' or e['reason'].startswith('terrain_sampling_failure') for e in events)
        rows=0;crossing=False
        for path in blocks:
            m=json.loads(path.read_text());data=np.asarray(PilImage.open(path.with_suffix('.pgm'))).tobytes()
            assert images.get(round(m['last']['time_s']*1e9))==(hashlib.sha256(data).hexdigest(),m['width'],m['rows'])
            assert m['end_reason'] in ('full','capture_toggle')
            if m['end_reason']=='full':assert m['width']==m['rows']==4096
            else:assert m['rows']>=1000
            assert m['last']['global_line']-m['first']['global_line']+1==m['rows'];rows+=m['rows']
            for l,r in zip(m['pose_tags'],m['pose_tags'][1:]):
                if r['global_line']==l['global_line']+1 and r['time_s']-l['time_s']>2:assert m['rows']==4096;crossing=True
        if not a.no_pause:assert crossing,'missing retained full frame across pause'
        assert all(s['command_body'][0]>=-1e-9 for s in states if s.get('kind')=='PASS')
        motion=np.asarray(actual);intervals=np.diff(motion,axis=0)
        steady=(motion[1:,2]>=.9*a.speed)&(motion[:-1,2]>=.9*a.speed)
        steady_rtf=float(intervals[steady,1].sum()/intervals[steady,0].sum()) if np.any(steady) else None
        result=dict(first_odometry_wall_s=actual[0][0]-process_start,steady_rtf=steady_rtf,steady_min_speed_m_s=.9*a.speed,steady_wall_s=float(intervals[steady,0].sum()),passed=a.no_cancel_probe,capture_passed=True,gui_rviz=True,blocks=len(blocks),rows=rows,ros_archive_identical=True,pause_retains_frame=crossing if not a.no_pause else None,requested_speed_m_s=a.speed,region_m=[a.length,a.width],discarded_tail_rows=[e["rows"] for e in discarded],actual_peak_speed_m_s=max(x[2] for x in actual if x[1]>=mission_start_time),
            observed_rtf=(actual[-1][1]-actual[0][1])/(actual[-1][0]-actual[0][0]),road_display=json.loads((session/"rviz_road.json").read_text()),
            coverage_status=latest['coverage']['status'],coverage_tracks=latest['coverage']['tracks'],final_state=latest['status']['state'],final_motion=latest['status']['motion_state'],
            wheel_steering_range_rad={k:[float(np.min(np.array(joints)[:,i+1])),float(np.max(np.array(joints)[:,i+1]))] for i,k in enumerate(('fl','fr','rl','rr'))})
        result['late_static_display_received']=retained
        if a.broker_stall:
            records=[json.loads(v) for v in next(session.glob('tasks/*/mission/execution.jsonl')).read_text().splitlines()]
            # A newly spawned node initially has ROS time zero until its first /clock.
            gaps=[r['time_s']-l['time_s'] for l,r in zip(records,records[1:]) if l['state']==r['state']=='RUNNING']
            assert gaps
            max_gap=float(max(gaps))
            assert max_gap<.1,max_gap
            result['broker_stall']={'duration_wall_s':.6,'control_max_step_s':max_gap,'continued_without_fault':True}
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');np.save(a.output/'wheel_steering.npy',np.array(joints));print(json.dumps(result),flush=True)
        (a.output/'finished_for_ui').touch();delay(a.inspect_seconds)
        result['repeat_rounds']=[]
        for extra in range(1,a.rounds):
            # Each round asks for a different rectangle, so a retained plan or a
            # stale coverage report cannot pass by looking like the last one.
            assert latest.get('editable'),'the broker must accept a new task once the last one is archived'
            length=round(a.length-extra*.5,3)
            send('preview',fields={'length':length,'width':a.width,'start_x':a.start_x,
                                   'start_y':-a.width/2 if a.start_y is None else a.start_y,'speed':a.speed})
            assert latest['request']['region']['length_m']==length,latest['request']['region']
            send('prepare');wait(lambda:latest['status'].get('ready_to_start'))
            # What the previous task counted must be gone before this one starts.
            assert not latest.get('coverage'),'a stale coverage report survived into a new task'
            assert latest['status'].get('captured_rows')==0 and latest['status'].get('captured_blocks')==0,latest['status']
            send('start')
            wait(lambda:latest['status']['state'] in ('ACQUIRED','FAULT'),a.run_timeout)
            assert latest['status']['state']=='ACQUIRED',latest['status']
            assert latest['status']['motion_state']=='HOLD',latest['status']
            wait(lambda:latest.get('editable'));send('audit');assert latest.get('coverage')
            assert latest['coverage']['request']['region']['length_m']==length,'the audit reported the previous region'
            tasks=sorted(session.glob('tasks/*/mission'))
            result['repeat_rounds'].append(dict(round=extra+1,region_m=[length,a.width],
                state=latest['status']['state'],motion_state=latest['status']['motion_state'],
                coverage_status=latest['coverage']['status'],
                unverified_tracks=sum(1 for t in latest['coverage']['tracks'] if t['unverified_along_m']),
                quality_flagged_tracks=sum(1 for t in latest['coverage']['tracks'] if t['quality_flags']),
                captured_rows=latest['status'].get('captured_rows'),
                task_dir=tasks[-1].parent.name))
        if result['repeat_rounds']:
            assert len(sorted(session.glob('tasks/*')))==a.rounds,'a round did not get its own task directory'
            (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
        if a.no_cancel_probe:return
        if latest['coverage'].get('rescan_candidates'):
            send('rescan',index=0)
        else:send('preview',fields={'length':2.,'width':1.})
        send('prepare');wait(lambda:latest['status'].get('ready_to_start'));send('start')
        wait(lambda:latest['status'].get('profile_time_s',0)>.7 or latest['status']['state']=='FAULT')
        assert latest['status']['state']!='FAULT',latest['status']
        if a.executor_exit_probe:
            wait(lambda:latest['status'].get('kind')=='PASS' and latest['status'].get('capture_active') and latest['status'].get('profile_time_s',0)>1.,180)
            workers=[p for p in psutil.Process(sim.pid).children(recursive=True) if 'from agv_mission.execution_node import main; main()' in p.cmdline()]
            assert len(workers)==1,[p.pid for p in workers]
            workers[0].kill()
            wait(lambda:latest['status']['state']=='FAULT' and latest.get('editable'))
            assert latest['status']['reason']=='EXECUTOR_PROCESS_EXITED'
            result['executor_exit']={'state':'FAULT','motion_state':latest['status']['motion_state'],'capture_active':latest['status']['capture_active']}
        elif a.fault_probe:
            wait(lambda:latest['status'].get('kind')=='PASS' and latest['status'].get('capture_active') and latest['status'].get('profile_time_s',0)>1.,180)
            fault_pub=n.create_publisher(String,'/linescan/status',10)
            wait(lambda:fault_pub.get_subscription_count()>0)
            fault_pub.publish(String(data=json.dumps(dict(reason='unsupported_scan_motion',injected_for_validation=True))))
            wait(lambda:latest['status']['state']=='FAULT' and latest.get('editable'))
            assert 'unsupported_scan_motion' in latest['status']['reason']
            result['injected_segmentation_fault']={'state':'FAULT','motion_state':'HOLD','capture_active':False}
        else:
            send('cancel');wait(lambda:latest['status']['state']=='CANCELED' and latest.get('editable'))
            result['new_task_cancel']={'state':'CANCELED','motion_state':'HOLD','capture_active':False}
        assert latest['status']['motion_state']=='HOLD' and latest['status']['capture_active'] is False
        result['passed']=True
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    finally:
        (a.output/'localization_health.json').write_text(json.dumps(list(health_history),indent=2)+'\n')
        np.save(a.output/'wheel_steering.npy',np.array(joints))
        np.save(a.output/'actual_motion.npy',np.array(actual))
        # On unexpected failures explicitly request cancellation and observe HOLD before teardown.
        if sim.poll() is None and latest.get('status',{}).get('state') not in ('IDLE',None):
            pub.publish(String(data=json.dumps(dict(id=uuid.uuid4().hex,action='cancel'))))
            until=time.monotonic()+12
            while time.monotonic()<until and latest.get('status',{}).get('motion_state')!='HOLD':rclpy.spin_once(n,timeout_sec=.02)
            (a.output/'cleanup_stop.json').write_text(json.dumps(latest.get('status',{})))
        n.destroy_node();rclpy.try_shutdown();resources.close();stop_tree(sim,known_children=list(resources.owned.values()));log.close()
if __name__=='__main__':main()
