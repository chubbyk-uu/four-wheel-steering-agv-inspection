#!/usr/bin/env python3
"""Force a legal limit reconfiguration while an encoder-triggered camera stays armed."""
import argparse,json,math,os,subprocess,time
from pathlib import Path
import numpy as np
import rclpy
from std_msgs.msg import String,Float64MultiArray
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from validate_motion import Evaluator
from validate_rectangle_execution import stop_tree
from process_resources import ResourceMonitor


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--scene',type=Path,required=True);p.add_argument('--platform',type=Path);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(100+os.getpid()%70),GZ_PARTITION='scan_limit_'+str(os.getpid()))
    log=(a.output/'simulation.log').open('w');sim=subprocess.Popen(['ros2','launch','agv_bringup','inspection.launch.py','session_dir:='+str((a.output/'session').resolve()),'scene_manifest:='+str(a.scene.resolve()),'spawn_x:=20']+(['platform:='+str(a.platform.resolve())] if a.platform else []),stdout=log,stderr=log,start_new_session=True)
    monitor=ResourceMonitor(sim.pid,a.output/'resources.jsonl');rclpy.init();node=Evaluator();events=[];reasons=[];samples=[];commands=[]
    node.create_subscription(Float64MultiArray,'/drive_controller/commands',lambda m:commands.append([node.get_clock().now().nanoseconds*1e-9,*m.data]),100)
    node.create_subscription(String,'/linescan/status',lambda m:events.append(json.loads(m.data)),100)
    node.create_subscription(String,'/motion_transition_reason',lambda m:reasons.append([node.get_clock().now().nanoseconds*1e-9,m.data]),20)
    def joints(m):
        keys={k:i for i,k in enumerate(m.name)};names=[w+'_steer_joint' for w in ('fl','fr','rl','rr')]+[w+'_drive_joint' for w in ('fl','fr','rl','rr')]
        if all(k in keys for k in names):samples.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9]+[m.position[keys[k]] for k in names]+[m.velocity[keys[k]] for k in names])
    node.create_subscription(JointState,'/joint_states',joints,100)
    client=node.create_client(SetBool,'/linescan/set_enabled')
    def settle(cmd):
        deadline=time.monotonic()+50;since=None;expected='DRIVE' if any(cmd) else 'HOLD'
        while True:
            assert sim.poll() is None,'simulation exited'
            node.command(cmd);rclpy.spin_once(node,timeout_sec=.02)
            now=node.get_clock().now().nanoseconds*1e-9
            if node.state==expected:
                if since is None:since=now
                if now-since>.2:return
            else:since=None
            assert time.monotonic()<deadline,(node.state,cmd)
    def enabled(value):
        assert client.wait_for_service(timeout_sec=5)
        future=client.call_async(SetBool.Request(data=value));end=time.monotonic()+15
        while not future.done():
            rclpy.spin_once(node,timeout_sec=.02);assert time.monotonic()<end
        assert future.result().success,future.result().message
    try:
        end=time.monotonic()+180
        while not(node.odom and node.joints and node.state):
            rclpy.spin_once(node,timeout_sec=.05);assert time.monotonic()<end
        settle((0,0,0));enabled(False)
        settle((0,.1,0));node.run_for(1,(0,.1,0));settle((0,0,0))
        settle((0,0,.25));node.run_for(12,(0,0,.25));settle((0,0,0))
        settle((.02,0,0));enabled(True)
        capture_start=node.get_clock().now().nanoseconds*1e-9
        node.run_for(3,(.5,0,0))
        # Minimal steering has equivalent +/-180 branches; follow the measured one.
        steering=[v for k,v in zip(node.joints.name,node.joints.position) if k.endswith('_steer_joint')]
        outer=max(steering,key=abs);assert abs(outer)>math.radians(175),steering
        sign=math.copysign(1,outer)
        node.run_for(2,(.5,.5*math.tan(math.radians(sign)),0))
        request=(.5,.5*math.tan(math.radians(3*sign)),0)
        node.run_for(8,request);settle(request);node.run_for(3,request)
        settle((0,0,0));enabled(False);node.run_for(1,(0,0,0))
        limit_times=[t for t,reason in reasons if t>=capture_start and reason=='LIMIT_RECONFIGURE']
        assert limit_times,'test did not reach steering limit recovery during capture'
        assert not any(x['reason'] not in ('capture_toggle','tail_discarded') for x in events),events
        blocks=[json.loads(p.read_text()) for p in sorted((a.output/'session/raw').glob('**/block_*.json'))]
        assert blocks and sum(b['rows'] for b in blocks)>4096
        segments={b['segment_id'] for b in blocks};assert len(segments)==1,segments
        assert any(b['first']['time_s']<limit_times[0]<b['last']['time_s'] for b in blocks),'no retained image across limit recovery'
        blocks.sort(key=lambda b:b['first']['global_line'])
        assert all(r['first']['global_line']==l['last']['global_line']+1 for l,r in zip(blocks,blocks[1:]))
        measured=np.asarray(samples)
        max_angle=float(np.max(np.abs(np.degrees(measured[:,1:5]))))
        assert max_angle<185,max_angle
        assert np.max(np.abs(measured[-1,13:17]))<.01
        result=dict(max_actual_steering_deg=max_angle,final_drive_rad_s=measured[-1,13:17].tolist(),passed=True,image_quality_accepted=False,final_motion=node.state,limit_reconfiguration=True,blocks=len(blocks),rows=sum(b['rows'] for b in blocks),continuous_segment=True,max_encoder_retrace_m=max(e.get('encoder_max_retrace_m',0) for e in events),retrace_episodes=sum(e.get('encoder_retrace_episodes',0) for e in events),scope='Scripted stamped body commands, physical steering/drive and OptiX; not a production mission or full-area coverage')
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        try:settle((0,0,0))
        finally:
            np.save(a.output/'drive_commands.npy',np.asarray(commands))
            np.save(a.output/'joints.npy',np.asarray(samples));(a.output/'reasons.json').write_text(json.dumps(reasons));(a.output/'events.json').write_text(json.dumps(events))
            monitor.close();node.destroy_node();rclpy.try_shutdown();stop_tree(sim,known_children=list(monitor.owned.values()));log.close()

if __name__=='__main__':main()
