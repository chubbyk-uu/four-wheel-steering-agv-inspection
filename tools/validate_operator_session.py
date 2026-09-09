#!/usr/bin/env python3
"""Exercise the RViz broker with real GUI, localization, wheels and OptiX archives."""
import argparse,hashlib,json,os,subprocess,time,uuid
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import Image,JointState
from PIL import Image as PilImage
from validate_rectangle_execution import stop_tree


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--inspect-seconds',type=float,default=0);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False);session=a.output/'session'
    os.environ.update(ROS_DOMAIN_ID='98',GZ_PARTITION='agv_operator_'+str(os.getpid()))
    log=(a.output/'simulation.log').open('w');sim=subprocess.Popen(['ros2','launch','agv_bringup','inspection.launch.py','session_dir:='+str(session.resolve())],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();n=Node('operator_evaluator');latest={};images={};joints=[];states=[]
    def status(m):
        latest.clear();latest.update(json.loads(m.data));states.append(latest.get('status',{}))
        (a.output/'latest.json').write_text(m.data)
    n.create_subscription(String,'/mission/operator/state',status,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    n.create_subscription(Image,'/linescan/image_raw',lambda m:images.update({m.header.stamp.sec*10**9+m.header.stamp.nanosec:(hashlib.sha256(m.data).hexdigest(),m.width,m.height)}),2)
    def joint(m):
        indices={v:i for i,v in enumerate(m.name)}
        if all(k+'_steer_joint' in indices for k in ('fl','fr','rl','rr')):joints.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9]+[m.position[indices[k+'_steer_joint']] for k in ('fl','fr','rl','rr')])
    n.create_subscription(JointState,'/joint_states',joint,qos_profile_sensor_data)
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
        wait(lambda:latest.get('editable') and pub.get_subscription_count()>0)
        send('preview',expected=False,fields={'length':100.})
        send('preview',fields={'length':3.,'width':2.,'start_x':6.,'start_y':-1.})
        send('save',path=str((a.output/'saved_request.yaml').resolve()))
        send('load',path=str((a.output/'saved_request.yaml').resolve()))
        send('preview')
        wait(lambda:any(x.get('motion_state')=='HOLD' for x in states) or len(joints)>800)
        send('prepare');wait(lambda:latest.get('status',{}).get('ready_to_start'))
        (a.output/'ready_for_ui').touch()
        until=time.monotonic()+a.inspect_seconds
        while time.monotonic()<until and latest['status']['state']=='READY':rclpy.spin_once(n,timeout_sec=.02)
        if latest['status']['state']=='READY':send('start')
        wait(lambda:latest.get('status',{}).get('kind')=='PASS' and latest['status'].get('profile_time_s',0)>2.5,180)
        send('preview',expected=False,fields={'length':4.})
        send('pause');wait(lambda:latest['status']['state']=='PAUSED');before=len(images);delay(3);assert len(images)==before
        send('resume');wait(lambda:latest['status']['state'] in ('ACQUIRED','FAULT'),400)
        assert latest['status']['state']=='ACQUIRED',latest['status']
        wait(lambda:latest.get('editable'));send('audit');assert latest.get('coverage')
        delay(2);blocks=sorted(session.glob('raw/*/block_*.json'));assert blocks
        rows=0;crossing=False
        for path in blocks:
            m=json.loads(path.read_text());data=np.asarray(PilImage.open(path.with_suffix('.pgm'))).tobytes()
            assert images.get(round(m['last']['time_s']*1e9))==(hashlib.sha256(data).hexdigest(),m['width'],m['rows'])
            assert m['last']['global_line']-m['first']['global_line']+1==m['rows'];rows+=m['rows']
            for l,r in zip(m['pose_tags'],m['pose_tags'][1:]):
                if r['global_line']==l['global_line']+1 and r['time_s']-l['time_s']>2:assert m['rows']==4096;crossing=True
        assert crossing,'missing retained full frame across pause'
        assert all(s['command_body'][0]>=-1e-9 for s in states if s.get('kind')=='PASS')
        result=dict(passed=True,gui_rviz=True,blocks=len(blocks),rows=rows,ros_archive_identical=True,pause_retains_frame=True,
            coverage_status=latest['coverage']['status'],coverage_tracks=latest['coverage']['tracks'],final_state=latest['status']['state'],final_motion=latest['status']['motion_state'],
            wheel_steering_range_rad={k:[float(np.min(np.array(joints)[:,i+1])),float(np.max(np.array(joints)[:,i+1]))] for i,k in enumerate(('fl','fr','rl','rr'))})
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n');np.save(a.output/'wheel_steering.npy',np.array(joints));print(json.dumps(result),flush=True)
        (a.output/'finished_for_ui').touch();delay(a.inspect_seconds)
        if latest['coverage'].get('rescan_candidates'):
            send('rescan',index=0)
        else:send('preview',fields={'length':2.,'width':1.})
        send('prepare');wait(lambda:latest['status'].get('ready_to_start'));send('start')
        wait(lambda:latest['status'].get('profile_time_s',0)>.7 or latest['status']['state']=='FAULT')
        assert latest['status']['state']!='FAULT',latest['status']
        send('cancel');wait(lambda:latest['status']['state']=='CANCELED' and latest.get('editable'))
        assert latest['status']['motion_state']=='HOLD' and latest['status']['capture_active'] is False
        result['new_task_cancel']={'state':'CANCELED','motion_state':'HOLD','capture_active':False}
        (a.output/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    finally:
        # On unexpected failures explicitly request cancellation and observe HOLD before teardown.
        if sim.poll() is None and latest.get('status',{}).get('state') not in ('IDLE',None):
            pub.publish(String(data=json.dumps(dict(id=uuid.uuid4().hex,action='cancel'))))
            until=time.monotonic()+12
            while time.monotonic()<until and latest.get('status',{}).get('motion_state')!='HOLD':rclpy.spin_once(n,timeout_sec=.02)
            (a.output/'cleanup_stop.json').write_text(json.dumps(latest.get('status',{})))
        n.destroy_node();rclpy.try_shutdown();stop_tree(sim);log.close()
if __name__=='__main__':main()
