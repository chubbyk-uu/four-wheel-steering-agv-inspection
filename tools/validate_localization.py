#!/usr/bin/env python3
"""Whole-vehicle localization evaluation; truth is confined to this evaluator."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from validate_motion import Evaluator


def motion_tilt_summary(health,start):
    """Per-cause accounting for the driving gravity reference.

    A single reject counter could not be read as a rate: the interval throttle
    is only reset by a successful update, so once a gate closes every IMU sample
    at 100 Hz is counted again while accepted updates are capped at 10 Hz. What
    is meaningful is how often an accepted reference actually arrives and which
    exit the rest take.
    """
    samples=[(t,v) for t,v in health if t>=start and v.get('motion_tilt_outcomes')]
    if not samples:return None
    first,last=samples[0][1]['motion_tilt_outcomes'],samples[-1][1]['motion_tilt_outcomes']
    span=samples[-1][0]-samples[0][0]
    delta={k:last[k]-first.get(k,0) for k in last}
    attempts=sum(v for k,v in delta.items() if k!='rate_limited')
    return dict(observed_span_s=span,outcomes=delta,
        accepted_update_rate_hz=delta['accepted']/span if span>0 else None,
        attempts_excluding_throttle=attempts,
        accepted_fraction_of_attempts=delta['accepted']/attempts if attempts else None,
        note=('rate_limited is the configured throttle, not a rejection, and is excluded from '
              'the fraction; the remaining exits are still sampled at IMU rate while a gate '
              'stays closed, so the fraction is not a per-opportunity acceptance probability'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',choices=['zero','normal'],default='zero')
    parser.add_argument('--config',default='')
    parser.add_argument('--gui',action='store_true')
    parser.add_argument('--expect-outage',action='store_true')
    parser.add_argument('--spawn-yaw',type=float,default=0.)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args()
    os.environ['ROS_DOMAIN_ID']='94';os.environ['GZ_PARTITION']='agv_loc_'+str(os.getpid())
    a.output.parent.mkdir(parents=True,exist_ok=True)
    log_path=a.output.with_suffix('.log')
    log=log_path.open('w')
    command=['ros2','launch','agv_bringup','sim.launch.py','localization:=true',
             'localization_profile:='+a.profile,'headless:='+str(not a.gui).lower(),
             'rviz:='+str(a.gui).lower(),'follow_camera:=false','spawn_x:=3','spawn_y:=0',
             'localization_output_dir:='+str(a.output.with_suffix('')),'spawn_yaw:='+str(a.spawn_yaw)]
    if a.config:command+=['localization_config:='+str(Path(a.config).resolve())]
    proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Evaluator()
    truth=[];estimates=[];observations=[];health=[];imu=[];wheel=[];tf_edges={};local_est=[]
    def stamp(msg):return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    def pose(msg):
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        return [p.x,p.y,p.z],[q.x,q.y,q.z,q.w]
    def truth_cb(msg):truth.append((stamp(msg),*pose(msg)))
    def filter_cb(msg):estimates.append((stamp(msg),*pose(msg)))
    def nav_cb(msg):observations.append((stamp(msg),*pose(msg)))
    def wheel_cb(msg):wheel.append((stamp(msg),node.get_clock().now().nanoseconds*1e-9,msg.twist.twist.linear.y))
    def imu_cb(msg):imu.append((stamp(msg),node.get_clock().now().nanoseconds*1e-9,msg.orientation_covariance[0],msg.linear_acceleration.z))
    def tf_cb(msg):
        for t in msg.transforms:tf_edges.setdefault(t.child_frame_id,set()).add(t.header.frame_id)
    node.create_subscription(Odometry,'/ground_truth/odom',truth_cb,100)
    node.create_subscription(Odometry,'/odometry/global',filter_cb,100)
    node.create_subscription(Odometry,'/odometry/local',lambda m:local_est.append(stamp(m)),100)
    node.create_subscription(PoseWithCovarianceStamped,'/localization/gnss_pose',nav_cb,100)
    node.create_subscription(Odometry,'/localization/wheel_odom',wheel_cb,100)
    node.create_subscription(Imu,'/localization/imu',imu_cb,100)
    node.create_subscription(String,'/localization/status',lambda m:health.append((node.get_clock().now().nanoseconds*1e-9,json.loads(m.data))),100)
    node.create_subscription(TFMessage,'/tf',tf_cb,100)
    try:
        deadline=time.monotonic()+80
        while not (node.odom and node.joints and node.state and estimates):
            assert proc.poll() is None,'launch exited; see saved log'
            assert time.monotonic()<deadline,'localization startup timed out'
            rclpy.spin_once(node,timeout_sec=.1)
        node.run_for(5,(0,0,0))
        assert health and health[-1][1]['state']=='READY',('initialization not READY',health[-1:] )
        start=stamp(node.odom)
        for name,cmd,duration in [('forward',(.5,0,0),4),('lateral',(0,.5,0),4),
                                 ('rotate',(0,0,.3),13),('reverse',(-.5,0,0),4)]:
            node.run_for(duration,cmd)
            node.run_for(3,(0,0,0));assert node.state=='HOLD',(name,node.state)
        assert estimates and observations and imu and wheel
        t=np.array([v[0] for v in truth]);xyz=np.array([v[1] for v in truth]);quat=np.array([v[2] for v in truth])
        unique=np.r_[True,np.diff(t)>0];t=t[unique];xyz=xyz[unique];quat=quat[unique]
        sl=Slerp(t,Rotation.from_quat(quat))
        evaluation=a.output.with_suffix('')/'evaluation';evaluation.mkdir(parents=True,exist_ok=True)
        (evaluation/'trajectory_truth.jsonl').write_text(''.join(json.dumps({'time_s':float(stamp),
            'position_m':p.tolist(),'orientation_xyzw':q.tolist(),'role':'evaluation_truth'})+'\n' for stamp,p,q in zip(t,xyz,quat)))
        def errors(samples):
            samples=[s for s in samples if max(t[0],start)<=s[0]<=t[-1]]
            times=np.array([s[0] for s in samples])
            predicted=np.column_stack([np.interp(times,t,xyz[:,i]) for i in range(3)])
            delta=np.array([s[1] for s in samples])-predicted
            rotation=Rotation.from_quat([s[2] for s in samples])
            yaw_error=(sl(times).inv()*rotation).as_rotvec()[:,2]
            return {'samples':len(samples),'horizontal_rmse_m':float(np.sqrt(np.mean(np.sum(delta[:,:2]**2,axis=1)))),
                    'position_max_m':float(np.linalg.norm(delta,axis=1).max()),
                    'vertical_rmse_m':float(np.sqrt(np.mean(delta[:,2]**2))),
                    'yaw_rmse_deg':float(np.degrees(np.sqrt(np.mean(yaw_error**2)))),
                    'yaw_mean_deg':float(np.degrees(yaw_error.mean())),
                    'rate_hz':float((len(times)-1)/(times[-1]-times[0]))}
        fusion=errors(estimates);measurement=errors(observations)
        assert fusion['horizontal_rmse_m']<(.05 if a.profile=='zero' else .15),fusion
        assert fusion['position_max_m']<(.12 if a.profile=='zero' else .4),fusion
        assert fusion['yaw_rmse_deg']<(1 if a.profile=='zero' else 4),fusion
        assert fusion['vertical_rmse_m']<.1,fusion
        assert all(s[2]==-1 for s in imu),'ideal IMU attitude leaked'
        assert max(abs(s[2]) for s in wheel)>.3,'lateral odometry absent'
        assert tf_edges.get('base_link')=={'odom'} and tf_edges.get('odom')=={'map'},tf_edges
        states=[v['state'] for timestamp,v in health if timestamp>=start]
        if a.expect_outage:
            assert 'NOT_READY' in states and states[-1]=='READY','FIX outage/recovery not observed'
        else:
            assert states.count('READY')/len(states)>.98,dict(ready=states.count('READY'),total=len(states))
        archive=a.output.with_suffix('')/'navigation.jsonl'
        records=[json.loads(line) for line in archive.read_text().splitlines()]
        assert len(records)>100 and all(r['frame_id']=='map' for r in records)
        assert all(b['time_s']>a['time_s'] for a,b in zip(records,records[1:]))
        assert all('truth' not in r for r in records)
        nodes={name for name,_ in node.get_node_names_and_namespaces()}
        for name in ('measurement_adapter','ekf_local','ekf_global'):
            assert name in nodes
            topics=node.get_subscriber_names_and_types_by_node(name,'/')
            assert not any('ground_truth' in topic for topic,_ in topics),(name,topics)
        result={'passed':True,'profile':a.profile,'gui_rviz':a.gui,'configuration':Path(a.config).name if a.config else 'measurements.yaml',
                'spawn_yaw_rad':a.spawn_yaw,'navigation_records':len(records),'outage_recovery_tested':a.expect_outage,'fusion':fusion,'gnss_observation':measurement,'tf_edges':{k:sorted(v) for k,v in tf_edges.items() if k in ('odom','base_link')},
                'imu_rate_hz':(len(imu)-1)/(imu[-1][0]-imu[0][0]),
                'wheel_rate_hz':(len(wheel)-1)/(wheel[-1][0]-wheel[0][0]),
                'imu_delivery_age_mean_s':float(np.mean([s[1]-s[0] for s in imu])),
                'wheel_delivery_age_mean_s':float(np.mean([s[1]-s[0] for s in wheel])),
                'not_ready_samples':states.count('NOT_READY'),'ready_fraction':states.count('READY')/len(states),'final_motion_state':node.state,
                'motion_tilt':motion_tilt_summary(health,start),
                'scope':'measurement/EKF motion regression; not mission tracking or image tagging'}
        a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        node.command((0,0,0));node.destroy_node();rclpy.shutdown()
        if proc.poll() is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=10)
        log.close()


if __name__=='__main__':main()
