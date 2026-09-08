#!/usr/bin/env python3
"""Independent closed-loop trials; never commands wheels or uses truth for control."""
import argparse,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=8)


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--profile',choices=['zero','normal'],default='zero');p.add_argument('--gui',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID='95',GZ_PARTITION='agv_tracking_'+str(os.getpid()))
    launch_log=(a.output/'simulation.log').open('w')
    sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','localization:=true',
        'localization_profile:='+a.profile,'localization_output_dir:='+str(a.output/'navigation'),
        'headless:='+str(not a.gui).lower(),'rviz:='+str(a.gui).lower(),'follow_camera:=false',
        'spawn_x:=3'],stdout=launch_log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();n=Node('tracking_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)])
    latest={};records=[];joint_samples=[];truth_samples=[];process=None
    n.create_subscription(String,'/localization/status',lambda m:latest.update(health=json.loads(m.data)),20)
    n.create_subscription(String,'/motion_state',lambda m:latest.update(motion=m.data),20)
    def status(m):
        v=json.loads(m.data);latest['tracking']=v;records.append(v)
    def truth(m):
        latest['truth']=m
        q=m.pose.pose.orientation;v=m.pose.pose.position
        truth_samples.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,v.x,v.y,v.z,q.x,q.y,q.z,q.w])
    def joints(m):
        names={name:i for i,name in enumerate(m.name)}
        if all(k+'_steer_joint' in names for k in ('fl','fr','rl','rr')):
            joint_samples.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9]+[m.position[names[k+'_steer_joint']] for k in ('fl','fr','rl','rr')])
    n.create_subscription(String,'/mission/tracking_status',status,100)
    n.create_subscription(Odometry,'/ground_truth/odom',truth,100)
    n.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
    results=[]
    try:
        deadline=time.monotonic()+80
        while not (latest.get('health',{}).get('state')=='READY' and latest.get('motion')=='HOLD'):
            assert time.monotonic()<deadline and sim.poll() is None,'simulation initialization failed'
            rclpy.spin_once(n,timeout_sec=.05)
        for name,kind,displacement,angle,speed,offset,heading in [
            ('forward','translate',[4.,0.],0.,.5,[0.,.1],.05),
            ('lateral','translate',[0.,1.],0.,.5,[0.,0.],0.),
            ('rotate','rotate',[0.,0.],float(np.pi),.25,[0.,0.],0.),
            ('reverse','translate',[-2.,0.],0.,.5,[0.,0.],0.),
            ('short_triangle','translate',[.12,0.],0.,.5,[0.,0.],.10)]:
            records.clear();joint_samples.clear();truth_samples.clear();latest.pop('tracking',None)
            directory=a.output/name
            log=(a.output/(name+'.log')).open('w')
            process=subprocess.Popen(['ros2','run','agv_mission','track_segment','--ros-args',
                '-p','use_sim_time:=true','-p','autostart:=true','-p','kind:='+kind,
                '-p','displacement_xy_m:='+json.dumps(displacement),'-p','angle_rad:='+str(angle),
                '-p','speed:='+str(speed),'-p','start_offset_xy_m:='+json.dumps(offset),
                '-p','heading_offset_rad:='+str(heading),'-p','output_dir:='+str(directory)],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline=time.monotonic()+160
            while latest.get('tracking',{}).get('state') not in ('COMPLETED','FAULT'):
                assert process.poll() is None,'tracking node exited'
                assert time.monotonic()<deadline,'tracking did not finish'
                rclpy.spin_once(n,timeout_sec=.02)
            end=latest['tracking'];success=end['state']=='COMPLETED' and latest['motion']=='HOLD'
            # Save evaluation data even on failure, for tuning without hiding failed runs.
            np.save(directory/'joint_evaluation.npy',np.asarray(joint_samples))
            np.save(directory/'truth_evaluation.npy',np.asarray(truth_samples))
            request=json.loads((directory/'request.json').read_text())
            true=latest['truth'].pose.pose;qp=true.orientation;pos=true.position
            true_p=np.array([pos.x,pos.y,pos.z]);true_r=Rotation.from_quat([qp.x,qp.y,qp.z,qp.w])
            goal_r=Rotation.from_quat(request['goal_orientation_xyzw'])
            error=(true_r.inv()*goal_r).as_rotvec()
            running=[r for r in records if r['state']=='RUNNING']
            rp=np.array([r['rpy_rad'][:2] for r in records]);j=np.array(joint_samples)
            result={'case':name,'passed':success,'final_state':end['state'],'reason':end['reason'],
                'terminal_trims':end['terminal_trims'],'reconfigurations':end['reconfigurations'],'estimated_goal_error_m':end.get('goal_error_m'),
                'estimated_heading_error_rad':end.get('goal_heading_error_rad'),
                'truth_goal_error_m':float(np.linalg.norm(true_p-np.array(request['goal_position_m']))),
                'truth_heading_error_rad':float(error[2]),
                'truth_roll_pitch_peak_deg':np.degrees(np.max(np.abs(Rotation.from_quat(np.array(truth_samples)[:,4:]).as_euler('xyz')[:,:2]),axis=0)).tolist(),
                'roll_pitch_peak_deg':np.degrees(np.max(np.abs(rp),axis=0)).tolist(),
                'reference_error_rmse_m':float(np.sqrt(np.mean([r.get('reference_position_error_m',0)**2 for r in running]))),
                'wheel_steer_total_travel_rad':np.sum(np.abs(np.diff(j[:,1:],axis=0)),axis=0).tolist(),
                'nominal_profile_duration_s':request['profile_duration_s'],'elapsed_sim_s':records[-1]['time_s']-records[0]['time_s']}
            results.append(result)
            (a.output/'results.json').write_text(json.dumps({'profile':a.profile,'gui_rviz':a.gui,'cases':results},indent=2)+'\n')
            stop(process);process=None;log.close()
            assert success,result
            assert result['truth_goal_error_m']<(.1 if a.profile=='zero' else .18),result
            assert abs(result['truth_heading_error_rad'])<(.04 if a.profile=='zero' else .08),result
            # Allow subscriptions from the just-stopped one-shot process to drain.
            start=time.monotonic()
            while time.monotonic()-start<.3:rclpy.spin_once(n,timeout_sec=.02)
        print(json.dumps({'passed':True,'cases':results}))
    finally:
        if process is not None:stop(process)
        n.destroy_node();rclpy.shutdown();stop(sim);launch_log.close()


if __name__=='__main__':main()
