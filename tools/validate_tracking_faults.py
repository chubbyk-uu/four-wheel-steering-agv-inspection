#!/usr/bin/env python3
"""Cancel and GNSS-loss stopping through the real single-segment controller."""
import argparse,json,os,signal,subprocess,time
import yaml
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from validate_tracking import stop


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    os.environ.update(ROS_DOMAIN_ID='97',GZ_PARTITION='agv_tracking_fault_'+str(os.getpid()))
    noise=yaml.safe_load(Path('src/agv_localization/config/outage_probe.yaml').read_text())
    noise['gnss_outage_start_s']=25.
    probe=a.output/'outage.yaml';probe.write_text(yaml.safe_dump(noise))
    log=(a.output/'simulation.log').open('w')
    sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','headless:=true','localization:=true',
        'localization_profile:=normal','localization_config:='+str(probe.resolve()),
        'localization_output_dir:='+str(a.output/'navigation'),'spawn_x:=3'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=Node('tracking_fault_evaluator',parameter_overrides=[Parameter('use_sim_time',value=True)])
    state={};tracker=None;results=[]
    node.create_subscription(String,'/localization/status',lambda m:state.update(health=json.loads(m.data)),20)
    node.create_subscription(String,'/motion_state',lambda m:state.update(motion=m.data),20)
    node.create_subscription(String,'/mission/tracking_status',lambda m:state.update(tracking=json.loads(m.data)),20)
    node.create_subscription(Odometry,'/ground_truth/odom',lambda m:state.update(truth=m),20)
    client=node.create_client(Trigger,'/mission/cancel_segment')
    def wait_for(test,seconds=40):
        deadline=time.monotonic()+seconds
        while not test():
            assert sim.poll() is None and time.monotonic()<deadline, state.get('tracking',state)
            rclpy.spin_once(node,timeout_sec=.02)
    try:
        wait_for(lambda:state.get('health',{}).get('state')=='READY' and state.get('motion')=='HOLD',80)
        for case in ('cancel','gnss_loss'):
            state.pop('tracking',None)
            stream=(a.output/(case+'.log')).open('w')
            tracker=subprocess.Popen(['ros2','run','agv_mission','track_segment','--ros-args',
                '-p','use_sim_time:=true','-p','autostart:=true','-p','displacement_xy_m:=[20.0, 0.0]',
                '-p','output_dir:='+str(a.output/case)],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            wait_for(lambda:state.get('tracking',{}).get('state')=='RUNNING')
            if case=='cancel':
                wait_for(lambda:state['tracking']['profile_time_s']>.8)
                future=client.call_async(Trigger.Request());wait_for(future.done,5)
                assert future.result().success
            wait_for(lambda:state.get('tracking',{}).get('state')=='FAULT')
            fault_time=state['tracking']['time_s'];reason=state['tracking']['reason']
            expected='CANCELED' if case=='cancel' else 'STALE_OR_UNREADY_LOCALIZATION'
            assert reason==expected,(case,reason)
            wait_for(lambda:state.get('motion')=='HOLD',5)
            velocity=state['truth'].twist.twist
            assert abs(velocity.linear.x)<.03 and abs(velocity.linear.y)<.03
            stopped_time=node.get_clock().now().nanoseconds*1e-9
            if case=='gnss_loss':
                wait_for(lambda:state['health']['state']=='READY',8)
                assert state['tracking']['state']=='FAULT','must not resume on FIX recovery'
            result={'case':case,'passed':True,'reason':reason,'fault_to_hold_sim_s':stopped_time-fault_time,
                    'final_state':state['tracking']['state'],'final_motion_state':state['motion']}
            results.append(result);stop(tracker);tracker=None;stream.close()
            # Clear queued status from the previous node before starting the next.
            end=time.monotonic()+.3
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
        (a.output/'results.json').write_text(json.dumps({'passed':True,'cases':results},indent=2)+'\n')
        print(json.dumps(results))
    finally:
        if tracker is not None:stop(tracker)
        node.destroy_node();rclpy.shutdown();stop(sim);log.close()


if __name__=='__main__':main()
