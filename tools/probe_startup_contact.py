#!/usr/bin/env python3
"""Matched stamped-command startup experiment; truth/contact data are evaluation only."""
import argparse,json,os,signal,subprocess,sys,time
from pathlib import Path
import yaml

def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=False)
    from prepare_mission_camera import prepare
    camera=prepare(Path('src/agv_description/config/linescan.yaml'),a.output/'camera.yaml')
    cfg=yaml.safe_load(camera.read_text());cfg['flight_recorder'].update(span_s=20,max_samples=24000,probe_contact_kinematics=True)
    camera.write_text(yaml.safe_dump(cfg))
    env=dict(os.environ,ROS_DOMAIN_ID='98',GZ_PARTITION='startup_probe_'+str(os.getpid()))
    os.environ.update(ROS_DOMAIN_ID=env['ROS_DOMAIN_ID'],GZ_PARTITION=env['GZ_PARTITION'])
    log=(a.output/'simulation.log').open('w')
    cmd=['ros2','launch','agv_bringup','sim.launch.py','headless:=true','rviz:=false','localization:=false','linescan:=true','linescan_backend:=optix','camera_config:='+str(camera),'scene_manifest:='+str(a.scene.resolve()),'capture_dir:='+str((a.output/'raw').resolve()),'spawn_x:=6','spawn_y:=0']
    sim=subprocess.Popen(cmd,env=env,stdout=log,stderr=log,start_new_session=True)
    import rclpy
    from std_srvs.srv import SetBool,Trigger
    from validate_motion import Evaluator
    rclpy.init();n=Evaluator();commands=[]
    def call(client,request):
        assert client.wait_for_service(timeout_sec=180),'probe service absent'
        f=client.call_async(request);rclpy.spin_until_future_complete(n,f,timeout_sec=90);assert f.done(),'probe service timeout'
        r=f.result();assert r.success,r.message
    try:
        end=time.monotonic()+180
        while not(n.odom and n.joints and n.state):
            assert sim.poll() is None,'simulation exited';assert time.monotonic()<end,'feedback absent';rclpy.spin_once(n,timeout_sec=.05)
        n.run_for(5,(0,0,0));assert n.state=='HOLD'
        assert all(abs(pos)<.005 for name,pos in zip(n.joints.name,n.joints.position) if name.endswith('_steer_joint')),'not aligned'
        enable=n.create_client(SetBool,'/linescan/set_enabled');dump=n.create_client(Trigger,'/linescan/dump_probe')
        req=SetBool.Request();req.data=True;call(enable,req)
        start=n.get_clock().now().nanoseconds/1e9
        n.run_for(.25,(0,0,0))
        # Controller enforces the unchanged 0.8 m/s² acceleration limit.
        commands.append({'t':n.get_clock().now().nanoseconds/1e9,'body':[10/3.6,0,0]})
        n.run_for(5,(10/3.6,0,0));n.run_for(2,(10/3.6,0,0))
        commands.append({'t':n.get_clock().now().nanoseconds/1e9,'body':[0,0,0]})
        n.run_for(4,(0,0,0));assert n.state=='HOLD' and abs(n.odom.twist.twist.linear.x)<.001
        call(dump,Trigger.Request());req.data=False;call(enable,req);n.run_for(2,(0,0,0))
        (a.output/'probe.json').write_text(json.dumps({'passed':True,'capture_enable_sim_s':start,'commands':commands,'final_state':n.state,'actual_final_steering_rad':{k:v for k,v in zip(n.joints.name,n.joints.position) if k.endswith('_steer_joint')},'scene':str(a.scene),'scope':'single straight stamped body command; physical controller acceleration, static aligned capture start'},indent=2)+'\n')
    finally:
        n.command((0,0,0));n.destroy_node();rclpy.shutdown()
        if sim.poll() is None:
            os.killpg(sim.pid,signal.SIGINT)
            try:sim.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
        log.close()
    print('PASS: startup contact probe, parked and diagnostic drained')
if __name__=='__main__':main()
