#!/usr/bin/env python3
"""Sustained streamed OptiX fixture, raw durable archive and independent ROS receiver."""
import argparse,json,os,subprocess,sys
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',required=True);p.add_argument('--output',required=True)
    p.add_argument('--blocks',type=int,default=160);p.add_argument('--rate',type=float,default=11200);p.add_argument('--domain',type=int,default=83)
    p.add_argument('--pass-blocks',type=int,default=80);p.add_argument('--start-x',type=float,default=2);p.add_argument('--track-y',type=float,default=0)
    a=p.parse_args();root=Path(__file__).resolve().parents[1];out=Path(a.output).resolve()
    if out.exists():raise ValueError('output exists')
    fixture=out.with_name(out.name+'_fixture');fixture.mkdir()
    import yaml
    config=yaml.safe_load((root/'src/agv_description/config/linescan.yaml').read_text())
    config['stream_benchmark']=dict(pass_blocks=a.pass_blocks,start_x_m=a.start_x,track_y_m=a.track_y)
    camera=fixture/'camera.yaml';camera.write_text(yaml.safe_dump(config,sort_keys=False))
    import xacro
    sys.path.insert(0,str(root/'src/agv_linescan'))
    from agv_linescan.robot_scene import export,split_visual_links
    robot=export(split_visual_links(xacro.process_file(str(root/'src/agv_description/urdf/agv.urdf.xacro')).toxml()),fixture)
    m=json.loads(robot.read_text());m['groups']=[g for g in m['groups'] if g['name']==m['led_emitters']['link']];robot.write_text(json.dumps(m))
    env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain))
    with out.with_suffix('.receiver.log').open('w') as log:
        receiver=subprocess.Popen([sys.executable,str(root/'tools/benchmark_cuda_linescan.py'),'--receiver','--output',str(out),'--blocks',str(a.blocks)],env=env,stdout=log,stderr=log)
        try:
            subprocess.run([str(root/'build/agv_linescan/benchmark_optix_stream'),str(camera),str(out),str(a.blocks),str(a.rate),'1','1',str(Path(a.scene).resolve()),str(robot),str(root/'build/agv_linescan/agv_optix_scan.ptx')],env=env,check=True,timeout=180)
            assert receiver.wait(timeout=10)==0
            r=json.loads((out/'summary.json').read_text());t=r['tiles']
            assert r['wall_seconds']>=30 and r['effective_lines_per_wall_second']>=11000
            assert r['invalid_pixels']==0 and r['ros_acknowledged_blocks']==a.blocks
            assert t['required_tile_misses_after_warm']==0 and t['tile_loads']>t['cache_slots'] and t['evictions']>0
            assert t['slot_pins']==0
            assert r['batch_seconds_max']<=r['continuous_delay_budget']['batch_seconds']
            assert r['block_receive_interval_seconds_max']<=r['continuous_delay_budget']['block_interval_seconds']
            r['streamed_raw_capture_11khz_passed']=True
            (out/'summary.json').write_text(json.dumps(r,indent=2)+'\n')
            print('11 kHz streamed raw capture passed; '+str(out/'summary.json'))
        finally:
            if receiver.poll() is None:
                receiver.terminate()
                try:receiver.wait(timeout=5)
                except subprocess.TimeoutExpired:receiver.kill();receiver.wait()


if __name__=='__main__':main()
