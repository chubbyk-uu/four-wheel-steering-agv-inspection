#!/usr/bin/env python3
"""Matched physics tests plus fixed-pose optical identity for collision-only simplification."""
import argparse
import json
import subprocess
import sys
from pathlib import Path
import numpy as np
from PIL import Image
import xacro

ROOT=Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.robot_scene import export,split_visual_links
from agv_linescan.shared_scene import validate,digest


def main():
    p=argparse.ArgumentParser();p.add_argument('--detailed',required=True);p.add_argument('--proxy',required=True);p.add_argument('--output',required=True)
    a=p.parse_args();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False)
    paths={k:Path(getattr(a,k)).resolve() for k in ('detailed','proxy')}
    manifests={k:validate(v) for k,v in paths.items()}
    d,s=manifests['detailed'],manifests['proxy']
    assert d['ground_material']==s['ground_material'] and d['display_materials']==s['display_materials']
    assert d['assets'][0]['sha256']==s['assets'][0]['sha256']
    robot=export(split_visual_links(xacro.process_file(str(ROOT/'src/agv_description/urdf/agv.urdf.xacro')).toxml()),out/'robot')
    r=json.loads(robot.read_text());r['groups']=[g for g in r['groups'] if g['name']==r['led_emitters']['link']];robot.write_text(json.dumps(r))
    for name,path in paths.items():
        with (out/(name+'_optical.log')).open('w') as log:
            subprocess.run([str(ROOT/'build/agv_linescan/benchmark_grooves'),str(path),str(robot),
                str(ROOT/'build/agv_linescan/agv_optix_scan.ptx'),str(ROOT/'src/agv_description/config/linescan.yaml'),
                str(out/(name+'_optical')),'32','16'],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    first=out/'detailed_optical/samples_16.pgm';second=out/'proxy_optical/samples_16.pgm'
    assert np.array_equal(np.array(Image.open(first)),np.array(Image.open(second)))
    report=dict(scope='local 2.1 m passes at 0.5 m/s; no full-road or 11 kHz end-to-end acceptance',
        display_displacement_scope='unmatched display timestamps; use motion_evaluation for timestamp-aligned velocity comparisons',
        collision_proxy=s['assets'][0]['collision_proxy'],optical_mesh_sha256=s['assets'][0]['sha256'],
        optical_materials_identical=True,fixed_trajectory_images_identical=True,fixed_trajectory_pgm_sha256=digest(first),
        runs={},order=['detailed_gui','proxy_gui','detailed_headless','proxy_headless'])
    for label in report['order']:
        name,mode=label.split('_');logpath=out/(label+'.log')
        command=[sys.executable,str(ROOT/'tools/validate_rendered_linescan.py'),
            '--backend','optix','--scene',str(paths[name]),'--spawn-x','.55','--speed','.5',
            '--warmup','.8','--distance','2.1','--domain','84']
        if mode=='gui':command+=['--gui','--rviz']
        with logpath.open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=480)
        lines=logpath.read_text().splitlines()
        result=next(json.loads(line) for line in reversed(lines) if line.startswith('{') and '"passed"' in line)
        assert result['passed'] and result['ros_all_blocks_byte_identical'] and result['final_motion_state']=='HOLD'
        # Archive path stays only in local raw run logs, not in the portable report.
        result.pop('archive',None)
        report['runs'][label]=result
    for label,r in report['runs'].items():
        r['wheel_max_z_step_m']=max(v['max_z_step_m'] for v in r['wheel_motion_diagnostics'].values())
        if r['rviz_world_start']:
            r['measured_display_displacement_m']=(np.array(r['rviz_world_end'])-r['rviz_world_start']).tolist()
    report['proxy_gui_realtime']=report['runs']['proxy_gui']['real_time_factor']>=.97
    report['proxy_headless_realtime']=report['runs']['proxy_headless']['real_time_factor']>=.97
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:dict(rtf=v['real_time_factor'],rows=v['rows'],wheel_max_z_step_m=v['wheel_max_z_step_m']) for k,v in report['runs'].items()}),flush=True)


if __name__=='__main__':main()
