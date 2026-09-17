#!/usr/bin/env python3
"""Noiseless shadow convergence on the production sampler and current lamp mounting."""
import argparse
import json
import subprocess
from pathlib import Path
import sys
import numpy as np
from PIL import Image
import xacro
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.robot_scene import export,split_visual_links
from agv_linescan.shared_scene import digest

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',required=True)
    a=p.parse_args();out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False)
    robot=split_visual_links(xacro.process_file(str(ROOT/'src/agv_description/urdf/agv.urdf.xacro')).toxml())
    rp=export(robot,out/'robot');r=json.loads(rp.read_text())
    # Keep the actual body, camera, brackets and LED. Wheels are irrelevant to this isolated optical fixture.
    r['groups']=[g for g in r['groups'] if g['name']==r['led_emitters']['link']]
    rp.write_text(json.dumps(r))
    cfg=yaml.safe_load((ROOT/'src/agv_description/config/linescan.yaml').read_text())
    lamp=np.mean(r['led_emitters']['positions_m'],axis=0)
    lamp_forward=float(lamp[0])-cfg['camera_x_m']
    lamp_height=float(lamp[2])+cfg['base_nominal_height_m']
    results={}
    for height in (0.,.03,.10,.20):
        scene=out/f'height_{height:.2f}';scene.mkdir()
        assets=[]
        center=3.+lamp_forward*height/lamp_height
        for name,vertices in [('terrain',[(0,-2,0),(6,-2,0),(6,2,0),(0,2,0)])]+(
            [('occluder',[(center-.006,-.06,height),(center+.006,-.06,height),(center+.006,.08,height),(center-.006,.08,height)])] if height else []):
            f=scene/f'{name}.obj'
            f.write_text('vn 0 0 1\n'+''.join('v '+' '.join(map(str,v))+'\n' for v in vertices)+'f 1//1 2//1 3//1\nf 1//1 3//1 4//1\n')
            assets.append(dict(name=name,mesh=f.name,sha256=digest(f),triangles=2,linear_reflectance=.75))
        mp=scene/'manifest.json'
        mp.write_text(json.dumps(dict(schema='agv.shared.static_scene.v1',units='m',frame='world',transform='identity_world_baked',assets=assets)))
        command=[str(ROOT/'build/agv_linescan/benchmark_led_convergence'),str(mp),str(rp),
                 str(ROOT/'build/agv_linescan/agv_optix_scan.ptx'),str(ROOT/'src/agv_description/config/linescan.yaml'),str(scene/'images')]
        with (scene/'run.log').open('w') as log:subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
        results[str(height)]=json.loads((scene/'images/timing.json').read_text())
    clear=np.asarray(Image.open(out/'height_0.00/images/samples_256.pgm'),dtype=np.int16)
    rows=[]
    for height in (.03,.10,.20):
        folder=out/f'height_{height:.2f}/images'
        reference=np.asarray(Image.open(folder/'samples_256.pgm'),dtype=np.int16)
        mask=clear-reference>2
        if not np.any(mask):raise RuntimeError('fixture failed to produce shadows')
        for n in (4,8,16,32,64,128):
            im=np.asarray(Image.open(folder/f'samples_{n}.pgm'),dtype=np.int16)
            delta=np.abs(im-reference)[mask]
            rows.append(dict(height_m=height,samples=n,shadow_pixels=int(mask.sum()),max_dn=int(delta.max()),
                p95_dn=float(np.percentile(delta,95)),rms_dn=float(np.sqrt(np.mean(delta.astype(float)**2))),
                fraction_over_2dn=float(np.mean(delta>2))))
    report=dict(scope='isolated static optical fixture; actual base/LED mount; 3 exposure samples at 10 km/h; no physics or wheel meshes',
                reference_samples=256,reference_check_samples=128,noise=False,prnu=False,metrics=rows,timing=results,
                source_urdf_sha256=r['source_sha256'],camera_sha256=digest(ROOT/'src/agv_description/config/linescan.yaml'))
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    # Three shadow fixtures, same row and unaltered DN scaling.
    panels=[]
    for height in (.03,.10,.20):
        ims=[np.asarray(Image.open(out/f'height_{height:.2f}/images/samples_{n}.pgm')) for n in (4,8,32,64,256)]
        panels.append(np.concatenate(ims,axis=0))
    Image.fromarray(np.concatenate(panels,axis=1)).save(out/'comparison.png')
    print(json.dumps(rows))
if __name__=='__main__':main()
