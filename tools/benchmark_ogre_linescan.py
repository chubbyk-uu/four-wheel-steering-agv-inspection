#!/usr/bin/env python3
"""Bounded standalone Ogre2 batching experiment; not full line-scan acceptance."""
import argparse
import json
import os
import signal
from pathlib import Path
import subprocess
import numpy as np
from PIL import Image
import yaml


def validate_pixels(folder, config):
    meta=json.loads((folder/'summary.json').read_text())
    with Image.open(folder/'scan.pgm') as image:
        pixels=np.asarray(image).astype(float)
    width=config['width'];rows=pixels.shape[0]
    q=np.linspace(-1,1,65537)
    rays=np.polynomial.polynomial.polyval(q,config['ray_polynomial'])
    scale=width*config['pixel_pitch_m']/(2*config['focal_length_m'])
    height=config['nominal_width_m']/(2*scale)
    x=meta['start_x_m']+np.arange(rows)*config['line_spacing_m']+meta['speed_m_s']*config['exposure_s']/2
    # Independent geometry oracle: diagonal painted line y=x on z=.0003.
    expected=np.interp(x/((height-.0003)*scale),rays,q)*(width/2)+(width-1)/2
    centers=[]
    for row,u in zip(pixels,expected):
        left=int(u)-15;right=int(u)+16
        region=row[left:right]
        assert np.ptp(region)>25, 'fiducial disappeared or clipped'
        indices=np.flatnonzero(region<(region.min()+region.max())/2)
        centers.append(left+indices.mean())
    error=abs(np.asarray(centers)-expected)
    assert error.max()<2, ('distinct-time diagonal projection failed', float(error.max()))
    assert centers[-1]-centers[0]>.8*(rows-1), 'stale/repeated camera poses'
    return pixels,dict(max_diagonal_center_error_px=float(error.max()),first_center_px=centers[0],last_center_px=centers[-1])


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--rows',type=int,default=192)
    p.add_argument('--quick',action='store_true')
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1]
    config_path=root/'src/agv_description/config/linescan.yaml'
    config=yaml.safe_load(config_path.read_text())
    env=dict(os.environ,GALLIUM_DRIVER='d3d12',MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA')
    # Same scene/pixels in each family; scene is deliberately fixed during each batch.
    cases=[(1,6,21,1),(4,6,21,1),(16,6,21,1),(16,96,21,1),
           (1,6,0,1),(16,96,0,1),(16,96,21,0)]
    if a.quick: cases=cases[:1]
    reports=[];references={}
    for batch,flush,lights,copy in cases:
        name=f'b{batch}_f{flush}_led{lights}_copy{copy}';folder=a.output/name
        with (a.output/(name+'.log')).open('w') as log:
            command=['ros2','run','agv_linescan','benchmark_ogre_batch',str(config_path),str(folder),
                str(batch),str(a.rows),str(flush),str(lights),str(copy)]
            process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                code=process.wait(timeout=240)
                if code: raise subprocess.CalledProcessError(code,command)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try: process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL);process.wait()
        report=json.loads((folder/'summary.json').read_text())
        if copy:
            pixels,geometry=validate_pixels(folder,config);report['geometry']=geometry
            if lights not in references:references[lights]=pixels
            error=abs(pixels-references[lights])
            report['serial_comparison_max_dn']=float(error.max())
            report['serial_comparison_p999_dn']=float(np.percentile(error,99.9))
            assert error.max()<=1, ('batch output differs from sequential independent views',name,float(error.max()))
        reports.append(report)
        (a.output/'results.json').write_text(json.dumps({'cases':reports,'full_acceptance_passed':False},indent=2)+'\n')
        print(json.dumps({'case':name,'line_rate':report['lines_per_wall_second'],'wall_seconds':report['wall_seconds'],'geometry':report.get('geometry')}),flush=True)


if __name__=='__main__':main()
