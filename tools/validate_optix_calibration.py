#!/usr/bin/env python3
"""Image-estimated references captured by the actual GZ/OptiX AGV sensor."""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
import yaml
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/agv_linescan'))
from agv_linescan.shared_scene import calibration_scene
from agv_linescan.calibration import Correction, capture_signature, stripe_centers


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--grid-scene',type=Path,required=True);a=p.parse_args()
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    config=yaml.safe_load((ROOT/'src/agv_description/config/linescan.yaml').read_text())
    worlds={name:calibration_scene(out/(name+'_scene'),ROOT/'src/agv_bringup/worlds/flat.sdf',phase,width=4.)
            for name,phase in [('flat',None),('board',0.),('holdout',.025)]}
    sessions={}
    for name in ('dark','flat','board','holdout','grid'):
        c=copy.deepcopy(config)
        if name=='dark':c['radiometry'].update(led_peak_relative=0.,ambient_relative=0.)
        cfg=out/(name+'.yaml');cfg.write_text(yaml.safe_dump(c))
        scene=a.grid_scene if name=='grid' else worlds['flat' if name=='dark' else name]
        log=out/(name+'.log')
        with log.open('w') as f:
            subprocess.run([sys.executable,str(ROOT/'tools/validate_rendered_linescan.py'),
                '--backend','optix','--scene',str(scene),'--camera-config',str(cfg),'--reference-target',
                '--spawn-x','2','--speed','.5','--warmup','2','--distance','3.5','--domain','94'],
                stdout=f,stderr=subprocess.STDOUT,check=True,timeout=180)
        report=next(json.loads(line) for line in log.read_text().splitlines() if line.startswith('{"passed":'))
        assert report['passed'] and report['ros_all_blocks_byte_identical']
        sessions[name]=Path(report['archive'])
        (out/'sessions.json').write_text(json.dumps({k:str(v) for k,v in sessions.items()},indent=2))
    # Conditions are from the real capture, including the OptiX model suffix.
    source=yaml.safe_load((sessions['flat']/'calibration.yaml').read_text())
    first=json.loads((sessions['flat']/'block_000000.json').read_text())
    conditions=dict(exposure_s=source['exposure_s'],source_calibration_id=source['calibration_id'],
        capture_signature=capture_signature(source),scene_backend=first['scene_backend'],
        robot_source_sha256=first['robot_contract']['source_sha256'],
        note='Fixed vertical camera on horizontal diffuse reference; renew after geometry or illumination changes.')
    (out/'conditions.json').write_text(json.dumps(conditions))
    width=config['nominal_width_m'];pixels=config['width'];spacing=width/pixels
    count=int(round(width/2/.05))
    (out/'target.json').write_text(json.dumps(dict(across_m=(np.arange(-count,count+1)*.05).tolist(),output_width_m=width)))
    with (out/'fit.log').open('w') as log:
        subprocess.run([sys.executable,str(ROOT/'tools/calibrate_linescan.py'),
            '--dark',str(sessions['dark']/'block_000000.pgm'),'--flat',str(sessions['flat']/'block_000000.pgm'),
            '--board',str(sessions['board']/'block_000000.pgm'),'--target',str(out/'target.json'),
            '--conditions',str(out/'conditions.json'),'--output',str(out/'measured.json')],stdout=log,stderr=subprocess.STDOUT,check=True)
    profile=json.loads((out/'measured.json').read_text());corr=Correction(profile);corr.check_capture(source)
    def read(name,index):return np.array(Image.open(sessions[name]/f'block_{index:06d}.pgm'))
    flat=read('flat',1);fixed,_=corr.apply(flat);valid=corr.valid
    raw_cv=float(flat[:,valid].mean(0).std()/flat[:,valid].mean())
    fixed_cv=float(fixed[:,valid].mean(0).std()/fixed[:,valid].mean())
    held=read('holdout',1);corrected,_=corr.apply(held);corrected[:,~valid]=round(profile['flat']['target_signal_dn'])
    expected=(np.arange(-count,count)*.05+.025)/spacing+(pixels-1)/2
    centers=stripe_centers(corrected);assert len(centers)==len(expected),(len(centers),len(expected))
    error=float(max(abs(centers-expected)))
    raw_centers=stripe_centers((held-np.array(profile['flat']['offset']))*np.array(profile['flat']['gain']))
    # Overscan can expose an extra pair of shifted holdout stripes outside the
    # corrected swath. Compare only the symmetric interior reference points.
    extra=len(raw_centers)-len(expected)
    assert extra>=0 and extra%2==0
    raw_centers=raw_centers[extra//2:extra//2+len(expected)]
    before=float(max(abs(raw_centers-expected)))
    chunks=[read('grid',i) for i in range(2)];joined=np.concatenate(chunks)
    combined,_=corr.apply(joined);separate=np.concatenate([corr.apply(x)[0] for x in chunks])
    assert np.array_equal(combined,separate)
    for i in range(2):
        meta=json.loads((sessions['grid']/f'block_{i:06d}.json').read_text());corrected_meta=corr.metadata(meta)
        assert all(meta[k]==corrected_meta[k] for k in ('first','last','pose_tags','rows','reference'))
        Image.fromarray(separate[i*4096:(i+1)*4096]).save(out/f'corrected_{i}.png')
        (out/f'corrected_{i}.json').write_text(json.dumps(corrected_meta,indent=2))
    report=dict(passed=fixed_cv<.002 and error<1,backend=first['scene_backend'],calibration_id=profile['calibration_id'],
        flat_cv_before=raw_cv,flat_cv_after=fixed_cv,holdout_error_px_before=before,holdout_error_px_after=error,
        fit_error_px_max=profile['geometry']['fit_error_px_max'],valid_columns=int(valid.sum()),
        partition_identical=True,metadata_preserved=True,full_throughput_acceptance=False,
        scope='GZ capture, fixed horizontal plane; flat and holdout block 1 excluded from fitting')
    (out/'validation.json').write_text(json.dumps(report,indent=2)+'\n')
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for ax,im,title in zip(axes.flat,[chunks[0],separate[:4096],flat,fixed],['OptiX raw scene','Measured correction','Independent flat: raw','Independent flat: corrected']):
        ax.imshow(im,cmap='gray',vmin=0,vmax=220,interpolation='antialiased');ax.set_title(title);ax.axis('off')
    fig.tight_layout();fig.savefig(out/'comparison.png',dpi=130);plt.close(fig)
    assert report['passed'],report
    print(json.dumps(report))


if __name__=='__main__':main()
