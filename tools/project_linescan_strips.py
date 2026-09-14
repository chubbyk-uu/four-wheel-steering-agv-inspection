#!/usr/bin/env python3
"""Diagnostic flat-road strip projection; no feature matching or seam optimization."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image
from numpy.polynomial.polynomial import polyval
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/agv_linescan'),str(ROOT/'src/agv_mission')]
from agv_linescan.calibration import Correction
from agv_linescan.strip_projection import line_times,navigation_at,project_flat,projection_pose
from agv_mission.capture_audit import Navigation


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('mission','navigation','profile','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--camera-x-residual-mm',type=float,default=0.,help='Additional fixed body-X estimated lever-arm error; replay experiment only')
    p.add_argument('--resolution-m',type=float,default=1.5/4096)
    p.add_argument('--pose-mode',choices=['dynamic','fixed_height','fixed_height_tilt'],default='dynamic')
    a=p.parse_args();plan=json.loads((a.mission/'plan.json').read_text())
    road=plan['request']['road'];region=plan['request']['region']
    if road['yaw_rad']!=0 or road['origin_xyz_m']!=[0,0,0]:raise ValueError('first diagnostic requires world-aligned z=0 road')
    if not np.isfinite([a.resolution_m,a.camera_x_residual_mm]).all() or a.resolution_m<=0:raise ValueError('invalid projection settings')
    nav=Navigation(a.navigation);profile=json.loads(a.profile.read_text());correction=Correction(profile)
    x0,y0=region['start_xy_m'];nr=int(np.ceil(region['length_m']/a.resolution_m));nc=int(np.ceil(region['width_m']/a.resolution_m))
    if nr*nc>100_000_000:raise ValueError('diagnostic limited to 100 million output pixels; use a short mission')
    a.output.mkdir(parents=True,exist_ok=False)
    offset=nav.offset+np.array([a.camera_x_residual_mm*.001,0,0])
    reports=[]
    intervals=json.loads((a.mission/'capture_intervals.json').read_text())
    for interval in intervals:
        if 'disabled_ack_time_s' not in interval:raise ValueError('capture interval is not closed')
        archive=Path(interval['archive']);config=yaml.safe_load((archive/'calibration.yaml').read_text());correction.check_capture(config)
        if nav.cal['optical_intrinsic_id'] not in (config['calibration_id'],config['calibration_id'].removesuffix('-optix-strip-v1')):
            raise ValueError('navigation optical calibration mismatch')
        width=config['width'];q=(np.arange(width)-(width-1)/2)/(width/2)
        height=config['nominal_width_m']*config['focal_length_m']/(width*config['pixel_pitch_m'])
        # Fixed-height board calibration cannot separately identify focal length,
        # camera height and mount tilt. This is the declared nominal-height model.
        across=polyval(q,profile['geometry']['metric_polynomial'])
        rays=np.column_stack([across/height,np.zeros(width),np.ones(width)])
        measured=profile['geometry']['measured_raw_columns']
        valid_columns=np.asarray(profile['flat']['valid']) & (np.arange(width)>=measured[0]) & (np.arange(width)<=measured[-1])
        sums=np.zeros((nr,nc),np.float32);weights=np.zeros_like(sums);blocks=0;rows=0;shift_sum=np.zeros(2);shift_count=0
        for path in sorted(archive.glob('block_*.json')):
            m=json.loads(path.read_text())
            if not interval['enabled_ack_time_s']-.05<=m['first']['time_s']<=m['last']['time_s']<=interval['disabled_ack_time_s']+.05:continue
            correction.metadata(m);times=line_times(m)
            with Image.open(path.with_suffix('.pgm')) as im:raw=np.array(im)
            if raw.shape!=(m['rows'],width) or raw.dtype!=np.uint8:raise ValueError('invalid source pixels')
            for start in range(0,len(raw),64):
                values=raw[start:start+64];positions,rotations=navigation_at(nav,times[start:start+64])
                positions,rotations=projection_pose(positions,rotations,offset,a.pose_mode,height)
                points=project_flat(positions,rotations,offset,nav.mount,rays)
                shift=rotations.apply([a.camera_x_residual_mm*.001,0,0])[:,:2]
                shift_sum+=shift.sum(0);shift_count+=len(shift)
                rr=(points[:,:,0]-x0)/a.resolution_m-.5;cc=(points[:,:,1]-y0)/a.resolution_m-.5
                ri=np.floor(rr).astype(np.int64);ci=np.floor(cc).astype(np.int64)
                gray=(values.astype(np.float32)-correction.offset)*correction.gain
                valid=(values!=255)&valid_columns[None,:]
                for dr in (0,1):
                    for dc in (0,1):
                        r=ri+dr;c=ci+dc;w=(1-abs(rr-r))*(1-abs(cc-c))
                        keep=valid&(r>=0)&(r<nr)&(c>=0)&(c<nc)
                        np.add.at(sums,(r[keep],c[keep]),(gray*w)[keep])
                        np.add.at(weights,(r[keep],c[keep]),w[keep])
            blocks+=1;rows+=len(raw)
        valid=weights>1e-6;image=np.zeros((nr,nc),np.uint8)
        image[valid]=np.rint(np.clip(sums[valid]/weights[valid],0,255)).astype(np.uint8)
        name=f"track_{interval['track_id']:02d}"
        Image.fromarray(image).save(a.output/(name+'.png'));Image.fromarray(valid.astype(np.uint8)*255).save(a.output/(name+'_valid.png'))
        reports.append(dict(track_id=interval['track_id'],blocks=blocks,rows=rows,covered_pixels=int(valid.sum()),
                            mean_injected_map_shift_m=(shift_sum/max(1,shift_count)).tolist()))
    report=dict(schema='agv.strip_projection.diagnostic.v1',tracks=reports,shape=[nr,nc],resolution_m=a.resolution_m,
                origin_xy_m=[x0,y0],image_axes='rows road +X, columns road +Y; pixel centers at half-cell',
                pose_mode=a.pose_mode,fixed_optical_height_m=height if a.pose_mode!='dynamic' else None,
                fixed_body_roll_pitch_rad=[0.,0.] if a.pose_mode=='fixed_height_tilt' else None,
                camera_x_residual_mm=a.camera_x_residual_mm,navigation_calibration_id=nav.cal['calibration_id'],
                optical_profile_id=profile['calibration_id'],
                scope='Independent track rasters from fused navigation and calibrated rays. Bilinear forward splat, no gap filling, matching or stitching. Sparse line-time interpolation; fixed-plane nominal-height optics. Raw truth poses are not read.')
    (a.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':main()
