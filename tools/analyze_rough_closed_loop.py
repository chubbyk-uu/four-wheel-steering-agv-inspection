#!/usr/bin/env python3
"""Truth-only evaluation and unwarped image comparisons of deadband trials."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from scipy.spatial.transform import Rotation, Slerp


def pose_metrics(root):
    truth=np.load(root/'truth_evaluation.npy')
    _,indices=np.unique(truth[:,0],return_index=True)
    truth=truth[indices]
    nav=[json.loads(s) for s in (root/'navigation/navigation.jsonl').read_text().splitlines()]
    nav=[r for r in nav if truth[0,0]<r['time_s']<truth[-1,0]]
    t=np.array([r['time_s'] for r in nav]);idx=np.searchsorted(truth[:,0],t)
    bracket=truth[idx,0]-truth[idx-1,0]
    p=np.column_stack([np.interp(t,truth[:,0],truth[:,i]) for i in (1,2,3)])
    velocity=np.gradient(truth[:,1:3],truth[:,0],axis=0)
    speed=np.interp(t,truth[:,0],np.linalg.norm(velocity,axis=1))
    q=Slerp(truth[:,0],Rotation.from_quat(truth[:,4:8]))(t)
    estimate=np.array([r['position_m'] for r in nav])
    estimate_q=Rotation.from_quat([r['orientation_xyzw'] for r in nav])
    angular_error=np.degrees((q.inv()*estimate_q).as_rotvec())
    # World X of the camera mounting point is used only to select the interior
    # of the requested road ROI; neither truth nor navigation warps any image.
    config=yaml.safe_load((root/'camera.yaml').read_text())
    height=config['nominal_width_m']*config['focal_length_m']/(config['width']*config['pixel_pitch_m'])
    camera=p+q.apply([config['camera_x_m'],0,height-config['base_nominal_height_m']])
    execution=[json.loads(s) for s in (root/'mission/execution.jsonl').read_text().splitlines()]
    intervals=json.loads((root/'mission/capture_intervals.json').read_text())
    plan=json.loads((root/'mission/plan.json').read_text())
    report=[];curves=[]
    for track in plan['tracks']:
        window=np.zeros(len(t),bool)
        for event in intervals:
            if event['track_id']==track['id']:
                window|=(t>=event['enabled_ack_time_s'])&(t<=event['disabled_ack_time_s'])
        xmin,xmax=sorted((track['scan_start_xyz_m'][0],track['scan_end_xyz_m'][0]))
        selected=window&(camera[:,0]>xmin+.1)&(camera[:,0]<xmax-.1)&(bracket<=.06)
        if selected.sum()<20:raise ValueError('insufficient synchronized ROI truth')
        cy=track['center_road_y_m'];cross=p[selected,1]-cy
        e=estimate[selected]-p[selected]
        commands=[r['command_body'] for r in execution if r.get('kind')=='PASS'
                  and r.get('track_id')==track['id'] and t[selected][0]<=r['time_s']<=t[selected][-1]]
        if not commands:raise ValueError('missing ROI commands')
        commands=np.asarray(commands)
        report.append(dict(track=track['id'],samples=int(selected.sum()),
            true_cross_rmse_mm=float(np.sqrt(np.mean(cross**2))*1000),
            true_cross_max_abs_mm=float(max(abs(cross))*1000),
            true_cross_peak_to_peak_mm=float(np.ptp(cross)*1000),
            true_xy_speed_m_s_p5_median_p95=np.percentile(speed[selected],[5,50,95]).tolist(),
            command_body_vy_rms_m_s=float(np.sqrt(np.mean(commands[:,1]**2))),
            command_body_yaw_rate_rms_rad_s=float(np.sqrt(np.mean(commands[:,2]**2))),
            fusion_xyz_rmse_mm=(np.sqrt(np.mean(e**2,axis=0))*1000).tolist(),
            fusion_attitude_rotvec_rmse_deg=np.sqrt(np.mean(angular_error[selected]**2,axis=0)).tolist()))
        curves.append((camera[selected,0],cross*1000))
    return report,curves


def marking_centers(image):
    measurements=[]
    for row in range(14000,min(24000,len(image)-32),32):
        profile=np.median(image[row:row+32],axis=0)
        boundaries=np.flatnonzero(np.diff(np.r_[False,profile>90,False]))
        runs=[(l,r) for l,r in zip(boundaries[::2],boundaries[1::2])
              if l>2000 and r<len(profile) and 200<r-l<600]
        if len(runs)==2:
            measurements.append([row+15.5,*[(l+r-1)/2 for l,r in runs]])
    a=np.asarray(measurements)
    if len(a)<200:raise ValueError('double-line image ROI not reliably detected')
    return a


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--case',action='append',required=True,help='label=trial_directory')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--figures',type=Path,required=True)
    args=parser.parse_args();args.figures.mkdir(parents=True,exist_ok=True)
    cases=[value.split('=',1) for value in args.case]
    fig,axes=plt.subplots(1,3,figsize=(15,5))
    photos,photo_axes=plt.subplots(1,len(cases),figsize=(5*len(cases),11),squeeze=False)
    edges,edge_axes=plt.subplots(1,2,figsize=(12,5))
    results={}
    for label,directory in cases:
        root=Path(directory);trial=json.loads((root/'results.json').read_text())
        if not trial['passed']:raise ValueError('failed mission is not an accepted capture comparison')
        metrics,curves=pose_metrics(root)
        for ax,(x,y) in zip(axes,curves):ax.plot(x,y,label=label)
        # Known local experiment artifact, bounded at 4096 x 40000 pixels.
        Image.MAX_IMAGE_PIXELS=170_000_000
        with Image.open(root/'strips/track_02.png') as file:
            if file.width!=4096 or not 24000<=file.height<=40000:raise ValueError('unexpected strip dimensions')
            im=np.array(file)
        centers=marking_centers(im)
        straight=np.column_stack([np.polyval(np.polyfit(centers[:,0],centers[:,k],1),centers[:,0]) for k in (1,2)])
        for k,ax in enumerate(edge_axes):
            ax.plot(centers[:,0],centers[:,k+1]-centers[:,k+1].mean(),label=label)
        ax=photo_axes[0,len(results)]
        Image.fromarray(im[14000:24000,2400:3900]).save(root/'strips/double_line_detail.png')
        ax.imshow(im[14000:24000,2400:3900],cmap='gray',vmin=0,vmax=160,
                  extent=[2400,3900,24000,14000],aspect='equal',interpolation='antialiased')
        ax.set_title(label+' cross deadband');ax.set_xlabel('Original column');ax.set_ylabel('Original row')
        results[label]=dict(motion=metrics,capture=trial['capture'],
            final_state=trial['final']['state'],final_motion=trial['final']['motion_state'],
            double_line_centers_peak_to_peak_px=np.ptp(centers[:,1:],axis=0).tolist(),
            double_line_center_std_px=np.std(centers[:,1:],axis=0).tolist(),
            double_line_nonlinear_peak_to_peak_px=np.ptp(centers[:,1:]-straight,axis=0).tolist(),
            valid_image_windows=len(centers))
    for k,ax in enumerate(axes):
        ax.set_title('Track '+str(k));ax.set_xlabel('Road X (m)');ax.set_ylabel('True body cross-track error (mm)');ax.grid(alpha=.3);ax.legend()
    fig.suptitle('Closed-loop ROI truth: normal sensor noise, same rough road')
    fig.tight_layout();fig.savefig(args.figures/'rough_closed_loop_tracking.png',dpi=130);plt.close(fig)
    for k,ax in enumerate(edge_axes):
        ax.set_title('Double-line marking '+str(k));ax.set_xlabel('Row in own strip');ax.set_ylabel('Image center minus its mean (px)');ax.grid(alpha=.3);ax.legend()
    edges.suptitle('Image measurements only: no pose warp, no curve correction\nEach independent strip has its own acquisition origin; 32-row median, 90 DN threshold')
    edges.tight_layout(rect=[0,0,1,.88]);edges.savefig(args.figures/'rough_closed_loop_markings.png',dpi=130);plt.close(edges)
    photos.suptitle('Track 2: corrected original pixels, same crop and brightness\nRows 14000:24000; independent origins, no registration or stretching')
    photos.tight_layout(rect=[0,0,1,.94]);photos.savefig(args.figures/'rough_closed_loop_photos.png',dpi=150);plt.close(photos)
    report=dict(cases=results,
        scope='One three-track mission per setting on 20 m rough scene, 4 m requested ROI at 10 km/h. '
              'Not identical noise replay or proof of best default. Pose errors are evaluated against synchronized truth; '
              'attitude errors are small-angle body-frame rotation vectors. Images use only fixed flat/optical correction and row concatenation.',
        roi_margin_m=.1,truth_bracket_limit_s=.06,image_row_window=[14000,24000])
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(results))


if __name__=='__main__':main()
