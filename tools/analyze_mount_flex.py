#!/usr/bin/env python3
"""Evaluate physical camera mounting against sparse camera tags; no image warp."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import yaml
from PIL import Image


def analyze(path):
    truth=np.load(path/'truth_evaluation.npy');mount=np.load(path/'mount_evaluation.npy')
    clock=np.load(path/'clock_evaluation.npy');cfg=yaml.safe_load((path/'camera.yaml').read_text())
    flexible=cfg['mount_flex']['enabled'];times=truth[:,0]
    unique=np.r_[True,np.diff(times)>0];truth=truth[unique];times=truth[:,0]
    turns=Rotation.from_quat(truth[:,4:8]);slerp=Slerp(times,turns)
    intervals=json.loads((path/'mission/capture_intervals.json').read_text())
    assert len(intervals)==1,'single straight pass required'
    interval=intervals[0];blocks=[]
    for p in sorted(Path(interval['archive']).glob('block_*.json')):
        b=json.loads(p.read_text())
        if interval['enabled_ack_time_s']-.05<=b['first']['time_s']<=b['last']['time_s']<=interval['disabled_ack_time_s']+.05:blocks.append(b)
    assert blocks
    tags={t['global_line']:t for b in blocks for t in [b['first'],*b.get('pose_tags',[]),b['last']]}
    # Archived tag key is checked explicitly; first/last remain independent anchors.
    taglist=sorted(tags.values(),key=lambda t:t['time_s'])
    taglist=[t for t in taglist if times[0]<=t['time_s']<=times[-1] and (not flexible or mount[0,0]<=t['time_s']<=mount[-1,0])]
    tt=np.array([t['time_s'] for t in taglist]);navpos=np.column_stack([np.interp(tt,times,truth[:,c]) for c in (1,2,3)])
    rb=slerp(tt);hc=cfg['nominal_width_m']*cfg['focal_length_m']/(cfg['width']*cfg['pixel_pitch_m'])-cfg['base_nominal_height_m']
    angles=np.zeros((len(tt),3))
    if flexible:
        angles[:,:2]=np.column_stack([np.interp(tt,mount[:,0],mount[:,c]) for c in (1,2)])
    # Intrinsic X then Y: physical nested roll/pitch joints.
    rm=Rotation.from_euler('XYZ',angles)
    pivot=np.array([1.005,0,.14]);arm=np.array([cfg['camera_x_m'],0,hc])-pivot
    expected=navpos+rb.apply(pivot+rm.apply(arm))
    actual=np.array([t['camera_position_world_m'] for t in taglist])
    camera_rotation=Rotation.from_matrix(np.array([t['camera_rotation_world'] for t in taglist]))
    optical=Rotation.from_euler('xyz',[np.pi,0,np.pi/2])
    expected_rot=rb*rm*optical
    error=np.linalg.norm(expected-actual,axis=1)*1000
    re=(expected_rot.inv()*camera_rotation).magnitude()*180/np.pi
    # Use true body location only to select the evaluated ROI, never to reshape pixels.
    roi=(truth[:,1]>=8.1)&(truth[:,1]<=11.9)&(times>=interval['enabled_ack_time_s'])&(times<=interval['disabled_ack_time_s'])
    rt=times[roi];assert len(rt)>30
    m=np.zeros((len(rt),2)) if not flexible else np.column_stack([np.interp(rt,mount[:,0],mount[:,c]) for c in (1,2)])
    body=turns.as_euler('xyz')[roi,:2];m=np.rad2deg(m);body=np.rad2deg(body)
    w=np.interp(rt,clock[:,0],clock[:,1]);rtf=(rt[-1]-rt[0])/(w[-1]-w[0])
    capture_mount=mount[(mount[:,0]>=interval['enabled_ack_time_s'])&(mount[:,0]<=interval['disabled_ack_time_s'])] if flexible else np.zeros((1,3))
    report=dict(mount_capture_peak_abs_deg=np.rad2deg(abs(capture_mount[:,1:]).max(0)).tolist(),passed=json.loads((path/'results.json').read_text())['passed'],mount=cfg['mount_flex'],roi_samples=len(rt),rtf=float(rtf),
        mount_roi_std_deg=np.std(m,0).tolist(),mount_roi_p2p_deg=np.ptp(m,0).tolist(),
        mount_all_peak_abs_deg=(np.rad2deg(abs(mount[:,1:]).max(0)).tolist() if flexible else [0.,0.]),
        body_roi_std_deg=np.std(body,0).tolist(),tag_fk_max_position_error_mm=float(error.max()),tag_fk_max_rotation_error_deg=float(re.max()),
        tag_fk_samples=len(tt),blocks=len(blocks),rows=sum(b['rows'] for b in blocks))
    # 50 Hz body + 100 Hz joint diagnostic interpolation is coarser than sensor physics.
    assert error.max()<2. and re.max()<.15,report
    return report,(truth[roi,1],m,body)


def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--figures',type=Path,required=True);a=p.parse_args()
    names=['rigid','600','150','40'];report={};fig,axes=plt.subplots(2,1,figsize=(10,7),sharex=True)
    photo,pa=plt.subplots(1,len(names),figsize=(16,7))
    for name,ax in zip(names,pa):
        path=a.root/(name+'_single');r,(x,m,b)=analyze(path);report[name]=r
        for j in range(2):axes[j].plot(x,m[:,j],label=name)
        file=path/'fixed_strips/track_00.png'
        if file.exists():
            image=np.asarray(Image.open(file));crop=image[14000:24000]
            centers=[]
            for row in range(0,len(crop)-31,32):
                line=np.median(crop[row:row+32],axis=0);ids=np.flatnonzero(line[2200:]>90)+2200
                groups=np.split(ids,np.where(np.diff(ids)>1)[0]+1)
                groups=[g for g in groups if len(g)>30 and g[0]>2200 and g[-1]<image.shape[1]-1]
                if len(groups)==2:centers.append([row,*[float((g[0]+g[-1])/2) for g in groups]])
            center=np.asarray(centers);assert len(center)>100,'yellow line diagnostic unavailable'
            fit=np.column_stack([center[:,0],np.ones(len(center))]);residual=center[:,1:]-fit@np.linalg.lstsq(fit,center[:,1:],rcond=None)[0]
            r['image_center_p2p_px']=np.ptp(center[:,1:],axis=0).tolist()
            r['image_center_detrended_p2p_px']=np.ptp(residual,axis=0).tolist()
            r['image_center_windows']=len(center)
            r['image_rows_window']=[14000,24000]
            ax.imshow(crop[::4,::4],cmap='gray',vmin=0,vmax=160,aspect='equal');ax.set_title(name+' N m/rad' if name!='rigid' else 'Rigid');ax.axis('off')
    for ax,label in zip(axes,['Relative roll (deg)','Relative pitch (deg)']):ax.set_ylabel(label);ax.grid();ax.legend()
    axes[-1].set_xlabel('True body road X (m); evaluation only');fig.tight_layout()
    a.figures.mkdir(parents=True,exist_ok=True);fig.savefig(a.figures/'mount_flex_motion.png',dpi=140);plt.close(fig)
    photo.suptitle('Same native row window 14000:24000; fixed correction + direct concatenation\nNo pose warp or registration; independent acquisition origins');photo.tight_layout();photo.savefig(a.figures/'mount_flex_photos.png',dpi=140);plt.close(photo)
    normal=a.root/'150_normal'
    if (normal/'results.json').exists():report['150_normal']=analyze(normal)[0]
    failures={}
    for name in ('run600','rigid_zero'):
        path=a.root/name/'results.json'
        if path.exists():
            f=json.loads(path.read_text())['final'];failures[name]={k:f.get(k) for k in ('state','reason','motion_state','track_id','reference_heading_error_rad')}
    report['two_track_failures']=failures
    report['scope']='Single straight closed-loop pass, zero localization noise, GUI/RViz; not a multi-track or full-road acceptance. FK check uses coarser independent body/joint recordings.'
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

if __name__=='__main__':main()
