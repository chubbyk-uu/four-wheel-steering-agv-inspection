#!/usr/bin/env python3
"""Compare identical strip images projected with three height/tilt policies."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter1d,label
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def measure(image,spacing,y0,threshold=90,valid=None):
    centers=[]
    for row in range(0,len(image),32):
        pair=[]
        for y in [-.15,.15]:
            j=int((y-y0)/spacing);lo=j-320
            p=gaussian_filter1d(np.median(image[row:row+32,lo:j+320],axis=0).astype(float),2)
            ids,n=label(p>threshold);sizes=np.bincount(ids);sizes[0]=0
            ii=np.where(ids==sizes.argmax())[0] if sizes.max()>150 else []
            if len(ii)>150 and valid is not None:
                left=max(0,lo+ii[0]-8);right=min(image.shape[1]-1,lo+ii[-1]+8)
                if np.min(valid[row:row+32,left:right+1].mean(axis=0))<.75:ii=[]
            pair.append((lo+(ii[0]+ii[-1])/2+.5)*spacing+y0 if len(ii)>150 else np.nan)
        centers.append(pair)
    return np.array(centers)


def main():
    p=argparse.ArgumentParser();p.add_argument('--inputs',nargs=3,type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(exist_ok=False,parents=True)
    titles=['Dynamic height + dynamic tilt','Fixed height + dynamic tilt','Fixed height + fixed tilt']
    summaries=[json.loads((d/'summary.json').read_text()) for d in a.inputs]
    for s in summaries[1:]:
        for k in ['shape','resolution_m','origin_xy_m','optical_profile_id','navigation_calibration_id']:assert s[k]==summaries[0][k],k
    s=summaries[0];spacing=s['resolution_m'];x0,y0=s['origin_xy_m'];nr,nc=s['shape'];reports=[]
    extracted={};common={}
    for track in (0,1):
        for mode,d in enumerate(a.inputs):
            im=np.array(Image.open(d/f'track_{track:02d}.png'))
            valid=np.array(Image.open(d/f'track_{track:02d}_valid.png'))>0
            for threshold in [85,90,95]:extracted[mode,track,threshold]=measure(im,spacing,y0,threshold,valid)
        common[track]=np.logical_and.reduce([np.isfinite(extracted[mode,track,t]) for mode in range(3) for t in [85,90,95]])
        assert common[track].sum()>50,'insufficient common untruncated marking samples'
    fig,axes=plt.subplots(2,3,figsize=(15,12),sharex=True,sharey=True)
    curves,ca=plt.subplots(2,2,figsize=(13,8),sharex=True)
    for mode,d in enumerate(a.inputs):
        for track in (0,1):
            image=np.array(Image.open(d/f'track_{track:02d}.png'));assert list(image.shape)==s['shape']
            axes[track,mode].imshow(image[::4,::4],cmap='gray',vmin=0,vmax=160,extent=[y0,y0+nc*spacing,x0+nr*spacing,x0],aspect='equal')
            axes[track,mode].set_title(f'Track {track}: {titles[mode]}');axes[track,mode].set_xlabel('Road Y (m)');axes[track,mode].set_ylabel('Road X (m)')
            keep=common[track];c=np.where(keep,extracted[mode,track,90],np.nan)
            x=x0+(np.arange(len(keep))*32+16)*spacing
            widths=c[:,1]-c[:,0];offsets=np.nanmean(c-[-.15,.15],axis=1)
            def stat(v,fn):
                v=np.asarray(v);v=v[np.isfinite(v)]
                return float(fn(v)*1000) if len(v) else None
            report=dict(mode=d.name,track=track,samples_per_marking=keep.sum(0).tolist(),
                center_mean_bias_mm=[stat(c[:,k]-[-.15,.15][k],np.mean) for k in (0,1)],
                center_std_mm=[stat(c[:,k],np.std) for k in (0,1)],
                separation_mean_mm=stat(widths,np.mean),separation_std_mm=stat(widths,np.std))
            report['threshold_sensitivity_center_std_mm']={str(t):[stat(extracted[mode,track,t][:,k][keep[:,k]],np.std) for k in (0,1)] for t in [85,95]}
            reports.append(report)
            ca[track,0].plot(x,offsets*1000,label=titles[mode]);ca[track,1].plot(x,widths*1000,label=titles[mode]) if np.isfinite(widths).any() else None
            ca[track,0].set_title(f'Track {track}: mean available marking offset');ca[track,1].set_title(f'Track {track}: marking center separation')
            ca[track,0].set_ylabel('Common lateral offset (mm)');ca[track,1].set_ylabel('Separation (mm)')
    if not np.isfinite(extracted[0,1,90][:,0]).any():
        ca[1,1].text(.5,.5,'Not measurable: one marking is clipped',ha='center',transform=ca[1,1].transAxes)
    for row in ca:
        for ax in row:ax.grid(alpha=.3);ax.set_xlabel('Road X (m)');ax.legend(fontsize=8) if ax.lines else None
    fig.tight_layout();fig.savefig(a.output/'comparison.png',dpi=140);plt.close(fig)
    curves.tight_layout();curves.savefig(a.output/'marking_metrics.png',dpi=140);plt.close(curves)
    result=dict(scope='Same six raw blocks, calibration and fused navigation; only projection height/tilt policy changes. No matching or truth-pose substitution.',fixed_optical_height_m=summaries[1]['fixed_optical_height_m'],fixed_body_roll_pitch_rad=[0,0],method='32-row median, Gaussian sigma=2 columns, dominant component >90 DN, center=edge midpoint; 85/95 DN sensitivity. All policies use identical valid rows per marking; null means marking is clipped, not measurable; reject components within eight pixels of invalid coverage. Values describe projected line straightness, not longitudinal or absolute survey accuracy.',measurements=reports)
    (a.output/'metrics.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
