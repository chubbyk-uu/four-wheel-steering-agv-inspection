#!/usr/bin/env python3
"""Concrete image quality report and actual-pixel samples; no image synthesis."""
import argparse,json,sys,yaml
from pathlib import Path
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.calibration import Correction


def display_srgb(pixels, black=0):
    """Screen-only transfer; quantitative checks above/below use original DN."""
    linear=np.clip((np.arange(256,dtype=float)-black)/(255-black),0,1)
    lut=np.rint(255*np.where(linear<=.0031308,12.92*linear,1.055*linear**(1/2.4)-.055)).astype(np.uint8)
    return lut[pixels]


def main():
    p=argparse.ArgumentParser();p.add_argument('--archive',required=True);p.add_argument('--profile',required=True);p.add_argument('--output',required=True);p.add_argument('--corrected-dir');a=p.parse_args()
    archive=Path(a.archive);corrected_dir=Path(a.corrected_dir) if a.corrected_dir else archive.parent/'corrected';out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    correction=Correction(json.loads(Path(a.profile).read_text()));metas=[json.loads(p.read_text()) for p in sorted(archive.glob('block_*.json'))]
    totals=dict(pixels=0,raw_saturated_pixels=0,corrected_clipped_pixels=0);candidates=[]
    for m in metas:
        path=archive/f'block_{m["block_id"]:06d}.pgm';raw=np.array(Image.open(path));fixed,quality=correction.apply(raw)
        expected=np.array(Image.open(corrected_dir/path.name));assert np.array_equal(fixed,expected)
        totals['pixels']+=raw.size
        for key in ('raw_saturated_pixels','corrected_clipped_pixels'):totals[key]+=quality[key]
        candidates.append((abs(m['pose_tags'][len(m['pose_tags'])//2]['camera_position_world_m'][0]-22.5),m))
    m=min(candidates,key=lambda x:x[0])[1];name=f'block_{m["block_id"]:06d}.pgm'
    raw=np.array(Image.open(archive/name));fixed=np.array(Image.open(corrected_dir/name))
    # An independent metric edge check on the painted bands; known world Y edges.
    expected_edges=np.array([-.225,-.075,.075,.225])/(1.2/4096)+2047.5
    profile=fixed.astype(float).mean(axis=0);derivative=np.diff(profile);observed=[]
    for edge in expected_edges:
        candidates=np.arange(max(1,int(edge)-12),min(len(derivative)-1,int(edge)+13))
        i=candidates[np.argmax(np.abs(derivative[candidates]))];v=np.abs(derivative[i-1:i+2]);den=v[0]-2*v[1]+v[2]
        shift=.5*(v[0]-v[2])/den if abs(den)>1e-8 else 0
        observed.append(float(i+.5+np.clip(shift,-.5,.5)))
    # Evaluation only: account for recorded camera lateral displacement/attitude.
    # These simulator tags are not used by the correction algorithm.
    cfg=yaml.safe_load((archive/'calibration.yaml').read_text())
    h=cfg['nominal_width_m']*cfg['focal_length_m']/(cfg['width']*cfg['pixel_pitch_m'])
    pose_expected=[]
    for tag in m['pose_tags']:
        origin=np.array(tag['camera_position_world_m']);rotation=np.array(tag['camera_rotation_world'])
        across,down=rotation[:,0],rotation[:,2];dy=np.array([-.225,-.075,.075,.225])-origin[1]
        q=(-origin[2]*down[1]-dy*down[2])/(dy*across[2]+origin[2]*across[1])
        pose_expected.append(h*q/(1.2/4096)+2047.5)
    pose_expected=np.mean(pose_expected,axis=0)
    # Partition invariance around an actual image boundary, not a tile-border copy.
    current=m['block_id'];next_meta=next(v for v in metas if v['block_id']==current+1)
    following=np.array(Image.open(archive/f'block_{current+1:06d}.pgm'))
    joined=np.concatenate([raw[-256:],following[:256]])
    separate=np.concatenate([correction.apply(raw[-256:])[0],correction.apply(following[:256])[0]])
    assert np.array_equal(correction.apply(joined)[0],separate)
    assert next_meta['first']['global_line']==m['last']['global_line']+1
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    raw_display=display_srgb(raw,cfg.get('radiometry',{}).get('black_level_dn',0))
    fixed_display=display_srgb(fixed)
    fig,axes=plt.subplots(1,2,figsize=(10,5))
    for ax,img,title in zip(axes,[raw_display,fixed_display],['Raw capture (sRGB display)','Offline corrected (sRGB display)']):
        ax.imshow(img,cmap='gray',vmin=0,vmax=255,interpolation='antialiased');ax.set_title(title);ax.axis('off')
    fig.tight_layout();fig.savefig(out/'concrete_correction.png',dpi=150);plt.close(fig)
    # Readable 1:1 crop in corrected coordinates around the retained crack near y=-.45 m.
    # Raw columns use the measured inverse map so both panels refer to the same surface region.
    left,right=180,850;top,bottom=512,1536
    rleft=int(np.floor(correction.profile['geometry']['lookup'][left]));rright=int(np.ceil(correction.profile['geometry']['lookup'][right]))
    fig,axes=plt.subplots(1,2,figsize=(8,6))
    for ax,img,title in zip(axes,[raw_display[top:bottom,rleft:rright],fixed_display[top:bottom,left:right]],['Raw crack (sRGB display)','Corrected crack (sRGB display)']):
        ax.imshow(img,cmap='gray',vmin=0,vmax=255,interpolation='nearest');ax.set_title(title);ax.axis('off')
    fig.tight_layout();fig.savefig(out/'concrete_crack_correction.png',dpi=160);plt.close(fig)
    report=dict(blocks=len(metas),sample_block=current,sample_first_camera_x_m=m['first']['camera_position_world_m'][0],sample_last_camera_x_m=m['last']['camera_position_world_m'][0],
        **totals,raw_saturated_fraction=totals['raw_saturated_pixels']/totals['pixels'],corrected_clipped_fraction=totals['corrected_clipped_pixels']/totals['pixels'],
        stripe_expected_edge_columns=expected_edges.tolist(),stripe_observed_edge_columns=observed,stripe_edge_error_px_max=float(np.max(np.abs(np.array(observed)-expected_edges))),
        evaluation_pose_adjusted_edge_columns=pose_expected.tolist(),evaluation_pose_adjusted_edge_error_px_max=float(np.max(np.abs(np.array(observed)-pose_expected))),
        evaluation_camera_lateral_m=m['pose_tags'][len(m['pose_tags'])//2]['camera_position_world_m'][1],
        ground_truth_used_only_for_independent_evaluation=True,
        corrected_pixels_match_reference=True,partition_invariance=True,within_track_rows_overlap=0,figure_display_transfer='srgb; raw black pedestal removed for display only; metrics use linear DN',
        scope='Actual Mono8 captures; saturation counts cannot imply recovered detail. Crack crops are visual inspection, not measured defect-width ground truth or detection acceptance.')
    (out/'quality.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()
