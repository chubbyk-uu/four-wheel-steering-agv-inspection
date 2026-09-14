#!/usr/bin/env python3
"""Measure rendered no-parking-box row spans; no navigation warp/rescaling.

Specific to road_markings_probe_v1, forward scan at Y=-2.4 including the whole
box. The two wide transverse paint bands have row mean >105 DN in this exposure.
This is an image-landmark check, not a general-purpose strip matcher.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image


def measure(session):
    metadata=[json.loads(p.read_text()) for p in sorted(session.glob('block_*.json'))]
    for a,b in zip(metadata,metadata[1:]):
        assert b['first']['global_line']==a['last']['global_line']+1
    image=np.concatenate([np.asarray(Image.open(session/f"block_{m['block_id']:06d}.pgm")) for m in metadata])
    # Ignore the leading partial arrow. Require two broad, contiguous bands.
    rows=np.flatnonzero((image.mean(1)>105)&(np.arange(len(image))>5000))
    bands=[g for g in np.split(rows,np.flatnonzero(np.diff(rows)>1)+1) if len(g)>100]
    assert len(bands)>=2, 'expected both transverse no-parking box borders'
    bands=bands[:2]  # a later arrow can enter the end of the longer 0.42 m run
    centers=[float(g.mean()) for g in bands]
    contract=metadata[0]['wheel_encoder']
    assert all(m['wheel_encoder']==contract for m in metadata)
    tags=[t for m in metadata for t in m['pose_tags']]
    return image,dict(actual_diameter_m=contract['actual_diameter_m_truth'],
        calibrated_diameter_m=contract['calibrated_diameter_m'],
        lines_per_revolution=contract['lines_per_revolution'],
        rows=len(image),border_centers_rows=centers,border_span_rows=centers[1]-centers[0],
        median_camera_height_m_truth=float(np.median([t['camera_position_world_m'][2] for t in tags])),
        longitudinal_spacing_m=metadata[0]['line_spacing_m'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('sessions',type=Path,nargs=3,help='0.39, 0.40 and 0.42 m raw sessions, any order')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    pairs=sorted((measure(s) for s in args.sessions),key=lambda x:x[1]['actual_diameter_m'])
    baseline=next(m for _,m in pairs if m['actual_diameter_m']==.4)
    for _,m in pairs:
        m['measured_scale_vs_040']=m['border_span_rows']/baseline['border_span_rows']
        m['expected_scale_vs_040']=.4/m['actual_diameter_m']
        m['scale_error']=m['measured_scale_vs_040']-m['expected_scale_vs_040']
        assert abs(m['scale_error'])<.001, m
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,3,figsize=(12,12),sharey=True)
    # Identical raw-pixel scale in all panels; only integer row translation.
    # Show same-width crops; do not normalize every strip to the same length.
    for ax,(image,m) in zip(axes,pairs):
        origin=round(m['border_centers_rows'][0])-500
        crop=image[origin:origin+10500]
        ax.imshow(crop[::8,::8],cmap='gray',vmin=0,vmax=180,
                  extent=[0,4096,origin+len(crop)-m['border_centers_rows'][0],origin-m['border_centers_rows'][0]])
        ax.set_ylim(10000,-500)
        ax.set_title(f"Tyre {m['actual_diameter_m']:.2f} m\n{m['border_span_rows']:.0f} rows; {(m['measured_scale_vs_040']-1)*100:+.2f}%")
        ax.set_xlabel('Raw column (same display scale)')
        ax.axhline(baseline['border_span_rows'],color='tab:red',lw=.7,ls='--')
    axes[0].set_ylabel('Raw rows from first transverse border')
    fig.suptitle('Actual OptiX captures: fixed encoder ratio and calibrated diameter 0.40 m\nNo geometric correction or vertical resizing; red = baseline second border')
    fig.tight_layout();fig.savefig(args.output/'comparison.png',dpi=150);plt.close(fig)
    report=dict(passed=True,method='two raw image transverse border centers; no truth position used in scale measurement',
        cases=[m for _,m in pairs],preview='common raw pixel scale; integer translation; shared fixed grayscale window')
    (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':main()
