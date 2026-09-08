#!/usr/bin/env python3
"""Export a geometric preview of a saved mission (not a simulator screenshot)."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('plan', type=Path)
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
p = json.loads(args.plan.read_text())
fig, ax = plt.subplots(figsize=(12, 7))
ax.add_patch(Polygon([v[:2] for v in p['region_xyz_m']], fill=False, color='#c29400', lw=3, label='Requested region'))
for t in p['tracks']:
    ax.add_patch(Polygon([v[:2] for v in t['nominal_footprint_xyz_m']], color='#19ac88', alpha=.09))
    a,b = t['scan_start_xyz_m'],t['scan_end_xyz_m']
    ax.annotate('',xy=b[:2],xytext=a[:2],arrowprops=dict(arrowstyle='->',color='#148065',lw=2))
    ax.text((a[0]+b[0])/2,a[1]+.07,f"Scan {t['id']+1}",color='#12634f')
for seg in p['segments']:
    xy = [pt['pose']['position'][:2] for pt in seg['points']]
    ax.plot([v[0] for v in xy],[v[1] for v in xy],color='#376bc0' if seg['capture'] else '#cc4580',alpha=.7,lw=1.5,
            linestyle='--' if seg['capture'] else '-')
    if seg['kind']=='ROTATE_180':
        ax.add_patch(Circle(xy[0],p['sweep_radius_m'],fill=False,color='#e38d20',alpha=.4,linestyle=':'))
ax.plot([],[],color='#148065',label='Camera scan line center')
ax.plot([],[],color='#376bc0',linestyle='--',label='Base during scan (1.15 m offset)')
ax.plot([],[],color='#cc4580',label='Acceleration / braking / transfer')
ax.plot([],[],color='#e38d20',linestyle=':',label='Conservative turn envelope')
ax.set_aspect('equal'); ax.grid(alpha=.2); ax.margins(.08)
ax.set_xlabel(f"{p['frame_id']} X (m)"); ax.set_ylabel(f"{p['frame_id']} Y (m)")
ax.set_title(f"8 x 4 m example | {p['track_count']} tracks | fixed {p['actual_track_spacing_m']:.1f} m spacing | preview only")
ax.legend(loc='upper center',bbox_to_anchor=(.5,-.12),ncol=2)
fig.tight_layout()
args.output.parent.mkdir(parents=True,exist_ok=True)
fig.savefig(args.output,dpi=160,bbox_inches='tight')
