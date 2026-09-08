#!/usr/bin/env python3
"""Display equal-sized raw blocks from similar road positions; no correction."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

p=argparse.ArgumentParser();p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
fig,axes=plt.subplots(1,2,figsize=(12,6),layout='constrained')
for ax,root,title in zip(axes,(a.before,a.after),('Before: global feedback baseline','After: tuned global feedback')):
    blocks=json.loads((root/'mission/capture_manifest.json').read_text())['blocks'];candidates=[]
    for b in blocks:
        if b['track_id']!=1 or b['rows']!=4096:continue
        m=json.loads(Path(b['image']).with_suffix('.json').read_text())
        x=(m['first']['camera_position_world_m'][0]+m['last']['camera_position_world_m'][0])/2
        candidates.append((abs(x-9),b,x))
    _,block,x=min(candidates,key=lambda b:b[0]);raw=np.asarray(Image.open(block['image']))
    ax.imshow(raw,cmap='gray',vmin=0,vmax=140);ax.set_title(f'{title}\n4096 x 4096, road midpoint x={x:.2f} m')
    ax.set_xlabel('Raw column');ax.set_ylabel('Scan row')
fig.savefig(a.output,dpi=160)
