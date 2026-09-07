#!/usr/bin/env python3
"""Compare two completed bakes, rejecting mismatched metric defect layouts."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image
import numpy as np
p=argparse.ArgumentParser();p.add_argument('first',type=Path);p.add_argument('second',type=Path);p.add_argument('--output',required=True,type=Path);a=p.parse_args()
roots=[a.first,a.second];ms=[json.loads((r/'manifest.json').read_text()) for r in roots]
for key in ('length_m','width_m','texel_m','slab_size_m','joint_width_m','lane_markings','instances','cracks'):
    if ms[0][key]!=ms[1][key]:raise ValueError('Mismatched layout: '+key)
if ms[0]['previews']['crop_pixels_xy']!=ms[1]['previews']['crop_pixels_xy']:raise ValueError('Different crop position')
for name in ('crack_256mm_labels.png','crack_256mm_support.png'):
    if not np.array_equal(np.asarray(Image.open(roots[0]/name)),np.asarray(Image.open(roots[1]/name))):
        raise ValueError('Different defect raster: '+name)
fig,axes=plt.subplots(2,2,figsize=(12,11),layout='constrained')
for row,(r,m) in enumerate(zip(roots,ms)):
    v=m['previews'];xmin,xmax,ymin,ymax=v['crop_bounds_xy_m']
    axes[row,0].imshow(Image.open(r/v['overview']),extent=(0,m['length_m'],-m['width_m']/2,m['width_m']/2))
    axes[row,0].add_patch(Rectangle((xmin,ymin),xmax-xmin,ymax-ymin,fill=False,edgecolor='red',linewidth=2))
    axes[row,0].set(title=m.get('material_name','Gravel Concrete 03')+' | 10 x 10 m',xlabel='World X (m)',ylabel='World Y (m)')
    axes[row,1].imshow(Image.open(r/v['after']),extent=(0,256,0,256))
    axes[row,1].set(title='Same defect and position | 256 x 256 mm',xlabel='Local X (mm)',ylabel='Local Y (mm)')
fig.suptitle('Baked texture candidates | identical slabs, markings and AI crack layout\nConcrete047A source mapped to 2.1 m for comparison; physical scale unspecified by provider')
a.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.output,dpi=150);plt.close(fig)
print(json.dumps(dict(layout_match=True,crop_match=True,defect_raster_match=True,output=str(a.output))))
