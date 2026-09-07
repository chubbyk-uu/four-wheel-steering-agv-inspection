#!/usr/bin/env python3
"""Compare two archive-derived views of the same physical patch."""
import argparse,json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image

p=argparse.ArgumentParser();p.add_argument('directory');a=p.parse_args()
root=Path(a.directory);m=json.loads((root/'manifest.json').read_text());v=m['previews']
xmin,xmax,ymin,ymax=v['crop_bounds_xy_m']
fig,axes=plt.subplots(1,3,figsize=(17,6),layout='constrained')
axes[0].imshow(Image.open(root/v['overview']),extent=(0,m['length_m'],-m['width_m']/2,m['width_m']/2))
axes[0].add_patch(Rectangle((xmin,ymin),xmax-xmin,ymax-ymin,fill=False,edgecolor='red',linewidth=2))
axes[0].set(title=f"{m.get('material_name','Gravel Concrete 03')} | {m['length_m']:g} x {m['width_m']:g} m",xlabel='World X (m)',ylabel='World Y (m)')
axes[0].text(.02,.98,f'Red box: X {xmin:.3f}..{xmax:.3f} m\nY {ymin:.3f}..{ymax:.3f} m',
             transform=axes[0].transAxes,va='top',color='white',bbox=dict(facecolor='black',alpha=.65))
for ax,key,title in zip(axes[1:],('before','after'),('Before defects','After defects')):
    ax.imshow(Image.open(root/v[key]),extent=(0,256,0,256))
    ax.set(title=title+' | same 256 x 256 mm patch',xlabel='Local X (mm)',ylabel='Local Y (mm)')
fig.suptitle('Same source, same position, same scale | baked texture preview (not a camera image)')
fig.savefig(root/'comparison.png',dpi=140)
plt.close(fig)
