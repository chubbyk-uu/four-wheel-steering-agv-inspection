#!/usr/bin/env python3
"""Independent global-coordinate float64 oracle for tiled diagnostic scan data."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from generate_runway_tiles import pattern

p=argparse.ArgumentParser()
p.add_argument('archive',type=Path)
a=p.parse_args()
c=yaml.safe_load((a.archive/'calibration.yaml').read_text())
if c.get('radiometry', {}).get('enabled', False):
    raise SystemExit('This oracle checks geometric texture sampling only; disable radiometry for this test. Use validate_radiometry.py for illumination checks.')
s=json.loads((a.archive/'summary.json').read_text())
t=json.loads(Path(s['terrain_manifest']).read_text())
width=c['width']; spacing=c['line_spacing_m']; texel=t['texel_m']
q=(np.arange(width)-(width-1)/2)/(width/2)
yray=c['nominal_width_m']/2*np.polynomial.polynomial.polyval(q,c['ray_polynomial'])
comparisons=0; maximum=0; differences=0; checked_blocks=[]
# Every block: samples on each side of the tile boundary and image boundary.
for b in range(s['blocks']):
    meta=json.loads((a.archive/f'block_{b}.json').read_text())
    x0,y0,_=meta['first']['camera_position_world_m'];direction=meta['first']['scan_direction']
    nominal=np.array([0,1,511,1023,2047,3071,4094,4095])
    tile_size=t['core_pixels']*texel
    lo,hi=sorted((x0,x0+direction*(meta['rows']-1)*spacing))
    boundaries=np.arange(np.floor(lo/tile_size),np.ceil(hi/tile_size)+1)*tile_size
    cross=np.rint((boundaries-x0)/(direction*spacing)).astype(int)
    rows=np.unique(np.concatenate([nominal]+[cross+i for i in range(-8,9)]))
    rows=rows[(rows>=0)&(rows<meta['rows'])]
    y=y0+yray
    gy=(y-t['origin_y_m'])/texel-.5
    iy=np.floor(gy).astype(int);fy=gy-iy
    values=np.zeros((len(rows),width))
    for phase in (-1/3,0,1/3):
        x=x0+direction*rows*spacing+direction*s['target_lines_per_second']*spacing*c['exposure_s']*phase
        gx=(x-t['origin_x_m'])/texel-.5
        ix=np.floor(gx).astype(int);fx=gx-ix
        # Global texel-center values, generated independently of tile selection,
        # local GPU anchors, disk reads and gutter indexing.
        xlo=t['origin_x_m']+(ix+.5)*texel; ylo=t['origin_y_m']+(iy+.5)*texel
        v00=pattern(xlo,ylo).T.astype(float);v10=pattern(xlo+texel,ylo).T.astype(float)
        v01=pattern(xlo,ylo+texel).T.astype(float);v11=pattern(xlo+texel,ylo+texel).T.astype(float)
        values+=((v00*(1-fx[:,None])+v10*fx[:,None])*(1-fy)+
                 (v01*(1-fx[:,None])+v11*fx[:,None])*fy)/3
    with Image.open(a.archive/f'block_{b}.pgm') as im:
        actual=np.asarray(im)[rows].astype(int)
    error=np.abs(actual-np.rint(values).astype(int))
    maximum=max(maximum,int(error.max()));differences+=int(np.count_nonzero(error));comparisons+=error.size
    assert error.max()<=1, (b,int(error.max()),int(rows[np.unravel_index(error.argmax(),error.shape)[0]]))
    checked_blocks.append(b)
result=dict(passed=True,blocks=len(checked_blocks),compared_pixels=comparisons,max_gray_error=maximum,
            differing_pixels=differences,scope='Every block boundary plus tile crossings, both travel directions and three lanes; float64 global bilinear oracle; 1 DN tolerance')
(a.archive/'pixel_validation.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
