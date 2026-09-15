#!/usr/bin/env python3
"""Layer distributed arrow/box road paint onto a compact recipe scene."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
from bake_concrete_road import LUT,srgb
from road_test_markings import polygons,paint,seam_crossings


def generate(source,out,block_length_m=1.5):
    source=source.resolve();out=out.resolve()
    m=json.loads((source/'manifest.json').read_text())
    if m.get('ground_material',{}).get('schema')!='agv.ground_material.recipe.v1':
        raise ValueError('requires a compact recipe scene')
    length=float(m['length_m']);width=float(m['width_m'])
    items=polygons(length,width);crossings=seam_crossings(items,length,block_length_m)
    # Immutable payloads share disk blocks. Every modified file is unlinked first.
    shutil.copytree(source,out,copy_function=os.link)
    def write(path,data):
        path.unlink(missing_ok=True);path.write_text(data)
    def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
    for i,asset in enumerate(m['assets']):
        uv=asset['display_uv_projection'];x0,y0=uv['origin_xy_m'];dx,dy=uv['span_xy_m']
        cp=out/f'display_color_{i}.png';np_=out/f'display_normal_{i}.png'
        rgb=np.array(Image.open(cp));normal=np.array(Image.open(np_));h,w=rgb.shape[:2]
        xs=x0+(np.arange(w)+.5)*dx/w
        for start in range(0,h,128):
            stop=min(start+128,h);ys=y0+(np.arange(start,stop)+.5)*dy/h
            c=LUT[rgb[start:stop]];mask=paint(c,xs,ys,items)
            rgb[start:stop][mask]=srgb(c)[mask]
            n=normal[start:stop].astype(np.float32)/255*2-1;n[mask,:2]*=.25
            n/=np.linalg.norm(n,axis=2)[...,None]
            normal[start:stop][mask]=np.uint8(np.clip((n[mask]*.5+.5)*255+.5,0,255))
        for path,array in [(cp,rgb),(np_,normal)]:
            path.unlink();Image.fromarray(array).save(path);m['display_materials'][path.name]=digest(path)
    rp=out/m['ground_material']['recipe']['file'];recipe=json.loads(rp.read_text());recipe['inspection_paint']=items
    write(rp,json.dumps(recipe,indent=2)+'\n');m['ground_material']['recipe']['sha256']=digest(rp)
    world=out/m['world'];text=world.read_text().replace(str(source),str(out));write(world,text);m['world_sha256']=digest(world)
    m['inspection_paint']=dict(profile='distributed_arrows_box_v2',polygons=items,
        distribution=dict(period_m=20.,road_length_m=length,road_width_m=width,
                          capture_block_length_m=block_length_m,seam_crossings=crossings),
        purpose='Independent geometry reference for longitudinal scale and shear evaluation; not a solver input',
        scope='Example road paint, not a regulatory traffic layout; zero added geometry')
    write(out/'manifest.json',json.dumps(m,indent=2)+'\n')
    # Old overview represents the source road; do not publish it as this trial.
    (out/'overview.png').unlink(missing_ok=True)
    write(out/'COLCON_IGNORE','')
    return out/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=Path('assets/road/runtime_fullwidth_20m_v1'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--block-length-m',type=float,default=1.5)
    a=p.parse_args();print(generate(a.source,a.output,a.block_length_m))
