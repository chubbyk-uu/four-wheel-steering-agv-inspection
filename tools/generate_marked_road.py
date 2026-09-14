#!/usr/bin/env python3
"""Create an independent 20 m arrow/box-junction paint trial from compact assets."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
from bake_concrete_road import LUT,srgb
from road_test_markings import polygons,paint


def generate(source,out):
    source=source.resolve();out=out.resolve()
    m=json.loads((source/'manifest.json').read_text())
    if m['length_m']!=20 or m['width_m']!=10 or m['ground_material']['schema']!='agv.ground_material.recipe.v1':
        raise ValueError('requires the compact 20 x 10 m recipe scene')
    items=polygons()
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
    m['inspection_paint']=dict(profile='arrows_box_v1',polygons=items,
        purpose='Independent geometry reference for longitudinal scale and shear evaluation; not a solver input',
        scope='Example road paint, not a regulatory traffic layout; zero added geometry')
    write(out/'manifest.json',json.dumps(m,indent=2)+'\n')
    # Old overview represents the source road; do not publish it as this trial.
    (out/'overview.png').unlink(missing_ok=True)
    write(out/'COLCON_IGNORE','')
    return out/'manifest.json'


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=Path('assets/road/runtime_fullwidth_20m_v1'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(generate(a.source,a.output))
