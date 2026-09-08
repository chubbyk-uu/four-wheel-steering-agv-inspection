#!/usr/bin/env python3
"""Create a separate collision-only variant; do not rebake/change optical assets."""
import argparse
import json
import shutil
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/agv_linescan'))
from agv_linescan.shared_scene import validate,digest
from agv_linescan.collision_proxy import mesh_arrays,shallow_rectangle


def simplify(source,output,max_depth=.002):
    source=Path(source).resolve();out=Path(output).resolve()
    m=validate(source)
    if len(m['assets'])!=1 or m['assets'][0]['name']!='terrain':
        raise ValueError('first proxy tool only supports a single terrain asset')
    asset=m['assets'][0]
    if 'collision_proxy' in asset: raise ValueError('source already has a collision proxy')
    v,f=mesh_arrays(source.parent/asset['mesh'])
    z=float(v[:,2].max());lo,hi,fraction=shallow_rectangle(v,f,z,max_depth)
    # Validate before creating output. Copies preserve the original archive independently.
    if out==source.parent or source.parent in out.parents:
        raise ValueError('output must be separate from source scene')
    shutil.copytree(source.parent,out)
    proxy=out/'terrain_collision.obj'
    corners=np.array([[lo[0],lo[1],z],[hi[0],lo[1],z],[hi[0],hi[1],z],[lo[0],hi[1],z]])
    with proxy.open('w') as stream:
        np.savetxt(stream,corners,fmt='v %.9f %.9f %.9f')
        stream.write('vn 0 0 1\nf 1//1 2//1 3//1\nf 1//1 3//1 4//1\n')
    asset['collision_proxy']=dict(method='shallow_horizontal_rectangle_v1',mesh=proxy.name,
        sha256=digest(proxy),plane_z_m=z,max_surface_deviation_m=max_depth,
        omitted_depression_area_fraction=fraction,triangles=2,
        purpose='wheel contact only; detailed optical geometry and shadows unchanged')
    tree=ET.parse(out/m['world'])
    for tag in ('uri','albedo_map','normal_map'):
        for element in tree.findall('.//'+tag):
            old=Path(element.text)
            if old.parent.resolve()!=source.parent: raise ValueError('nonlocal scene resource')
            element.text=str(out/old.name)
    tree.find('.//collision/geometry/mesh/uri').text=str(proxy)
    tree.write(out/m['world'],encoding='unicode')
    m['world_sha256']=digest(out/m['world'])
    m['collision_approximation']='Only sparse <= bound shallow depressions omitted from wheel contact; image geometry unchanged'
    target=out/'manifest.json';target.write_text(json.dumps(m,indent=2)+'\n')
    validate(target)
    print(json.dumps(dict(collision_triangles_before=len(f),collision_triangles_after=2,
        max_surface_deviation_m=max_depth,omitted_area_fraction=fraction,
        optical_mesh_unchanged=digest(out/asset['mesh'])==asset['sha256'])),flush=True)
    return target


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--scene',required=True);p.add_argument('--output',required=True)
    p.add_argument('--max-depth',type=float,default=.002)
    a=p.parse_args();simplify(a.scene,a.output,a.max_depth)
