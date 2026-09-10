#!/usr/bin/env python3
"""Independently inspect the generated full-road layout and archived tile bytes."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from PIL import Image
from budget_full_road import budget,ROOT
sys.path.insert(0,str(ROOT/'src/agv_linescan'))
from agv_linescan.shared_scene import validate


def check(manifest):
    path=Path(manifest);root=path.parent;start=time.monotonic()
    scene=validate(path);expected,request=budget();bounds=expected['road']
    for key in ('inspection_bounds_xy_m','drivable_bounds_xy_m','optical_valid_bounds_xy_m'):
        if any(abs(a-b)>1e-8 for a,b in zip(scene[key],bounds[key])):raise ValueError('unexpected full-road '+key)
    material=scene['ground_material'];stride=material['core_pixels']+2*material['gutter_pixels']
    if material['tiles_x']!=bounds['tiles_x'] or material['tiles_y']!=bounds['tiles_y'] or material['texel_m']!=bounds['texel_m']:
        raise ValueError('unexpected full-road tile layout')
    total=0
    for tile in material.get('tiles',[]):
        for channel,channels in (('color',1),('normal',2)):
            entry=tile[channel];data=(root/entry['file']).read_bytes();total+=len(data)
            if len(data)!=stride*stride*channels or hashlib.sha256(data).hexdigest()!=entry['sha256']:
                raise ValueError('archived material hash/size mismatch')
    runtime=material['schema']=='agv.ground_material.recipe.v1'
    if runtime:
        recipe=json.loads((root/material['recipe']['file']).read_text())
        total=sum((root/name).stat().st_size for name in recipe['payload_sha256'])
    elif total!=bounds['texture_bytes']:raise ValueError('texture storage differs from budget')
    rectangles=[];triangles=0;collision=0;display_pixels=0
    for asset in scene['assets']:
        projection=asset['display_uv_projection'];x,y=projection['origin_xy_m'];sx,sy=projection['span_xy_m']
        rectangles.append((x,x+sx,y,y+sy));triangles+=asset['triangles'];collision+=asset['collision_proxy']['triangles']
        if asset['collision_proxy']['triangles']!=2:raise ValueError('detailed groove mesh unexpectedly used for collisions')
        name='display_color_'+asset['name'].rsplit('_',1)[1]+'.png'
        with Image.open(root/name) as im:
            if im.width>2048 or im.height>2048:raise ValueError('oversized display partition')
            display_pixels+=im.width*im.height
    # Area equality alone could hide overlap + a hole. Check non-overlap too.
    for i,a in enumerate(rectangles):
        for b in rectangles[i+1:]:
            if min(a[1],b[1])-max(a[0],b[0])>1e-8 and min(a[3],b[3])-max(a[2],b[2])>1e-8:
                raise ValueError('overlapping display/geometry partitions')
    x0,x1,y0,y1=bounds['optical_valid_bounds_xy_m']
    if any(a<x0-1e-8 or b>x1+1e-8 or c<y0-1e-8 or d>y1+1e-8 for a,b,c,d in rectangles):
        raise ValueError('geometry partition outside declared domain')
    if abs(sum((b-a)*(d-c) for a,b,c,d in rectangles)-(x1-x0)*(y1-y0))>1e-6:
        raise ValueError('geometry/display partition gap')
    return dict(passed=True,scope='static asset integrity and geometry/texture contracts; not motion or rendered acceptance',
                tile_count=material['tiles_x']*material['tiles_y'],runtime_recipe=runtime,all_tile_hashes_verified=not runtime,source_payload_hashes_verified=runtime,texture_bytes=total,
                display_partitions=len(rectangles),display_pixels=display_pixels,visual_optix_triangles=triangles,collision_triangles=collision,
                bounds={k:scene[k] for k in ('inspection_bounds_xy_m','drivable_bounds_xy_m','optical_valid_bounds_xy_m')},
                disk_bytes=sum(p.stat().st_size for p in root.rglob('*') if p.is_file()),elapsed_seconds=time.monotonic()-start)


def main():
    p=argparse.ArgumentParser();p.add_argument('manifest');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=check(a.manifest);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))

if __name__=='__main__':main()
