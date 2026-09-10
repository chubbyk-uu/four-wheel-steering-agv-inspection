#!/usr/bin/env python3
"""Compare sampled generated tiles against CPU quilting, markings and AI fields."""
import argparse,json,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from PIL import Image
from concrete_quilt import ConcreteQuilt
from bake_concrete_road import LUT
from fullwidth_road import Features
from road_markings import paint
from probe_runtime_material import Sampler


def main():
    p=argparse.ArgumentParser();p.add_argument('scene',type=Path);p.add_argument('--library',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    root=a.scene.parent;scene=json.loads(a.scene.read_text());m=scene['ground_material'];r=json.loads((root/m['recipe']['file']).read_text());qinfo=json.loads((root/'quilt/layout.json').read_text())
    # Replay the independently archived CPU recipe, without recomputing minimum cuts.
    q=ConcreteQuilt.__new__(ConcreteQuilt);q.source=np.fromfile(root/r['color'],np.uint8).reshape(r['source_height'],r['source_width'],3);q.lut=LUT
    normal=np.fromfile(root/r['normal'],np.uint8).reshape(q.source.shape);q.origin=qinfo['origin_xy_m'];q.gsd=qinfo['guide_texel_m'];q.patch=qinfo['patch_guide_pixels'];q.guide_side=qinfo['guide_side']
    q.placements=[{**v,'alpha':np.asarray(Image.open(root/'quilt'/v['alpha']))} for v in qinfo['placements']]
    q.guide=SimpleNamespace(shape=(max(v['top'] for v in q.placements)+q.patch,max(v['left'] for v in q.placements)+q.patch))
    features=Features(scene['length_m']);sampler=Sampler(a.library,root/m['recipe']['file']);linear=np.arange(256,dtype=np.float32)/255
    core=m['core_pixels'];g=m['gutter_pixels'];step=m['texel_m'];ox,oy=m['origin_xy_m'];nx=m['tiles_x'];ny=m['tiles_y'];stride=core+2*g
    points=[(-8,-6.5),(0,0),(2.5,2.5),(12.5,-2.5),(50,4.5),(52.5,-2.5),(92.5,2.5),(99.9,.15),(108,6.5)]
    keys={(min(nx-1,max(0,int((x-ox)/(core*step)))),min(ny-1,max(0,int((y-oy)/(core*step))))) for x,y in points}
    keys|={(0,0),(nx-1,ny-1),(nx-1,0),(0,ny-1)};results=[]
    try:
        for ix,iy in sorted(keys):
            start=time.monotonic();xs=ox+(ix*core+np.arange(stride)-g+.5)*step;ys=oy+(iy*core+np.arange(stride)-g+.5)*step
            actual=sampler.bake(ix,iy).copy();expected=np.empty_like(actual);plane=stride*stride
            # Bounded CPU strips, not a full expanded road.
            for row in range(0,stride,128):
                yy=ys[row:row+128];rgb=q.sample(xs,yy);before=rgb.copy();mark=paint(rgb,xs,yy,10);outside=(xs<0)|(xs>scene['length_m']);rgb[:,outside]=before[:,outside];mark[:,outside]=False
                n=q.sample(xs,yy,source=normal,lut=linear)*2-1;n/=np.linalg.norm(n,axis=2)[:,:,None];n[:,:,1]*=-1;n[mark,:2]*=.25;n/=np.linalg.norm(n,axis=2)[:,:,None];rgb,n=features.apply(rgb,n,xs,yy)
                mono=np.uint8(np.clip((rgb@np.array([.2126,.7152,.0722],np.float32))*255+.5,0,255));packed=np.uint8(np.clip((n[:,:,:2]*.5+.5)*255+.5,0,255))
                expected[row*stride:(row+len(yy))*stride]=mono.ravel();expected[plane+row*stride*2:plane+(row+len(yy))*stride*2]=packed.ravel()
            delta=np.abs(actual.astype(np.int16)-expected.astype(np.int16));entry=dict(tile=[ix,iy],max_dn=int(delta.max()),different_bytes=int(np.count_nonzero(delta)),seconds=time.monotonic()-start);results.append(entry)
        result=dict(passed=all(t['different_bytes']==0 for t in results),scope='Selected full-resolution CPU/GPU tile comparisons including gutters, source pixels, markings and flipped AI crack fields',tiles=results,device_bytes=sampler.bytes)
        a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
        if not result['passed']:raise RuntimeError('CPU/GPU material difference')
    finally:sampler.close()

if __name__=='__main__':main()
