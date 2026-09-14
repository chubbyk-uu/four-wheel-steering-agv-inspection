#!/usr/bin/env python3
"""Compare optional CUDA paint against unpainted tiles and CPU metric masks."""
import argparse,json,time
from pathlib import Path
import numpy as np
from probe_runtime_material import Sampler
from road_test_markings import paint


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--trial',type=Path,required=True)
    p.add_argument('--library',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    m=json.loads(a.trial.read_text());items=m['inspection_paint'];stride=m['core']+2*m['gutter'];plane=stride**2
    points=[(3.5,-2.4),(5.3,-2.4),(5.8,2.4),(8.85,-4),(10,-2.4),(12.2,-.6),(14.2,-2.4),(0,0),(18,3)]
    keys=list(dict.fromkeys((int((x-m['ox'])/(m['core']*m['texel'])),int((y-m['oy'])/(m['core']*m['texel']))) for x,y in points))
    base=Sampler(a.library,a.source);reference={};base_times=[]
    try:
        for key in keys:
            start=time.perf_counter();reference[key]=base.bake(*key).copy();base_times.append(time.perf_counter()-start)
    finally:base.close()
    sampler=Sampler(a.library,a.trial);records=[]
    try:
        for key in keys:
            start=time.perf_counter();actual=sampler.bake(*key).copy();elapsed=time.perf_counter()-start
            original=reference[key];gray=original[:plane].reshape(stride,stride)/255.
            rgb=np.repeat(gray[...,None],3,axis=2)
            xs=m['ox']+(key[0]*m['core']+np.arange(stride)-m['gutter']+.5)*m['texel']
            ys=m['oy']+(key[1]*m['core']+np.arange(stride)-m['gutter']+.5)*m['texel']
            mask=paint(rgb,xs,ys,items)
            expected=np.uint8(np.clip((rgb@[.2126,.7152,.0722])*255+.5,0,255))
            diff=np.abs(actual[:plane].reshape(stride,stride).astype(int)-expected.astype(int))
            assert diff.max()<=1,(key,diff.max())
            assert np.array_equal(actual[:plane].reshape(stride,stride)[~mask],original[:plane].reshape(stride,stride)[~mask])
            assert np.array_equal(actual[plane:].reshape(stride,stride,2)[~mask],original[plane:].reshape(stride,stride,2)[~mask])
            records.append(dict(tile=key,paint_pixels=int(mask.sum()),max_color_dn=int(diff.max()),bake_readback_s=elapsed))
    finally:sampler.close()
    result=dict(passed=True,scope='Actual CUDA tile bake vs independent CPU polygon masks; unpainted pixels including normals bit-identical. Not end-to-end throughput.',
        tiles=records,baseline_median_s=float(np.median(base_times)),paint_median_s=float(np.median([r['bake_readback_s'] for r in records])))
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
