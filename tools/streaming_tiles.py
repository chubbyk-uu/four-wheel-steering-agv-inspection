"""Bounded parallel tile baking; one immutable sampler, disjoint output files."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time
import numpy as np
from bake_concrete_road import sha


def bake_tiles(sample,out,nx,ny,ox,oy,core,gutter,texel,reuse=False,workers=1):
    if not 1<=workers<=4:raise ValueError('tile workers must be 1..4')
    out=Path(out);stride=core+2*gutter;start=time.monotonic()
    def bake(index):
        ix,iy=divmod(index,ny)
        files=[(kind,out/f'{kind}_{ix}_{iy}.raw',channels) for kind,channels in [('color',1),('normal',2)]]
        cached=reuse and all(path.is_file() and path.stat().st_size==stride*stride*channels for _,path,channels in files)
        if not cached:
            xs=ox+(ix*core+np.arange(-gutter,core+gutter)+.5)*texel
            ys=oy+(iy*core+np.arange(-gutter,core+gutter)+.5)*texel
            rgb,n=sample(xs,ys)
            mono=np.uint8(np.clip((rgb@np.array([.2126,.7152,.0722],np.float32))*255+.5,0,255))
            norm=np.uint8(np.clip((n[:,:,:2]*.5+.5)*255+.5,0,255))
            for (_,path,_),data in zip(files,(mono,norm)):
                temporary=path.with_suffix('.part');data.tofile(temporary);temporary.replace(path)
        return dict(ix=ix,iy=iy,**{kind:dict(file=path.name,sha256=sha(path)) for kind,path,_ in files})
    tiles=[]
    with ThreadPoolExecutor(max_workers=workers) as pool:
        # Only small descriptors/results are queued, never full pixel buffers.
        for entry in pool.map(bake,range(nx*ny)):
            tiles.append(entry)
            if len(tiles)%ny==0 or len(tiles)==nx*ny:
                progress=dict(stage='tiles',completed=len(tiles),total=nx*ny,seconds=time.monotonic()-start)
                tmp=out/'bake_progress.tmp';tmp.write_text(json.dumps(progress));tmp.replace(out/'bake_progress.json')
    worst=0
    for entry in tiles:
        ix,iy=entry['ix'],entry['iy']
        for kind,channels in [('color',1),('normal',2)]:
            shape=(stride,stride) if channels==1 else (stride,stride,channels)
            data=np.memmap(out/entry[kind]['file'],dtype='uint8',mode='r',shape=shape)
            for dx,dy in ((-1,0),(0,-1)):
                if ix+dx<0 or iy+dy<0:continue
                old=np.memmap(out/f'{kind}_{ix+dx}_{iy+dy}.raw',dtype='uint8',mode='r',shape=shape)
                left,right=(old[:,core:core+2*gutter],data[:,:2*gutter]) if dx else (old[core:core+2*gutter],data[:2*gutter])
                delta=int(np.max(np.abs(left.astype(np.int16)-right.astype(np.int16))));worst=max(worst,delta)
                if delta:raise ValueError(f'Nonidentical material gutter {kind} {ix} {iy}: {delta}')
    return tiles,worst
