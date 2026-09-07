# Copyright 2026 chubbyk-uu
# SPDX-License-Identifier: Apache-2.0
"""Low-resolution minimum-cut layout replayed at native texture scale.

Minimum-cut/overlap selection adapted from climbot_sim/tools/bake_wall_texture.py
(Apache-2.0). No dependency on the sibling checkout. This quilts the BACKGROUND,
never generates cracks. Full-resolution canvas is not allocated.
"""
import math
import json
from pathlib import Path
import cv2
import numpy as np

def minimum_cut(error):
    rows,columns=error.shape
    total=error.astype(np.float64).copy();back=np.zeros((rows,columns),np.int8)
    for row in range(1,rows):
        prev=total[row-1]
        options=np.vstack((np.r_[np.inf,prev[:-1]],prev,np.r_[prev[1:],np.inf]))
        choice=np.argmin(options,axis=0)
        back[row]=choice-1;total[row]+=options[choice,np.arange(columns)]
    seam=np.empty(rows,np.int32);seam[-1]=np.argmin(total[-1])
    for row in range(rows-2,-1,-1):seam[row]=seam[row+1]+back[row+1,seam[row+1]]
    return seam

class ConcreteQuilt:
    def __init__(self,source,lut,length,width,margin,source_width=2.1,seed=20260908,
                 guide_side=512,patch=320,overlap=96,candidates=32,feather=3):
        if not 0<overlap<patch<=guide_side or source_width<=0:
            raise ValueError('invalid quilting geometry')
        self.source=source;self.lut=lut;self.source_width=source_width
        self.guide_side=guide_side;self.patch=patch;self.overlap=overlap;self.seed=seed
        self.origin=(-margin,-width/2-margin)
        self.gsd=source_width/guide_side
        guide=cv2.resize(source,(guide_side,guide_side),interpolation=cv2.INTER_AREA)
        gray=lut[guide]@np.array([.2126,.7152,.0722],np.float32)
        self.guide_source=gray
        step=patch-overlap
        rows=max(1,math.ceil(((width+2*margin)/self.gsd-overlap)/step))
        cols=max(1,math.ceil(((length+2*margin)/self.gsd-overlap)/step))
        canvas=np.zeros((rows*step+overlap,cols*step+overlap),np.float32)
        rng=np.random.default_rng(seed);self.placements=[]
        for row in range(rows):
            for col in range(cols):
                top,left=row*step,col*step
                window=canvas[top:top+patch,left:left+patch]
                occupied=np.zeros((patch,patch),bool)
                if col:occupied[:,:overlap]=True
                if row:occupied[:overlap,:]=True
                offsets=rng.integers(0,guide_side-patch+1,size=(candidates,2))
                costs=[]
                for sy,sx in offsets:
                    delta=(gray[sy:sy+patch,sx:sx+patch]-window)[occupied]
                    costs.append(float(delta@delta))
                costs=np.array(costs);best=costs.min()
                pick=int(rng.choice(np.flatnonzero(costs<=best*1.1+1e-12)))
                sy,sx=map(int,offsets[pick]);block=gray[sy:sy+patch,sx:sx+patch]
                mask=np.ones((patch,patch),np.uint8)
                guard=min(overlap//4,math.ceil(feather*4)+1)
                if col:
                    error=(block[:,:overlap]-window[:,:overlap])**2
                    guard=min(overlap//4,math.ceil(feather*4)+1)
                    error[:,:guard]=np.inf;error[:,-guard:]=np.inf
                    cut=minimum_cut(error)
                    mask[:,:overlap]&=np.arange(overlap)[None,:]>=cut[:,None]
                if row:
                    error=((block[:overlap,:]-window[:overlap,:])**2).T
                    error[:,:guard]=np.inf;error[:,-guard:]=np.inf
                    cut=minimum_cut(error)
                    mask[:overlap,:]&=np.arange(overlap)[:,None]>=cut[None,:]
                alpha=cv2.GaussianBlur(mask.astype(np.float32),(0,0),feather,borderType=cv2.BORDER_REPLICATE) if feather else mask.astype(np.float32)
                alpha[~occupied]=1
                # Store quantized alpha, and use that exact alpha during layout/replay.
                alpha=np.uint8(np.clip(alpha*255+.5,0,255))
                af=alpha.astype(np.float32)/255
                window[:]=window*(1-af)+block*af
                self.placements.append(dict(top=top,left=left,sy=sy,sx=sx,alpha=alpha))
        self.guide=canvas
        self.feather=feather;self.candidates=candidates

    def sample(self,xs,ys):
        xs=np.asarray(xs,dtype=np.float64);ys=np.asarray(ys,dtype=np.float64)
        gx=(xs-self.origin[0])/self.gsd
        gy=(ys-self.origin[1])/self.gsd
        if min(gx.min(),gy.min())<0 or gx.max()>=self.guide.shape[1] or gy.max()>=self.guide.shape[0]:
            raise ValueError('sampling outside quilt domain')
        rgb=np.zeros((len(ys),len(xs),3),np.float32)
        filled=np.zeros((len(ys),len(xs)),np.float32)
        ratio=self.source.shape[1]/self.guide_side
        for p in self.placements:
            xi=np.flatnonzero((gx>=p['left'])&(gx<p['left']+self.patch))
            yi=np.flatnonzero((gy>=p['top'])&(gy<p['top']+self.patch))
            if not len(xi) or not len(yi):continue
            sl=np.s_[yi[0]:yi[-1]+1,xi[0]:xi[-1]+1]
            # Pixel-centre convention is identical for guide and source.
            px,py=np.meshgrid(gx[xi]-p['left'],gy[yi]-p['top'])
            amx=np.float32(px-.5);amy=np.float32(py-.5)
            alpha=cv2.remap(p['alpha'],amx,amy,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE).astype(np.float32)/255
            # Never wrap a source border: each candidate lies wholly inside the source.
            u=np.float32((px+p['sx'])*ratio-.5)
            v=np.float32((py+p['sy'])*ratio-.5)
            color=cv2.remap(self.source,u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)
            rgb[sl]=rgb[sl]*(1-alpha[:,:,None])+self.lut[color]*alpha[:,:,None]
            filled[sl]=filled[sl]*(1-alpha)+alpha
        if np.any(filled<.99999):raise ValueError('quilt contains unfilled samples')
        return rgb

    def save(self,directory):
        directory=Path(directory);directory.mkdir()
        placements=[]
        for i,p in enumerate(self.placements):
            name=f'alpha_{i:04d}.png';cv2.imwrite(str(directory/name),p['alpha'])
            placements.append({**{k:v for k,v in p.items() if k!='alpha'},'alpha':name})
        info=dict(method='minimum_error_patch_quilting',seed=self.seed,source_width_m=self.source_width,
                  guide_side=self.guide_side,guide_texel_m=self.gsd,patch_guide_pixels=self.patch,
                  overlap_guide_pixels=self.overlap,feather_sigma_guide_pixels=self.feather,
                  candidates=self.candidates,origin_xy_m=self.origin,placements=placements,
                  source_rotations=False,source_scaling='native physical 2.1m; translations only',
                  note='no fixed 2.1m wrapping; local content reuse remains possible')
        (directory/'layout.json').write_text(json.dumps(info,indent=2)+'\n')
        return {k:v for k,v in info.items() if k!='placements'}|{'patch_count':len(placements),'layout':'quilt/layout.json'}
