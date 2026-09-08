#!/usr/bin/env python3
"""Metric baking of sourced concrete and AI-created defect decals. No procedural crack paths."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time
import cv2
import numpy as np
from scipy.ndimage import distance_transform_edt
from PIL import Image
from concrete_quilt import ConcreteQuilt
from road_materials import MATERIALS, DEFAULT_MATERIAL
from road_markings import paint as paint_markings, metadata as marking_metadata

ROOT=Path(__file__).resolve().parents[1]
CRACKS=ROOT/'assets/road/generated/concrete_cracks_ai_v1.png'
SPALLS=ROOT/'assets/road/generated/concrete_spalls_ai_v1.png'
LUT=np.where(np.arange(256)/255<=.04045,np.arange(256)/255/12.92,((np.arange(256)/255+.055)/1.055)**2.4).astype(np.float32)

def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()

def srgb(linear):
    return np.uint8(np.clip(np.where(linear<=.0031308,12.92*linear,1.055*np.maximum(linear,0)**(1/2.4)-.055)*255+.5,0,255))

def largest(mask):
    n,labels,stats,_=cv2.connectedComponentsWithStats(np.uint8(mask),8)
    if n<2:raise ValueError('AI decal lacks foreground')
    return labels==(1+np.argmax(stats[1:,cv2.CC_STAT_AREA]))

def prepare_crack(index,texel):
    raw=np.array(Image.open(CRACKS).convert('RGBA'))
    raw=raw[index*256:(index+1)*256]
    foreground=largest(raw[:,:,3]>=128)
    ys,xs=np.nonzero(foreground);box=(slice(ys.min(),ys.max()+1),slice(xs.min(),xs.max()+1))
    raw=raw[box];foreground=foreground[box]
    # Centerlines and local width variation come exclusively from imagegen output.
    source_radius=distance_transform_edt(foreground)
    length=2.2+.2*index
    w=round(length/texel);h=round(.14/texel)
    stretched=cv2.resize(np.uint8(foreground)*255,(w,h),interpolation=cv2.INTER_NEAREST)
    skeleton=cv2.ximgproc.thinning(stretched)>0
    distance,nearest=distance_transform_edt(~skeleton,return_indices=True)
    measured_width=cv2.resize(np.float32(source_radius*2*.00016),(w,h))
    nominal_width=np.clip(measured_width,.0008,.002)
    radii=nominal_width[tuple(nearest)]/(2*texel)
    # Alpha coverage, preserving the image-derived trajectory while calibrating cross-section.
    alpha=np.clip(radii+.5-distance,0,1).astype(np.float32)
    # Retain imagegen's fine tonal variation, with a narrow core and antialiased concrete edge.
    color=cv2.resize(raw[:,:,:3],(w,h),interpolation=cv2.INTER_LINEAR)
    gray=LUT[color].mean(axis=2)
    edge=np.clip(distance/np.maximum(radii,1),0,1)
    reflectance=np.clip(.025+.11*gray+.08*edge,.02,.22)
    rgb=np.repeat(reflectance[:,:,None],3,axis=2).astype(np.float32)
    # Geometric support radius is the label definition; junction/tip raster widths are separate.
    stats=dict(source='imagegen',source_sha256=sha(CRACKS),variant=index,
               length_extent_m=w*texel,transverse_extent_m=h*texel,
               nominal_main_width_min_mm=float(nominal_width[skeleton].min()*1000),
               nominal_main_width_max_mm=float(nominal_width[skeleton].max()*1000),
               nominal_main_width_median_mm=float(np.median(nominal_width[skeleton])*1000),
               width_semantics='image-derived centerline, calibrated local diameter; alpha edge, tips and branch unions not independent width samples',
               procedural_crack_paths=False)
    return rgb,alpha,stats

def prepare_spall(index,texel):
    raw=np.array(Image.open(SPALLS).convert('RGBA'))
    edges=[0,700,1550,2048]
    raw=raw[:,edges[index]:edges[index+1]]
    mask=largest(raw[:,:,3]>=128);ys,xs=np.nonzero(mask)
    raw=raw[ys.min():ys.max()+1,xs.min():xs.max()+1]
    width=.025+.01*index;w=round(width/texel);h=max(4,round(w*raw.shape[0]/raw.shape[1]))
    # Monochrome defect reflectance suppresses generated RGB fringe; source original is retained.
    rgb=cv2.resize(raw[:,:,:3],(w,h),interpolation=cv2.INTER_AREA)
    lum=LUT[rgb].mean(axis=2)
    alpha=cv2.resize(raw[:,:,3].astype(np.float32)/255,(w,h),interpolation=cv2.INTER_AREA)
    return np.repeat(np.clip(lum,.04,.55)[:,:,None],3,axis=2),alpha,dict(width_m=width,geometry_depth_m=0)

class Baker:
    def __init__(self,length,width,texel,margin=.6,material=DEFAULT_MATERIAL):
        self.material=MATERIALS[material]
        self.source=ROOT/'assets/road/source'/self.material['file']
        self.length,self.width,self.texel=length,width,texel
        self.base=np.asarray(Image.open(self.source).convert('RGB'))
        self.quilt=ConcreteQuilt(self.base,LUT,length,width,margin,source_width=self.material['width_m'])
        self.cracks=[prepare_crack(i,texel) for i in range(3)]
        self.spalls=[prepare_spall(i,texel) for i in range(3)]
        self.instances=[]
        # Fixed layout seed controls placement only; it never generates a crack trajectory.
        rng=np.random.default_rng(20260907)
        for ix in range(math.ceil(length/5)):
            for iy in range(math.ceil(width/5)):
                center=np.array([ix*5+2.5,-width/2+iy*5+2.5])
                for kind,offset in [('crack',[-.4,-.35]),('spall',[.5,.7])]:
                    variant=(ix+iy)%3
                    angle=float(rng.uniform(-.65,.65))
                    self.instances.append(dict(kind=kind,variant=variant,center_m=(center+offset).tolist(),angle_rad=angle,
                                               slab=[ix,iy]))
        self.labels={'background':0,'joint':1,'lane_paint':2,'crack':3,'spall':4}

    def render(self,xs,ys,with_background=False):
        x,y=np.meshgrid(np.float32(xs),np.float32(ys))
        rgb=self.quilt.sample(xs,ys);labels=np.zeros(x.shape,np.uint8)
        background=rgb.copy() if with_background else None
        coverage=np.zeros(x.shape,bool) if with_background else None
        joint=(np.minimum(x%5,5-x%5)<=.004)|(np.minimum((y+self.width/2)%5,5-(y+self.width/2)%5)<=.004)
        rgb[joint]=.045;labels[joint]=1
        if with_background:coverage[joint]=True
        lane=paint_markings(rgb,xs,ys,self.width);labels[lane]=2
        if with_background:coverage[lane]=True
        for inst in self.instances:
            cx,cy=inst['center_m'];a=inst['angle_rad'];c,s=math.cos(a),math.sin(a)
            texture,alpha,_=(self.cracks if inst['kind']=='crack' else self.spalls)[inst['variant']]
            hh,ww=alpha.shape
            radius=.5*self.texel*math.hypot(ww,hh)
            if xs[-1]<cx-radius or xs[0]>cx+radius or ys[-1]<cy-radius or ys[0]>cy+radius:continue
            # Limit temporary maps to the tile/defect intersection.
            xi=np.flatnonzero((xs>=cx-radius)&(xs<=cx+radius));yi=np.flatnonzero((ys>=cy-radius)&(ys<=cy+radius))
            if not len(xi) or not len(yi):continue
            sl=np.s_[yi[0]:yi[-1]+1,xi[0]:xi[-1]+1]
            dx=x[sl]-cx;dy=y[sl]-cy
            mapx=np.float32((c*dx+s*dy)/self.texel+(ww-1)/2)
            mapy=np.float32((-s*dx+c*dy)/self.texel+(hh-1)/2)
            aa=cv2.remap(alpha,mapx,mapy,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
            cc=cv2.remap(texture,mapx,mapy,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
            if inst['kind']=='crack':
                # Crevice attenuates the local concrete reflectance; a neutral RGB overlay
                # would add an artificial blue/metallic rim to this warm-brown substrate.
                rgb[sl]=rgb[sl]*(1-aa[:,:,None]+cc*aa[:,:,None])
            else:
                rgb[sl]=rgb[sl]*(1-aa[:,:,None])+cc*aa[:,:,None]
            region=labels[sl];region[aa>=.5]=self.labels[inst['kind']]
            if with_background:coverage[sl]|=aa>0
        return (rgb,labels,background,coverage) if with_background else (rgb,labels)

def bake(out,length=10,width=10,texel=.00025,core=2048,material=DEFAULT_MATERIAL):
    if not 0<length<=200 or not 5<=width<=30 or length%5 or width%5 or texel!=.00025 or not 64<=core<=4096 or core%16:
        raise ValueError('5m slabs, <=200x30m, texel 0.25mm, core 64..4096 multiple of 16')
    out=Path(out);out.mkdir(parents=True,exist_ok=False);start=time.monotonic()
    b=Baker(length,width,texel,margin=core*texel+.01,material=material);gutter=2
    quilt_info=b.quilt.save(out/'quilt')
    nx,ny=math.ceil(length/(core*texel)),math.ceil(width/(core*texel))
    tiles=[];stride=16
    overview=np.zeros((ny*core//stride,nx*core//stride,3),np.uint8)
    overview_mono=np.zeros(overview.shape[:2],np.uint8)
    # Choose one crop now, align it to archived texel centres, and fill it FROM the
    # actual tiles as they are baked. No separate rendering path for the preview.
    inst=b.instances[0];_,alpha,_=b.cracks[inst['variant']]
    row=int(np.argmax(alpha[:,alpha.shape[1]//2]))
    local_y=(row-(alpha.shape[0]-1)/2)*texel
    c,s=math.cos(inst['angle_rad']),math.sin(inst['angle_rad'])
    cx=inst['center_m'][0]-s*local_y;cy=inst['center_m'][1]+c*local_y
    px0=int(round(cx/texel))-512;py0=int(round((cy+width/2)/texel))-512
    after=np.zeros((1024,1024,3),np.uint8);before=after.copy()
    crop_labels=np.zeros((1024,1024),np.uint8);crop_support=crop_labels.copy();crop_mono=crop_labels.copy()
    crop_filled=np.zeros((1024,1024),bool)
    for iy in range(ny):
        ys=-width/2+(iy*core+np.arange(-gutter,core+gutter)+.5)*texel
        for ix in range(nx):
            xs=(ix*core+np.arange(-gutter,core+gutter)+.5)*texel
            x0,x1=max(ix*core,px0),min((ix+1)*core,px0+1024)
            y0,y1=max(iy*core,py0),min((iy+1)*core,py0+1024)
            intersects=x0<x1 and y0<y1
            result=b.render(xs,ys,with_background=intersects)
            rgb,labels=result[:2]
            mono=np.uint8(np.clip(rgb@np.array([.2126,.7152,.0722],np.float32)*255+.5,0,255))
            name=f'tile_{ix}_{iy}';path=out/(name+'.pgm');label_path=out/(name+'_labels.png')
            with path.open('wb') as f:
                f.write(f'P5\n{mono.shape[1]} {mono.shape[0]}\n255\n'.encode());f.write(mono.tobytes())
            Image.fromarray(labels).save(label_path)
            tiles.append(dict(ix=ix,iy=iy,mono=path.name,sha256=sha(path),
                              labels=label_path.name,labels_sha256=sha(label_path)))
            # Mean in linear light from ALL native tile samples, not sparse samples.
            small=rgb[gutter:gutter+core,gutter:gutter+core].reshape(core//stride,stride,core//stride,stride,3).mean(axis=(1,3))
            overview[iy*core//stride:(iy+1)*core//stride,ix*core//stride:(ix+1)*core//stride]=srgb(small)
            mono_small=np.uint8(mono[gutter:gutter+core,gutter:gutter+core].reshape(core//stride,stride,core//stride,stride).mean(axis=(1,3))+.5)
            overview_mono[iy*core//stride:(iy+1)*core//stride,ix*core//stride:(ix+1)*core//stride]=mono_small
            if intersects:
                src=np.s_[y0-iy*core+gutter:y1-iy*core+gutter,x0-ix*core+gutter:x1-ix*core+gutter]
                dst=np.s_[y0-py0:y1-py0,x0-px0:x1-px0]
                after[dst]=srgb(rgb[src]);before[dst]=srgb(result[2][src])
                crop_labels[dst]=labels[src];crop_support[dst]=result[3][src]*255
                crop_mono[dst]=mono[src];crop_filled[dst]=True
    if not crop_filled.all():raise ValueError('preview crop not fully archived')
    unchanged=np.max(np.abs(before.astype(np.int16)-after.astype(np.int16))[crop_support==0],initial=0)
    if unchanged:raise ValueError('background changed outside defect support')
    overview=overview[:round(width/(texel*stride)),:round(length/(texel*stride))]
    Image.fromarray(overview[::-1]).save(out/'overview.png')
    Image.fromarray(overview_mono[:overview.shape[0],:overview.shape[1]][::-1]).save(out/'overview_mono.png')
    for name,pixels in [('crack_256mm_closeup',after),('crack_256mm_before',before),
                        ('crack_256mm_labels',crop_labels),('crack_256mm_support',crop_support),
                        ('crack_256mm_mono',crop_mono)]:
        Image.fromarray(pixels[::-1]).save(out/(name+'.png'))
    guide=b.quilt.guide;shift=b.quilt.guide_side
    period_mae=float(np.mean(np.abs(guide[:,shift:]-guide[:,:-shift])))
    report=dict(schema='agv.road.baked.v1',length_m=length,width_m=width,origin_xy_m=[0,-width/2],
                texel_m=texel,core_pixels=core,gutter_pixels=gutter,tiles_x=nx,tiles_y=ny,
                layout='row_y_column_x',encoding='linear_reflectance_mono8',slab_size_m=[5,5],joint_width_m=.008,
                lane_markings=marking_metadata(width),
                cracks=[x[2] for x in b.cracks],instances=b.instances,labels=b.labels,tiles=tiles,
                sources={str(p.relative_to(ROOT)):sha(p) for p in [b.source,CRACKS,SPALLS]},
                material_id=material,material_name=b.material['name'],
                source_width_m=b.material['width_m'],source_url=b.material['url'],source_license='CC0',
                source_scale_basis=b.material['scale_basis'],
                background_quilting=quilt_info,guide_source_width_shift_mean_abs_difference=period_mae,
                previews=dict(crop_pixels_xy=[px0,py0,1024,1024],
                              crop_bounds_xy_m=[px0*texel,(px0+1024)*texel,-width/2+py0*texel,-width/2+(py0+1024)*texel],
                              crop_span_m=[.256,.256],overview_texel_m=texel*stride,
                              provenance='extracted/area-averaged from actual native tile render buffers',
                              unchanged_background_max_delta_dn=int(unchanged),
                              mono_crop='crack_256mm_mono.png',before='crack_256mm_before.png',
                              after='crack_256mm_closeup.png',support='crack_256mm_support.png',
                              overview='overview.png',overview_mono='overview_mono.png'),
                crack_path_source='imagegen raster, no random/procedural path generation',
                geometry='flat: cracks/spalls are appearance labels, not measured geometry',
                limits=['quilt reuses source patches; no guarantee of globally unique texture',
                        'width range specifies centerline diameter; junction unions and resampling need separate raster validation',
                        'not connected to OptiX texture sampling yet; GUI overview is coarse'],
                elapsed_s=time.monotonic()-start)
    (out/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('schema','length_m','width_m','tiles_x','tiles_y','elapsed_s')}))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',required=True)
    p.add_argument('--length',type=float,default=10);p.add_argument('--width',type=float,default=10)
    p.add_argument('--texel',type=float,default=.00025);p.add_argument('--tile-pixels',type=int,default=2048)
    p.add_argument('--material',choices=list(MATERIALS),default=DEFAULT_MATERIAL)
    a=p.parse_args();bake(a.output,a.length,a.width,a.texel,a.tile_pixels,a.material)
