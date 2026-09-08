#!/usr/bin/env python3
"""Matched Concrete047A channels on the same true groove fixture, staged GPU measurements."""
import argparse,json,subprocess,sys,time,copy
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw
import xacro
from concrete_quilt import ConcreteQuilt
from bake_concrete_road import LUT,sha
from validate_groove_cost import geometry,write_scene
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/agv_linescan'))
from agv_linescan.robot_scene import export,split_visual_links

def bake(out):
    out.mkdir();source=ROOT/'assets/road/source'
    color=np.array(Image.open(source/'Concrete047A_8K-PNG_Color.png').convert('RGB'))
    normal=np.array(Image.open(source/'Concrete047A_8K-PNG_NormalGL.png').convert('RGB'))
    rough=np.array(Image.open(source/'Concrete047A_8K-PNG_Roughness.png').convert('RGB'))
    q=ConcreteQuilt(color,LUT,5,5,0,source_width=2.1)
    q.save(out/'quilt')
    w,h=10400,6400;step=.00025;origin=[1.2,-.8]
    arrays={k:np.memmap(out/(k+'.raw'),dtype='uint8',mode='w+',shape=(h,w,c) if c==2 else (h,w)) for k,c in [('color',1),('normal',2),('roughness_map',1)]}
    xs=origin[0]+(np.arange(w)+.5)*step
    linear=np.arange(256,dtype=np.float32)/255
    rough_sum=0.;normal_max=0.;t=time.monotonic()
    for start in range(0,h,128):
        ys=origin[1]+(np.arange(start,min(start+128,h))+.5)*step
        rgb=q.sample(xs,ys);mono=rgb@np.array([.2126,.7152,.0722],np.float32)
        arrays['color'][start:start+len(ys)]=np.uint8(np.clip(mono*255+.5,0,255))
        n=q.sample(xs,ys,source=normal,lut=linear)*2-1
        if np.any(n[:,:,2]<=0):raise ValueError('XY encoding requires upper-hemisphere source normals')
        n/=np.linalg.norm(n,axis=2)[:,:,None]
        normal_max=max(normal_max,float(np.max(np.arccos(np.clip(n[:,:,2],-1,1))))*180/np.pi)
        # Source NormalGL is +V up; quilt image row / world Y increases down the source.
        n[:,:,1]*=-1
        arrays['normal'][start:start+len(ys)]=np.uint8(np.clip((n[:,:,:2]*.5+.5)*255+.5,0,255))
        r=q.sample(xs,ys,source=rough,lut=linear)[:,:,0]
        arrays['roughness_map'][start:start+len(ys)]=np.uint8(np.clip(r*255+.5,0,255));rough_sum+=float(r.astype(np.float64).sum())
    for a in arrays.values():a.flush()
    m=dict(schema='agv.ground_material.xy.v1',width=w,height=h,origin_xy_m=origin,span_xy_m=[w*step,h*step],roughness=rough_sum/(w*h))
    for k in arrays:m[k]=dict(file=k+'.raw',sha256=sha(out/(k+'.raw')))
    (out/'material.json').write_text(json.dumps(m,indent=2)+'\n')
    (out/'bake.json').write_text(json.dumps(dict(seconds=time.monotonic()-t,texel_m=step,roughness_mean=m['roughness'],normal_max_tilt_deg=normal_max,
         layout='same color-derived placements/alpha for all maps; normalize blended normals; flip NormalGL Y for world XY',
         source_sha256={k:sha(source/f'Concrete047A_8K-PNG_{k}.png') for k in ('Color','NormalGL','Roughness')}),indent=2)+'\n')
    return m

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--textures',required=True);a=p.parse_args()
    out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False);textures=Path(a.textures).resolve()
    if textures.exists():
        material=json.loads((textures/'material.json').read_text())
        for k in ('color','normal','roughness_map'):
            if sha(textures/material[k]['file'])!=material[k]['sha256']:raise ValueError('Baked map checksum mismatch')
    else:material=bake(textures)
    robot=split_visual_links(xacro.process_file(str(ROOT/'src/agv_description/urdf/agv.urdf.xacro')).toxml())
    rp=export(robot,out/'robot');r=json.loads(rp.read_text());r['groups']=[g for g in r['groups'] if g['name']==r['led_emitters']['link']];rp.write_text(json.dumps(r))
    vertices,faces,stats=geometry()
    cases={'color':('color',),'normal':('color','normal'),'roughness':('color','normal','roughness_map')}
    for name,channels in cases.items():
        folder=out/name;write_scene(folder,vertices,faces)
        mp=folder/'manifest.json';scene=json.loads(mp.read_text());m=copy.deepcopy(material)
        for k in ('color','normal','roughness_map'):
            if k not in channels:m.pop(k,None)
            else:
                # Local symlinks keep manifests relocatable within the fixture; report omits local paths.
                (folder/(k+'.raw')).symlink_to(textures/material[k]['file']);m[k]['file']=k+'.raw'
        scene['ground_material']=m;scene['assets'][0]['material']='ground';mp.write_text(json.dumps(scene,indent=2)+'\n')
    report=dict(scope='production OptixScene staged material + GPU readback; no GUI/physics/ROS/calibration/archive; short active throughput not acceptance',
       target_hz=11000,texel_m=.00025,texture_resolution=[material['width'],material['height']],roughness_constant=material['roughness'],
       crack=stats,triangles=len(faces),test_orders=[list(cases),list(reversed(cases))],fixtures={},bake=json.loads((textures/'bake.json').read_text()))
    for repeat,order in enumerate(report['test_orders']):
        for name in order:
            folder=out/name;dest=folder/f'images_{repeat}'
            cmd=[str(ROOT/'build/agv_linescan/benchmark_grooves'),str(folder/'manifest.json'),str(rp),str(ROOT/'build/agv_linescan/agv_optix_scan.ptx'),str(ROOT/'src/agv_description/config/linescan.yaml'),str(dest)]
            with (folder/f'run_{repeat}.log').open('w') as log:subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,check=True)
            report['fixtures'].setdefault(name,[]).append(json.loads((dest/'timing.json').read_text()))
    images={name:np.array(Image.open(out/name/'images_0/samples_16.pgm'),dtype=np.int16) for name in cases}
    metrics={}
    for previous,name in [('color','normal'),('normal','roughness')]:
        delta=np.abs(images[name]-images[previous]);metrics[name]=dict(mean_abs_dn=float(delta.mean()),p95_dn=float(np.percentile(delta,95)),max_dn=int(delta.max()),fraction_over_2dn=float((delta>2).mean()))
        if not np.any(delta):raise ValueError(name+' has no image effect')
    for name in cases:
        duplicate=np.array(Image.open(out/name/'images_1/samples_16.pgm'),dtype=np.int16)
        if not np.array_equal(images[name],duplicate):raise ValueError('Repeated capture not deterministic')
    report['image_metrics']=metrics;report['deterministic_images']=True
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    comparison=Image.new('RGB',(1536,540),'white')
    for i,name in enumerate(cases):
        crop=Image.fromarray(images[name].astype('uint8')).crop((1792,3840,2304,4352)).convert('RGB')
        comparison.paste(crop,(512*i,28));ImageDraw.Draw(comparison).text((512*i+8,8),name+' / 16 LED samples',fill=(0,0,0))
    comparison.save(out/'comparison.png')
    print(json.dumps(dict(metrics=metrics,timings={name:[[(m['samples'],round(m['active_lines_per_second']),round(m['allocated_device_bytes']/2**20,2)) for m in run['measurements']] for run in runs] for name,runs in report['fixtures'].items()})))
if __name__=='__main__':main()
