#!/usr/bin/env python3
"""Export an immutable source recipe and compare CUDA replay against archived pixels."""
import argparse,ctypes,hashlib,json,time
from pathlib import Path
import numpy as np
from PIL import Image
from bake_concrete_road import LUT
from fullwidth_road import Features


def export(scene_path,out):
    out.mkdir(parents=True,exist_ok=False);scene=json.loads(scene_path.read_text());m=scene['ground_material']
    q=json.loads((scene_path.parent/'quilt/layout.json').read_text());source=Path(__file__).resolve().parents[1]/'assets/road/source'
    color=np.asarray(Image.open(source/'Concrete047A_8K-PNG_Color.png').convert('RGB'))
    normal=np.asarray(Image.open(source/'Concrete047A_8K-PNG_NormalGL.png').convert('RGB'))
    features=Features(scene['length_m']);h,w=features.field['pigment'].shape
    files={}
    def write(name,data):
        path=out/(name+'.raw');np.ascontiguousarray(data).tofile(path);files[path.name]=hashlib.sha256(path.read_bytes()).hexdigest();return path.name
    info=dict(schema='agv.material.recipe.probe.v1',source_width=color.shape[1],source_height=color.shape[0],patch=q['patch_guide_pixels'],ratio=color.shape[1]/q['guide_side'],
        length=scene['length_m'],field_width=w,field_height=h,core=m['core_pixels'],gutter=m['gutter_pixels'],nx=m['tiles_x'],ny=m['tiles_y'],ox=m['origin_xy_m'][0],oy=m['origin_xy_m'][1],texel=m['texel_m'],
        qx=q['origin_xy_m'][0],qy=q['origin_xy_m'][1],gsd=q['guide_texel_m'],
        placements=[[p['left'],p['top'],p['sx'],p['sy']] for p in q['placements']],
        cracks=[[*p['center'],int(p['flip_x']),int(p['flip_y'])] for p in features.instances],
        color=write('color',color),normal=write('normal',normal),lut=write('lut',LUT),
        pigment=write('pigment',features.field['pigment']),strength=write('strength',features.field['normal_strength']))
    info['alpha']=write('alpha',np.stack([np.asarray(Image.open(scene_path.parent/'quilt'/p['alpha'])) for p in q['placements']]))
    info['payload_sha256']=files
    info['source_scene_sha256']=hashlib.sha256(scene_path.read_bytes()).hexdigest()
    (out/'recipe.json').write_text(json.dumps(info,indent=2)+'\n')
    return out/'recipe.json'


class Sampler:
    def __init__(self,library,recipe):
        self.lib=ctypes.CDLL(str(library));self.lib.recipe_create.argtypes=[ctypes.c_char_p];self.lib.recipe_create.restype=ctypes.c_void_p
        self.lib.recipe_error.restype=ctypes.c_char_p
        self.lib.recipe_bake.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_void_p];self.lib.recipe_bake.restype=ctypes.c_int
        self.lib.recipe_bytes.argtypes=[ctypes.c_void_p];self.lib.recipe_bytes.restype=ctypes.c_size_t
        self.lib.recipe_destroy.argtypes=[ctypes.c_void_p]
        start=time.perf_counter();self.handle=self.lib.recipe_create(str(recipe).encode())
        if not self.handle:raise RuntimeError(self.lib.recipe_error().decode())
        self.init_seconds=time.perf_counter()-start;self.bytes=self.lib.recipe_bytes(self.handle)
        m=json.loads(recipe.read_text());self.stride=m['core']+2*m['gutter'];self.buffer=np.empty(self.stride*self.stride*3,np.uint8)
    def bake(self,x,y):
        if self.lib.recipe_bake(self.handle,x,y,self.buffer.ctypes.data):raise RuntimeError(self.lib.recipe_error().decode())
        return self.buffer
    def close(self):self.lib.recipe_destroy(self.handle)


def main():
    p=argparse.ArgumentParser();p.add_argument('--scene',type=Path,default=Path('assets/road/baked_fullwidth_20m_v1/manifest.json'))
    p.add_argument('--output',type=Path,required=True);p.add_argument('--library',type=Path,default=Path('/tmp/libagv_recipe_probe.so'));p.add_argument('--reuse',action='store_true');p.add_argument('--all',action='store_true');p.add_argument('--repeats',type=int,default=4);a=p.parse_args()
    if a.repeats<1:p.error('--repeats must be positive')
    recipe=a.output/'recipe.json' if a.reuse else export(a.scene,a.output)
    scene=json.loads(a.scene.read_text());m=scene['ground_material'];sampler=Sampler(a.library,recipe)
    indices={(t['ix'],t['iy']):t for t in m['tiles']};tests=[]
    # Explicit plain substrate, yellow/white lines, slab seams, both crack flips,
    # arbitrary quilt overlap boundaries, and the outer background apron.
    points=[(0,0),(1,0),(2.5,-2.5),(2.5,2.5),(12.5,-2.5),(12.5,2.5),(5,0),(10,4.5),(17,1),(-.8,-5.8)]
    keys=list(dict.fromkeys((int(np.floor((x-m['origin_xy_m'][0])/(m['core_pixels']*m['texel_m']))),int(np.floor((y-m['origin_xy_m'][1])/(m['core_pixels']*m['texel_m'])))) for x,y in points))
    if a.all:keys=list(indices)
    try:
        for x,y in keys:
            elapsed=[];output=None
            for _ in range(a.repeats):
                start=time.perf_counter();output=sampler.bake(x,y).copy();elapsed.append(time.perf_counter()-start)
            entry=indices[x,y];plane=sampler.stride**2;channels={}
            for channel,slice_ in [('color',slice(0,plane)),('normal',slice(plane,None))]:
                expected=np.fromfile(a.scene.parent/entry[channel]['file'],np.uint8);actual=output[slice_];d=np.abs(actual.astype(np.int16)-expected.astype(np.int16))
                channels[channel]=dict(max_dn=int(d.max()),different_fraction=float(np.mean(d!=0)),mean_absolute_dn=float(d.mean()),above_one_dn=int(np.count_nonzero(d>1)))
                if d.max()>1:np.save(a.output/f'difference_{x}_{y}_{channel}.npy',d.reshape(sampler.stride,sampler.stride,-1))
            tests.append(dict(tile=[x,y],channels=channels,bake_readback_seconds=elapsed));print(json.dumps(tests[-1]),flush=True)
        result=dict(exact=all(v['max_dn']==0 for t in tests for v in t['channels'].values()),scope='Isolated recipe replay including GPU bake and host readback; no OptiX/cache integration yet',init_seconds=sampler.init_seconds,device_bytes=sampler.bytes,
            recipe_disk_bytes=sum(p.stat().st_size for p in a.output.glob('*.raw')),tiles=tests)
        (a.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
        if not result['exact']:raise RuntimeError('runtime material differs from archived reference pixels')
    finally:sampler.close()

if __name__=='__main__':main()
