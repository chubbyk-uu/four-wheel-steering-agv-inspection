#!/usr/bin/env python3
"""Layer a bounded common road heightfield under existing pigment and shallow defects."""
import argparse,json,os,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.shared_scene import validate,digest
from agv_linescan.obj_arrays import read_obj
from agv_linescan.heightfield import Heightfield
from probe_rough_road import height

def write_obj(path,v,f,uv=None):
    tri=v[f];n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);n/=np.linalg.norm(n,axis=1)[:,None]
    with path.open('w') as out:
        np.savetxt(out,v,fmt='v %.9f %.9f %.9f')
        if uv is not None:np.savetxt(out,uv,fmt='vt %.9f %.9f')
        np.savetxt(out,n,fmt='vn %.9f %.9f %.9f')
        for k,face in enumerate(f,1):out.write('f '+' '.join(f'{i+1}/{i+1 if uv is not None else ""}/{k}' for i in face)+'\n')

def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--peak-mm',type=float,default=3);p.add_argument('--step-m',type=float,default=.1,help='Reference heightfield spacing, not texture resolution');a=p.parse_args()
    if not np.isfinite(a.peak_mm) or not 0<a.peak_mm<=3:raise ValueError('peak must be in (0,3] mm')
    if not np.isfinite(a.step_m) or not .05<=a.step_m<=.5:raise ValueError('step must be finite and in [0.05,0.5] m')
    source=a.source.resolve();m=validate(source/'manifest.json');out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
    # Hardlink immutable payloads only; replacement files always unlink first.
    for path in source.iterdir():
        if path.is_file():os.link(path,out/path.name)
    bounds=m.get('optical_valid_bounds_xy_m',[-2,22,-7,7]);x=np.arange(np.floor(bounds[0]/a.step_m),np.ceil(bounds[1]/a.step_m)+1)*a.step_m;y=np.arange(np.floor(bounds[2]/a.step_m),np.ceil(bounds[3]/a.step_m)+1)*a.step_m
    # A production road is physically continuous through the acceleration and
    # turn-around buffers.  The old isolated suspension probe intentionally
    # tapered from a flat lead-in; reusing that profile here made x < 2 m a
    # dense but exactly coplanar collision mesh; an equal-shape proxy A/B tied
    # that region to the measured position-dependent DART contact cost.
    xx,yy=np.meshgrid(x,y);z=height(xx,yy,taper=False);scale=a.peak_mm*.001/np.max(abs(z));z*=scale
    field_path=out/'road_heightfield.json';field_path.write_text(json.dumps(dict(schema='agv.reference_heightfield.v1',x=x.tolist(),y=y.tolist(),z=z.tolist(),seed=20260915,scale=scale)))
    field=Heightfield(field_path);tree=ET.parse(source/m['world']);world=tree.getroot().find('world')
    ground_min,ground_max=float('inf'),-float('inf')
    for asset in m['assets']:
        old=asset['collision_proxy']
        if old['method']!='shallow_horizontal_rectangle_v1':raise ValueError('input needs checked flat shallow proxies')
        optical=out/asset['mesh'];reference=out/('reference_'+optical.name);os.link(source/asset['mesh'],reference)
        v,f,uv=read_obj(optical,with_uv=True);lo=v[:,:2].min(axis=0);hi=v[:,:2].max(axis=0);v[:,2]+=field.sample(v[:,:2])
        if asset.get('material')=='ground':
            ground_min=min(ground_min,float(v[:,2].min()))
            ground_max=max(ground_max,float(v[:,2].max()))
        optical.unlink();write_obj(optical,v,f,uv);asset['sha256']=digest(optical)
        proxy=out/old['mesh'];pv,pf=field.mesh(lo,hi);pv[:,2]+=old['plane_z_m'];proxy.unlink();write_obj(proxy,pv,pf)
        asset['collision_proxy']=dict(method='layered_heightfield_shallow_v1',mesh=proxy.name,sha256=digest(proxy),triangles=len(pf),reference_surface=dict(mesh=reference.name,sha256=digest(reference)),heightfield=dict(mesh=field_path.name,sha256=digest(field_path)),reference_plane_z_m=old['plane_z_m'],max_surface_deviation_m=old['max_surface_deviation_m'])
    # Rewrite only filesystem locations; all shared UVs/texture payload bytes remain unchanged.
    for element in world.iter():
        if element.text and str(source) in element.text:element.text=element.text.replace(str(source),str(out))
    world_path=out/m['world'];world_path.unlink();tree.write(world_path,encoding='unicode');m['world_sha256']=digest(world_path)
    # Streaming footprints must bound the displaced optical geometry, including
    # negative groove depth. Do not retain the flat road's old Z interval.
    if 'ground_material' in m:
        m['ground_material']['height_bounds_m']=[ground_min-1e-6,ground_max+1e-6]
    m['roughness_experiment']=dict(step_m=a.step_m,max_abs_height_m=float(np.max(abs(z))),rms_height_m=float(np.std(z)),min_height_m=float(z.min()),max_height_m=float(z.max()),source_manifest_sha256=digest(source/'manifest.json'),seed=20260915,profile='full-domain shared heightfield plus unchanged shallow defects',tapered_lead_in=False)
    manifest=out/'manifest.json';manifest.unlink();manifest.write_text(json.dumps(m,indent=2));validate(manifest)
    print(json.dumps(dict(passed=True,assets=len(m['assets']),collision_triangles=sum(e['collision_proxy']['triangles'] for e in m['assets']),roughness=m['roughness_experiment'])))
if __name__=='__main__':main()
