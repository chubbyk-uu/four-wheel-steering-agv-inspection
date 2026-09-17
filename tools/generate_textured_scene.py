#!/usr/bin/env python3
"""Small shared road scene: same geometry/coordinates, bounded high-detail optical corridor."""
import argparse,json,sys,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import Delaunay
from PIL import Image
from concrete_quilt import ConcreteQuilt
from bake_concrete_road import LUT,prepare_crack,sha,srgb
from road_markings import paint,metadata
ROOT=Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.shared_scene import validate

def road_geometry(length=10):
    step=.00025;_,alpha,stats=prepare_crack(0,step)
    iy,ix=np.nonzero(binary_dilation(alpha>0,iterations=2))
    v=np.column_stack((1.4+(ix+.5)*step,-.45+(iy+.5)*step-.07,-.002*alpha[iy,ix]))
    xx,yy=np.meshgrid(np.linspace(0,length,round(length*10)+1),np.linspace(-5,5,101))
    flat=np.column_stack((xx.ravel(),yy.ravel(),np.zeros(xx.size)))
    cracks=[]
    for offset in np.arange(0,length,20):
        part=v.copy();part[:,0]+=offset;cracks.append(part)
        flat=flat[~((flat[:,0]>=1.4+offset)&(flat[:,0]<=3.6+offset)&(abs(flat[:,1]+.45)<=.071))]
    parts=cracks+[flat]
    for seam in np.arange(5,length,5):
        a,b=np.meshgrid(seam+np.array([-.004,-.003,.003,.004]),np.linspace(-5,5,501))
        parts.append(np.column_stack((a.ravel(),b.ravel(),np.zeros(a.size))))
    a,b=np.meshgrid(np.array([-.004,-.003,.003,.004]),np.linspace(0,length,round(length*50)+1))
    parts.append(np.column_stack((b.ravel(),a.ravel(),np.zeros(a.size))))
    vertices=np.vstack(parts);_,ids=np.unique(vertices[:,:2],axis=0,return_index=True);vertices=vertices[ids]
    distance=np.minimum(np.abs((vertices[:,0]+2.5)%5-2.5),abs(vertices[:,1]))
    distance[(vertices[:,0]<.004)|(vertices[:,0]>length-.004)]=abs(vertices[(vertices[:,0]<.004)|(vertices[:,0]>length-.004),1])
    vertices[:,2]=np.minimum(vertices[:,2],-.003*np.clip((.004-distance)/.001,0,1))
    faces=Delaunay(vertices[:,:2]).simplices
    a,b,c=vertices[faces[:,0],:2],vertices[faces[:,1],:2],vertices[faces[:,2],:2]
    area=(b[:,0]-a[:,0])*(c[:,1]-a[:,1])-(b[:,1]-a[:,1])*(c[:,0]-a[:,0])
    assert np.all(area>0) and abs(area.sum()/2-length*10)<1e-7
    return vertices,faces,stats

def write_world(out,path,collision_path):
    tree=ET.parse(ROOT/'src/agv_bringup/worlds/flat.sdf');world=tree.getroot().find('world')
    for m in list(world.findall('model')):world.remove(m)
    model=ET.SubElement(world,'model',name='terrain');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='road')
    for kind in ('visual','collision'):
        item=ET.SubElement(link,kind,name=kind);mesh=ET.SubElement(ET.SubElement(item,'geometry'),'mesh')
        ET.SubElement(mesh,'uri').text=str(path if kind=='visual' else collision_path);ET.SubElement(mesh,'scale').text='1 1 1'
        if kind=='visual':
            material=ET.SubElement(item,'material');ET.SubElement(material,'diffuse').text='1 1 1 1'
            metal=ET.SubElement(ET.SubElement(material,'pbr'),'metal')
            ET.SubElement(metal,'albedo_map').text=str(out/'display_color.png')
            ET.SubElement(metal,'normal_map',type='tangent').text=str(out/'display_normal.png')
            ET.SubElement(metal,'roughness').text='0.60';ET.SubElement(metal,'metalness').text='0'
    tree.write(out/'world.sdf',encoding='unicode')

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False)
    source=ROOT/'assets/road/source'
    color=np.array(Image.open(source/'Concrete047A_8K-PNG_Color.png').convert('RGB'))
    normal=np.array(Image.open(source/'Concrete047A_8K-PNG_NormalGL.png').convert('RGB'))
    q=ConcreteQuilt(color,LUT,10,10,.1,source_width=2.1);q.save(out/'quilt')
    lin=np.arange(256,dtype=np.float32)/255
    def sample(xs,ys):
        rgb=q.sample(xs,ys)
        lane=paint(rgb,xs,ys,10)
        n=q.sample(xs,ys,source=normal,lut=lin)*2-1;n/=np.linalg.norm(n,axis=2)[:,:,None]
        n[:,:,1]*=-1
        # Paint fills some substrate relief; this is a declared appearance approximation.
        n[lane,:2]*=.25;n/=np.linalg.norm(n,axis=2)[:,:,None]
        return rgb,n
    # 8 m x 1.6 m camera corridor; do not allocate the full 100 m road at sensor resolution.
    w,h=32000,6400;step=.00025
    raw=np.memmap(out/'color.raw',dtype='uint8',mode='w+',shape=(h,w))
    nr=np.memmap(out/'normal.raw',dtype='uint8',mode='w+',shape=(h,w,2))
    xs=(np.arange(w)+.5)*step
    for start in range(0,h,64):
        ys=-.8+(np.arange(start,min(h,start+64))+.5)*step
        rgb,n=sample(xs,ys)
        raw[start:start+len(ys)]=np.uint8(np.clip((rgb@np.array([.2126,.7152,.0722],np.float32))*255+.5,0,255))
        nr[start:start+len(ys)]=np.uint8(np.clip((n[:,:,:2]*.5+.5)*255+.5,0,255))
    raw.flush();nr.flush()
    # GUI map uses the same quilt, pigment and physical coordinates, at 4 mm/texel.
    # It is an overview, not a substitute for full-resolution sensor data.
    size=2500;display=np.empty((size,size,3),np.uint8);dn=display.copy()
    xs=(np.arange(size)+.5)*.004
    for start in range(0,size,128):
        ys=-5+(np.arange(start,min(size,start+128))+.5)*.004
        rgb,n=sample(xs,ys);display[start:start+len(ys)]=srgb(rgb)
        dn[start:start+len(ys)]=np.uint8(np.clip((n*.5+.5)*255+.5,0,255))
    Image.fromarray(display).save(out/'display_color.png');Image.fromarray(dn).save(out/'display_normal.png')
    # Overhead project preview only: actual camera image is captured separately in GZ.
    Image.fromarray(display[::-1]).save(out/'road_overview.png')
    v,f,stats=road_geometry()
    path=out/'terrain.obj'
    with path.open('w') as stream:
        np.savetxt(stream,v,fmt='v %.9f %.9f %.9f')
        np.savetxt(stream,np.column_stack((v[:,0]/10,1-(v[:,1]+5)/10)),fmt='vt %.9f %.9f')
        # Explicit per-face normals preserve the physical groove surface for both readers.
        n=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]]);n/=np.linalg.norm(n,axis=1)[:,None]
        np.savetxt(stream,n,fmt='vn %.9f %.9f %.9f')
        for i,face in enumerate(f):stream.write('f '+' '.join(f'{j+1}/{j+1}/{i+1}' for j in face)+'\n')
    from fullwidth_road import collision_mesh
    proxy=collision_mesh(out,'terrain',v,f)
    write_world(out,path,out/proxy['mesh'])
    mat=dict(schema='agv.ground_material.xy.v1',width=w,height=h,origin_xy_m=[0,-.8],span_xy_m=[8,1.6],roughness=.60,
             color=dict(file='color.raw',sha256=sha(out/'color.raw')),normal=dict(file='normal.raw',sha256=sha(out/'normal.raw')))
    manifest=dict(schema='agv.shared.static_scene.v1',units='m',frame='world',transform='identity_world_baked',
        profile='textured_road_probe',length_m=10,width_m=10,world='world.sdf',world_sha256=sha(out/'world.sdf'),
        assets=[dict(name='terrain',mesh=path.name,sha256=sha(path),triangles=len(f),material='ground',collision_proxy=proxy)],
        ground_material=mat,display_materials={n:sha(out/n) for n in ('display_color.png','display_normal.png')},
        lane_markings=metadata(10),slab_size_m=[5,5],joint_width_m=.008,joint_depth_m=.003,crack=stats,
        optical_valid_bounds_xy_m=[0,8,-.8,.8],display_texel_m=.004,display_uv='u=x/10; v=1-(y+5)/10; image rows increase with world y',
        display_uv_projection=dict(origin_xy_m=[0,-5],span_xy_m=[10,10],v_direction='decreasing_world_y'),
        source_scale_m=2.1,source_scale_basis='project setting; supplier dimensions unspecified',
        paint_normal_xy_strength=.25,material_scope='same geometry, quilt and marking coordinates; GUI overview and monochrome calibrated sensor use different renderers; not photometric parity',
        sources={n:sha(source/n) for n in ('Concrete047A_8K-PNG_Color.png','Concrete047A_8K-PNG_NormalGL.png')})
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');validate(out/'manifest.json')
    print(json.dumps(dict(triangles=len(f),texture_payload_MiB=w*h*3/2**20,markings=manifest['lane_markings'],optical_valid_bounds_xy_m=manifest['optical_valid_bounds_xy_m'])))
if __name__=='__main__':main()
