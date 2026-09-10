#!/usr/bin/env python3
"""100 m road, real color/normal tiles in a bounded-width optical corridor; coarse shared GZ assets."""
import argparse,json,sys,math,time
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
from concrete_quilt import ConcreteQuilt
from bake_concrete_road import LUT,sha,srgb
from road_markings import paint,metadata
from road_layout import layout,display_regions
from streaming_tiles import bake_tiles
from generate_textured_scene import road_geometry
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/agv_linescan'))
from agv_linescan.shared_scene import validate

def clip_partition(vertices, faces, lo, hi):
 """Split crossing triangles at display boundaries; preserve their original planes."""
 triangles=vertices[faces];mn=triangles[:,:,0].min(axis=1);mx=triangles[:,:,0].max(axis=1)
 interior=triangles[(mn>=lo)&(mx<=hi)]
 crossing=triangles[(mx>lo)&(mn<hi)&~((mn>=lo)&(mx<=hi))]
 extra=[]
 for tri in crossing:
  poly=list(tri)
  for boundary,sign in ((lo,1),(hi,-1)):
   result=[]
   for a,b in zip(poly,poly[1:]+poly[:1]):
    inside_a=sign*(a[0]-boundary)>=0;inside_b=sign*(b[0]-boundary)>=0
    if inside_a:result.append(a)
    if inside_a!=inside_b:
     p=a+(b-a)*((boundary-a[0])/(b[0]-a[0]));p[0]=boundary;result.append(p)
   poly=result
  for i in range(1,len(poly)-1):
   tri=np.array([poly[0],poly[i],poly[i+1]])
   if np.linalg.norm(np.cross(tri[1]-tri[0],tri[2]-tri[0]))>1e-14:extra.append(tri)
 all_tri=np.concatenate((interior,np.array(extra).reshape(-1,3,3)))
 vv,indices=np.unique(all_tri.reshape(-1,3),axis=0,return_inverse=True)
 return vv,indices.reshape(-1,3)

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--length',type=int,default=100);p.add_argument('--full-width',action='store_true',help='Whole 10m road plus optical margins, branched cracks and bounded collision proxies');p.add_argument('--runtime-material',action='store_true',help='Store immutable source recipe, generate high-resolution tiles on GPU at runtime');p.add_argument('--reuse-tiles',action='store_true',help='Reassemble local interrupted bake; recheck existing tile sizes and gutters');p.add_argument('--end-buffer',type=float,default=0.);p.add_argument('--side-buffer',type=float,default=0.);p.add_argument('--workers',type=int,default=1,choices=range(1,5));a=p.parse_args()
 if a.runtime_material and (not a.full_width or a.reuse_tiles):raise ValueError('--runtime-material requires --full-width and a new output directory')
 if a.length<20 or a.length>200 or a.length%10:raise ValueError('length must be 20..200 m, multiple of 10')
 if (a.end_buffer or a.side_buffer) and not a.full_width:raise ValueError('physical buffers require --full-width')
 bounds=layout(a.length,a.end_buffer,a.side_buffer)
 out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=a.reuse_tiles);start=time.monotonic()
 source=ROOT/'assets/road/source';color=np.array(Image.open(source/'Concrete047A_8K-PNG_Color.png').convert('RGB'));normal=np.array(Image.open(source/'Concrete047A_8K-PNG_NormalGL.png').convert('RGB'))
 buffered=a.full_width and (a.end_buffer or a.side_buffer)
 if buffered:
  bx0,bx1,by0,by1=bounds['optical_valid_bounds_xy_m']
  q=ConcreteQuilt(color,LUT,bx1-bx0,by1-by0,.01,source_width=2.1,origin_x=bx0)
 else:q=ConcreteQuilt(color,LUT,a.length,10,1.6 if a.full_width else .6,source_width=2.1)
 from fullwidth_road import Features,collision_mesh
 features=Features(a.length) if a.full_width else None
 core,gutter,texel=2048,2,.00025;stride=core+2*gutter
 nx=bounds['tiles_x'] if a.full_width else math.ceil(a.length/(core*texel));ny=bounds['tiles_y'] if a.full_width else 3
 ox=bounds['optical_valid_bounds_xy_m'][0] if a.full_width else 0;oy=-ny*core*texel/2
 xmax=ox+nx*core*texel
 recipe=dict(length_m=a.length,source_scale_m=2.1,texel_m=.00025,core=2048,gutter=2,tiles_y=ny,full_width=a.full_width,origin_xy_m=[ox,oy],
  end_buffer_m=a.end_buffer,side_buffer_m=a.side_buffer,
  tile_code_sha256=sha(ROOT/'tools/streaming_tiles.py'),layout_code_sha256=sha(ROOT/'tools/road_layout.py'),
  color_sha256=sha(source/'Concrete047A_8K-PNG_Color.png'),normal_sha256=sha(source/'Concrete047A_8K-PNG_NormalGL.png'),
  baker_sha256=sha(Path(__file__)),quilt_code_sha256=sha(ROOT/'tools/concrete_quilt.py'),marking_code_sha256=sha(ROOT/'tools/road_markings.py'),
  features_code_sha256=sha(ROOT/'tools/fullwidth_road.py'),crack_code_sha256=sha(ROOT/'tools/branch_crack_fixture.py'),
  crack_source_sha256=sha(ROOT/'assets/road/generated/concrete_crack_branch_ai_v2.png'))
 if a.reuse_tiles:
  if json.loads((out/'tile_recipe.json').read_text())!=recipe:raise ValueError('Cannot reuse tiles from a different bake recipe; use a new output directory')
 else:
  q.save(out/'quilt');(out/'tile_recipe.json').write_text(json.dumps(recipe,indent=2)+'\n')
 lin=np.arange(256,dtype=np.float32)/255
 def sample(xs,ys):
  rgb=q.sample(xs,ys);before=rgb.copy() if a.full_width else None;mark=paint(rgb,xs,ys,10)
  if a.full_width:
   outside=(xs<0)|(xs>a.length);rgb[:,outside]=before[:,outside];mark[:,outside]=False
  n=q.sample(xs,ys,source=normal,lut=lin)*2-1;n/=np.linalg.norm(n,axis=2)[:,:,None];n[:,:,1]*=-1;n[mark,:2]*=.25;n/=np.linalg.norm(n,axis=2)[:,:,None]
  return features.apply(rgb,n,xs,ys) if features else (rgb,n)
 tiles,worst=([],None) if a.runtime_material else bake_tiles(sample,out,nx,ny,ox,oy,core,gutter,texel,a.reuse_tiles,a.workers)
 (out/'bake_progress.json').write_text(json.dumps(dict(stage='geometry_and_display',tiles=len(tiles))))
 if not features:v,faces,stats=road_geometry(a.length)
 else:stats=dict(template=features.field['stats'],instances=features.instances)
 tree=ET.parse(ROOT/'src/agv_bringup/worlds/flat.sdf');world=tree.getroot().find('world')
 for model in list(world.findall('model')):world.remove(model)
 assets=[];display_files={};total_triangles=0
 if features:
  regions=display_regions(a.length,[ox,xmax,oy,-oy]) if buffered else [(lo,hi,by,ey) for lo,hi in zip(([ox]+list(range(5,a.length,5))), (list(range(5,a.length,5))+[xmax])) for by,ey in ((oy,0),(0,-oy))]
 else:regions=[(i*10,i*10+10,-5,5) for i in range(a.length//10)]
 for i,(x0,x1,y0,y1) in enumerate(regions):
  vv,ff=features.geometry(x0,x1,y0,y1) if features else clip_partition(v,faces,x0,x1)
  total_triangles+=len(ff)
  if vv[:,0].min()<x0-1e-6 or vv[:,0].max()>x1+1e-6:raise ValueError('mesh spans a display-material partition')
  mesh=out/f'terrain_{i}.obj'
  with mesh.open('w') as f:
   # PNG row zero samples y0; OBJ V=1 must address that top image row.
   np.savetxt(f,vv,fmt='v %.9f %.9f %.9f');np.savetxt(f,np.column_stack(((vv[:,0]-x0)/(x1-x0),1-(vv[:,1]-y0)/(y1-y0))),fmt='vt %.9f %.9f')
   n=np.cross(vv[ff[:,1]]-vv[ff[:,0]],vv[ff[:,2]]-vv[ff[:,0]]);n/=np.linalg.norm(n,axis=1)[:,None];np.savetxt(f,n,fmt='vn %.9f %.9f %.9f')
   for j,tri in enumerate(ff):f.write('f '+' '.join(f'{k+1}/{k+1}/{j+1}' for k in tri)+'\n')
  dw=round((x1-x0)/.004);dh=round((y1-y0)/.004);display=np.empty((dh,dw,3),np.uint8);dn=display.copy();xs=x0+(np.arange(dw)+.5)*(x1-x0)/dw
  for row in range(0,dh,128):
   ys=y0+(np.arange(row,min(row+128,dh))+.5)*(y1-y0)/dh;rgb,n=sample(xs,ys);display[row:row+len(ys)]=srgb(rgb);dn[row:row+len(ys)]=np.uint8(np.clip((n*.5+.5)*255+.5,0,255))
  for kind,data in [('color',display),('normal',dn)]:
   name=f'display_{kind}_{i}.png';Image.fromarray(data).save(out/name);display_files[name]=sha(out/name)
  proxy=collision_mesh(out,f'terrain_{i}',vv,ff) if features else None
  model=ET.SubElement(world,'model',name=f'terrain_{i}');ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='road')
  for kind in ('visual','collision'):
   item=ET.SubElement(link,kind,name=kind);geo=ET.SubElement(ET.SubElement(item,'geometry'),'mesh');ET.SubElement(geo,'uri').text=str(out/proxy['mesh'] if kind=='collision' and proxy else mesh);ET.SubElement(geo,'scale').text='1 1 1'
   if kind=='visual':
    mat=ET.SubElement(item,'material');ET.SubElement(mat,'diffuse').text='1 1 1 1';metal=ET.SubElement(ET.SubElement(mat,'pbr'),'metal')
    for tag,value in [('albedo_map',str(out/f'display_color_{i}.png')),('normal_map',str(out/f'display_normal_{i}.png')),('roughness','.60'),('metalness','0')]:ET.SubElement(metal,tag).text=value
  assets.append(dict(name=f'terrain_{i}',mesh=mesh.name,sha256=sha(mesh),triangles=len(ff),material='ground',display_uv_projection=dict(origin_xy_m=[x0,y0],span_xy_m=[x1-x0,y1-y0],v_direction='decreasing_world_y'),**({'collision_proxy':proxy} if proxy else {})))
 rgb,_=sample((np.arange(a.length*20)+.5)*.05,-5+(np.arange(200)+.5)*.05)
 Image.fromarray(srgb(rgb)[::-1]).save(out/'overview.png');tree.write(out/'world.sdf',encoding='unicode')
 material=dict(schema='agv.ground_material.tiles.v1',tiles_x=nx,tiles_y=ny,core_pixels=core,gutter_pixels=gutter,texel_m=texel,origin_xy_m=[ox,oy],height_bounds_m=[-.003001,.000001],roughness=.60,cache_slots=32,prefetch_ahead_m=1.5,prefetch_behind_m=.5,required_wait_timeout_s=.05,tiles=tiles)
 manifest=dict(schema='agv.shared.static_scene.v1',units='m',frame='world',transform='identity_world_baked',profile='streaming_fullwidth_road' if features else 'streaming_road_corridor',length_m=a.length,width_m=10,assets=assets,world='world.sdf',world_sha256=sha(out/'world.sdf'),display_materials=display_files,ground_material=material,lane_markings=metadata(10),optical_valid_bounds_xy_m=[ox,xmax,oy,-oy],inspection_bounds_xy_m=[0,a.length,-5,5],drivable_bounds_xy_m=bounds['drivable_bounds_xy_m'],non_acquisition_buffers_m=dict(end=a.end_buffer,side=a.side_buffer),slab_size_m=[5,5],joint_width_m=.008,joint_depth_m=.003,crack=stats,display_texel_m=.004,source_scale_m=2.1,source_scale_basis='project mapping; not supplier measurement',max_gutter_delta=worst,elapsed_seconds=time.monotonic()-start)
 (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 if a.runtime_material:
  from probe_runtime_material import export
  export(out/'manifest.json',out,existing_directory=True)
  material.pop('tiles');material['schema']='agv.ground_material.recipe.v1'
  material['recipe']=dict(file='recipe.json',sha256=sha(out/'recipe.json'))
  manifest['profile']='runtime_recipe_fullwidth_road'
  (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 validate(out/'manifest.json')
 (out/'bake_progress.json').write_text(json.dumps(dict(stage='complete',tiles=len(tiles),seconds=time.monotonic()-start)))
 print(json.dumps(dict(length_m=a.length,tiles=len(tiles),triangles=total_triangles,max_gutter_delta=worst,texture_bytes=len(tiles)*stride*stride*3,seconds=time.monotonic()-start)))
if __name__=='__main__':main()
