#!/usr/bin/env python3
"""Reuse a 20m road with a 10m coordinate shift and plain run-up/braking aprons.

Texture files are hard-linked, never resampled. GZ and OptiX share shifted meshes.
"""
import argparse,json,os
from pathlib import Path
import xml.etree.ElementTree as ET
from agv_linescan.shared_scene import digest,validate
p=argparse.ArgumentParser();p.add_argument('--source',required=True);p.add_argument('--output',required=True);a=p.parse_args()
src=Path(a.source).resolve();dst=Path(a.output).resolve();dst.mkdir(parents=True,exist_ok=False)
m=json.loads((src/'manifest.json').read_text());shift=10.
for entry in src.iterdir():
 if entry.is_file() and entry.name not in ('world.sdf','manifest.json'):
  if entry.suffix=='.obj':
   with entry.open() as inp,(dst/entry.name).open('w') as out:
    for line in inp:
     if line.startswith('v '):
      f=line.split();line=f'v {float(f[1])+shift:.9f} {f[2]} {f[3]}\n'
     out.write(line)
  else:os.link(entry,dst/entry.name)
for asset in m['assets']:
 asset['sha256']=digest(dst/asset['mesh'])
 if 'collision_proxy' in asset:asset['collision_proxy']['sha256']=digest(dst/asset['collision_proxy']['mesh'])
 if 'display_uv_projection' in asset:asset['display_uv_projection']['origin_xy_m'][0]+=shift
m['ground_material']['origin_xy_m'][0]+=shift
for name in ('optical_valid_bounds_xy_m','inspection_bounds_xy_m'):
 m[name][0]+=shift;m[name][1]+=shift
for item in m.get('crack',{}).get('instances',[]):item['center'][0]+=shift
m['length_m']=42;m['profile']='20m_speed_demo_with_aprons'
m['demo_notes']='20m existing textured road shifted +10m; extra flat geometry for acceleration/braking only'
tree=ET.parse(src/'world.sdf');world=tree.getroot().find('world')
for e in world.iter():
 if e.text and str(src) in e.text:e.text=e.text.replace(str(src),str(dst))
for name,lo,hi in [('run_up',-1.,m['optical_valid_bounds_xy_m'][0]),('braking',m['optical_valid_bounds_xy_m'][1],42.)]:
 mesh=dst/(name+'.obj');mesh.write_text(f'v {lo} -6.144 0\nv {hi} -6.144 0\nv {hi} 6.144 0\nv {lo} 6.144 0\nvn 0 0 1\nf 1//1 2//1 3//1\nf 1//1 3//1 4//1\n')
 m['assets'].append(dict(name=name,mesh=mesh.name,sha256=digest(mesh),triangles=2,linear_reflectance=.45))
 model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true';link=ET.SubElement(model,'link',name='ground')
 for kind in ('visual','collision'):
  item=ET.SubElement(link,kind,name=kind);geo=ET.SubElement(ET.SubElement(item,'geometry'),'mesh');ET.SubElement(geo,'uri').text=str(mesh);ET.SubElement(geo,'scale').text='1 1 1'
  if kind=='visual':ET.SubElement(ET.SubElement(item,'material'),'diffuse').text='0.45 0.45 0.45 1'
tree.write(dst/'world.sdf',encoding='unicode');m['world_sha256']=digest(dst/'world.sdf');(dst/'manifest.json').write_text(json.dumps(m,indent=2)+'\n');validate(dst/'manifest.json')
print(dst/'manifest.json')
