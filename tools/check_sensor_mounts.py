#!/usr/bin/env python3
"""Nominal flat-road optical clearance and shared LED geometric footprint checks."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import xacro, yaml
from agv_linescan.robot_scene import triangles, transform
root=Path(__file__).resolve().parents[1]
r=ET.fromstring(xacro.process_file(str(root/'src/agv_description/urdf/agv.urdf.xacro')).toxml())
c=yaml.safe_load((root/'src/agv_description/config/linescan.yaml').read_text())
parents={j.find('child').get('link'):j for j in r.findall('joint')}
def pose(link):
 if link not in parents:return np.eye(4)
 j=parents[link]
 return pose(j.find('parent').get('link'))@transform(j.find('origin'))
t=[]
for link in r.findall('link'):
 for v in link.findall('visual'):
  m=pose(link.get('name'))@transform(v.find('origin'))
  t.append(triangles(v.find('geometry'))@m[:3,:3].T+m[:3,3])
t=np.concatenate(t);a=t[:,0];e1=t[:,1]-a;e2=t[:,2]-a
camera=pose('camera_optical_frame');o=camera[:3,3];ground=-c['base_nominal_height_m']
half=c['nominal_width_m']/2
end=half*sum(c['ray_polynomial'])
hits=0
for across in np.linspace(-end,end,257):
 target=np.array([c['camera_x_m'],across,ground]);d=target-o
 h=np.cross(np.broadcast_to(d,e2.shape),e2);det=np.einsum('ij,ij->i',e1,h)
 valid=np.abs(det)>1e-12;inv=np.divide(1,det,out=np.zeros_like(det),where=valid)
 s=o-a;u=inv*np.einsum('ij,ij->i',s,h);q=np.cross(s,e1)
 v=inv*(q@d);distance=inv*np.einsum('ij,ij->i',e2,q)
 hits+=bool(np.any(valid&(u>=0)&(v>=0)&(u+v<=1)&(distance>1e-6)&(distance<1-1e-6)))
assert hits==0, ('robot blocks camera rays',hits)
lights=r.findall('gazebo/light');covered=0
for across in np.linspace(-end,end,257):
 target=np.array([c['camera_x_m'],across,ground]);ok=False
 for light in lights:
  o=np.array(list(map(float,light.findtext('pose').split()[:3])))
  axis=np.array(list(map(float,light.findtext('direction').split())))
  d=target-o;d/=np.linalg.norm(d)
  # Conservative half of configured cone angle, geometric coverage only.
  ok|=np.dot(d,axis)>=np.cos(float(light.findtext('spot/outer_angle'))/2)
 covered+=ok
assert covered==257, ('uncovered scan samples',257-covered)
assert c['radiometry']['led_half_width_m']>end
print(json.dumps(dict(camera_height_m=float(camera[2,3]-ground),raw_swath_m=2*end,checked_rays=257,occluded_camera_rays=hits,gz_spot_covered_samples=int(covered),scope='nominal flat geometry; not radiometric uniformity or suspension envelope validation')))
