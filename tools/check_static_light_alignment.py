#!/usr/bin/env python3
"""Check actual recorded flat-road rest attitude against camera/LED geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import xacro
import yaml
from agv_linescan.robot_scene import transform

p=argparse.ArgumentParser();p.add_argument('--samples',type=Path,default=Path('local_data/rough_road_probe/flat_valid/samples.json'));p.add_argument('--output',type=Path,required=True);a=p.parse_args()
raw=json.loads(a.samples.read_text());rows=np.array([r[:-1] for r in raw if r[-1]=='brake'],float)[-100:]
assert len(rows)==100 and np.max(np.abs(rows[:,3]))<.001,'not a settled stopped window'
assert np.max(np.std(rows[:,4:6],axis=0))<np.radians(.001),'attitude not settled'
config_path=Path('src/agv_description/config/linescan.yaml');model_path=Path('src/agv_description/urdf/agv.urdf.xacro')
c=yaml.safe_load(config_path.read_text());root=ET.fromstring(xacro.process_file(str(model_path)).toxml())
parents={j.find('child').get('link'):j for j in root.findall('joint')}
def local(link):
 if link not in parents:return np.eye(4)
 joint=parents[link];return local(joint.find('parent').get('link'))@transform(joint.find('origin'))
base=np.eye(4);base[:3,:3]=Rotation.from_euler('xyz',np.mean(rows[:,4:7],axis=0)).as_matrix();base[2,3]=np.mean(rows[:,2])
# Remove world XY translation only; retain measured roll/pitch/yaw and height.
cam=base@local('camera_optical_frame');origin=cam[:3,3];across=cam[:3,0];down=cam[:3,2];forward=np.cross(down,across)
q=(np.arange(c['width'])-(c['width']-1)/2)/(c['width']/2)
rays=np.polynomial.polynomial.polyval(q,c['ray_polynomial'])*c['width']*c['pixel_pitch_m']/(2*c['focal_length_m'])
directions=down+np.outer(rays,across);assert np.all(directions[:,2]<0)
points=origin-directions*(origin[2]/directions[:,2])[:,None]
center=origin-down*origin[2]/down[2]
covered=np.zeros(len(points),bool);axis_hits=[]
for light in root.findall('gazebo/light'):
 lo=base[:3,3]+base[:3,:3]@np.array(list(map(float,light.findtext('pose').split()[:3])))
 ld=base[:3,:3]@np.array(list(map(float,light.findtext('direction').split())))
 ld/=np.linalg.norm(ld);axis_hits.append(lo-ld*lo[2]/ld[2])
 v=points-lo;v/=np.linalg.norm(v,axis=1)[:,None]
 covered|=(v@ld)>=np.cos(float(light.findtext('spot/outer_angle'))/2)
spot=axis_hits[len(axis_hits)//2]
# Match LedIrradiance's camera-relative longitudinal envelope, excluding material/shadows.
h=c['nominal_width_m']*c['focal_length_m']/(c['width']*c['pixel_pitch_m'])
delta=points-origin;pf=delta@forward;pd=delta@down
scale=(pd-(h-c['led_height_m']))/c['led_height_m']
band_center=c['led_forward_offset_m']*(1-scale)
offset=pf-band_center;half_depth=c['radiometry']['led_half_depth_m']*scale
factor=np.exp(-2*(offset/half_depth)**4)
assert covered.all() and np.max(abs(offset))<.001,'static line is no longer centered'
result=dict(schema='agv.static_light_alignment.v1',passed=True,
 source='Recorded final 2 s after braking in flat_valid; no new simulation launch',samples=100,
 relative_sample_path=str(a.samples),sample_sha256=hashlib.sha256(a.samples.read_bytes()).hexdigest(),
 measured_base_height_m=float(base[2,3]),measured_rpy_deg=np.degrees(np.mean(rows[:,4:7],axis=0)).tolist(),
 camera_optical_height_m=float(origin[2]),scan_center_xy_m=center[:2].tolist(),gz_center_spot_xy_m=spot[:2].tolist(),
 center_separation_mm=float(np.linalg.norm(center-spot)*1000),
 optix_max_longitudinal_envelope_offset_mm=float(np.max(abs(offset))*1000),
 optix_half_depth_mm=[float(half_depth.min()*1000),float(half_depth.max()*1000)],
 optix_min_longitudinal_profile_factor=float(factor.min()),checked_pixels=len(points),gz_cone_covered_pixels=int(covered.sum()),
 source_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in (config_path,model_path,Path('src/agv_description/config/platform.yaml'))},
 scope='Flat static geometry using recorded base truth plus fixed mounts; all raw pixel rays checked. Cone/envelope coverage, not calibrated lux, irradiance uniformity, occlusion rendering or dynamic pitch acceptance. Recorded configuration was unchanged during this check.')
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
