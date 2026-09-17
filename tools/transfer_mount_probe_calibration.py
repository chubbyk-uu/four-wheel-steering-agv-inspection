#!/usr/bin/env python3
"""Explicit experimental reuse of a fixed calibration after a zero-pose audit.

Does not recalibrate flex, correct its time-varying pose or weaken production
metadata checks. The resulting profile is tied to one target robot hash.
"""
import argparse,copy,hashlib,json,sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml
sys.path.append(str(Path(__file__).resolve().parents[1]/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.robot_scene import transform,triangles
from agv_linescan.calibration import capture_signature,make_profile


def nominal_visuals(xml):
    root=ET.fromstring(xml);parents={j.find('child').get('link'):j for j in root.findall('joint')}
    colors={m.get('name'):m.find('color') for m in root.findall('material')}
    def pose(name):
        j=parents.get(name)
        return np.eye(4) if j is None else pose(j.find('parent').get('link'))@transform(j.find('origin'))
    rows=[]
    for link in root.findall('link'):
        for v in link.findall('visual'):
            t=pose(link.get('name'))@transform(v.find('origin'));points=triangles(v.find('geometry'))
            points=points@t[:3,:3].T+t[:3,3]
            material=v.find('material');color=material.find('color') if material is not None else None
            if color is None and material is not None:color=colors.get(material.get('name'))
            rgba=[.5,.5,.5,1.] if color is None else list(map(float,color.get('rgba').split()))
            for tri in points:
                tri=np.round(tri,8);tri=tri[np.lexsort(tri.T[::-1])]
                rows.append([*tri.ravel(),*rgba])
    array=np.asarray(rows);return array[np.lexsort(array.T[::-1])],pose('camera_optical_frame'),pose('led_link')


def transfer(profile,source,target):
    src=(source/'robot.urdf').read_bytes();dst=(target/'robot.urdf').read_bytes()
    c=copy.deepcopy(profile['conditions']);assert hashlib.sha256(src).hexdigest()==c['robot_source_sha256']
    srcconfig=yaml.safe_load((source/'calibration.yaml').read_text());dstconfig=yaml.safe_load((target/'calibration.yaml').read_text())
    assert capture_signature(srcconfig)==capture_signature(dstconfig)==c['capture_signature'],'optics/radiometry changed'
    left=nominal_visuals(src);right=nominal_visuals(dst)
    for a,b in zip(left,right):np.testing.assert_allclose(a,b,rtol=0,atol=1e-7)
    robot_hash=hashlib.sha256(dst).hexdigest();block=json.loads(next(target.glob('block_*.json')).read_text())
    assert robot_hash==block['robot_contract']['source_sha256']
    c['robot_source_sha256']=robot_hash
    c['note']='EXPERIMENT ONLY: same fixed baseline coefficients; nominal visual mesh/colors and camera/LED transforms audited equal. No new reference capture; no compensation of flex/static loaded-pose differences.'
    sources=dict(profile['sources'],mount_probe_transfer=dict(source_calibration_id=profile['calibration_id'],source_robot_sha256=hashlib.sha256(src).hexdigest(),target_robot_sha256=robot_hash,nominal_geometry_max_error=float(np.max(abs(left[0]-right[0])))))
    return make_profile(profile['flat'],profile['geometry'],c,sources)


def main():
    p=argparse.ArgumentParser();p.add_argument('--profile',type=Path,required=True);p.add_argument('--source-session',type=Path,required=True);p.add_argument('--target-session',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=transfer(json.loads(a.profile.read_text()),a.source_session,a.target_session)
    with a.output.open('x') as f:f.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result['sources']['mount_probe_transfer']))

if __name__=='__main__':main()
