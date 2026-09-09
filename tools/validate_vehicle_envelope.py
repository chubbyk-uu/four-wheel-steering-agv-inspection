#!/usr/bin/env python3
"""Conservative planar sweep bound from visual AND collision URDF geometry.

Joint rotations are bounded over a full revolution, not sampled angles.
Prismatic motion includes both travel endpoints. This is a base-frame planar
bound, not a roll/pitch or obstacle clearance certificate.
"""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/agv_linescan'))
from agv_linescan.robot_scene import triangles, transform


def envelope(urdf):
    root = ET.fromstring(urdf)
    parents = {j.find('child').get('link'): j for j in root.findall('joint')}
    bounds = {}
    for link in root.findall('link'):
        name = link.get('name')
        maximum = 0.
        for shape in list(link.findall('visual'))+list(link.findall('collision')):
            geometry = shape.find('geometry')
            # A circumscribed cylinder mesh bounds the actual smooth surface.
            cylinder = geometry.find('cylinder')
            if cylinder is not None:
                geometry = ET.fromstring(ET.tostring(geometry))
                geometry.find('cylinder').set('radius', str(float(cylinder.get('radius'))/np.cos(np.pi/64)))
            centers = triangles(geometry).reshape(-1, 3)
            radii = np.zeros(len(centers))
            pose = transform(shape.find('origin'))
            centers = centers@pose[:3, :3].T+pose[:3, 3]
            current = name
            visited = set()
            while current in parents:
                if current in visited:
                    raise ValueError('joint cycle')
                visited.add(current)
                joint = parents[current]
                kind = joint.get('type')
                if kind != 'fixed':
                    element = joint.find('axis')
                    axis = np.array(list(map(float, element.get('xyz', '1 0 0').split()))) if element is not None else np.array([1., 0., 0.])
                    axis /= np.linalg.norm(axis)
                    if kind in ('revolute', 'continuous'):
                        # Each orbit fits a sphere about its axis projection.
                        projected = np.outer(centers@axis, axis)
                        radii += np.linalg.norm(centers-projected, axis=1)
                        centers = projected
                    elif kind == 'prismatic':
                        limit = joint.find('limit')
                        centers = np.vstack([centers+float(limit.get(k))*axis for k in ('lower','upper')])
                        radii = np.tile(radii, 2)
                    else:
                        raise ValueError('unsupported joint '+kind)
                pose = transform(joint.find('origin'))
                centers = centers@pose[:3, :3].T+pose[:3, 3]
                current = joint.find('parent').get('link')
            if current != 'base_link':
                raise ValueError('geometry must descend from base_link')
            maximum = max(maximum, float(np.max(np.linalg.norm(centers[:, :2], axis=1)+radii)))
        if maximum:
            bounds[name] = maximum
    if not bounds or not np.isfinite(list(bounds.values())).all():
        raise ValueError('invalid envelope')
    return bounds


def current_urdf():
    import xacro
    return xacro.process_file(str(ROOT/'src/agv_description/urdf/agv.urdf.xacro'), mappings={
        'platform':str(ROOT/'src/agv_description/config/platform.yaml'),
        'camera_config':str(ROOT/'src/agv_description/config/linescan.yaml')}).toxml()


if __name__ == '__main__':
    bounds = envelope(current_urdf())
    print(json.dumps({'maximum_radius_m':max(bounds.values()), 'links':bounds}, indent=2))
    assert max(bounds.values()) <= 1.5, 'model exceeds audited planning envelope'
