"""Derive ray geometry from the exact expanded URDF supplied to Gazebo.

Fixed descendants are baked into their moving parent. Cylinders use 64 facets;
visual geometry is used for visibility (collision approximations are not optics).
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def transform(origin):
    m = np.eye(4)
    if origin is None:
        return m
    xyz = list(map(float, origin.get('xyz', '0 0 0').split()))
    r, p, y = map(float, origin.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    m[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    m[:3, 3] = xyz
    if not np.isfinite(m).all():
        raise ValueError('nonfinite URDF origin')
    return m


def triangles(geometry, facets=64):
    box, cylinder = geometry.find('box'), geometry.find('cylinder')
    if box is not None:
        size = np.array(list(map(float, box.attrib['size'].split())))
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('invalid box')
        v = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])*size
        faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                 (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        return np.array([v[list(t)] for a, b, c, d in faces for t in ((a, b, c), (a, c, d))])
    if cylinder is not None:
        r, h = float(cylinder.attrib['radius']), float(cylinder.attrib['length'])/2
        if not (math.isfinite(r) and math.isfinite(h) and min(r, h) > 0):
            raise ValueError('invalid cylinder')
        result = []
        for i in range(facets):
            a, b = 2*math.pi*i/facets, 2*math.pi*(i+1)/facets
            v = [(r*math.cos(a), r*math.sin(a), -h), (r*math.cos(b), r*math.sin(b), -h),
                 (r*math.cos(b), r*math.sin(b), h), (r*math.cos(a), r*math.sin(a), h)]
            result.extend([[v[0], v[1], v[2]], [v[0], v[2], v[3]],
                           [(0, 0, -h), v[1], v[0]], [(0, 0, h), v[3], v[2]]])
        return np.array(result)
    mesh = geometry.find('mesh')
    if mesh is not None:
        uri = mesh.attrib['filename']
        if uri.startswith('package://'):
            from ament_index_python.packages import get_package_share_directory
            package, relative = uri[len('package://'):].split('/', 1)
            path = Path(get_package_share_directory(package))/relative
        else:
            path = Path(uri.removeprefix('file://'))
        if path.suffix.lower() != '.obj':
            raise ValueError('ray exporter currently accepts triangular OBJ meshes only')
        vertices, faces = [], []
        for line in path.read_text().splitlines():
            fields = line.split()
            if not fields: continue
            if fields[0] == 'v': vertices.append(list(map(float, fields[1:4])))
            if fields[0] == 'f':
                if len(fields) != 4: raise ValueError('OBJ must be triangulated')
                ids = [int(f.split('/')[0]) for f in fields[1:]]
                if any(i <= 0 or i > len(vertices) for i in ids):
                    raise ValueError('invalid OBJ index')
                faces.append([i-1 for i in ids])
        scale = np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
        points = np.asarray(vertices, dtype=float)
        if scale.shape != (3,) or np.any(scale <= 0) or not np.isfinite(scale).all() or not faces or not np.isfinite(points).all():
            raise ValueError('invalid OBJ geometry or scale')
        return (points*scale)[faces]
    raise ValueError('unsupported URDF visual geometry; refusing to omit it')


def export(urdf, destination):
    root = ET.fromstring(urdf)
    links = {n.attrib['name']: n for n in root.findall('link')}
    parents = {j.find('child').attrib['link']: j for j in root.findall('joint')}
    colors = {m.attrib['name']: m.find('color') for m in root.findall('material')}
    groups = {}
    def group(name, visited=()):
        if name in visited:
            raise ValueError('URDF joint cycle')
        j = parents.get(name)
        if j is None or j.attrib['type'] != 'fixed':
            return name, np.eye(4)
        parent, pose = group(j.find('parent').attrib['link'], visited+(name,))
        return parent, pose @ transform(j.find('origin'))
    for name, link in links.items():
        moving, fixed_pose = group(name)
        for visual in link.findall('visual'):
            vertices = triangles(visual.find('geometry')).reshape(-1, 3)
            pose = fixed_pose @ transform(visual.find('origin'))
            vertices = vertices @ pose[:3, :3].T + pose[:3, 3]
            material = visual.find('material')
            color = None if material is None else material.find('color')
            if color is None and material is not None:
                color = colors.get(material.get('name'))
            rgb = [0.5]*3 if color is None else list(map(float, color.attrib['rgba'].split()))[:3]
            if len(rgb) != 3 or not all(math.isfinite(c) and 0 <= c <= 1 for c in rgb):
                raise ValueError('invalid material color')
            reflectance = sum(a*b for a, b in zip(rgb, (.2126, .7152, .0722)))
            g = groups.setdefault(moving, {'name': moving, 'vertices': [], 'reflectance': []})
            g['vertices'].extend(vertices.astype(np.float32).tolist())
            g['reflectance'].extend([reflectance]*(len(vertices)//3))
    target = Path(destination)
    target.mkdir(parents=True, exist_ok=True)
    (target/'robot.urdf').write_text(urdf)
    document = {'schema': 'agv.robot.ray_scene.v1', 'source_urdf': 'robot.urdf',
                'source_sha256': hashlib.sha256(urdf.encode()).hexdigest(),
                'cylinder_facets': 64, 'groups': list(groups.values())}
    if 'led_link' in links:
        moving, pose = group('led_link')
        # Same emitter plane as the 21 GZ display spots: 22mm below bar centre.
        half_length = float(links['led_link'].find('visual/geometry/box').get('size').split()[1])/2-.02
        if half_length <= 0: raise ValueError('LED strip too short')
        light = []
        for i in range(4):
            point = pose @ np.array([0., -half_length+2*half_length*(i+.5)/4, -.022, 1.])
            light.append(point[:3].tolist())
        document['led_emitters'] = {'link': moving, 'positions_m': light}
    (target/'robot_scene.json').write_text(json.dumps(document, separators=(',', ':'))+'\n')
    return target/'robot_scene.json'


def split_visual_links(urdf):
    """One material per visual link for RViz Jazzy's primitive-material lookup.

    Fixed, massless child links preserve geometry, dynamics and joint controls;
    GZ and the ray exporter receive this same expanded description.
    """
    root=ET.fromstring(urdf)
    names={element.get('name') for element in root if element.get('name')}
    for link in list(root.findall('link')):
        for index,visual in enumerate(link.findall('visual')[1:],1):
            name=f"{link.attrib['name']}_visual_{index}"
            joint_name=name+'_fixed'
            if name in names or joint_name in names:
                raise ValueError('visual-link name collision')
            names.update((name,joint_name))
            link.remove(visual)
            child=ET.SubElement(root,'link',name=name);child.append(visual)
            joint=ET.SubElement(root,'joint',name=joint_name,type='fixed')
            ET.SubElement(joint,'parent',link=link.attrib['name'])
            ET.SubElement(joint,'child',link=name)
    return ET.tostring(root,encoding='unicode')
