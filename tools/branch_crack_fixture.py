#!/usr/bin/env python3
"""Local AI-derived branch trial; never replaces the production road assets."""
import argparse
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, convolve, distance_transform_edt, gaussian_filter
from scipy.spatial import Delaunay

from bake_concrete_road import LUT, largest, prepare_crack, sha, srgb
from concrete_quilt import ConcreteQuilt

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / 'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.shared_scene import validate

STEP = .00025
SOURCE = ROOT / 'assets/road/generated/concrete_crack_branch_ai_v2.png'


def clean_source_skeleton(mask):
    """Remove extraction burrs before upsampling; keep long source-drawn branches."""
    smooth = gaussian_filter(mask.astype('float32'), .85) >= .45
    sk = cv2.ximgproc.thinning(smooth.astype('uint8')*255)>0
    points = set(zip(*np.nonzero(sk)))
    def neighbors(p):
        y,x=p
        result=[]
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                q=(y+dy,x+dx)
                if (not dy and not dx) or q not in points: continue
                # Avoid redundant diagonal shortcuts around the same digital corner.
                if dy and dx and ((y+dy,x) in points or (y,x+dx) in points): continue
                result.append(q)
        return result
    # Prune only terminal stubs under 12 source pixels (~12 mm), not major forks.
    for _ in range(3):
        removed=set()
        for start in sorted(points):
            if len(neighbors(start)) != 1: continue
            path=[start];previous=None;current=start
            while len(path)<=12:
                following=[q for q in neighbors(current) if q!=previous]
                if len(following)!=1: break
                previous,current=current,following[0]
                if len(neighbors(current))!=2: break
                path.append(current)
            if len(path)<=12 and len(neighbors(current))>=3:
                removed.update(path)
        if not removed: break
        points-=removed
    # Trace existing centerline edges, then use subpixel interpolation at target scale.
    visited=set();paths=[]
    nodes=[p for p in sorted(points) if len(neighbors(p))!=2]
    for start in nodes+sorted(points):
        for nxt in neighbors(start):
            edge=tuple(sorted((start,nxt)))
            if edge in visited: continue
            path=[start];previous,current=start,nxt;visited.add(edge)
            while True:
                path.append(current)
                following=[q for q in neighbors(current) if q!=previous]
                if len(following)!=1: break
                nxt=following[0];edge=tuple(sorted((current,nxt)))
                if edge in visited: break
                visited.add(edge);previous,current=current,nxt
            paths.append(np.asarray(path,dtype='float32')[:,::-1])
    return paths


def branch_fields():
    raw = np.array(Image.open(SOURCE).convert('RGBA'))
    mask = largest(raw[:, :, 3] >= 128)
    y, x = np.nonzero(mask)
    raw = raw[y.min():y.max()+1, x.min():x.max()+1]
    mask = mask[y.min():y.max()+1, x.min():x.max()+1]
    # Isotropic scale retains source branch lengths, unlike the old 14 cm strip.
    w = round(1.8 / STEP)
    h = round(w * mask.shape[0] / mask.shape[1])
    source_radius = distance_transform_edt(mask)
    paths = clean_source_skeleton(mask)
    canvas = np.zeros((h,w),'uint8')
    for path in paths:
        # Half-source-pixel simplification removes grid stair steps, not source topology.
        path = cv2.approxPolyDP(path,.55,False).reshape(-1,2)
        path = (path+.5)*np.array([w/mask.shape[1],h/mask.shape[0]])-.5
        cv2.polylines(canvas,[np.rint(path*16).astype('int32')],False,255,1,cv2.LINE_AA,shift=4)
    skeleton = cv2.ximgproc.thinning((canvas>=64).astype('uint8')*255)>0
    del canvas
    # Short irregularities remain image-derived; no invented paths or random branch seed.
    distance, nearest = distance_transform_edt(~skeleton, return_indices=True)
    distance = distance.astype('float32')
    local = cv2.resize(source_radius.astype('float32'), (w, h))
    local = gaussian_filter(local, sigma=3)
    sampled = local[tuple(nearest)]
    lo, hi = np.percentile(local[skeleton], [10, 90])
    strength = np.clip((sampled-lo)/max(hi-lo, 1e-6), 0, 1)
    radius = (.0008 + .0012 * strength) / (2 * STEP)
    alpha = np.clip(radius + .5 - distance, 0, 1).astype('float32')
    # Rounded, variable-depth cross section; all depth stays within the same footprint.
    profile = np.maximum(0, 1 - (distance / (radius + .5))**2)**.75
    depth = (-profile * (.001 + .001 * strength)).astype('float32')
    # Neutral local reflectance attenuation, not the source's colored rims / baked shadows.
    # Optical shadowing comes from OptiX. The modest factor models dirt/mineral change only.
    pigment = (1 - alpha * (.25 + .15 * strength)).astype('float32')
    # Local micro-normal attenuation in the fissure; macro slopes come from the real mesh.
    normal_strength = (1 - .65 * alpha).astype('float32')
    widths = 2 * radius[skeleton] * STEP * 1000
    degree = convolve(skeleton.astype('uint8'), np.ones((3, 3), 'uint8')) - skeleton
    stats = dict(source='built-in image_gen', source_file=SOURCE.name, source_sha256=sha(SOURCE),
        source_canvas_px=list(Image.open(SOURCE).size), raster_shape=list(alpha.shape), texel_m=STEP,
        extent_m=[w*STEP, h*STEP], isotropic_resize=True, procedural_paths=False,
        nominal_width_mm=[float(widths.min()), float(np.median(widths)), float(widths.max())],
        width_semantics='local centerline diameter; branch unions and antialiased tips are not independent widths',
        depth_min_m=float(depth.min()), depth_max_m=float(depth.max()),
        skeleton_endpoints=int(np.count_nonzero(skeleton & (degree == 1))),
        extraction='source-scale thinning, <12 px terminal spur pruning, 0.55 px path simplification; isotropic subpixel rasterization',
        endpoint_count_scope='raster topology including edge irregularities; not manual count of major branches',
        substrate='Concrete047A unchanged outside defect', roughness=.60)
    assert widths.min() >= .7999 and widths.max() <= 2.0001
    assert np.count_nonzero(depth[alpha == 0]) == 0
    return dict(alpha=alpha, depth=depth, pigment=pigment, normal_strength=normal_strength,
                origin=[2.5-w*STEP/2, -h*STEP/2], stats=stats)


def old_fields():
    _, alpha, stats = prepare_crack(0, STEP)
    return dict(alpha=alpha, depth=-.002*alpha, pigment=np.ones_like(alpha),
                normal_strength=np.ones_like(alpha), origin=[1.4, -.07], stats=stats)


def geometry(fields):
    a = fields['alpha']
    yy, xx = np.nonzero(binary_dilation(a > 0, iterations=2))
    ox, oy = fields['origin']
    fine = np.column_stack((ox+(xx+.5)*STEP, oy+(yy+.5)*STEP, fields['depth'][yy, xx]))
    x, y = np.meshgrid(np.linspace(0, 5, 51), np.linspace(-2.5, 2.5, 51))
    base = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    # Remove only coarse vertices near actual fissure support, not a large rectangular hole.
    ix = np.floor((base[:, 0]-ox)/STEP).astype(int)
    iy = np.floor((base[:, 1]-oy)/STEP).astype(int)
    valid = (ix>=0)&(ix<a.shape[1])&(iy>=0)&(iy<a.shape[0])
    remove = np.zeros(len(base), bool)
    remove[valid] = binary_dilation(a>0, iterations=3)[iy[valid], ix[valid]]
    v = np.vstack((fine, base[~remove]))
    _, ids = np.unique(v[:, :2], axis=0, return_index=True)
    v = v[ids]
    f = Delaunay(v[:, :2]).simplices
    edges = v[f[:, 1:], :2] - v[f[:, :1], :2]
    areas = edges[:, 0, 0]*edges[:, 1, 1] - edges[:, 0, 1]*edges[:, 1, 0]
    assert np.all(areas > 0) and abs(areas.sum()/2-25) < 1e-7
    assert -.002001 <= v[:, 2].min() and v[:, 2].max() == 0
    return v, f


def sample_field(fields, key, xs, ys, default):
    ox, oy = fields['origin']
    x, y = np.meshgrid(np.float32((xs-ox)/STEP-.5), np.float32((ys-oy)/STEP-.5))
    return cv2.remap(fields[key], x, y, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=default)


def write_scene(out, v, f, material):
    path = out/'terrain.obj'
    with path.open('w') as stream:
        np.savetxt(stream, v, fmt='v %.9f %.9f %.9f')
        np.savetxt(stream, np.column_stack((v[:, 0]/5, (v[:, 1]+2.5)/5)), fmt='vt %.9f %.9f')
        n = np.cross(v[f[:, 1]]-v[f[:, 0]], v[f[:, 2]]-v[f[:, 0]])
        n /= np.linalg.norm(n, axis=1)[:, None]
        np.savetxt(stream, n, fmt='vn %.9f %.9f %.9f')
        for i, face in enumerate(f):
            stream.write('f '+' '.join(f'{j+1}/{j+1}/{i+1}' for j in face)+'\n')
    tree = ET.parse(ROOT/'src/agv_bringup/worlds/flat.sdf')
    world = tree.getroot().find('world')
    for m in list(world.findall('model')):
        world.remove(m)
    model = ET.SubElement(world, 'model', name='terrain')
    ET.SubElement(model, 'static').text = 'true'
    link = ET.SubElement(model, 'link', name='road')
    for kind in ('visual', 'collision'):
        item = ET.SubElement(link, kind, name=kind)
        mesh = ET.SubElement(ET.SubElement(item, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = str(path)
        ET.SubElement(mesh, 'scale').text = '1 1 1'
        if kind == 'visual':
            m = ET.SubElement(item, 'material')
            ET.SubElement(m, 'diffuse').text = '1 1 1 1'
            metal = ET.SubElement(ET.SubElement(m, 'pbr'), 'metal')
            ET.SubElement(metal, 'albedo_map').text = str(out/'display_color.png')
            ET.SubElement(metal, 'normal_map', type='tangent').text = str(out/'display_normal.png')
            ET.SubElement(metal, 'roughness').text = '.60'
            ET.SubElement(metal, 'metalness').text = '0'
    tree.write(out/'world.sdf', encoding='unicode')
    manifest = dict(schema='agv.shared.static_scene.v1', units='m', frame='world',
        transform='identity_world_baked', profile='branch_crack_local_trial',
        length_m=5, width_m=5,
        world='world.sdf', world_sha256=sha(out/'world.sdf'),
        assets=[dict(name='terrain', mesh=path.name, sha256=sha(path), triangles=len(f), material='ground')],
        ground_material=material,
        display_materials={k:sha(out/k) for k in ('display_color.png', 'display_normal.png')},
        display_uv_projection=dict(origin_xy_m=[0, -2.5], span_xy_m=[5, 5]),
        optical_valid_bounds_xy_m=[1.2, 3.8, -.8, .8],
        scope='local crack appearance trial; 5x5 m support, 2.6x1.6 m optical corridor; no production asset replacement')
    (out/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    validate(out/'manifest.json')


def build(out):
    out.mkdir(parents=True, exist_ok=False)
    source = ROOT/'assets/road/source'
    color = np.array(Image.open(source/'Concrete047A_8K-PNG_Color.png').convert('RGB'))
    normal = np.array(Image.open(source/'Concrete047A_8K-PNG_NormalGL.png').convert('RGB'))
    q = ConcreteQuilt(color, LUT, 5, 5, 0, source_width=2.1)
    q.save(out/'quilt')
    fields = {'old': old_fields(), 'branched': branch_fields()}
    w, h = 10400, 6400
    arrays = {}
    for name, field in fields.items():
        folder = out/name
        folder.mkdir()
        arrays[name] = (np.memmap(folder/'color.raw', dtype='uint8', mode='w+', shape=(h, w)),
                        np.memmap(folder/'normal.raw', dtype='uint8', mode='w+', shape=(h, w, 2)))
        # Source-derived quantitative masks, not pictures used as fake sensor captures.
        np.savez_compressed(folder/'defect_fields.npz', **{k:field[k] for k in ('alpha', 'depth')})
    linear = np.arange(256, dtype='float32')/255
    def sample(xs, ys):
        rgb = q.sample(xs, ys)
        n = q.sample(xs, ys, source=normal, lut=linear)*2-1
        n /= np.linalg.norm(n, axis=2)[:, :, None]
        n[:, :, 1] *= -1
        return rgb, n
    xs = 1.2+(np.arange(w)+.5)*STEP
    for start in range(0, h, 64):
        ys = -.8+(np.arange(start, min(h, start+64))+.5)*STEP
        rgb, n = sample(xs, ys)
        for name, field in fields.items():
            pigment = sample_field(field, 'pigment', xs, ys, 1)
            mono = rgb @ np.array([.2126, .7152, .0722], 'float32') * pigment
            nn = n.copy()
            nn[:, :, :2] *= sample_field(field, 'normal_strength', xs, ys, 1)[:, :, None]
            nn /= np.linalg.norm(nn, axis=2)[:, :, None]
            arrays[name][0][start:start+len(ys)] = np.uint8(np.clip(mono*255+.5, 0, 255))
            arrays[name][1][start:start+len(ys)] = np.uint8(np.clip((nn[:, :, :2]*.5+.5)*255+.5, 0, 255))
    for pair in arrays.values():
        for a in pair:
            a.flush()
    # Same geometry and physical coordinates in GZ, using a 2 mm overview map.
    xs = (np.arange(2500)+.5)*.002
    displays = {name:(np.empty((2500,2500,3),'uint8'),np.empty((2500,2500,3),'uint8')) for name in fields}
    for start in range(0, 2500, 64):
        ys = -2.5+(np.arange(start, min(start+64, 2500))+.5)*.002
        rgb, n = sample(xs, ys)
        for name, field in fields.items():
            pigment = sample_field(field, 'pigment', xs, ys, 1)
            nn = n.copy()
            nn[:, :, :2] *= sample_field(field, 'normal_strength', xs, ys, 1)[:, :, None]
            nn /= np.linalg.norm(nn, axis=2)[:, :, None]
            displays[name][0][start:start+len(ys)] = srgb(rgb*pigment[:, :, None])
            displays[name][1][start:start+len(ys)] = np.uint8(np.clip((nn*.5+.5)*255+.5,0,255))
    report = dict(texel_m=STEP, trial_footprint_m=[2,2], optical_corridor_m=[2.6,1.6],
        support_ground_m=[5,5], geometry_shared_with_GZ_visual_and_collision=True,
        roughness=.60, source_scale_m=2.1, source_scale_basis='project mapping, not measured',
        source_hashes={k:sha(source/f'Concrete047A_8K-PNG_{k}.png') for k in ('Color','NormalGL')},
        cases={})
    for name, field in fields.items():
        folder = out/name
        for im, file in zip(displays[name], ('display_color.png','display_normal.png')):
            Image.fromarray(im).save(folder/file)
        v, f = geometry(field)
        material = dict(schema='agv.ground_material.xy.v1', width=w, height=h,
            origin_xy_m=[1.2,-.8], span_xy_m=[2.6,1.6], roughness=.60,
            **{k:dict(file=k+'.raw',sha256=sha(folder/(k+'.raw'))) for k in ('color','normal')})
        write_scene(folder, v, f, material)
        report['cases'][name] = dict(**field['stats'], origin_xy_m=field['origin'],
            triangles=len(f), vertices=len(v), texture_payload_MiB=w*h*3/2**20,
            scene_manifest_sha256=sha(folder/'manifest.json'))
    # Pixel-exact background invariant over all high-resolution material pixels.
    for k, label in enumerate(('color', 'normal')):
        changed = 0
        for start in range(0,h,64):
            ys = -.8+(np.arange(start,min(h,start+64))+.5)*STEP
            xs = 1.2+(np.arange(w)+.5)*STEP
            touched = sample_field(fields['branched'], 'alpha', xs, ys, 0) > 0
            delta = arrays['branched'][k][start:start+len(ys)] != arrays['old'][k][start:start+len(ys)]
            if k: delta = np.any(delta,axis=2)
            assert not np.any(delta & ~touched), 'substrate changed outside crack'
            changed += int(delta.sum())
        report[label+'_changed_pixels'] = changed
    report['outside_defect_material_equal'] = True
    (out/'build.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    a = p.parse_args()
    build(Path(a.output).resolve())
