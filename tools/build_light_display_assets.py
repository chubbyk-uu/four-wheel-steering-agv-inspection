#!/usr/bin/env python3
"""Build the candidate lightweight GZ display assets and measure them.

Both gz processes build their own Ogre2 scene from the same optical meshes and
display textures, and that is most of the 100 m startup. Neither is what the
line-scan camera images under the optix backend, so a lighter display copy is
possible. This produces the candidates and the numbers; it changes nothing.

It writes only into a new output directory. The manifest, world.sdf and the
scene validator are untouched, so nothing here is wired in: adopting these
assets is a separate change to the scene contract, which currently requires
the visual URI to be the optical mesh.

Meshes reuse the collision-proxy geometry, which is already the source
heightfield sampled on its own 20 cm grid with the shallow defects flattened,
and which validate_layered_proxy has checked against that heightfield. Only
UVs are added, so the display surface differs from the imaged one exactly by
the declared defect depth, inside the declared defect area, and nowhere else.

Textures are halved with the two corrections a plain resize gets wrong:
normals are averaged as vectors and restored to unit length, and colour is
averaged in linear light rather than in sRGB.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.obj_arrays import read_obj                       # noqa: E402
from agv_linescan.shared_scene import digest, validate_display_uv  # noqa: E402

LAYERED = 'layered_heightfield_shallow_v1'


def checked(path, expected, what):
    if digest(path) != expected:
        raise ValueError('%s checksum mismatch: %s' % (what, path.name))
    return path


def display_mesh(root, asset, manifest, out):
    """The proxy surface with display UVs. Returns (triangles, path)."""
    proxy = asset.get('collision_proxy')
    projection = asset.get('display_uv_projection')
    if not proxy or not projection:
        return None
    # A shallow_horizontal_rectangle proxy is four vertices of flat plane. Using
    # it would silently replace the road with a quad, so refuse instead.
    if proxy.get('method') != LAYERED:
        raise ValueError('%s: display mesh needs a %s proxy, found %r'
                         % (asset['name'], LAYERED, proxy.get('method')))
    source = checked(root/proxy['mesh'], proxy['sha256'], 'collision proxy')
    v, f, _ = read_obj(source)
    ox, oy = projection['origin_xy_m']
    sx, sy = projection['span_xy_m']
    uv = (v[:, :2] - [ox, oy])/[sx, sy]
    if projection['v_direction'] == 'decreasing_world_y':
        uv[:, 1] = 1 - uv[:, 1]
    # The shipped checker, so the generated UVs meet the contract by construction.
    validate_display_uv(asset, manifest, v, uv)

    face = np.cross(v[f[:, 1]]-v[f[:, 0]], v[f[:, 2]]-v[f[:, 0]])
    face /= np.linalg.norm(face, axis=1)[:, None]
    normal = np.zeros_like(v)
    for k in range(3):
        np.add.at(normal, f[:, k], face)
    normal /= np.linalg.norm(normal, axis=1)[:, None]

    lines = ['v %.9f %.9f %.9f' % tuple(p) for p in v]
    lines += ['vt %.9f %.9f' % tuple(t) for t in uv]
    lines += ['vn %.9f %.9f %.9f' % tuple(q) for q in normal]
    lines += ['f %d/%d/%d %d/%d/%d %d/%d/%d' % (a+1, a+1, a+1, b+1, b+1, b+1, c+1, c+1, c+1)
              for a, b, c in f]
    path = out/('display_'+asset['name']+'.obj')
    path.write_text('\n'.join(lines)+'\n')
    return len(f), path


def box2(a):
    """Exact 2x2 area average; the image dimensions must both be even."""
    h, w = a.shape[:2]
    if h % 2 or w % 2:
        raise ValueError('cannot halve an odd dimension')
    return a.reshape(h//2, 2, w//2, 2, -1).mean(axis=(1, 3))


def half_texture(source, out):
    """Halve one display map. Returns the largest unit-length or sRGB error."""
    with Image.open(source) as im:
        a = np.asarray(im.convert('RGB'), np.float32)/255.
    if source.name.startswith('display_normal_'):
        vec = box2(a*2-1)
        length = np.linalg.norm(vec, axis=2, keepdims=True)
        error = float(np.abs(length-1).max())
        length[length == 0] = 1
        data = (vec/length+1)/2
    else:
        linear = np.where(a <= .04045, a/12.92, ((a+.055)/1.055)**2.4)
        small = box2(linear)
        data = np.where(small <= .0031308, small*12.92, 1.055*small**(1/2.4)-.055)
        error = float(np.abs(data-box2(a)).max()*255)
    Image.fromarray(np.clip(data*255+.5, 0, 255).astype(np.uint8)).save(out/source.name)
    return error


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--report', type=Path)
    p.add_argument('--skip-meshes', action='store_true')
    p.add_argument('--skip-textures', action='store_true')
    a = p.parse_args()
    root = a.scene.resolve().parent
    manifest = json.loads(a.scene.read_text())
    a.output.mkdir(parents=True, exist_ok=False)

    report = dict(schema='agv.light_display_assets.v1', scene=str(a.scene),
                  scope=('Candidate GZ-only display assets, generated for measurement. '
                         'Nothing is wired in: the scene contract still requires the visual '
                         'URI to be the optical mesh, and OptiX reads manifest["assets"], '
                         'which this does not touch.'))

    if not a.skip_meshes:
        start = time.perf_counter()
        built = [(asset, display_mesh(root, asset, manifest, a.output))
                 for asset in manifest['assets']]
        entries = [dict(name=asset['name'],
                        display_mesh=dict(mesh=path.name, sha256=digest(path), triangles=count))
                   for asset, made in built if made for count, path in [made]]
        built = [made for _, made in built if made]
        optical = sum(v['triangles'] for v in manifest['assets'])
        light = sum(t for t, _ in built)
        report['meshes'] = dict(
            assets=len(built), of_assets=len(manifest['assets']),
            optical_triangles=optical, display_triangles=light,
            triangle_ratio=optical/light,
            optical_obj_bytes=sum((root/v['mesh']).stat().st_size for v in manifest['assets']),
            display_obj_bytes=sum(p.stat().st_size for _, p in built),
            geometry_bytes_estimate=dict(optical=optical*3*32, display=light*3*32,
                                         basis='32 B per vertex for position, normal and UV'),
            uv_checked_with='agv_linescan.shared_scene.validate_display_uv',
            # Ready to merge into the manifest, in the shape validate() expects,
            # so adopting these does not reimplement the entry by hand.
            manifest_entries=entries,
            fidelity=('identical to the imaged surface outside the declared shallow defects; '
                      'inside them it is the flattened reference plane, so the difference is '
                      'bounded by max_surface_deviation_m over the area shallow_rectangle caps'),
            seconds=time.perf_counter()-start)

    if not a.skip_textures:
        start = time.perf_counter()
        names = sorted(manifest['display_materials'])
        errors = {'normal_unit_length': 0., 'srgb_vs_linear_grey_levels': 0.}
        for name in names:
            source = checked(root/name, manifest['display_materials'][name], 'display material')
            error = half_texture(source, a.output)
            key = 'normal_unit_length' if name.startswith('display_normal_') else 'srgb_vs_linear_grey_levels'
            errors[key] = max(errors[key], error)
        with Image.open(root/names[0]) as im:
            size = im.size
        halved = (size[0]//2, size[1]//2)
        report['textures'] = dict(
            count=len(names), source_pixels=list(size), display_pixels=list(halved),
            source_bytes=sum((root/n).stat().st_size for n in names),
            display_bytes=sum((a.output/n).stat().st_size for n in names),
            vram_estimate=dict(source=len(names)*size[0]*size[1]*4*4//3,
                               display=len(names)*halved[0]*halved[1]*4*4//3,
                               basis='RGBA with a full mipmap chain, 4/3 of the base level'),
            corrections=dict(
                normal_maps='averaged as vectors then restored to unit length',
                colour='averaged in linear light, not in sRGB',
                largest_error_avoided=errors),
            seconds=time.perf_counter()-start)

    text = json.dumps(report, indent=2)+'\n'
    (a.report or a.output/'report.json').write_text(text)
    print(text)


if __name__ == '__main__':
    main()
