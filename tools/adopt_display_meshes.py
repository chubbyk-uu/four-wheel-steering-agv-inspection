#!/usr/bin/env python3
"""Adopt lightweight GZ display meshes into a scene, reversibly.

Everything this writes is additive except two small files, manifest.json and
world.sdf, which are backed up first. So reverting restores the scene exactly,
and the multi-gigabyte optical meshes and textures are never touched.

The scene must validate before the change and must validate after it; if the
result does not, the backups are restored and the error is raised rather than
leaving an asset that cannot launch.

Generation reuses build_light_display_assets so what gets adopted is what was
measured, not a second implementation of the same mesh.
"""
import argparse
import json
import shutil
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/agv_linescan'))
sys.path.insert(0, str(ROOT/'tools'))
from agv_linescan.shared_scene import digest, validate   # noqa: E402
from build_light_display_assets import display_mesh      # noqa: E402

SUFFIX = '.before_display_mesh'
RECORD = 'display_mesh_adoption.json'


def backups(root):
    return [(root/name, root/(name+SUFFIX)) for name in ('manifest.json', 'world.sdf')]


def restore(root):
    for live, saved in backups(root):
        if saved.is_file():
            shutil.copy2(saved, live)


def revert(root):
    if not (root/RECORD).is_file():
        raise SystemExit('no adoption record in '+str(root))
    record = json.loads((root/RECORD).read_text())
    restore(root)
    for entry in record['entries']:
        (root/entry['display_mesh']['mesh']).unlink(missing_ok=True)
    for _, saved in backups(root):
        saved.unlink(missing_ok=True)
    (root/RECORD).unlink()
    validate(root/'manifest.json')
    print('reverted %d display meshes in %s' % (len(record['entries']), root))


def adopt(scene):
    root = scene.resolve().parent
    if (root/RECORD).is_file():
        raise SystemExit('already adopted; run with --revert first')
    validate(scene)                                   # refuse to start from a broken scene
    for live, saved in backups(root):
        shutil.copy2(live, saved)

    start = time.perf_counter()
    manifest = json.loads(scene.read_text())
    entries = {}
    try:
        for asset in manifest['assets']:
            made = display_mesh(root, asset, manifest, root)
            if not made:
                continue
            count, path = made
            asset['display_mesh'] = dict(mesh=path.name, sha256=digest(path), triangles=count)
            entries[asset['name']] = asset['display_mesh']
        if not entries:
            raise ValueError('no asset in this scene can carry a display mesh')

        tree = ET.parse(root/manifest['world'])
        world = tree.getroot().find('world')
        for model in world.findall('model'):
            entry = entries.get(model.get('name'))
            if not entry:
                continue
            uri = model.find('link/visual/geometry/mesh/uri')
            if uri is None:
                raise ValueError('%s has no visual mesh to redirect' % model.get('name'))
            uri.text = str(root/entry['mesh'])
        tree.write(root/manifest['world'], encoding='unicode')

        manifest['world_sha256'] = digest(root/manifest['world'])
        scene.write_text(json.dumps(manifest, indent=2)+'\n')
        shared = validate(scene)
    except BaseException:
        # Leave nothing half-adopted: the two small files come back and any
        # display mesh written so far goes away.
        restore(root)
        for entry in entries.values():
            (root/entry['mesh']).unlink(missing_ok=True)
        raise

    optical = sum(a['triangles'] for a in shared['assets'])
    light = sum(e['triangles'] for e in entries.values())
    record = dict(schema='agv.display_mesh_adoption.v1', scene=str(scene),
                  adopted=len(entries), of_assets=len(shared['assets']),
                  optical_triangles=optical, display_triangles=light,
                  triangle_ratio=optical/light,
                  previous_world_sha256=json.loads((root/('manifest.json'+SUFFIX)).read_text())['world_sha256'],
                  entries=[dict(name=k, display_mesh=v) for k, v in sorted(entries.items())],
                  reversible=('manifest.json and world.sdf are backed up beside themselves with '
                              'the %s suffix; every other change is a new file' % SUFFIX),
                  seconds=time.perf_counter()-start)
    (root/RECORD).write_text(json.dumps(record, indent=2)+'\n')
    print(json.dumps({k: v for k, v in record.items() if k != 'entries'}, indent=2))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--revert', action='store_true')
    a = p.parse_args()
    if a.revert:
        revert(a.scene.resolve().parent)
    else:
        adopt(a.scene)


if __name__ == '__main__':
    main()
