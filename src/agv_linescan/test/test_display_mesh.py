"""A separate GZ display mesh may be lighter, but not a different road.

The contract was written so the GZ visual and the OptiX reader are one file.
Letting them differ means the validator has to bound the difference instead of
asserting identity, and these are the bounds.
"""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from agv_linescan.shared_scene import digest, generate, validate

BASE = Path(__file__).resolve().parents[2]/'agv_bringup/worlds/flat.sdf'
ORIGIN = (0., -1.)
SPAN = (10., 2.)


def write_grid(path, nx, ny, height=lambda x, y: 0.):
    """A regular grid over the asset footprint, with the projected UVs."""
    xs = np.linspace(ORIGIN[0], ORIGIN[0]+SPAN[0], nx+1)
    ys = np.linspace(ORIGIN[1], ORIGIN[1]+SPAN[1], ny+1)
    v = np.array([[x, y, height(x, y)] for x in xs for y in ys])
    uv = (v[:, :2]-ORIGIN)/SPAN
    uv[:, 1] = 1-uv[:, 1]                       # decreasing_world_y
    quads = [(i*(ny+1)+j, (i+1)*(ny+1)+j, (i+1)*(ny+1)+j+1, i*(ny+1)+j+1)
             for i in range(nx) for j in range(ny)]
    faces = [t for a, b, c, d in quads for t in ((a, b, c), (a, c, d))]
    lines = ['v %.9f %.9f %.9f' % tuple(p) for p in v]
    lines += ['vt %.9f %.9f' % tuple(t) for t in uv]
    lines += ['vn 0 0 1']
    lines += ['f %d/%d/1 %d/%d/1 %d/%d/1' % (a+1, a+1, b+1, b+1, c+1, c+1)
              for a, b, c in faces]
    path.write_text('\n'.join(lines)+'\n')
    return len(faces)


@pytest.fixture
def scene(tmp_path):
    """A scene whose one ground asset declares a lighter display mesh."""
    path = tmp_path/'scene'
    generate(path, BASE, length=10, width=2, margin=0)
    mp = path/'manifest.json'
    m = json.loads(mp.read_text())
    asset = m['assets'][0]
    asset['display_uv_projection'] = dict(origin_xy_m=list(ORIGIN), span_xy_m=list(SPAN),
                                          v_direction='decreasing_world_y')
    display = path/'display.obj'
    count = write_grid(display, 5, 2)
    asset['display_mesh'] = dict(mesh=display.name, sha256=digest(display), triangles=count)
    point_visual_at(path, m, display.name)
    return path


def point_visual_at(path, m, name):
    tree = ET.parse(path/'world.sdf')
    tree.find('.//visual/geometry/mesh/uri').text = str((path/name).resolve())
    tree.write(path/'world.sdf', encoding='unicode')
    m['world_sha256'] = digest(path/'world.sdf')
    (path/'manifest.json').write_text(json.dumps(m))


def test_a_declared_display_mesh_is_accepted_and_is_what_gz_shows(scene):
    m = validate(scene/'manifest.json')
    assert m['assets'][0]['display_mesh']['mesh'] == 'display.obj'
    tree = ET.parse(scene/'world.sdf')
    assert tree.findtext('.//visual/geometry/mesh/uri').endswith('display.obj')
    assert m['assets'][0]['mesh'] == 'terrain.obj'          # OptiX input untouched


def test_without_the_field_the_visual_must_still_be_the_optical_mesh(tmp_path):
    """The default path has to behave exactly as before."""
    path = tmp_path/'plain'
    generate(path, BASE, length=10, width=2, margin=0)
    m = json.loads((path/'manifest.json').read_text())
    assert 'display_mesh' not in m['assets'][0]
    tree = ET.parse(path/'world.sdf')
    assert tree.findtext('.//visual/geometry/mesh/uri').endswith('terrain.obj')
    validate(path/'manifest.json')
    write_grid(path/'display.obj', 5, 2)
    point_visual_at(path, m, 'display.obj')
    with pytest.raises(ValueError, match='geometry mismatch'):
        validate(path/'manifest.json')


def test_declaring_it_but_showing_the_optical_mesh_is_rejected(scene):
    m = json.loads((scene/'manifest.json').read_text())
    point_visual_at(scene, m, 'terrain.obj')
    with pytest.raises(ValueError, match='geometry mismatch'):
        validate(scene/'manifest.json')


def test_tampered_display_mesh_is_rejected(scene):
    validate(scene/'manifest.json')
    with (scene/'display.obj').open('a') as f:
        f.write('# changed\n')
    with pytest.raises(ValueError, match='checksum'):
        validate(scene/'manifest.json')


def test_a_cache_hit_still_rejects_a_tampered_display_mesh(scene):
    """The entry stands for bytes, so reuse must not smuggle a changed mesh in."""
    validate(scene/'manifest.json')                          # populate
    obj = scene/'display.obj'
    obj.write_text(obj.read_text().replace('v 10.000000000', 'v 9.000000000'))
    with pytest.raises(ValueError, match='checksum'):
        validate(scene/'manifest.json')


def rehash(scene, build, triangles=20):
    """Rewrite display.obj with build() and re-declare it honestly."""
    m = json.loads((scene/'manifest.json').read_text())
    build(scene/'display.obj')
    m['assets'][0]['display_mesh'].update(sha256=digest(scene/'display.obj'),
                                          triangles=triangles)
    (scene/'manifest.json').write_text(json.dumps(m))


def test_a_smaller_footprint_is_rejected_even_when_rehashed(scene):
    def build(path):
        v = np.array([[0., -1., 0.], [8., -1., 0.], [8., 1., 0.], [0., 1., 0.]])
        uv = (v[:, :2]-ORIGIN)/SPAN
        uv[:, 1] = 1-uv[:, 1]
        path.write_text('\n'.join(
            ['v %.9f %.9f %.9f' % tuple(p) for p in v]
            + ['vt %.9f %.9f' % tuple(t) for t in uv] + ['vn 0 0 1']
            + ['f 1/1/1 2/2/1 3/3/1', 'f 1/1/1 3/3/1 4/4/1'])+'\n')
    rehash(scene, build, triangles=2)
    with pytest.raises(ValueError, match='footprint'):
        validate(scene/'manifest.json')


def test_a_height_outside_the_declared_band_is_rejected_even_when_rehashed(scene):
    rehash(scene, lambda p: write_grid(p, 5, 2, lambda x, y: .02))
    with pytest.raises(ValueError, match='deviation band'):
        validate(scene/'manifest.json')


def test_wrong_uvs_are_rejected_even_when_rehashed(scene):
    def build(path):
        nx, ny = 5, 2
        xs = np.linspace(ORIGIN[0], ORIGIN[0]+SPAN[0], nx+1)
        ys = np.linspace(ORIGIN[1], ORIGIN[1]+SPAN[1], ny+1)
        v = np.array([[x, y, 0.] for x in xs for y in ys])
        uv = (v[:, :2]-ORIGIN)/SPAN                     # V not flipped
        quads = [(i*(ny+1)+j, (i+1)*(ny+1)+j, (i+1)*(ny+1)+j+1, i*(ny+1)+j+1)
                 for i in range(nx) for j in range(ny)]
        faces = [t for a, b, c, d in quads for t in ((a, b, c), (a, c, d))]
        path.write_text('\n'.join(
            ['v %.9f %.9f %.9f' % tuple(p) for p in v]
            + ['vt %.9f %.9f' % tuple(t) for t in uv] + ['vn 0 0 1']
            + ['f %d/%d/1 %d/%d/1 %d/%d/1' % (a+1, a+1, b+1, b+1, c+1, c+1)
               for a, b, c in faces])+'\n')
    rehash(scene, build)
    with pytest.raises(ValueError, match='UV'):
        validate(scene/'manifest.json')


def test_a_display_mesh_outside_the_scene_directory_is_rejected(scene, tmp_path):
    outside = tmp_path/'elsewhere.obj'
    write_grid(outside, 5, 2)
    m = json.loads((scene/'manifest.json').read_text())
    m['assets'][0]['display_mesh'] = dict(mesh='../elsewhere.obj', sha256=digest(outside),
                                          triangles=20)
    point_visual_at(scene, m, '../elsewhere.obj')
    with pytest.raises(ValueError, match='beside the manifest|checksum'):
        validate(scene/'manifest.json')


def test_the_light_mesh_really_is_lighter(scene):
    """Guards the point of the exercise, not just its correctness."""
    from agv_linescan.obj_arrays import read_obj
    optical = len(read_obj(scene/'terrain.obj')[1])
    display = len(read_obj(scene/'display.obj')[1])
    assert display < optical


def test_a_wrong_triangle_count_is_rejected(scene):
    """The checker that reports the split must not be able to report a lie."""
    m = json.loads((scene/'manifest.json').read_text())
    m['assets'][0]['display_mesh']['triangles'] += 1
    (scene/'manifest.json').write_text(json.dumps(m))
    with pytest.raises(ValueError, match='triangle count'):
        validate(scene/'manifest.json')
