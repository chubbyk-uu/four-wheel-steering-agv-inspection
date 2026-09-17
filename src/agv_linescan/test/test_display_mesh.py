"""A separate GZ display mesh may be lighter, but not a different road.

The contract was written so the GZ visual and the OptiX reader are one file.
Letting them differ means the validator has to check the light mesh against
something, and the only thing available that has itself been verified is the
collision proxy: validate_layered_proxy pins it vertex for vertex to the
heightfield the optical mesh is built from, and the light mesh is that proxy
with UVs added. Bounding footprint and height range instead, as this first
did, bounds nothing -- one triangle plus the unused corner vertices, or the
whole road flattened onto its reference plane, passes both.
"""
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest

from agv_linescan.heightfield import Heightfield
from agv_linescan.shared_scene import digest, generate, validate

BASE = Path(__file__).resolve().parents[2]/'agv_bringup/worlds/flat.sdf'
ORIGIN = (0., -1.)
SPAN = (10., 2.)
LO, HI = (0., -1.), (10., 1.)


def write_obj(path, vertices, faces, uv=None):
    lines = ['v %.9f %.9f %.9f' % tuple(p) for p in vertices]
    if uv is not None:
        lines += ['vt %.9f %.9f' % tuple(t) for t in uv]
    lines += ['vn 0 0 1']
    lines += [('f %d/%d/1 %d/%d/1 %d/%d/1' % (a+1, a+1, b+1, b+1, c+1, c+1)) if uv is not None
              else ('f %d//1 %d//1 %d//1' % (a+1, b+1, c+1)) for a, b, c in faces]
    path.write_text('\n'.join(lines)+'\n')
    return len(faces)


def projected_uv(vertices):
    uv = (vertices[:, :2]-ORIGIN)/SPAN
    uv[:, 1] = 1-uv[:, 1]                       # decreasing_world_y
    return uv


def fine_grid():
    """Twice the heightfield's resolution, so the light mesh is really lighter."""
    xs = np.linspace(LO[0], HI[0], 5)
    ys = np.linspace(LO[1], HI[1], 5)
    v = np.array([[x, y, 0.] for x in xs for y in ys])
    n = len(ys)
    quads = [(i*n+j, (i+1)*n+j, (i+1)*n+j+1, i*n+j+1)
             for i in range(len(xs)-1) for j in range(n-1)]
    return v, np.array([t for a, b, c, d in quads for t in ((a, b, c), (a, c, d))])


@pytest.fixture
def scene(tmp_path):
    """A ground asset with a layered proxy and the light display mesh built on it.

    The heightfield varies along x alone and the reference grid carries a node
    on every one of its breakpoints, so the optical surface interpolates it
    exactly and the proxy's own sampling budget is not what is under test here.
    """
    path = tmp_path/'scene'
    generate(path, BASE, length=10, width=2, margin=0)
    m = json.loads((path/'manifest.json').read_text())
    asset = m['assets'][0]

    (path/'field.json').write_text(json.dumps(dict(
        schema='agv.reference_heightfield.v1', x=[0., 5., 10.], y=[-1., 1.],
        z=[[0., .001, 0.], [0., .001, 0.]])))
    field = Heightfield(path/'field.json')

    reference, faces = fine_grid()
    write_obj(path/'reference.obj', reference, faces)
    optical = reference.copy()
    optical[:, 2] += field.sample(optical[:, :2])
    write_obj(path/'terrain.obj', optical, faces, projected_uv(optical))
    contact, contact_faces = field.mesh(LO, HI)
    write_obj(path/'contact.obj', contact, contact_faces)
    write_obj(path/'display.obj', contact, contact_faces, projected_uv(contact))

    def entry(name):
        return dict(mesh=name, sha256=digest(path/name))
    asset.update(sha256=digest(path/'terrain.obj'), triangles=len(faces),
                 display_uv_projection=dict(origin_xy_m=list(ORIGIN), span_xy_m=list(SPAN),
                                            v_direction='decreasing_world_y'),
                 collision_proxy=dict(method='layered_heightfield_shallow_v1',
                                      triangles=len(contact_faces),
                                      reference_surface=entry('reference.obj'),
                                      heightfield=entry('field.json'),
                                      reference_plane_z_m=0., max_surface_deviation_m=.002,
                                      **entry('contact.obj')),
                 display_mesh=dict(triangles=len(contact_faces), **entry('display.obj')))
    point_geometry_at(path, m, 'display.obj', 'contact.obj')
    return path


def point_geometry_at(path, m, visual, collision='contact.obj'):
    tree = ET.parse(path/'world.sdf')
    tree.find('.//visual/geometry/mesh/uri').text = str((path/visual).resolve())
    tree.find('.//collision/geometry/mesh/uri').text = str((path/collision).resolve())
    tree.write(path/'world.sdf', encoding='unicode')
    m['world_sha256'] = digest(path/'world.sdf')
    (path/'manifest.json').write_text(json.dumps(m))


def redeclare(scene, **changes):
    """Rewrite display.obj honestly: whatever it now is, say so in the manifest."""
    m = json.loads((scene/'manifest.json').read_text())
    m['assets'][0]['display_mesh'].update(sha256=digest(scene/'display.obj'), **changes)
    (scene/'manifest.json').write_text(json.dumps(m))


def test_a_declared_display_mesh_is_accepted_and_is_what_gz_shows(scene):
    m = validate(scene/'manifest.json')
    assert m['assets'][0]['display_mesh']['mesh'] == 'display.obj'
    tree = ET.parse(scene/'world.sdf')
    assert tree.findtext('.//visual/geometry/mesh/uri').endswith('display.obj')
    assert m['assets'][0]['mesh'] == 'terrain.obj'          # OptiX input untouched


def test_the_light_mesh_really_is_lighter(scene):
    """Guards the point of the exercise, not just its correctness."""
    from agv_linescan.obj_arrays import read_obj
    assert len(read_obj(scene/'display.obj')[1]) < len(read_obj(scene/'terrain.obj')[1])


def test_without_the_field_the_visual_must_still_be_the_optical_mesh(tmp_path):
    """The default path has to behave exactly as before."""
    path = tmp_path/'plain'
    generate(path, BASE, length=10, width=2, margin=0)
    m = json.loads((path/'manifest.json').read_text())
    assert 'display_mesh' not in m['assets'][0]
    tree = ET.parse(path/'world.sdf')
    assert tree.findtext('.//visual/geometry/mesh/uri').endswith('terrain.obj')
    validate(path/'manifest.json')
    v, f = fine_grid()
    write_obj(path/'display.obj', v, f, projected_uv(v))
    tree.find('.//visual/geometry/mesh/uri').text = str((path/'display.obj').resolve())
    tree.write(path/'world.sdf', encoding='unicode')
    m['world_sha256'] = digest(path/'world.sdf')
    (path/'manifest.json').write_text(json.dumps(m))
    with pytest.raises(ValueError, match='geometry mismatch'):
        validate(path/'manifest.json')


def test_declaring_it_but_showing_the_optical_mesh_is_rejected(scene):
    m = json.loads((scene/'manifest.json').read_text())
    point_geometry_at(scene, m, 'terrain.obj')
    with pytest.raises(ValueError, match='geometry mismatch'):
        validate(scene/'manifest.json')


# --- integrity: the declared bytes, on every path -------------------------

def test_a_tampered_display_mesh_is_rejected_on_a_cold_cache(scene):
    """The first launch on a new machine is the one that has no entry to lean on."""
    with (scene/'display.obj').open('a') as f:
        f.write('# changed\n')
    with pytest.raises(ValueError, match='checksum'):
        validate(scene/'manifest.json')


def test_a_tampered_display_mesh_is_rejected_with_the_cache_disabled(scene, monkeypatch):
    monkeypatch.setenv('AGV_DERIVED_CACHE', 'off')
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


def test_a_display_mesh_outside_the_scene_directory_is_rejected(scene, tmp_path):
    outside = tmp_path/'elsewhere.obj'
    v, f = fine_grid()
    write_obj(outside, v, f, projected_uv(v))
    m = json.loads((scene/'manifest.json').read_text())
    m['assets'][0]['display_mesh'] = dict(mesh='../elsewhere.obj', sha256=digest(outside),
                                          triangles=len(f))
    point_geometry_at(scene, m, '../elsewhere.obj')
    with pytest.raises(ValueError, match='beside the manifest|checksum'):
        validate(scene/'manifest.json')


# --- geometry: the checked surface, not merely its bounding box -----------

def test_a_mesh_keeping_only_one_triangle_is_rejected_even_when_rehashed(scene):
    """Unused vertices hold the bounding box and the height range steady."""
    from agv_linescan.obj_arrays import read_obj
    v, f, uv = read_obj(scene/'display.obj', with_uv=True)
    write_obj(scene/'display.obj', v, f[:1], uv)
    redeclare(scene, triangles=1)
    with pytest.raises(ValueError, match='not the checked collision surface'):
        validate(scene/'manifest.json')


def test_a_flattened_road_is_rejected_even_when_rehashed(scene):
    """Every height inside the declared band, and still the wrong surface."""
    from agv_linescan.obj_arrays import read_obj
    v, f, uv = read_obj(scene/'display.obj', with_uv=True)
    v[:, 2] = 0.
    write_obj(scene/'display.obj', v, f, uv)
    redeclare(scene, triangles=len(f))
    with pytest.raises(ValueError, match='not the checked collision surface'):
        validate(scene/'manifest.json')


def test_a_retriangulated_surface_is_rejected_even_when_rehashed(scene):
    """The same points, the same footprint, a different topology."""
    from agv_linescan.obj_arrays import read_obj
    v, f, uv = read_obj(scene/'display.obj', with_uv=True)
    write_obj(scene/'display.obj', v, f[:, ::-1], uv)
    redeclare(scene, triangles=len(f))
    with pytest.raises(ValueError, match='not the checked collision surface'):
        validate(scene/'manifest.json')


def test_a_smaller_footprint_is_rejected_even_when_rehashed(scene):
    v = np.array([[0., -1., 0.], [8., -1., 0.], [8., 1., 0.], [0., 1., 0.]])
    write_obj(scene/'display.obj', v, np.array([[0, 1, 2], [0, 2, 3]]), projected_uv(v))
    redeclare(scene, triangles=2)
    with pytest.raises(ValueError, match='not the checked collision surface'):
        validate(scene/'manifest.json')


def test_wrong_uvs_are_rejected_even_when_rehashed(scene):
    """The one degree of freedom the proxy does not pin down."""
    from agv_linescan.obj_arrays import read_obj
    v, f, _ = read_obj(scene/'display.obj', with_uv=True)
    uv = (v[:, :2]-ORIGIN)/SPAN                              # V not flipped
    write_obj(scene/'display.obj', v, f, uv)
    redeclare(scene, triangles=len(f))
    with pytest.raises(ValueError, match='UV'):
        validate(scene/'manifest.json')


def test_a_wrong_triangle_count_is_rejected(scene):
    """The checker that reports the split must not be able to report a lie."""
    m = json.loads((scene/'manifest.json').read_text())
    m['assets'][0]['display_mesh']['triangles'] += 1
    (scene/'manifest.json').write_text(json.dumps(m))
    with pytest.raises(ValueError, match='triangle count'):
        validate(scene/'manifest.json')


# --- the reference the check needs must exist -----------------------------

def test_a_display_mesh_without_a_collision_proxy_is_rejected(scene):
    m = json.loads((scene/'manifest.json').read_text())
    del m['assets'][0]['collision_proxy']
    point_geometry_at(scene, m, 'display.obj', 'terrain.obj')
    with pytest.raises(ValueError, match='needs a collision proxy'):
        validate(scene/'manifest.json')


def test_a_display_mesh_on_a_flat_rectangle_proxy_is_rejected(tmp_path):
    """That proxy is four vertices of plane; inheriting it would show a quad."""
    path = tmp_path/'rect'
    generate(path, BASE, length=10, width=2, margin=0)
    m = json.loads((path/'manifest.json').read_text())
    asset = m['assets'][0]
    quad = np.array([[0., -1., 0.], [10., -1., 0.], [10., 1., 0.], [0., 1., 0.]])
    faces = np.array([[0, 1, 2], [0, 2, 3]])
    write_obj(path/'contact.obj', quad, faces)
    write_obj(path/'display.obj', quad, faces, projected_uv(quad))
    asset.update(display_uv_projection=dict(origin_xy_m=list(ORIGIN), span_xy_m=list(SPAN),
                                            v_direction='decreasing_world_y'),
                 collision_proxy=dict(method='shallow_horizontal_rectangle_v1',
                                      mesh='contact.obj', sha256=digest(path/'contact.obj'),
                                      triangles=2, plane_z_m=0., max_surface_deviation_m=.002),
                 display_mesh=dict(mesh='display.obj', sha256=digest(path/'display.obj'),
                                   triangles=2))
    point_geometry_at(path, m, 'display.obj')
    with pytest.raises(ValueError, match='layered_heightfield_shallow_v1'):
        validate(path/'manifest.json')


def test_the_shipped_generator_produces_what_the_validator_now_demands(scene):
    """Tightening the contract is only safe if the tool that feeds it agrees.

    build_light_display_assets writes the light mesh from the proxy, which is
    why requiring them to be equal is a checkable contract and not a new
    burden. If that ever stops being true this fails here rather than on a
    machine restoring the road.
    """
    import sys
    tools = str(Path(__file__).resolve().parents[3]/'tools')
    sys.path.insert(0, tools)
    try:
        from build_light_display_assets import display_mesh
    finally:
        sys.path.remove(tools)
    m = json.loads((scene/'manifest.json').read_text())
    asset = m['assets'][0]
    (scene/'display.obj').unlink()
    count, path = display_mesh(scene, asset, m, scene)
    assert path.name == 'display_'+asset['name']+'.obj'
    asset['display_mesh'] = dict(mesh=path.name, sha256=digest(path), triangles=count)
    point_geometry_at(scene, m, path.name)
    assert validate(scene/'manifest.json')['assets'][0]['display_mesh']['triangles'] == count
