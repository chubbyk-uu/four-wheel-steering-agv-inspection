"""Independent small meshes exercise the shared rough-road collision contract."""
import json
from pathlib import Path

import numpy as np
import pytest

from agv_linescan.collision_proxy import validate_proxy
from agv_linescan.heightfield import Heightfield
from agv_linescan.shared_scene import digest


def test_production_roughness_does_not_flatten_negative_buffer():
    import sys
    tools = Path(__file__).resolve().parents[3] / 'tools'
    sys.path.insert(0, str(tools))
    try:
        from probe_rough_road import height
        x = np.array([-8., -3., 0., 2.])
        y = np.array([-.47, .47, -.47, .47])
        assert np.array_equal(height(x, y), np.zeros(4))
        full = height(x, y, taper=False)
        assert np.all(np.isfinite(full))
        assert np.any(np.abs(full) > 1e-6)
        np.testing.assert_array_equal(full, height(x, y, taper=False))
    finally:
        sys.path.remove(str(tools))


def field_file(path):
    path.write_text(json.dumps(dict(schema='agv.reference_heightfield.v1',
        x=[0, .5, 1], y=[0, .5, 1],
        z=[[0, 0, 0], [0, .001, 0], [0, 0, 0]])))
    return Heightfield(path)


def write_mesh(path, vertices, faces):
    with path.open('w') as stream:
        np.savetxt(stream, vertices, fmt='v %.9f %.9f %.9f')
        np.savetxt(stream, faces + 1, fmt='f %d %d %d')


def test_piecewise_triangle_interpolation(tmp_path):
    field = field_file(tmp_path / 'field.json')
    # First cell's diagonal is 00->11: linear triangle interpolation, not bilinear.
    np.testing.assert_allclose(field.sample([[.25, .25], [.25, .125], [1, 1]]),
                               [.0005, .00025, 0], atol=1e-15)
    with pytest.raises(ValueError, match='bounds'):
        field.sample([[-.001, 0]])
    with pytest.raises(ValueError, match='coordinates'):
        field.sample([[np.nan, 0]])


@pytest.fixture
def layered(tmp_path):
    field = field_file(tmp_path / 'field.json')
    v, f = field.mesh([0, 0], [1, 1])
    reference = v.copy()
    reference[:, 2] = 0
    write_mesh(tmp_path / 'reference.obj', reference, f)
    write_mesh(tmp_path / 'optical.obj', v, f)
    write_mesh(tmp_path / 'contact.obj', v, f)
    def entry(name):
        return dict(mesh=name, sha256=digest(tmp_path / name))
    proxy = dict(method='layered_heightfield_shallow_v1',
        **entry('contact.obj'), reference_surface=entry('reference.obj'),
        heightfield=entry('field.json'), reference_plane_z_m=0,
        max_surface_deviation_m=.002)
    return tmp_path, dict(mesh='optical.obj', collision_proxy=proxy), v, f


def test_accept_shared_geometry(layered):
    root, asset, _, _ = layered
    assert validate_proxy(root, asset) == root / 'contact.obj'


@pytest.mark.parametrize('target', ['optical', 'contact'])
def test_reject_flattening_even_with_new_hash(layered, target):
    root, asset, v, f = layered
    v[:, 2] = 0
    path = root / (target + '.obj')
    write_mesh(path, v, f)
    if target == 'contact':
        asset['collision_proxy']['sha256'] = digest(path)
    with pytest.raises(ValueError, match='optical surface|collision mesh'):
        validate_proxy(root, asset)


@pytest.mark.parametrize('target', ['reference.obj', 'field.json', 'contact.obj'])
def test_reject_changed_sources(layered, target):
    root, asset, _, _ = layered
    with (root / target).open('a') as stream:
        stream.write('\n')
    with pytest.raises(ValueError, match='checksum'):
        validate_proxy(root, asset)


def test_reject_overlarge_field(tmp_path):
    path = tmp_path / 'field.json'
    field_file(path)
    data = json.loads(path.read_text())
    data['z'][1][1] = .0031
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='bounded'):
        Heightfield(path)


def test_reject_large_face_interpolation_discrepancy(layered):
    root, asset, _, _ = layered
    # A quad's corner heights miss the centre bump entirely, although every
    # optical vertex satisfies the shared heightfield and both hashes are valid.
    v = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]], float)
    f = np.array([[0, 1, 2], [0, 2, 3]])
    for name in ('reference.obj', 'optical.obj'):
        write_mesh(root / name, v, f)
    asset['collision_proxy']['reference_surface']['sha256'] = digest(root / 'reference.obj')
    with pytest.raises(ValueError, match='interpolation discrepancy'):
        validate_proxy(root, asset)


def test_streaming_bounds_include_bumps_and_groove_depth():
    from agv_linescan.shared_scene import validate_material_height_bounds
    vertices = np.array([[0, 0, -.004], [1, 0, .003]])
    material = dict(schema='agv.ground_material.recipe.v1', height_bounds_m=[-.003001, .000001])
    with pytest.raises(ValueError, match='prefetch height bounds'):
        validate_material_height_bounds(material, vertices)
    material['height_bounds_m'] = [-.004001, .003001]
    validate_material_height_bounds(material, vertices)
