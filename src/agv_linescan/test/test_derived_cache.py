"""Reusing derived work must not reuse a verdict about bytes that changed."""
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from agv_linescan import derived_cache, shared_scene
from agv_linescan.shared_scene import checked_geometry_files, digest, generate, validate

BASE = Path(__file__).resolve().parents[2]/'agv_bringup/worlds/flat.sdf'


@pytest.fixture
def proxy_bundle(tmp_path):
    """A scene whose one asset carries a checked collision proxy."""
    path = tmp_path/'proxy'
    generate(path, BASE, length=10, width=2, margin=0)
    mp = path/'manifest.json'
    m = json.loads(mp.read_text())
    obj = path/'contact.obj'
    obj.write_text('v 0 -1 0\nv 10 -1 0\nv 10 1 0\nv 0 1 0\nvn 0 0 1\n'
                   'f 1//1 2//1 3//1\nf 1//1 3//1 4//1\n')
    m['assets'][0]['collision_proxy'] = dict(
        method='shallow_horizontal_rectangle_v1', mesh=obj.name, sha256=digest(obj),
        plane_z_m=0, max_surface_deviation_m=.002)
    tree = ET.parse(path/'world.sdf')
    tree.find('.//collision/geometry/mesh/uri').text = str(obj)
    tree.write(path/'world.sdf', encoding='unicode')
    m['world_sha256'] = digest(path/'world.sdf')
    mp.write_text(json.dumps(m))
    return path


def test_second_validation_agrees_with_the_first(proxy_bundle):
    first = validate(proxy_bundle/'manifest.json')
    assert validate(proxy_bundle/'manifest.json') == first


def test_a_hit_still_rejects_a_tampered_proxy_file(proxy_bundle):
    """The whole point: the entry stands for bytes, not for a past verdict."""
    validate(proxy_bundle/'manifest.json')                    # populate
    obj = proxy_bundle/'contact.obj'
    obj.write_text(obj.read_text().replace('v 10 1 0', 'v 9 1 0'))
    with pytest.raises(ValueError, match='checksum'):
        validate(proxy_bundle/'manifest.json')


def test_a_hit_still_rejects_a_tampered_optical_mesh(proxy_bundle):
    validate(proxy_bundle/'manifest.json')
    mesh = proxy_bundle/'terrain.obj'
    with mesh.open('a') as f:
        f.write('# changed\n')
    with pytest.raises(ValueError, match='checksum'):
        validate(proxy_bundle/'manifest.json')


def _hashed_entries(node):
    """Every {mesh, sha256} pair anywhere in an asset's geometry declaration."""
    found = []
    if isinstance(node, dict):
        if 'mesh' in node and 'sha256' in node:
            found.append(node['mesh'])
        for value in node.values():
            found += _hashed_entries(value)
    return found


@pytest.mark.parametrize('asset', [
    dict(collision_proxy=dict(method='shallow_horizontal_rectangle_v1',
                              mesh='contact.obj', sha256='a')),
    dict(collision_proxy=dict(method='layered_heightfield_shallow_v1', mesh='c.obj', sha256='a',
                              reference_surface=dict(mesh='r.obj', sha256='b'),
                              heightfield=dict(mesh='h.json', sha256='c'))),
    dict(display_mesh=dict(mesh='d.obj', sha256='e')),
    dict(collision_proxy=dict(method='shallow_horizontal_rectangle_v1',
                              mesh='contact.obj', sha256='a'),
         display_mesh=dict(mesh='d.obj', sha256='e')),
])
def test_the_hit_path_lists_every_hashed_geometry_file(tmp_path, asset):
    """A field added without updating the list would let a hit skip its bytes."""
    listed = {p.name for p, _ in checked_geometry_files(tmp_path, asset)}
    assert listed == set(_hashed_entries(asset))


def test_a_plain_asset_has_nothing_extra_to_verify(tmp_path):
    assert checked_geometry_files(tmp_path, dict(mesh='terrain.obj', sha256='a')) == []


def test_a_changed_check_invalidates_every_entry(proxy_bundle, monkeypatch, tmp_path):
    """The fingerprint covers the checking code, not just the assets."""
    before = derived_cache.code_fingerprint(shared_scene)
    edited = tmp_path/'shared_scene_copy.py'
    edited.write_text(Path(shared_scene.__file__).read_text()+'\n# a changed check\n')
    monkeypatch.setattr(shared_scene, '__file__', str(edited))
    derived_cache._fingerprints.clear()
    assert derived_cache.code_fingerprint(shared_scene) != before


def test_disabling_the_cache_skips_the_store(proxy_bundle, monkeypatch):
    # generate() validates the scene it writes, so clear what the fixture left.
    shutil.rmtree(derived_cache.root(), ignore_errors=True)
    monkeypatch.setenv('AGV_DERIVED_CACHE', 'off')
    validate(proxy_bundle/'manifest.json')
    assert not list(derived_cache.root().rglob('value.json'))


def test_a_partial_directory_entry_is_not_served(tmp_path):
    """An interrupted write leaves no marker, so it reads as a miss."""
    source = tmp_path/'src'
    source.mkdir()
    (source/'a.txt').write_text('payload')
    key = derived_cache.key('unit', 1)
    derived_cache.write_directory('unit', key, source)
    slot = derived_cache.root()/derived_cache.LAYOUT/'unit'/key
    assert derived_cache.read_directory('unit', key, tmp_path/'hit')
    assert (tmp_path/'hit'/'a.txt').read_text() == 'payload'
    assert not (tmp_path/'hit'/'.complete').exists()
    (slot/'.complete').unlink()
    assert not derived_cache.read_directory('unit', key, tmp_path/'miss')


def test_publishing_leaves_no_staging_directory_behind(tmp_path):
    """The staging area is cleaned up, not moved into place and then hunted for."""
    source = tmp_path/'src'
    source.mkdir()
    (source/'a.txt').write_text('payload')
    derived_cache.write_value('unit', derived_cache.key('unit', 3), {'ok': True})
    derived_cache.write_directory('unit', derived_cache.key('unit', 4), source)
    left = [p.name for p in (derived_cache.root()/derived_cache.LAYOUT/'unit').iterdir()]
    assert sorted(left) == sorted([derived_cache.key('unit', 3), derived_cache.key('unit', 4)])


def test_a_damaged_value_entry_reads_as_a_miss(tmp_path):
    key = derived_cache.key('unit', 2)
    derived_cache.write_value('unit', key, {'ok': True})
    assert derived_cache.read_value('unit', key) == {'ok': True}
    (derived_cache.root()/derived_cache.LAYOUT/'unit'/key/'value.json').write_text('{ not json')
    assert derived_cache.read_value('unit', key) is None


def test_keys_separate_different_content():
    a = derived_cache.key('asset_geometry', {'sha256': 'aa'}, None, None, 'fp')
    b = derived_cache.key('asset_geometry', {'sha256': 'bb'}, None, None, 'fp')
    c = derived_cache.key('asset_geometry', {'sha256': 'aa'}, None, None, 'other')
    assert len({a, b, c}) == 3


def test_directory_hit_never_removes_existing_output(tmp_path):
    source=tmp_path/'source';source.mkdir();(source/'a').write_text('cache')
    derived_cache.write_directory('unit','existing',source)
    output=tmp_path/'owned';output.mkdir();(output/'evidence').write_text('keep')
    with pytest.raises(FileExistsError):
        derived_cache.read_directory('unit','existing',output)
    assert (output/'evidence').read_text()=='keep'


@pytest.mark.parametrize('damage',['remove','edit','metadata'])
def test_corrupt_directory_is_miss_and_can_be_repaired(tmp_path,damage):
    source=tmp_path/'source';source.mkdir();(source/'a').write_text('correct')
    derived_cache.write_directory('unit','corrupt',source)
    slot=derived_cache.root()/derived_cache.LAYOUT/'unit'/'corrupt'
    if damage=='remove':(slot/'a').unlink()
    elif damage=='edit':(slot/'a').write_text('wrong')
    else:(slot/'.complete').write_text('[]')
    assert not derived_cache.read_directory('unit','corrupt',tmp_path/'out')
    assert not (tmp_path/'out').exists()
    derived_cache.write_directory('unit','corrupt',source)
    assert derived_cache.read_directory('unit','corrupt',tmp_path/'out')
    assert (tmp_path/'out'/'a').read_text()=='correct'


@pytest.mark.parametrize('stored',[[],{}, {'collision':'other.obj'}])
def test_bad_geometry_cache_value_rederives(proxy_bundle,monkeypatch,stored):
    monkeypatch.setattr(derived_cache,'read_value',lambda *a:stored)
    assert validate(proxy_bundle/'manifest.json')
