from pathlib import Path
import json
import xml.etree.ElementTree as ET
import pytest
from agv_linescan.shared_scene import generate, validate, digest

BASE=Path(__file__).resolve().parents[2]/'agv_bringup/worlds/flat.sdf'

@pytest.fixture
def bundle(tmp_path):
    path=tmp_path/'scene'
    generate(path,BASE,length=10,width=2,profile="optical_stress")
    return path

def test_asset_identity_and_no_hidden_flat_ground(bundle):
    m=validate(bundle/'manifest.json')
    assert sum(a['triangles'] for a in m['assets'])==4024
    assert len(m['assets'])==3
    assert not ET.parse(bundle/'world.sdf').findall('.//plane')
    for a in m['assets']:
        lines=(bundle/a['mesh']).read_text().splitlines()
        assert sum(line.startswith('vn ') for line in lines)==a['triangles']
        assert all(all(len(t.split('/'))==3 for t in line.split()[1:]) for line in lines if line.startswith('f '))

def test_changed_mesh_rejected(bundle):
    with (bundle/'terrain.obj').open('a') as f:f.write('# changed\n')
    with pytest.raises(ValueError,match='checksum'):validate(bundle/'manifest.json')

def test_collision_offset_rejected_even_with_updated_world_hash(bundle):
    path=bundle/'world.sdf';tree=ET.parse(path)
    ET.SubElement(tree.find('.//collision'),'pose').text='0 0 .02 0 0 0'
    tree.write(path,encoding='unicode')
    mp=bundle/'manifest.json';m=json.loads(mp.read_text());m['world_sha256']=digest(path);mp.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='transform'):validate(mp)

def test_collision_scale_rejected_even_with_updated_world_hash(bundle):
    path=bundle/'world.sdf';tree=ET.parse(path)
    tree.find('.//collision/geometry/mesh/scale').text='1 1 2';tree.write(path,encoding='unicode')
    mp=bundle/'manifest.json';m=json.loads(mp.read_text());m['world_sha256']=digest(path);mp.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='geometry mismatch'):validate(mp)


def test_default_driving_scene_is_flat_without_obstacles(tmp_path):
    path=tmp_path/'flat'
    generate(path,BASE,length=10,width=2)
    m=validate(path/'manifest.json')
    assert m['profile']=='flat'
    assert [a['name'] for a in m['assets']]==['terrain']
    vertices=[list(map(float,line.split()[1:])) for line in (path/'terrain.obj').read_text().splitlines() if line.startswith('v ')]
    assert vertices and all(v[2]==0 for v in vertices)
    assert len(ET.parse(path/'world.sdf').findall('.//model'))==1


def test_calibration_board_normals_and_material_contract(tmp_path):
    from agv_linescan.shared_scene import calibration_scene
    path=calibration_scene(tmp_path/'board',BASE,.025)
    m=validate(path)
    for asset in m['assets']:
        text=(path.parent/asset['mesh']).read_text()
        assert text.startswith('vn 0 0 1\n')  # DART's ODE mesh importer needs normals.
        assert all(float(line.split()[3])==0 for line in text.splitlines() if line.startswith('v '))
    m['assets'][0]['linear_reflectance']=.123
    path.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='material mismatch'):validate(path)


def test_flat_margin_extends_ground_not_inspection_region(tmp_path):
    path=tmp_path/'extended'
    generate(path,BASE,length=10,width=2,margin=2)
    m=validate(path/'manifest.json')
    assert m['inspection_bounds_xy_m']==[0,10,-1,1]
    assert m['ground_bounds_xy_m']==[-2,12,-3,3]
    vertices=[];area=0
    for line in (path/'terrain.obj').read_text().splitlines():
        if line.startswith('v '):
            vertices.append(list(map(float,line.split()[1:])))
        if line.startswith('f '):
            a,b,c=[vertices[int(t.split('/')[0])-1] for t in line.split()[1:]]
            signed=((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/2
            assert signed>0
            area+=signed
    assert area==pytest.approx(14*6)
    assert min(v[0] for v in vertices)==-2
    assert max(v[1] for v in vertices)==3
    assert m['length_m']==10 and m['width_m']==2


@pytest.mark.parametrize('margin',[-1,float('nan'),float('inf'),11])
def test_invalid_ground_margin(tmp_path,margin):
    with pytest.raises(ValueError,match='margin'):
        generate(tmp_path/'bad',BASE,margin=margin)


def test_distorted_footprints_at_all_roi_edges_hit_extended_ground(tmp_path):
    import numpy as np
    path=tmp_path/'edges'
    generate(path,BASE,length=10,width=2,margin=2)
    vertices=[];faces=[]
    for line in (path/'terrain.obj').read_text().splitlines():
        if line.startswith('v '):vertices.append(list(map(float,line.split()[1:])))
        if line.startswith('f '):faces.append([int(t.split('/')[0])-1 for t in line.split()[1:]])
    triangles=np.array(vertices)[faces][:,:,:2]
    # Raw q+.04q^3 footprint including both outermost pixels, rotated at ROI corners/edges.
    for center in ([0,-1],[0,1],[10,-1],[10,1],[5,-1],[5,1],[0,0],[10,0]):
        for angle in np.arange(8)*np.pi/4:
            for q in (-1.,0.,1.):
                point=np.array(center)+.6*(q+.04*q**3)*np.array([np.cos(angle),np.sin(angle)])
                relative=point-triangles
                edges=np.roll(triangles,-1,axis=1)-triangles
                cross=edges[:,:,0]*relative[:,:,1]-edges[:,:,1]*relative[:,:,0]
                assert np.any(np.all(cross>=-1e-8,axis=1))
