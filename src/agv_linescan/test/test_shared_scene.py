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

@pytest.fixture
def textured_bundle(tmp_path):
    path=tmp_path/'pbr';generate(path,BASE,length=10,width=2,margin=0)
    mp=path/'manifest.json';m=json.loads(mp.read_text())
    for name,data in [('color.raw',bytes([128]*4)),('normal.raw',bytes([128]*8))]:(path/name).write_bytes(data)
    m['ground_material']=dict(schema='agv.ground_material.xy.v1',width=2,height=2,
        origin_xy_m=[0,0],span_xy_m=[1,1],roughness=.60,
        color=dict(file='color.raw',sha256=digest(path/'color.raw')),
        normal=dict(file='normal.raw',sha256=digest(path/'normal.raw')))
    m['assets'][0]['material']='ground'
    m['display_uv_projection']=dict(origin_xy_m=[0,0],span_xy_m=[.1,.1])
    tree=ET.parse(path/'world.sdf');visual=tree.find('.//visual')
    metal=ET.SubElement(ET.SubElement(ET.SubElement(visual,'material'),'pbr'),'metal')
    for tag,text in [('albedo_map',str(path/'grid.png')),('normal_map',str(path/'grid.png')),('roughness','0.60')]:ET.SubElement(metal,tag).text=text
    tree.write(path/'world.sdf',encoding='unicode');m['world_sha256']=digest(path/'world.sdf');mp.write_text(json.dumps(m))
    validate(mp)
    return path

def test_pbr_raw_integrity_and_display_roughness(textured_bundle):
    p=textured_bundle;mp=p/'manifest.json'
    (p/'normal.raw').write_bytes(bytes([100]*8))
    with pytest.raises(ValueError,match='integrity'):validate(mp)
    m=json.loads(mp.read_text());m['ground_material']['normal']['sha256']=digest(p/'normal.raw')
    tree=ET.parse(p/'world.sdf');tree.find('.//roughness').text='.9';tree.write(p/'world.sdf',encoding='unicode')
    m['world_sha256']=digest(p/'world.sdf');mp.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='roughness mismatch'):validate(mp)

def test_rehashed_uv_coordinate_change_rejected(textured_bundle):
    p=textured_bundle;mp=p/'manifest.json';obj=p/'terrain.obj'
    s=obj.read_text();lines=s.splitlines();i=next(i for i,v in enumerate(lines) if v.startswith('vt '));lines[i]='vt 9 9'
    obj.write_text('\n'.join(lines)+'\n');m=json.loads(mp.read_text());m['assets'][0]['sha256']=digest(obj);mp.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='UV/world-coordinate'):validate(mp)


def test_display_v_direction_is_explicit_and_checked(textured_bundle):
    p=textured_bundle;mp=p/'manifest.json';m=json.loads(mp.read_text());obj=p/'terrain.obj'
    m['display_uv_projection']['v_direction']='decreasing_world_y'
    mp.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='UV/world-coordinate'):validate(mp)
    lines=obj.read_text().splitlines()
    for i,line in enumerate(lines):
        if line.startswith('vt '):
            values=line.split();lines[i]=f'vt {values[1]} {1-float(values[2]):.9f}'
    obj.write_text('\n'.join(lines)+'\n');m['assets'][0]['sha256']=digest(obj);mp.write_text(json.dumps(m))
    validate(mp)
    m['display_uv_projection']['v_direction']='unknown';mp.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='V direction'):validate(mp)


@pytest.fixture
def proxy_bundle(tmp_path):
    path=tmp_path/'proxy';generate(path,BASE,length=10,width=2,margin=0)
    mp=path/'manifest.json';m=json.loads(mp.read_text())
    obj=path/'contact.obj'
    obj.write_text('v 0 -1 0\nv 10 -1 0\nv 10 1 0\nv 0 1 0\nvn 0 0 1\nf 1//1 2//1 3//1\nf 1//1 3//1 4//1\n')
    m['assets'][0]['collision_proxy']=dict(method='shallow_horizontal_rectangle_v1',mesh=obj.name,
        sha256=digest(obj),plane_z_m=0,max_surface_deviation_m=.002)
    tree=ET.parse(path/'world.sdf');tree.find('.//collision/geometry/mesh/uri').text=str(obj)
    tree.write(path/'world.sdf',encoding='unicode');m['world_sha256']=digest(path/'world.sdf')
    mp.write_text(json.dumps(m));return path


def test_explicit_collision_proxy_leaves_optical_mesh_unchanged(proxy_bundle):
    p=proxy_bundle;m=validate(p/'manifest.json')
    assert m['assets'][0]['mesh']=='terrain.obj'
    assert m['assets'][0]['collision_proxy']['mesh']=='contact.obj'
    tree=ET.parse(p/'world.sdf')
    assert tree.findtext('.//visual/geometry/mesh/uri')!=tree.findtext('.//collision/geometry/mesh/uri')


@pytest.mark.parametrize('fault',['deep_pit','raised','footprint','loose_bound','undeclared','checksum'])
def test_proxy_cannot_silently_remove_large_geometry_or_change_extent(proxy_bundle,fault):
    p=proxy_bundle;mp=p/'manifest.json';m=json.loads(mp.read_text());a=m['assets'][0]
    if fault in ('deep_pit','raised'):
        obj=p/a['mesh'];s=obj.read_text().splitlines()
        i=next(i for i,line in enumerate(s) if line.startswith('v '))
        xyz=s[i].split();xyz[3]='-.02' if fault=='deep_pit' else '.01';s[i]=' '.join(xyz)
        obj.write_text('\n'.join(s)+'\n');a['sha256']=digest(obj)
    elif fault in ('footprint','checksum'):
        obj=p/'contact.obj';obj.write_text(obj.read_text().replace('v 10 1 0','v 9 1 0'))
        if fault=='footprint': a['collision_proxy']['sha256']=digest(obj)
    elif fault=='loose_bound':a['collision_proxy']['max_surface_deviation_m']=.1
    elif fault=='undeclared':a.pop('collision_proxy')
    mp.write_text(json.dumps(m))
    with pytest.raises(ValueError,match='proxy|geometry mismatch'):validate(mp)


def test_probe_generator_uses_checked_proxy_for_collision(tmp_path):
    import sys
    import numpy as np
    sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
    from generate_textured_scene import write_world
    from fullwidth_road import collision_mesh
    vertices=np.array([[0.,-5,0],[10,-5,0],[10,5,0],[0,5,0]])
    faces=np.array([[0,1,2],[0,2,3]])
    proxy=collision_mesh(tmp_path,'terrain',vertices,faces)
    optical=tmp_path/'terrain.obj'
    optical.write_text('optical geometry is deliberately separate')
    original=optical.read_bytes()
    write_world(tmp_path,optical,tmp_path/proxy['mesh'])
    tree=ET.parse(tmp_path/'world.sdf')
    assert tree.findtext('.//visual/geometry/mesh/uri')==str(optical)
    assert tree.findtext('.//collision/geometry/mesh/uri')==str(tmp_path/proxy['mesh'])
    assert optical.read_bytes()==original and proxy['triangles']==2
    vertices[0,2]=-.02
    with pytest.raises(ValueError):collision_mesh(tmp_path,'deep',vertices,faces)


def test_native_heightmap_is_collision_only_and_checked(tmp_path,proxy_bundle):
    # Exercise the validator contract without making the command-line generator
    # depend on this synthetic fixture's absent reference heightfield.
    p=proxy_bundle;mp=p/'manifest.json';m=json.loads(mp.read_text())
    image=p/'height.png';image.write_bytes(b'heightmap')
    source=p/'height.json';source.write_text('{}')
    tree=ET.parse(p/'world.sdf');world=tree.getroot().find('world')
    physics=world.find('physics');dart=ET.SubElement(physics,'dart');ET.SubElement(dart,'collision_detector').text='ode'
    for model in world.findall('model'):
        for collision in list(model.find('link').findall('collision')):model.find('link').remove(collision)
    model=ET.SubElement(world,'model',name='road_native_heightmap_collision');ET.SubElement(model,'static').text='true'
    link=ET.SubElement(model,'link',name='road');collision=ET.SubElement(link,'collision',name='collision')
    shape=ET.SubElement(ET.SubElement(collision,'geometry'),'heightmap')
    ET.SubElement(shape,'uri').text=str(image);ET.SubElement(shape,'size').text='10 2 .006';ET.SubElement(shape,'pos').text='5 0 -.003'
    tree.write(p/'world.sdf',encoding='unicode');m['world_sha256']=digest(p/'world.sdf')
    m['physics_heightmap']=dict(model_name='road_native_heightmap_collision',image=image.name,sha256=digest(image),
        source_heightfield=source.name,source_sha256=digest(source),encoding='png_uint16_min_to_max_v1',
        size_m=[10.,2.,.006],position_m=[5.,0.,-.003],collision_detector='ode')
    mp.write_text(json.dumps(m));validate(mp)
    image.write_bytes(b'changed')
    with pytest.raises(ValueError,match='heightmap checksum'):validate(mp)


def test_spawn_uses_drivable_apron_and_full_vehicle_envelope():
    from agv_linescan.shared_scene import validate_spawn_position
    scene=dict(length_m=100,width_m=10,inspection_bounds_xy_m=[0,100,-5,5],drivable_bounds_xy_m=[-8,108,-6.5,6.5])
    validate_spawn_position(scene,-3,0)
    validate_spawn_position(scene,105,-4.5)
    for x,y in [(-7,0),(107,0),(50,6),(float('nan'),0)]:
        with pytest.raises(ValueError):validate_spawn_position(scene,x,y)
    scene.pop('drivable_bounds_xy_m')
    with pytest.raises(ValueError):validate_spawn_position(scene,-3,0)
    validate_spawn_position(scene,3,0)
