import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from agv_linescan.robot_scene import export, triangles


def test_fixed_geometry_baked_with_rotation_and_moving_child_separate(tmp_path):
    urdf='''<robot name="fixture"><link name="base"/><link name="fixed">
    <visual><geometry><box size="2 4 6"/></geometry></visual></link>
    <link name="wheel"><visual><geometry><cylinder radius=".1" length=".075"/></geometry></visual></link>
    <joint name="mount" type="fixed"><parent link="base"/><child link="fixed"/>
    <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/></joint>
    <joint name="drive" type="continuous"><parent link="base"/><child link="wheel"/>
    <origin xyz="100 0 0"/></joint></robot>'''
    m=json.loads(export(urdf,tmp_path).read_text())
    assert m['source_sha256']==hashlib.sha256(urdf.encode()).hexdigest()
    assert [g['name'] for g in m['groups']]==['base','wheel']
    box=np.array(m['groups'][0]['vertices'])
    np.testing.assert_allclose(box.min(0),[-1,1,0],atol=1e-6)
    np.testing.assert_allclose(box.max(0),[3,3,6],atol=1e-6)
    wheel=np.array(m['groups'][1]['vertices'])
    assert abs(wheel[:,0]).max()<.101  # Moving joint origin is supplied by ECM, not baked twice.
    assert len(wheel)==64*4*3
    assert .1*(1-math.cos(math.pi/64))<.000121


def test_unsupported_geometry_is_not_silently_dropped(tmp_path):
    urdf='<robot name="x"><link name="b"><visual><geometry><sphere radius="1"/></geometry></visual></link></robot>'
    with pytest.raises(ValueError,match='unsupported'):export(urdf,tmp_path)


def test_box_normals_point_outward():
    t=triangles(ET.fromstring('<geometry><box size="2 4 6"/></geometry>'))
    n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0])
    assert np.all((n*t.mean(1)).sum(1)>0)


@pytest.mark.parametrize("length", [.60, 1.20])
def test_led_emitters_follow_same_fixed_mount(tmp_path, length):
    urdf='''<robot name="lamp"><link name="base"/><link name="led_link">
    <visual><geometry><box size=".035 .60 .035"/></geometry></visual></link>
    <joint name="mount" type="fixed"><parent link="base"/><child link="led_link"/>
    <origin xyz=".95 0 -.06" rpy="0 0 0"/></joint></robot>'''
    urdf=urdf.replace('.035 .60 .035', f'.035 {length} .035')
    light=json.loads(export(urdf,tmp_path).read_text())['led_emitters']
    assert light['link']=='base'
    np.testing.assert_allclose(light['positions_m'], [[.95,y*(length/2-.02),-.082] for y in [-.75,-.25,.25,.75]],atol=1e-8)


def test_split_material_links_preserves_baked_geometry_and_dynamics(tmp_path):
    from agv_linescan.robot_scene import split_visual_links
    urdf='''<robot name="r"><link name="base"><inertial><mass value="65"/></inertial>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
    <visual><geometry><box size="1 1 1"/></geometry><material name="red"><color rgba="1 0 0 1"/></material></visual>
    <visual><origin xyz="1 2 3" rpy=".1 .2 .3"/><geometry><box size=".2 .3 .4"/></geometry>
    <material name="white"><color rgba="1 1 1 1"/></material></visual></link></robot>'''
    converted=split_visual_links(urdf);root=ET.fromstring(converted)
    assert all(len(link.findall('visual'))==1 for link in root.findall('link'))
    assert len(root.findall('.//inertial'))==1 and len(root.findall('.//collision'))==1
    before=json.loads(export(urdf,tmp_path/'before').read_text())
    after=json.loads(export(converted,tmp_path/'after').read_text())
    assert before['groups']==after['groups']

@pytest.mark.parametrize('diameter', [.39, .40, .42])
def test_physical_tyres_change_without_recalibrating_control(diameter):
    from pathlib import Path
    import xacro
    import yaml
    root = Path(__file__).resolve().parents[3]
    platform = root/'src/agv_description/config/platform.yaml'
    config = yaml.safe_load(platform.read_text())
    xml = xacro.process_file(str(root/'src/agv_description/urdf/agv.urdf.xacro'), mappings={
        'platform': str(platform), 'actual_wheel_diameter': str(diameter),
        'camera_config': str(root/'src/agv_description/config/linescan.yaml'),
        'controllers': str(root/'src/agv_bringup/config/controllers.yaml')}).toxml()
    robot = ET.fromstring(xml)
    for wheel in ('fl','fr','rl','rr'):
        link=robot.find(f"link[@name='{wheel}_wheel_link']")
        assert float(link.find('collision/geometry/cylinder').get('radius')) == diameter/2
        assert float(link.find('inertial/inertia').get('iyy')) == pytest.approx(18*(diameter/2)**2/2)
        limit=robot.find(f"joint[@name='{wheel}_drive_joint']/limit")
        assert float(limit.get('velocity')) == pytest.approx(config['max_speed']/.2)
    assert config['wheel_radius'] == .2


def test_camera_bracket_flex_preserves_mass_geometry_and_uses_dynamic_group(tmp_path):
    import xacro
    import yaml
    from pathlib import Path
    from agv_linescan.robot_scene import transform
    repo=Path(__file__).resolve().parents[3]
    cfg=yaml.safe_load((repo/'src/agv_description/config/linescan.yaml').read_text())
    masses=[];inertias=[];bounds=[]
    for enabled in (False,True):
        cfg['mount_flex']['enabled']=enabled
        path=tmp_path/f'camera_{enabled}.yaml';path.write_text(yaml.safe_dump(cfg))
        xml=xacro.process_file(str(repo/'src/agv_description/urdf/agv.urdf.xacro'),mappings={'camera_config':str(path)}).toxml()
        root=ET.fromstring(xml);joints={j.find('child').get('link'):j for j in root.findall('joint')}
        def world(name):
            j=joints.get(name)
            return np.eye(4) if j is None else world(j.find('parent').get('link'))@transform(j.find('origin'))
        inertia=[ET.tostring(x) for x in root.findall('link/inertial')];inertias.append(inertia)
        masses.append(sum(float(x.get('value')) for x in root.findall('link/inertial/mass')))
        vertices=[]
        for link in root.findall('link'):
            for v in link.findall('visual'):
                t=world(link.get('name'))@transform(v.find('origin'));p=triangles(v.find('geometry')).reshape(-1,3)
                vertices.append(p@t[:3,:3].T+t[:3,3])
        bounds.append(np.concatenate(vertices))
        group=json.loads(export(xml,tmp_path/str(enabled)).read_text())['groups']
        assert ('camera_carrier_link' in [g['name'] for g in group]) == enabled
        assert root.find("joint[@name='camera_pitch_joint']").get('type') == ('revolute' if enabled else 'fixed')
        states=root.findall("ros2_control/joint[@name='camera_pitch_joint']")
        assert bool(states)==enabled
        if enabled:
            assert not states[0].findall('command_interface')
            rest=float(root.find("gazebo[@reference='camera_pitch_joint']/springReference").text)
            assert rest*cfg['mount_flex']['stiffness_nm_rad']==pytest.approx(-9.81*(2.9*.07+.145))
    assert masses==pytest.approx([550,550])
    assert inertias[0]==inertias[1]
    np.testing.assert_allclose(bounds[0],bounds[1],atol=1e-12)
