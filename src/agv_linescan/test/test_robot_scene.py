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


def test_led_emitters_follow_same_fixed_mount(tmp_path):
    urdf='''<robot name="lamp"><link name="base"/><link name="led_link">
    <visual><geometry><box size=".035 .60 .035"/></geometry></visual></link>
    <joint name="mount" type="fixed"><parent link="base"/><child link="led_link"/>
    <origin xyz=".95 0 -.06" rpy="0 0 0"/></joint></robot>'''
    light=json.loads(export(urdf,tmp_path).read_text())['led_emitters']
    assert light['link']=='base'
    np.testing.assert_allclose(light['positions_m'], [[.95,y,-.082] for y in [-.21,-.07,.07,.21]],atol=1e-8)


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
