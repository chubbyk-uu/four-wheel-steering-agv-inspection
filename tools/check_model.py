#!/usr/bin/env python3
"""Check model mass, inertia positivity, mechanical joints and parameter variants."""
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import numpy as np
import xacro
import yaml
root=Path(__file__).resolve().parents[1]
base=yaml.safe_load((root/'src/agv_description/config/platform.yaml').read_text())
for mass in [500.0,550.0,600.0]:
    p=dict(base,mass=mass)
    with tempfile.TemporaryDirectory() as d:
        cfg=Path(d)/'platform.yaml';cfg.write_text(yaml.safe_dump(p))
        robot=ET.fromstring(xacro.process_file(str(root/'src/agv_description/urdf/agv.urdf.xacro'),mappings={'platform':str(cfg)}).toxml())
    inertials=robot.findall('link/inertial')
    assert abs(sum(float(i.find('mass').get('value')) for i in inertials)-mass)<1e-9
    for i in inertials:
        t={k:float(v) for k,v in i.find('inertia').attrib.items()}
        matrix=np.array([[t['ixx'],t['ixy'],t['ixz']],[t['ixy'],t['iyy'],t['iyz']],[t['ixz'],t['iyz'],t['izz']]])
        assert np.linalg.eigvalsh(matrix).min()>0
    joints=[j for j in robot.findall('joint') if j.get('type') in ('revolute','continuous')]
    assert len(joints)==8
    assert sum(j.get('type')=='revolute' for j in joints)==4
    for j in joints:
        lim=j.find('limit')
        assert float(lim.get('effort'))>0 and float(lim.get('velocity'))>0
        if j.get('type')=='revolute':
            assert math.isclose(float(lim.get('upper'))-float(lim.get('lower')),math.radians(380))
    interfaces=robot.findall('ros2_control/joint')
    assert len(interfaces)==12
    assert sum(len(j.findall('command_interface')) for j in interfaces)==8
    assert len([j for j in robot.findall('joint') if j.get('type')=='prismatic'])==4
    gps=[robot.find("joint[@name='gnss_"+n+"_mount']/origin") for n in ('left','right')]
    xyz=[list(map(float,g.get('xyz').split())) for g in gps]
    assert math.isclose(xyz[0][1]-xyz[1][1],base['gnss_baseline'])
    assert math.isclose(xyz[0][2]+base['base_height']+.0175,1.2)
    battery=robot.find("link[@name='battery_link']/collision/geometry/box")
    length,width,height=map(float,battery.get('size').split())
    z=float(robot.find("joint[@name='battery_mount']/origin").get('xyz').split()[2])
    assert z+height/2>=.08, 'battery must meet visible shell'
    assert math.isclose(base['base_height']+z-height/2,.2516), 'battery nominal ground clearance'
    assert math.isclose(width,base['body_width']), 'battery housing matches body width'
    # Conservative wheel/fork swept bounding disk, independent of steering angle.
    swept=math.hypot(base['wheel_radius'],.118)
    assert base['wheelbase']/2-length/2-swept>.05, 'battery/wheel sweep clearance'
print('PASS: 500/550/600 kg variants, four passive suspensions, GNSS baseline/heights, positive inertia, 8 actuated joints and 380-degree steering travel.')
