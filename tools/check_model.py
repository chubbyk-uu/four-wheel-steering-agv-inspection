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
for mass in [50.0,65.0,80.0]:
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
    joints=[j for j in robot.findall('joint') if j.get('type')!='fixed']
    assert len(joints)==8
    assert sum(j.get('type')=='revolute' for j in joints)==4
    for j in joints:
        lim=j.find('limit')
        assert float(lim.get('effort'))>0 and float(lim.get('velocity'))>0
        if j.get('type')=='revolute':
            assert math.isclose(float(lim.get('upper'))-float(lim.get('lower')),math.radians(380))
    interfaces=robot.findall('ros2_control/joint')
    assert len(interfaces)==8
    assert all(len(j.findall('command_interface'))==1 for j in interfaces)
print('PASS: 50/65/80 kg variants, positive inertia, 8 actuated joints and 380-degree steering travel.')
