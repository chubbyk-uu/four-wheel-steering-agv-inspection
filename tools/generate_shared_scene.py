#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path.append(str(root/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.shared_scene import generate
p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--length',type=float,default=100);p.add_argument('--width',type=float,default=10)
p.add_argument('--profile',choices=['flat','optical_stress'],default='flat',help='flat driving baseline; optical_stress is an isolated geometry/shadow regression fixture')
p.add_argument('--margin',type=float,default=2.,help='flat-scene ground extension outside unchanged inspection ROI (metres)')
a=p.parse_args();m=generate(a.output,root/'src/agv_bringup/worlds/flat.sdf',a.length,a.width,a.profile,a.margin)
print(f"Generated {len(m['assets'])} shared meshes, {sum(x['triangles'] for x in m['assets'])} triangles")
