#!/usr/bin/env python3
"""Estimate missing strips and write new, preview-only rectangle requests."""
import argparse,json
from pathlib import Path
import yaml
from agv_mission.capture_audit import audit_capture


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--mission',type=Path,required=True);p.add_argument('--navigation',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--uncertainty-m',type=float,default=.10)
    p.add_argument('--platform',type=Path,default=Path('src/agv_description/config/platform.yaml'))
    p.add_argument('--camera',type=Path,default=Path('src/agv_description/config/linescan.yaml'))
    a=p.parse_args();r=audit_capture(a.mission,a.navigation,a.output,yaml.safe_load(a.platform.read_text()),yaml.safe_load(a.camera.read_text()),a.uncertainty_m)
    print(json.dumps({k:r[k] for k in ('status','tracks','issues')}))


if __name__=='__main__':main()
