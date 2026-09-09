#!/usr/bin/env python3
"""Overlay the bounded closed-loop scan model without duplicating optical settings."""
import argparse
from pathlib import Path
import yaml
from agv_mission.camera_limits import inspection_camera_config


def prepare(source,destination,platform=Path("src/agv_description/config/platform.yaml")):
    config=yaml.safe_load(source.read_text())
    config=inspection_camera_config(config,yaml.safe_load(platform.read_text()))
    with destination.open('x') as stream:yaml.safe_dump(config,stream)
    return destination.resolve()


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=Path('src/agv_description/config/linescan.yaml'))
    p.add_argument('--platform',type=Path,default=Path('src/agv_description/config/platform.yaml'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.source,a.output,a.platform))


if __name__=='__main__':main()
