#!/usr/bin/env python3
"""Overlay the bounded closed-loop scan model without duplicating optical settings."""
import argparse
from pathlib import Path
import yaml


def prepare(source,destination):
    config=yaml.safe_load(source.read_text())
    config.update(projected_encoder=True,max_yaw_rate_rad_s=.08,
                  max_scan_lateral_m_s=.10,max_scan_residual_m_s=.03)
    with destination.open('x') as stream:yaml.safe_dump(config,stream)
    return destination.resolve()


def main():
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,default=Path('src/agv_description/config/linescan.yaml'))
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(prepare(a.source,a.output))


if __name__=='__main__':main()
