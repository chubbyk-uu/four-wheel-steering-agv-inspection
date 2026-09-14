#!/usr/bin/env python3
"""Write independent localization configs; never change physical camera geometry."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/agv_localization'))
from agv_localization.camera_residuals import cases


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=ROOT/'src/agv_localization/config/measurements.yaml')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    content=args.base.read_bytes();trials=list(cases(yaml.safe_load(content)))
    args.output.mkdir(parents=True,exist_ok=False)
    (args.output/'COLCON_IGNORE').touch()
    entries=[]
    for name,config,description in trials:
        filename=name+'.yaml';payload=yaml.safe_dump(config,sort_keys=False)
        (args.output/filename).write_text(payload)
        entries.append(dict(file=filename,sha256=hashlib.sha256(payload.encode()).hexdigest(),**description))
    report=dict(schema='agv.camera_residual_trials.v1',base_sha256=hashlib.sha256(content).hexdigest(),
                convention='fixed optical-origin right perturbation; requested axes are body X forward, Y left, Z up',
                scope='Only camera calibration changes. Existing sensor noise retained. No vibration or geometry changes. Sensitivity levels, not measured hardware specifications.',cases=entries)
    (args.output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(configurations=len(entries),output=str(args.output))))


if __name__=='__main__':main()
