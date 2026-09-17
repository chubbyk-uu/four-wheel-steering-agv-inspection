#!/usr/bin/env python3
"""Retrospective geometry-only audit of every archived tag interval, not pixels."""
import argparse,json,sys
from pathlib import Path
from collections import Counter
sys.path.append(str(Path(__file__).resolve().parents[1]/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.scan_footprint import Heightfield,interval_travel,ground_verdict

def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,action='append',required=True)
    p.add_argument('--heightfield',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();field=Heightfield.load(a.heightfield);runs=[]
    for root in a.run:
        counts=Counter();blocks=0
        for path in root.glob('session/raw/*/block_*.json'):
            b=json.loads(path.read_text());blocks+=1
            for first,last in zip(b['pose_tags'],b['pose_tags'][1:]):
                out=interval_travel(first,last,field,b['line_spacing_m'])
                counts[ground_verdict(out['encoder_m'],out['footprint_m']) if out else 'unmeasured']+=1
        runs.append(dict(run=root.name,blocks=blocks,counts=dict(counts),passed=bool(counts['pass']) and not counts['alert'] and not counts['unmeasured']))
    report=dict(scope='Geometry only; not a replacement for pixel/missing-image/coverage audit.',heightfield=str(a.heightfield),runs=runs,passed=all(r['passed'] for r in runs))
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    raise SystemExit(0 if report['passed'] else 1)
if __name__=='__main__':main()
