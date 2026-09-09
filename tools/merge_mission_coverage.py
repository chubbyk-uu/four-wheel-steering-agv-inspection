#!/usr/bin/env python3
"""Merge audited rescan footprints without modifying original imagery."""
import argparse,json
from pathlib import Path
from agv_mission.coverage import combine

def main():
    p=argparse.ArgumentParser();p.add_argument('--parent',type=Path,required=True)
    p.add_argument('--child',type=Path,action='append',required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();result=combine(json.loads(a.parent.read_text()),[json.loads(v.read_text()) for v in a.child])
    with a.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(result['status'])
if __name__=='__main__':main()
