#!/usr/bin/env python3
"""Offline dark/flat and horizontal distortion correction of an archived session."""
import argparse,json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/agv_linescan'))
from agv_linescan.offline_correction import process_session


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',required=True,help='Raw session_cpp_* directory containing calibration.yaml and block files')
    p.add_argument('--profile',required=True,help='Measured calibration JSON')
    p.add_argument('--output',required=True,help='New output directory; raw images remain untouched')
    a=p.parse_args();print(json.dumps(process_session(a.input,a.profile,a.output)))


if __name__=='__main__':main()
