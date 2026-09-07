#!/usr/bin/env python3
"""Estimate a measured profile from dark/flat/striped-board captures."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/agv_linescan'))
from agv_linescan.calibration import flat_field, stripe_centers, geometry, make_profile


def read(path):
    with Image.open(path) as im:
        if im.mode != 'L':
            raise ValueError('Mono8 required')
        return np.array(im)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('dark', 'flat', 'board', 'target', 'conditions', 'output'):
        p.add_argument('--'+name, required=True, type=Path)
    a = p.parse_args()
    if a.output.exists():
        raise FileExistsError(a.output)
    f = flat_field(read(a.dark), read(a.flat))
    board = read(a.board)
    board = (board-np.asarray(f['offset']))*np.asarray(f['gain'])
    centers = stripe_centers(board)
    target = json.loads(a.target.read_text())
    g = geometry(centers, target['across_m'], board.shape[1], target['output_width_m'])
    sources = {key: dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
               for key, path in [('dark', a.dark), ('flat', a.flat), ('board', a.board), ('target', a.target), ('conditions', a.conditions)]}
    profile = make_profile(f, g, json.loads(a.conditions.read_text()), sources)
    a.output.write_text(json.dumps(profile, indent=2)+'\n')
    print(json.dumps(dict(calibration_id=profile['calibration_id'], features=len(centers), fit_error_px_max=g['fit_error_px_max'])))


if __name__ == '__main__':
    main()
