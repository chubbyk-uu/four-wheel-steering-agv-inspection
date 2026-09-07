#!/usr/bin/env python3
"""Check visible longitudinal grid stripes in straight, flat OptiX captures.

This detects gaps/blank rows in the grid fixture, not arbitrary scene accuracy
or a substitute for encoder/ROS block continuity checks.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image


def check(session):
    results=[]
    for path in sorted(session.glob('block_*.pgm')):
        pixels=np.asarray(Image.open(path))
        if pixels.shape!=(4096,4096):
            continue  # A short final block may not span enough grid periods.
        # Local white reference across 64 columns: per-column normalization
        # would erase a stripe that remains at the same column for all rows.
        white=np.percentile(pixels.reshape(4096,64,64),90,axis=(0,2))
        normalized=pixels/np.repeat(np.maximum(white,1),64)[None,:]
        dark=normalized<.75
        candidates=np.flatnonzero(dark.mean(axis=0)>.5)
        groups=np.split(candidates,np.flatnonzero(np.diff(candidates)>1)+1)
        stripes=[g for g in groups if len(g)>=2]
        assert len(stripes)>=11, (path.name,'too few persistent stripes',len(stripes))
        missing=[]
        for g in stripes:
            # Include modest subpixel drift without joining adjacent grid lines.
            present=dark[:,max(0,g[0]-8):min(4096,g[-1]+9)].any(axis=1)
            missing.append(int((~present).sum()))
        results.append(dict(block=path.name,stripes=len(stripes),
                            maximum_missing_rows_per_stripe=max(missing)))
        assert max(missing)==0, (path.name,missing)
    assert results, 'no complete grid blocks'
    return dict(passed=True,full_blocks=len(results),checks=results,
                scope='persistent longitudinal grid stripes; final partial block excluded')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('session',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    report=check(args.session)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='checks'}))
