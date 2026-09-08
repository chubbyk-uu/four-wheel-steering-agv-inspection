#!/usr/bin/env python3
"""Independent straight-pass paint-position check using archived optics and known road dimensions."""
import argparse,json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml

def intervals(mask):
    edges=np.diff(np.r_[False,mask,False].astype(int))
    return [[int(a),int(b)] for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1)) if b-a>100]

def check(session):
    root=Path(session);cfg=yaml.safe_load((root/'calibration.yaml').read_text())
    image=np.array(Image.open(root/'block_000002.pgm'))
    w=cfg['width'];q=(np.arange(w)-(w-1)/2)/(w/2)
    y=cfg['nominal_width_m']/2*np.polynomial.polynomial.polyval(q,cfg['ray_polynomial'])
    expected=abs(abs(y)-.15)<=.075
    profile=image[200:600].mean(axis=0)
    threshold=(profile[(abs(y)<.35)&~expected].mean()+profile[expected].mean())/2
    measured=intervals(profile>threshold);predicted=intervals(expected)
    assert len(measured)==len(predicted)==2,(measured,predicted)
    error=int(np.max(np.abs(np.asarray(measured)-predicted)));assert error<=2,error
    return dict(measured_paint_intervals_pixels=measured,predicted_intervals_pixels=predicted,max_edge_error_pixels=error,
                scope='400 rows from block 2 in the centered straight-pass fixture; not general geometric calibration',passed=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('session');a=p.parse_args();print(json.dumps(check(a.session)))
