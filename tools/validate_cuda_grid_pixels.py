#!/usr/bin/env python3
"""Independent float64 geometry check of the CUDA testbench's first raw block."""
import argparse
import json
from pathlib import Path
import numpy as np
from PIL import Image
import yaml

p = argparse.ArgumentParser()
p.add_argument('archive', type=Path)
a = p.parse_args()
c = yaml.safe_load((a.archive/'calibration.yaml').read_text())
s = json.loads((a.archive/'summary.json').read_text())
width = c['width']
with Image.open(a.archive/'block_0.pgm') as im:
    image = np.asarray(im)
rows = np.unique(np.linspace(0, image.shape[0]-1, 128).astype(int))
q = (np.arange(width)-(width-1)/2)/(width/2)
# Independent nominal flat-ground projection; height/focal cancel to W/2.
y = c['nominal_width_m']/2*np.polynomial.polynomial.polyval(q, c['ray_polynomial'])
dy = np.abs((y+c['grid_spacing_m']/2) % c['grid_spacing_m']-c['grid_spacing_m']/2)
rate = s['target_lines_per_second'] or 22000
value = np.zeros((len(rows), width))
ambiguous = np.zeros_like(value, dtype=bool)
for fraction in (1/6, .5, 5/6):
    x = .05+rows*c['line_spacing_m']+rate*c['line_spacing_m']*c['exposure_s']*fraction
    dx = np.abs((x+c['grid_spacing_m']/2) % c['grid_spacing_m']-c['grid_spacing_m']/2)
    distance = np.minimum(dx[:, None], dy[None, :])
    value += np.where(distance <= c['grid_line_width_m']/2, c['grid_dark'], c['grid_light'])/3
    # A 2 micrometre guard only around discontinuous paint boundaries;
    # far smaller than one ground pixel. Not a general intensity tolerance.
    ambiguous |= np.abs(distance-c['grid_line_width_m']/2) < 2e-6
error = np.abs(image[rows].astype(int)-np.rint(value).astype(int))
assert np.max(error[~ambiguous]) == 0, int(np.max(error[~ambiguous]))
assert np.mean(ambiguous) < .01
result = dict(passed=True, compared_pixels=int(value.size),
              paint_boundary_guard_m=2e-6, guarded_pixels=int(ambiguous.sum()),
              mismatched_pixels=int((error>0).sum()),
              max_unguarded_gray_error=int(np.max(error[~ambiguous])))
(a.archive/'pixel_validation.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
