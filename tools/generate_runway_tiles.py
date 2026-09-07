#!/usr/bin/env python3
"""Materialize metric diagnostic grid tiles, with one-texel gutters; no real texture."""
import argparse
import json
import math
import os
from pathlib import Path
import time
import numpy as np
from PIL import Image


def pattern(x, y):
    # Unique cell tones expose wrong/stale tile selection, including periodic grid.
    cells_x = np.floor(x/.1).astype(np.int64)
    cells_y = np.floor(y/.1).astype(np.int64)
    gray = 170+(cells_y[:, None]*7+cells_x[None, :]*13) % 50
    dx = np.abs((x+.05) % .1-.05)
    dy = np.abs((y+.05) % .1-.05)
    return np.where((dy[:, None] <= .001) | (dx[None, :] <= .001), 25, gray).astype(np.uint8)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--length', type=float, default=100)
    p.add_argument('--width', type=float, default=10)
    p.add_argument('--tile-pixels', type=int, default=4096)
    a = p.parse_args()
    assert a.length > 0 and a.width > 0 and 32 <= a.tile_pixels <= 8192
    a.output.mkdir(parents=True, exist_ok=False)
    start = time.monotonic()
    spacing = 1.2/4096
    size = a.tile_pixels*spacing
    nx, ny = math.ceil(a.length/size), math.ceil(a.width/size)
    # No global cache dropping. Evict only each newly generated file's clean pages.
    for iy in range(ny):
        y = -a.width/2+(iy*a.tile_pixels+np.arange(-1,a.tile_pixels+1)+.5)*spacing
        for ix in range(nx):
            x = (ix*a.tile_pixels+np.arange(-1,a.tile_pixels+1)+.5)*spacing
            pixels = pattern(x,y)
            with (a.output/f'tile_{ix}_{iy}.pgm').open('wb') as f:
                f.write(f'P5\n{a.tile_pixels+2} {a.tile_pixels+2}\n255\n'.encode())
                f.write(pixels.tobytes()); f.flush(); os.fsync(f.fileno())
                os.posix_fadvise(f.fileno(),0,0,os.POSIX_FADV_DONTNEED)
    # A separate coarse GUI overview; never sampled by the line camera.
    x = (np.arange(math.ceil(a.length/.02))+.5)*.02
    y = -a.width/2+(np.arange(math.ceil(a.width/.02))+.5)*.02
    Image.fromarray(pattern(x,y)[::-1]).save(a.output/'overview.png')
    manifest = dict(schema='agv.terrain.tiles.v1', length_m=a.length, width_m=a.width,
                    origin_x_m=0., origin_y_m=-a.width/2, texel_m=spacing,
                    core_pixels=a.tile_pixels, gutter_pixels=1, tiles_x=nx, tiles_y=ny,
                    encoding='mono8', layout='row_y_column_x', plane_z_m=0.,
                    diagnostic_pattern='10cm_grid_2mm_lines_unique_cell_gray',
                    generation_seconds=time.monotonic()-start,
                    raw_texture_bytes=nx*ny*(a.tile_pixels+2)**2,
                    cache_advice='fsync + POSIX_FADV_DONTNEED per generated file; OS hint, not a hardware cold-cache guarantee')
    (a.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest))


if __name__ == '__main__':
    main()
