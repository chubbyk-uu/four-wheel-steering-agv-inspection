#!/usr/bin/env python3
"""Capture CUDA reference boards, estimate calibration, evaluate independent captures."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import yaml
from PIL import Image
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/agv_linescan'))
from agv_linescan.calibration import Correction, stripe_centers, capture_signature


def read(path):
    with Image.open(path) as im:
        return np.array(im)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--terrain', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    a = p.parse_args()
    out = a.output.resolve(); out.mkdir(parents=True, exist_ok=False)
    config = yaml.safe_load((ROOT/'src/agv_description/config/linescan.yaml').read_text())
    manifest = json.loads(a.terrain.read_text())
    n = manifest['core_pixels']; spacing = manifest['texel_m']
    assert abs(n*spacing/0.05-round(n*spacing/0.05)) < 1e-9
    terrains = {}
    for name, phase in [('flat', None), ('board', 0.), ('holdout', .025)]:
        folder = out/(name+'_terrain'); folder.mkdir()
        # These targets are periodic in y and constant in x, so identical tile
        # bytes are exact, including gutters. Hard links avoid redundant storage.
        pixels = np.full((n+2, n+2), 200, dtype=np.uint8)
        if phase is not None:
            y = manifest['origin_y_m']+(np.arange(-1, n+1)+.5)*spacing
            dark = abs((y-phase+.025) % .05-.025) <= .001
            pixels[dark] = 25
        master = folder/'tile_0_0.pgm'; Image.fromarray(pixels).save(master)
        for iy in range(manifest['tiles_y']):
            for ix in range(manifest['tiles_x']):
                dest = folder/f'tile_{ix}_{iy}.pgm'
                if dest != master:
                    os.link(master, dest)
        m = dict(manifest, diagnostic_pattern=name+'_calibration_target')
        (folder/'manifest.json').write_text(json.dumps(m))
        terrains[name] = folder/'manifest.json'
    for name in ['dark', 'flat', 'board', 'holdout', 'grid']:
        c = copy.deepcopy(config)
        if name == 'dark':
            c['radiometry']['led_peak_relative'] = 0
            c['radiometry']['ambient_relative'] = 0
        cfg = out/(name+'.yaml'); cfg.write_text(yaml.safe_dump(c))
        terrain = a.terrain if name == 'grid' else terrains['flat' if name == 'dark' else name]
        with (out/(name+'.log')).open('w') as log:
            subprocess.run(['ros2', 'run', 'agv_linescan', 'benchmark_cuda_grid', str(cfg), str(out/name),
                            '2', '19000', '0', '0', str(terrain.resolve())],
                           stdout=log, stderr=subprocess.STDOUT, check=True, timeout=90)
        summary = json.loads((out/name/'summary.json').read_text())
        assert summary['invalid_pixels'] == 0
    (out/'target.json').write_text(json.dumps(dict(across_m=(np.arange(-12, 13)*.05).tolist(), output_width_m=1.2)))
    (out/'conditions.json').write_text(json.dumps(dict(exposure_s=config['exposure_s'],
        source_calibration_id=config['calibration_id'], capture_signature=capture_signature(config),
        note='Same fixed camera/LED mounting, sensor gain and daylight as reference capture; recalibrate after changes.')))
    with (out/'fit.log').open('w') as log:
        subprocess.run([sys.executable, str(ROOT/'tools/calibrate_linescan.py'),
            '--dark', str(out/'dark/block_0.pgm'), '--flat', str(out/'flat/block_0.pgm'),
            '--board', str(out/'board/block_0.pgm'), '--target', str(out/'target.json'),
            '--conditions', str(out/'conditions.json'), '--output', str(out/'measured.json')],
            stdout=log, stderr=subprocess.STDOUT, check=True)
    profile = json.loads((out/'measured.json').read_text()); correction = Correction(profile)
    flat = read(out/'flat/block_1.pgm')  # Independent line-noise realization, not calibration block.
    fixed, _ = correction.apply(flat)
    valid = correction.valid
    raw_cv = float(flat[:, valid].mean(axis=0).std()/flat[:, valid].mean())
    corrected_cv = float(fixed[:, valid].mean(axis=0).std()/fixed[:, valid].mean())
    raw_holdout = read(out/'holdout/block_1.pgm')
    held, _ = correction.apply(raw_holdout)
    # Ignore explicitly invalid border; retain full coordinate origin.
    held[:, ~valid] = int(profile['flat']['target_signal_dn'])
    centers = stripe_centers(held)
    expected = (np.arange(-12, 12)*.05+.025)/spacing+(config['width']-1)/2
    assert len(centers) == len(expected), (len(centers), len(expected))
    geometry_error = float(np.max(abs(centers-expected)))
    raw_centers = stripe_centers((raw_holdout-np.asarray(profile['flat']['offset']))*np.asarray(profile['flat']['gain']))
    raw_error = float(np.max(abs(raw_centers-expected)))
    # Correct adjacent actual grid blocks, preserving every row and original tag.
    chunks = [read(out/f'grid/block_{i}.pgm') for i in range(2)]
    joined = np.concatenate(chunks)
    together, _ = correction.apply(joined)
    separate = np.concatenate([correction.apply(x)[0] for x in chunks])
    assert np.array_equal(together, separate)
    for i in range(2):
        meta = json.loads((out/f'grid/block_{i}.json').read_text())
        result = correction.metadata(meta)
        assert all(result[k] == meta[k] for k in ('first', 'last', 'pose_tags', 'rows', 'reference'))
        Image.fromarray(separate[i*4096:(i+1)*4096]).save(out/f'corrected_{i}.png')
        (out/f'corrected_{i}.json').write_text(json.dumps(result))
    # Informational offline CPU microbenchmark, NOT end-to-end acceptance.
    times = []
    for _ in range(8):
        start = time.perf_counter(); correction.apply(chunks[0]); times.append(time.perf_counter()-start)
    report = dict(calibration_id=profile['calibration_id'], source=str(out),
        flat_column_cv_before=raw_cv, flat_column_cv_after=corrected_cv,
        independent_geometry_error_px_before=raw_error, independent_geometry_error_px_after=geometry_error,
        fit_error_px_max=profile['geometry']['fit_error_px_max'], valid_columns=int(valid.sum()),
        partition_identical=True, metadata_preserved=True,
        offline_correction_lines_per_second=4096/float(np.mean(times)),
        full_pipeline_11_22khz_acceptance_passed=False)
    assert corrected_cv < .002 and corrected_cv < raw_cv/10, report
    assert geometry_error < 1 and raw_error > 10, report
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    for ax, im, title in zip(axes.flat, [chunks[0], separate[:4096], flat, fixed],
                            ['Raw grid', 'Measured flat + optical correction', 'Independent flat: raw', 'Independent flat: corrected']):
        ax.imshow(im[::4, ::4], cmap='gray', vmin=0, vmax=220); ax.set_title(title); ax.axis('off')
    fig.tight_layout(); fig.savefig(out/'comparison.png', dpi=130); plt.close(fig)
    (out/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()
