#!/usr/bin/env python3
"""Summarize matched ODE/Bullet collision-detector capture runs."""
import argparse
import json
from pathlib import Path

import numpy as np


def summarize(run: Path):
    blocks = {}
    for path in run.glob('raw/session_cpp_*/block_*.json'):
        doc = json.loads(path.read_text())
        blocks[doc['block_id']] = doc['scan_residual_statistics']
    if set(blocks) != set(range(34)):
        raise ValueError(f'{run}: expected block ids 0..33, got {sorted(blocks)}')
    result = json.loads((run / 'results.json').read_text())
    clock = np.load(run / 'clock_evaluation.npy')
    tracks = []
    for track_id, first, last in ((0, 0, 16), (1, 17, 33)):
        begin, end = blocks[first], blocks[last]
        histogram = np.asarray(end['histogram']) - np.asarray(begin['histogram'])
        if np.any(histogram < 0):
            raise ValueError(f'{run}: cumulative histogram moved backwards')
        samples = end['samples'] - begin['samples']
        if int(histogram.sum()) != samples:
            raise ValueError(f'{run}: histogram/sample mismatch')
        occupied = np.flatnonzero(histogram)
        tracks.append({
            'track_id': track_id,
            'samples_between_first_and_last_block': samples,
            'below_0p001935_m_s': int(histogram[0]),
            'at_or_above_0p001935_m_s': int(histogram[1:].sum()),
            'fraction_at_or_above_0p001935_m_s': float(histogram[1:].sum() / samples),
            'at_or_above_0p009677_m_s': int(histogram[5:].sum()),
            'at_or_above_0p015484_m_s': int(histogram[8:].sum()),
            'highest_occupied_bin_lower_bound_m_s': float(
                occupied[-1] * end['histogram_bin_width_m_s']),
        })
    return {
        'passed': result['passed'],
        'mission_simulation_time_s': result['final']['time_s'],
        'rtf': float((clock[-1, 0] - clock[0, 0]) / (clock[-1, 1] - clock[0, 1])),
        'whole_run_cumulative': {key: blocks[33][key] for key in (
            'samples', 'max_m_s', 'p95_m_s', 'p99_m_s', 'over_half_limit', 'over_limit')},
        'stable_capture_intervals': tracks,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    cases = {
        name: summarize(args.root / name)
        for name in ('run_ode2', 'run_ode3', 'run_bullet', 'run_bullet2')
    }
    report = {
        'schema': 'agv.collision_detector_ab.v1',
        'date': '2026-09-16',
        'scope': ('Matched 20 x 2 m, two-track, 10 km/h headless OptiX missions on the '
                  'same 10 cm / +/-3 mm collision mesh. DART and the Dantzig solver are '
                  'unchanged; only DART collision_detector is ode or bullet.'),
        'method': ('Each stable interval is the difference between the cumulative 1 kHz '
                   'residual histograms in the first and last image blocks of that track. '
                   'This excludes the approach and most turnaround samples that dominate '
                   'the whole-run tail; it also omits the first and last partial block.'),
        'histogram_bin_width_m_s': 0.001935483870967742,
        'cases': cases,
        'conclusion': ('Do not switch production to Bullet. It lowers the whole-run maximum '
                       'and turnaround over-limit count, but repeatedly makes 10.0-12.7% of '
                       'steady capture samples leave the first residual bin; both ODE repeats '
                       'keep every measured steady-capture sample in that bin. The A/B proves '
                       'that contact generation controls the residual signature, but does not '
                       'by itself prove the rare ODE outlier mechanism.'),
    }
    args.output.write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()
