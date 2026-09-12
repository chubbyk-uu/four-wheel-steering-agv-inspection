#!/usr/bin/env python3
"""Look for duplicate texture: lines emitted while the ground did not move.

Two independent views of the same question, because neither alone is conclusive.

The archived pose tags carry both an encoder distance and the camera's position,
so the ratio of ground travel to encoder travel over a tag interval says whether
the wheels turned without the vehicle advancing. That is cheap and covers every
line, but the tags are sparse: a short stall is diluted across its interval.

The pixels say it directly. Consecutive scan lines 0.3662 mm apart differ by a
measurable amount on real texture; lines imaging the same ground do not. The
detector is the mean absolute difference between adjacent rows, and the decision
threshold is read from the run's own distribution rather than assumed.

The original artifact (150 N.m holding torque, wheels rolling under a stopped
vehicle during a wheel alignment) advanced the encoder 8.057 mm while the camera
moved 0.968 mm -- a ratio of 0.12 across 23 lines.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


def tag_intervals(blocks):
    """Ground travel against encoder travel, per pose-tag interval."""
    for block in blocks:
        spacing = block['line_spacing_m']
        for start, end in zip(block['pose_tags'], block['pose_tags'][1:]):
            lines = end['global_line'] - start['global_line']
            if lines <= 0:
                continue
            # Ground distance along the scan axis, not 3-D: during a stop the
            # suspension alone moves millimetres vertically and would mask this.
            travel = abs(end['camera_position_world_m'][0]
                         - start['camera_position_world_m'][0])
            yield dict(block_id=block['block_id'], segment_id=block['segment_id'],
                       first_line=start['global_line'], last_line=end['global_line'],
                       lines=lines, encoder_m=lines*spacing, ground_m=travel,
                       ratio=travel/(lines*spacing), seconds=end['time_s']-start['time_s'])


def row_differences(path, stride):
    """Mean absolute difference between adjacent rows of one archived image."""
    with Image.open(path) as image:
        pixels = np.asarray(image, dtype=np.int16)
    return np.abs(np.diff(pixels[:, ::stride], axis=0)).mean(axis=1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission', type=Path, required=True,
                        help='mission directory holding capture_blocks.jsonl')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ratio-floor', type=float, default=.95,
                        help='ground/encoder below this is reported as slip')
    parser.add_argument('--column-stride', type=int, default=4)
    parser.add_argument('--duplicate-fraction', type=float, default=.4,
                        help='row difference below this fraction of the run median is a duplicate')
    args = parser.parse_args()

    blocks = [json.loads(line) for line in (args.mission/'capture_blocks.jsonl').read_text().splitlines()]
    if not blocks:
        raise SystemExit('no archived blocks')
    intervals = list(tag_intervals(blocks))
    ratios = np.array([v['ratio'] for v in intervals])
    slips = sorted((v for v in intervals if v['ratio'] < args.ratio_floor),
                   key=lambda v: v['ratio'])

    archives = {Path(entry['archive']) for entry in
                json.loads((args.mission/'capture_intervals.json').read_text())}
    images = sorted(path for archive in archives for path in archive.glob('block_*.pgm'))
    per_image, worst, floor_seen = [], [], math.inf
    for path in images:
        difference = row_differences(path, args.column_stride)
        floor_seen = min(floor_seen, float(difference.min()))
        per_image.append(dict(image=path.name, rows=int(difference.size)+1,
                              minimum=float(difference.min()),
                              median=float(np.median(difference)),
                              exact_duplicates=int((difference == 0).sum())))
    median = float(np.median([v['median'] for v in per_image])) if per_image else 0.
    threshold = median*args.duplicate_fraction
    for entry, path in zip(per_image, images):
        if entry['minimum'] < threshold or entry['exact_duplicates']:
            difference = row_differences(path, args.column_stride)
            rows = np.flatnonzero(difference < threshold)
            worst.append(dict(image=entry['image'], rows=rows.tolist()[:64],
                              count=int(rows.size), minimum=entry['minimum']))

    report = dict(
        schema='agv.scan_slip.v1', mission=str(args.mission),
        blocks=len(blocks), images=len(images), lines=sum(v['lines'] for v in intervals),
        encoder_vs_ground=dict(
            intervals=len(intervals), floor=args.ratio_floor,
            median=float(np.median(ratios)), minimum=float(ratios.min()),
            p0_1=float(np.percentile(ratios, .1)), p1=float(np.percentile(ratios, 1)),
            below_floor=len(slips), worst=slips[:8]),
        adjacent_rows=dict(
            column_stride=args.column_stride, run_median=median,
            duplicate_threshold=threshold, observed_minimum=floor_seen,
            images_flagged=len(worst), flagged=worst[:8],
            exact_duplicate_rows=sum(v['exact_duplicates'] for v in per_image)),
        scope=('Simulation truth pose tags and archived pixels. The ratio test needs '
               'the rendered camera position and cannot run against estimated '
               'navigation, whose centimetre noise hides a millimetre stall.'))
    report['passed'] = not slips and not worst
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print('slip intervals below %.2f: %d/%d' % (args.ratio_floor, len(slips), len(intervals)))
    print('row-difference median %.3f, threshold %.3f, observed floor %.3f'
          % (median, threshold, floor_seen))
    print('images with duplicate rows: %d/%d   exact duplicates: %d'
          % (len(worst), len(images), report['adjacent_rows']['exact_duplicate_rows']))
    print('passed:', report['passed'])


if __name__ == '__main__':
    main()
