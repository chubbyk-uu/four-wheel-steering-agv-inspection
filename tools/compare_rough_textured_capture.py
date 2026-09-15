#!/usr/bin/env python3
"""Compare two real sessions after column correction and exact row concatenation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/agv_linescan'))
from agv_linescan.offline_correction import process_session


def concatenate(source, corrected):
    pieces = []
    records = []
    previous = None
    for path in sorted(source.glob('block_*.json')):
        record = json.loads(path.read_text())
        if previous is not None:
            if (record['first']['global_line'] != previous['last']['global_line'] + 1
                    or record['segment_id'] != previous['segment_id']
                    or record['line_spacing_m'] != previous['line_spacing_m']):
                raise ValueError('only continuous rows from one segment can form this strip')
        pieces.append(np.array(Image.open(corrected / path.with_suffix('.pgm').name)))
        records.append(record)
        previous = record
    if not pieces:
        raise ValueError('empty capture')
    return np.concatenate(pieces), records


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--flat', type=Path, required=True)
    parser.add_argument('--rough', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    profile = json.loads(args.profile.read_text())
    width = profile['geometry']['output_width_m']
    reports = {}
    fig, axes = plt.subplots(1, 2, figsize=(10, 12))
    for ax, name, source in zip(axes, ('flat', 'rough'), (args.flat, args.rough)):
        corrected = args.output / (name + '_corrected')
        process_session(source, args.profile, corrected)
        strip, records = concatenate(source, corrected)
        target = args.output / (name + '_strip.png')
        Image.fromarray(strip).save(target)
        assert np.array_equal(np.array(Image.open(target)), strip)
        spacing = records[0]['line_spacing_m']
        ax.imshow(strip[::4, ::4], cmap='gray', vmin=0, vmax=160,
                  extent=[-width/2, width/2, len(strip)*spacing, 0], aspect='equal')
        ax.set_title(f'{name.capitalize()} road: {len(records)} blocks')
        ax.set_xlabel('Across strip (m)')
        ax.set_ylabel('Distance from first saved row (m)')
        reports[name] = dict(shape=list(strip.shape), block_rows=[r['rows'] for r in records],
            first_global_line=records[0]['first']['global_line'],
            last_global_line=records[-1]['last']['global_line'],
            line_spacing_m=spacing, pixel_sha256=hashlib.sha256(strip).hexdigest(),
            decoded_png_equals_corrected_rows=True)
    fig.suptitle('10 km/h: flat vs 10 cm / 3 mm rough road\n'
                 'Flat-field + optical correction; direct rows, no pose warp')
    fig.tight_layout(rect=[0, 0, 1, .95])
    fig.savefig(args.output / 'comparison.png', dpi=140)
    plt.close(fig)
    reports.update(calibration_id=profile['calibration_id'], preview_gray_range=[0, 160],
        scope='Independent acquisition origins; no matching, stretching or geometric alignment.')
    (args.output / 'summary.json').write_text(json.dumps(reports, indent=2) + '\n')
    print(json.dumps(reports))


if __name__ == '__main__':
    main()
