#!/usr/bin/env python3
"""Look for duplicate texture: lines emitted while the ground did not move.

Two independent views of the same question, because neither alone is conclusive.

The archived pose tags carry both an encoder distance and the camera's position,
so the ratio of ground travel to encoder travel over a tag interval says whether
the camera centre advanced less than nominal encoder travel. Suspension pitch
and mount flex also change that ratio, so it alone cannot establish tyre slip. That is cheap and covers every
line, but the tags are sparse: a short stall is diluted across its interval.

The pixels say it directly. Consecutive scan lines 0.3662 mm apart differ by a
measurable amount on real texture; lines imaging the same ground do not.

The plain adjacent-row difference cannot separate that from ground with nothing
to differ in. A painted marking collapses it just as a stall does: on the marked
road 31 of 704 images tripped it, and so did 32 of 699 on the triangle-mesh
version of the same road, while the unmarked road tripped none. So the decision
variable is the adjacent-row difference divided by the texture measured along
the row. A stall keeps each row's own contrast and loses only the difference to
its neighbour, so the ratio collapses; paint loses both together, so the ratio
does not move. Where the row itself carries too little texture to divide by, the
pair is reported as untestable rather than passed in silence.

The original artifact (150 N.m holding torque, wheels rolling under a stopped
vehicle during a wheel alignment) advanced the encoder 8.057 mm while the camera
moved 0.968 mm -- a ratio of 0.12 across 23 lines.
"""
import argparse
import json
import math
from pathlib import Path

import sys
import hashlib
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"src/agv_linescan"))
from agv_linescan.scan_footprint import Heightfield, interval_travel, ground_verdict

import numpy as np
from PIL import Image


def tag_intervals(blocks):
    """Camera-centre horizontal travel / nominal encoder distance, not tyre slip."""
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
                       lines=lines, encoder_m=lines*spacing, camera_horizontal_m=travel, ground_m=travel,
                       first_time_s=start["time_s"], last_time_s=end["time_s"],
                       camera_x_extent_m=sorted([start["camera_position_world_m"][0], end["camera_position_world_m"][0]]),
                       ratio=travel/(lines*spacing), seconds=end['time_s']-start['time_s'])


def identity(block):
    """What makes an archived block that block and no other."""
    return (block['block_id'], block['segment_id'], block['rows'],
            block['first']['global_line'], block['last']['global_line'])


def locate_images(blocks, archives):
    """Pair every archived block with its own pixels, by the sensor's naming.

    The two detectors read different files: the ratio test reads metadata, the
    duplicate-row test reads pixels. Left to find its images by globbing an
    archive directory, the pixel half tests nothing at all once that directory
    has been cleaned or moved -- and an audit that tested nothing still reported
    a pass. Pairing block by block makes a missing image a finding instead, and
    confines the pixel half to this mission's blocks rather than to whatever
    else shares the sensor session.
    """
    found, missing = [], []
    for block in blocks:
        name = 'block_%06d' % block['block_id']
        located = None
        for directory in archives:
            metadata = directory/(name+'.json')
            if not metadata.is_file():
                continue
            try:
                raw = json.loads(metadata.read_text())
            except ValueError:
                continue
            if identity(raw) == identity(block):
                located = directory/(name+'.pgm')
                break
        if located is not None and located.is_file():
            found.append(located)
        else:
            missing.append(dict(block_id=block['block_id'], image=name+'.pgm',
                                metadata_located=located is not None))
    return found, missing


def row_evidence(path, stride):
    """Adjacent-row difference and the texture each pair has to differ in."""
    with Image.open(path) as image:
        pixels = np.asarray(image, dtype=np.int16)[:, ::stride]
    difference = np.abs(np.diff(pixels, axis=0)).mean(axis=1)
    texture = np.abs(np.diff(pixels, axis=1)).mean(axis=1)
    # The weaker of the two rows bounds what their difference could have been.
    return difference, np.minimum(texture[:-1], texture[1:])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission', type=Path, required=True,
                        help='mission directory holding capture_blocks.jsonl')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ratio-floor', type=float, default=.95,
                        help='camera-centre/encoder below this raises a geometry alert, not confirmed tyre slip')
    parser.add_argument('--column-stride', type=int, default=4)
    parser.add_argument('--duplicate-fraction', type=float, default=.4,
                        help='difference/texture below this fraction of the run median is a duplicate')
    parser.add_argument('--texture-fraction', type=float, default=.1,
                        help='row pairs with less than this fraction of the run median texture '
                             'cannot be divided by and are counted as untestable')
    args = parser.parse_args()

    blocks = [json.loads(line) for line in (args.mission/'capture_blocks.jsonl').read_text().splitlines()]
    if not blocks:
        raise SystemExit('no archived blocks')
    intervals = list(tag_intervals(blocks))
    archives_config=json.loads((args.mission/'capture_intervals.json').read_text())
    field_paths=[Path(e['archive'])/'ground_geometry_heightfield.json' for e in archives_config]
    ground_mode=bool(field_paths) and all(p.is_file() for p in field_paths)
    if any(p.is_file() for p in field_paths) and not ground_mode:
        raise SystemExit('incomplete archived geometry reference')
    if ground_mode:
        if len({hashlib.sha256(p.read_bytes()).hexdigest() for p in field_paths})!=1:
            raise SystemExit('mixed geometry references')
        field=Heightfield.load(field_paths[0])
        tag_index={(b['block_id'],t['global_line']):t for b in blocks for t in b['pose_tags']}
        for v in intervals:
            out=interval_travel(tag_index[v['block_id'],v['first_line']],tag_index[v['block_id'],v['last_line']],field, v['encoder_m']/v['lines'])
            travel=out['footprint_m'] if out else None
            v.update(footprint_horizontal_m=travel, footprint_ratio=out['footprint_ratio'] if out else None,
                     ground_verdict=ground_verdict(v['encoder_m'],travel,args.ratio_floor))
    ratios = np.array([v['ratio'] for v in intervals])
    slips = sorted((v for v in intervals if v['ratio'] < args.ratio_floor),
                   key=lambda v: v['ratio'])

    centre_alerts=slips.copy()
    if ground_mode:
        slips=[v for v in intervals if v['ground_verdict'] in ('alert','unmeasured')]
    archives = []
    for entry in json.loads((args.mission/'capture_intervals.json').read_text()):
        directory = Path(entry['archive'])
        if directory not in archives:
            archives.append(directory)
    images, missing = locate_images(blocks, archives)
    per_image, worst, floor_seen, evidence = [], [], math.inf, []
    for path in images:
        difference, texture = row_evidence(path, args.column_stride)
        evidence.append((difference, texture))
        floor_seen = min(floor_seen, float(difference.min()))
        per_image.append(dict(image=path.name, rows=int(difference.size)+1,
                              minimum=float(difference.min()),
                              median=float(np.median(difference)),
                              texture_median=float(np.median(texture)),
                              exact_duplicates=int((difference == 0).sum())))
    median = float(np.median([v['median'] for v in per_image])) if per_image else 0.
    texture_median = float(np.median([v['texture_median'] for v in per_image])) if per_image else 0.
    texture_floor = texture_median*args.texture_fraction
    def ratio_of(difference, texture):
        return difference/np.maximum(texture, 1e-6)
    testable = [(texture > 0) & (texture >= texture_floor) for _, texture in evidence]
    pooled = np.concatenate([ratio_of(d, x)[ok] for (d, x), ok in zip(evidence, testable)]) \
        if evidence and any(ok.any() for ok in testable) else np.zeros(0)
    ratio_median = float(np.median(pooled)) if pooled.size else 0.
    threshold = ratio_median*args.duplicate_fraction
    untestable = int(sum(int((~ok).sum()) for ok in testable))
    ratio_floor_seen = float(pooled.min()) if pooled.size else None
    for entry, (difference, texture), ok in zip(per_image, evidence, testable):
        ratio = ratio_of(difference, texture)
        # A featureless row cannot establish motion, even if it repeats exactly.
        flagged = ok & ((ratio < threshold) | (difference == 0))
        if flagged.any():
            rows = np.flatnonzero(flagged)
            worst.append(dict(image=entry['image'], rows=rows.tolist()[:64],
                              count=int(rows.size), minimum=entry['minimum'],
                              ratio_minimum=float(ratio[flagged].min())))

    report = dict(
        schema='agv.scan_slip.v2', mission=str(args.mission),
        ratio_quantity='camera_centre_horizontal_travel_over_nominal_encoder_distance',
        mechanical_slip_confirmed=False,
        legacy_fields={'encoder_vs_ground': 'encoder_vs_camera alias', 'ground_m': 'camera_horizontal_m alias'},
        blocks=len(blocks), images=len(images), lines=sum(v['lines'] for v in intervals),
        pixel_evidence=dict(archives=[str(v) for v in archives], blocks=len(blocks),
                            images=len(images), missing=len(missing), first_missing=missing[:8]),
        encoder_vs_ground=dict(
            intervals=len(intervals), floor=args.ratio_floor,
            median=float(np.median(ratios)), minimum=float(ratios.min()),
            p0_1=float(np.percentile(ratios, .1)), p1=float(np.percentile(ratios, 1)),
            below_floor=len(centre_alerts), worst=centre_alerts[:8]),
        adjacent_rows=dict(
            column_stride=args.column_stride, run_median=median,
            texture_median=texture_median, texture_floor=texture_floor,
            ratio_median=ratio_median, duplicate_threshold=threshold,
            observed_minimum=floor_seen, observed_ratio_minimum=ratio_floor_seen,
            untestable_row_pairs=untestable,
            images_flagged=len(worst), flagged=worst[:8],
            exact_duplicate_rows=sum(v['exact_duplicates'] for v in per_image)),
        scope=('Every archived block must present its own image; a block whose pixels '
               'are missing fails the audit rather than going untested. '
               'Simulation truth pose tags and archived pixels. The ratio test needs '
               'the rendered camera position and cannot run against estimated '
               'navigation, whose centimetre noise hides a millimetre stall.'))
    # An empty image set satisfies "no duplicate rows" vacuously, so the pixel
    # half must first prove it had pixels to test.
    report['encoder_vs_camera'] = report['encoder_vs_ground'].copy()
    report['capture_integrity_passed'] = bool(images) and not missing and not worst
    report['pixel_motion_verifiable'] = bool(pooled.size)
    report['geometry_reference']='triangular_heightfield_centre_ray_v1' if ground_mode else 'legacy_camera_centre'
    report['ground_geometry'] = dict(minimum_encoder_m=.30, ratio_floor=args.ratio_floor,
        ratio_ceiling=1/args.ratio_floor, centre_diagnostic_alerts=len(centre_alerts),
        eligible_intervals=sum(v.get('ground_verdict')=='pass' or v.get('ground_verdict')=='alert' for v in intervals),
        short_intervals=[v for v in intervals if v.get('ground_verdict')=='short_interval'],
        findings=slips, intervals=intervals if ground_mode else [])
    report['geometry_requires_review'] = bool(slips) or (ground_mode and not report['ground_geometry']['eligible_intervals'])
    # Keep the conservative combined gate until the geometry criterion is agreed.
    # A geometry alert must not be renamed a capture failure or confirmed tyre slip.
    report['passed'] = report['capture_integrity_passed'] and report['pixel_motion_verifiable'] and not report['geometry_requires_review']
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print('geometry reference: %s; findings: %d/%d; centre diagnostic alerts: %d' % (report['geometry_reference'],len(slips),len(intervals),len(centre_alerts)))
    print('row difference median %.3f, texture median %.3f' % (median, texture_median))
    print('difference/texture median %.3f, threshold %.3f, observed floor %.3f, untestable pairs %d'
          % (ratio_median, threshold, ratio_floor_seen if ratio_floor_seen is not None else float('nan'), untestable))
    print('images with duplicate rows: %d/%d   exact duplicates: %d'
          % (len(worst), len(images), report['adjacent_rows']['exact_duplicate_rows']))
    if missing:
        print('blocks with no archived image: %d/%d   first: %s'
              % (len(missing), len(blocks), ', '.join(v['image'] for v in missing[:8])))
    print('passed:', report['passed'])
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
