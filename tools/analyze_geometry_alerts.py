#!/usr/bin/env python3
"""Re-measure camera/encoder geometry alerts against the ground footprint.

The alert divides encoder travel by the optical centre's travel. The centre
sits about a metre above the road on a body that pitches, so pitch moves it
with nothing sliding. This resolves the centre pixel's ray against the actual
surface and reports both ratios, so the candidate criterion can be judged
against the shipped one on data already collected.

An interval whose ray cannot be solved is reported as unmeasured. It is never
given the centre's value and never counted as a pass.
"""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.scan_footprint import Heightfield, interval_travel   # noqa: E402


def load_tags(run):
    """global_line -> pose tag, plus the line spacing the blocks agree on."""
    tags, spacing = {}, None
    for path in sorted(run.glob('session/raw/*/block_*.json')):
        block = json.loads(path.read_text())
        if spacing is None:
            spacing = block['line_spacing_m']
        elif spacing != block['line_spacing_m']:
            raise SystemExit('line spacing changed inside the run')
        for tag in block['pose_tags']:
            tags[tag['global_line']] = tag
    return tags, spacing


def alerts(run):
    for path in sorted(run.glob('session/raw/*/events.jsonl')):
        for line in path.read_text().splitlines():
            if not line.strip():
                continue
            event = json.loads(line)
            if event.get('reason') == 'camera_encoder_geometry_alert':
                yield event


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, action='append', required=True)
    p.add_argument('--heightfield', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--floor', type=float, default=.95)
    a = p.parse_args()
    field = Heightfield.load(a.heightfield)

    rows, missing = [], 0
    for run in a.run:
        tags, spacing = load_tags(run)
        for event in alerts(run):
            first, last = tags.get(event['first_line']), tags.get(event['last_line'])
            if first is None or last is None:
                missing += 1
                rows.append(dict(run=run.name, simulation_time_s=event['simulation_time_s'],
                                 measured=False, reason='pose tag not archived'))
                continue
            out = interval_travel(first, last, field, spacing)
            if out is None:
                missing += 1
                rows.append(dict(run=run.name, simulation_time_s=event['simulation_time_s'],
                                 measured=False, reason='degenerate interval'))
                continue
            rows.append(dict(run=run.name, simulation_time_s=event['simulation_time_s'],
                             first_line=event['first_line'], last_line=event['last_line'],
                             camera_x_to_m=event['camera_x_to_m'], measured=True, **out))

    solved = [r for r in rows if r.get('footprint_available')]
    unsolved = [r for r in rows if r.get('measured') and not r.get('footprint_available')]
    report = dict(
        schema='agv.geometry_alert_footprint.v1',
        heightfield=str(a.heightfield), floor=a.floor, runs=[str(v) for v in a.run],
        alerts=len(rows), footprint_solved=len(solved),
        footprint_unsolved=len(unsolved), intervals_without_tags=missing,
        centre_ratio_range=[min(r['centre_ratio'] for r in solved),
                            max(r['centre_ratio'] for r in solved)] if solved else None,
        footprint_ratio_range=[min(r['footprint_ratio'] for r in solved),
                               max(r['footprint_ratio'] for r in solved)] if solved else None,
        centre_below_floor=sum(r['centre_ratio'] < a.floor for r in solved),
        footprint_below_floor=sum(r['footprint_ratio'] < a.floor for r in solved),
        scope=('Footprint ratio is a candidate criterion only; the shipped floor and the '
               'shipped centre-based check are unchanged. An unsolved ray is reported, '
               'never replaced by the centre and never counted as a pass.'),
        intervals=rows)
    a.output.write_text(json.dumps(report, indent=2)+'\n')

    print('%-22s %-9s %-10s %-10s %-10s %-9s %-9s' % (
        'run', 'sim_t', 'encoder', 'centre', 'footprint', 'centre_r', 'foot_r'))
    for r in rows:
        if not r.get('measured'):
            print('%-22s %-9.3f  UNMEASURED: %s' % (r['run'], r['simulation_time_s'], r['reason']))
            continue
        foot = '%-10.4f' % r['footprint_m'] if r['footprint_available'] else '%-10s' % 'unsolved'
        fr = '%-9.4f' % r['footprint_ratio'] if r['footprint_available'] else '%-9s' % 'n/a'
        print('%-22s %-9.3f %-10.4f %-10.4f %s %-9.4f %s' % (
            r['run'], r['simulation_time_s'], r['encoder_m'], r['centre_m'], foot,
            r['centre_ratio'], fr))
    print('\nalerts %d, footprint solved %d, unsolved %d, intervals without tags %d'
          % (len(rows), len(solved), len(unsolved), missing))
    if solved:
        print('centre ratio    %.4f - %.4f, below %.2f: %d'
              % (*report['centre_ratio_range'], a.floor, report['centre_below_floor']))
        print('footprint ratio %.4f - %.4f, below %.2f: %d'
              % (*report['footprint_ratio_range'], a.floor, report['footprint_below_floor']))


if __name__ == '__main__':
    main()
