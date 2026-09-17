#!/usr/bin/env python3
"""Account for the difference between footprint travel and encoder travel.

The footprint ratio is not exactly one. This splits the difference into four
independently measured terms, so the remainder is a statement about what is
still unexplained rather than about what was never measured.

  mount deflection  the camera pose archived in the tag against the body pose
                    archived in the flight recorder. Their relative rotation is
                    constant for a rigid mount, so its change is the deflection.
                    Measured by re-solving the footprint with the mount frozen
                    at the interval start; no contact-kinematics probe needed.
  lever/geometry    rigid-mount footprint against the fl wheel centre rebuilt
                    from the body pose, suspension travel and the URDF chain.
  rolling remainder wheel centre travel against the rolling prediction. Not
                    called slip: it also carries steering, ground slope and the
                    error in rebuilding the hub from the body pose, and contact
                    tangential speed - the direct slip evidence - is gated
                    behind probe_contact_kinematics and was never collected.
  body pitch        r*dtheta. The encoder counts shaft rotation *relative to
                    the body*, so a body that pitches while rolling covers
                    ground the encoder never sees.
  shaft integration how far the integrated shaft rate lands from the line count
                    the encoder emitted. An accounting difference between two
                    readings of the same encoder, not a mechanical effect.

The terms telescope, so their sum is the observed difference by construction
and the sum is not a check on the attribution. The evidence is each
intermediate position, measured separately.

Nothing is filled in. An interval the flight window does not cover, or whose
ray will not solve, is reported as unmeasured and excluded from the summary.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.scan_footprint import Heightfield, ground_intersection  # noqa: E402
from agv_linescan.travel_budget import TERMS, integrate_rate, travel_budget  # noqa: E402

ALERT = 'camera_encoder_geometry_alert'


def read_run(run):
    tags = {}
    for path in sorted(run.glob('session/raw/*/block_*.json')):
        for tag in json.loads(path.read_text())['pose_tags']:
            tags[tag['global_line']] = tag
    dumps = [json.loads(p.read_text())
             for p in sorted(run.glob(f'session/raw/*/flight_*_{ALERT}.json'))]
    events = []
    for path in sorted(run.glob('session/raw/*/events.jsonl')):
        for line in path.read_text().splitlines():
            if line.strip() and json.loads(line).get('reason') == ALERT:
                events.append(json.loads(line))
    return tags, dumps, events


def budget(event, tags, dump, field, radius, wheelbase, track, hub_z):
    """One interval, or a dict saying why it could not be measured."""
    first, last = tags.get(event['first_line']), tags.get(event['last_line'])
    if first is None or last is None:
        return dict(measured=False, reason='pose tag not archived')
    t0, t1 = first['time_s'], last['time_s']
    records = dump['records']
    t = np.array([r['t'] for r in records])
    if not (t[0] <= t0 and t1 <= t[-1]):
        return dict(measured=False, reason='flight window does not cover the interval',
                    window_s=[float(t[0]), float(t[-1])], interval_s=[t0, t1])

    body = Slerp(t, Rotation.from_quat(np.array([r['body_quat_xyzw'] for r in records])))
    xyz = np.array([r['body_xyz'] for r in records])
    rb0, rb1 = body([t0])[0], body([t1])[0]
    pb0 = np.array([np.interp(t0, t, xyz[:, k]) for k in range(3)])
    pb1 = np.array([np.interp(t1, t, xyz[:, k]) for k in range(3)])

    pc0 = np.asarray(first['camera_position_world_m'], float)
    pc1 = np.asarray(last['camera_position_world_m'], float)
    rc0 = Rotation.from_matrix(np.asarray(first['camera_rotation_world'], float))
    rc1 = Rotation.from_matrix(np.asarray(last['camera_rotation_world'], float))

    # Camera in the body frame. Constant if the mount is rigid, so replaying the
    # start value onto the end body pose removes exactly the mount's motion.
    mount0 = rb0.inv()*rc0
    offset0 = rb0.inv().apply(pc0 - pb0)
    mount_change = (mount0.inv()*(rb1.inv()*rc1)).magnitude()

    g0 = ground_intersection(pc0, rc0.as_matrix(), field)
    g1 = ground_intersection(pc1, rc1.as_matrix(), field)
    g1r = ground_intersection(pb1 + rb1.apply(offset0), (rb1*mount0).as_matrix(), field)
    if g0 is None or g1 is None or g1r is None:
        return dict(measured=False, reason='centre ray does not solve against the surface')

    forward = rb0.apply([1., 0., 0.])
    footprint = float(np.dot(np.asarray(g1) - np.asarray(g0), forward))
    rigid = float(np.dot(np.asarray(g1r) - np.asarray(g0), forward))

    # Exact interval limits throughout; snapping to the nearest physics step is
    # worth a few tenths of a millimetre on these terms.
    susp = np.array([r['suspension_m'][0] for r in records])
    susp0, susp1 = float(np.interp(t0, t, susp)), float(np.interp(t1, t, susp))
    hub0 = pb0 + rb0.apply([wheelbase/2, track/2, hub_z + susp0])
    hub1 = pb1 + rb1.apply([wheelbase/2, track/2, hub_z + susp1])
    hub = float(np.dot(hub1 - hub0, forward))

    steer_series = [r['steer_rad'][0] for r in records]
    steer = integrate_rate(t, steer_series, t0, t1)
    shaft = integrate_rate(t, [r['drive_rate_rad_s'][0] for r in records], t0, t1)
    if steer is None or shaft is None:
        return dict(measured=False, reason='rate integration window is not inside the samples')
    steer /= (t1 - t0)
    axis = [-np.sin(steer), np.cos(steer), 0.]          # fl drive axis in the body frame
    pitch = float(np.dot((rb0.inv()*rb1).as_rotvec(), axis))
    encoder = event['encoder_m']

    terms = travel_budget(encoder, footprint, rigid, hub, shaft, pitch, radius)
    return dict(measured=True, lines=event['last_line']-event['first_line'],
                interval_s=[t0, t1], camera_x_to_m=event['camera_x_to_m'],
                probe_contact_kinematics=dump['probe_contact_kinematics'],
                encoder_m=encoder, footprint_m=footprint, hub_travel_m=hub,
                rigid_mount_footprint_m=rigid, mean_steer_rad=steer,
                shaft_relative_rad=shaft, body_pitch_about_axle_rad=pitch,
                mount_change_rad=float(mount_change),
                **{k[:-2]+'_mm': 1000*v for k, v in terms.items()})


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run', type=Path, action='append', required=True)
    p.add_argument('--heightfield', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--wheel-radius', type=float, default=.2)
    p.add_argument('--wheelbase', type=float, default=1.3)
    p.add_argument('--track', type=float, default=.94)
    p.add_argument('--hub-z', type=float, default=-.45,
                   help='base_link to wheel centre at zero suspension travel')
    a = p.parse_args()
    field = Heightfield.load(a.heightfield)

    rows = []
    for run in a.run:
        tags, dumps, events = read_run(run)
        for event in events:
            dump = min(dumps, key=lambda d: abs(d['fault_simulation_time_s']
                                                - event['simulation_time_s']))
            row = budget(event, tags, dump, field, a.wheel_radius,
                         a.wheelbase, a.track, a.hub_z)
            rows.append(dict(run=run.name, simulation_time_s=event['simulation_time_s'], **row))

    terms = [k[:-2]+'_mm' for k in TERMS] + ['total_mm']
    solved = [r for r in rows if r['measured']]
    summary = {k: dict(mean=float(np.mean([r[k] for r in solved])),
                       stdev=float(np.std([r[k] for r in solved])),
                       min=float(min(r[k] for r in solved)),
                       max=float(max(r[k] for r in solved))) for k in terms} if solved else None
    report = dict(schema='agv.geometry_residual_budget.v1',
                  heightfield=str(a.heightfield), runs=[str(v) for v in a.run],
                  alerts=len(rows), measured=len(solved),
                  unmeasured=[{k: r[k] for k in ('run', 'simulation_time_s', 'reason')}
                              for r in rows if not r['measured']],
                  scope=('Decomposition of footprint travel minus encoder travel into five '
                         'separately measured terms. The terms telescope, so their sum is the '
                         'observed difference BY CONSTRUCTION and is not a check on the '
                         'attribution; the evidence is each intermediate position. '
                         'rolling_remainder is not slip: it also carries steering, ground '
                         'slope and hub reconstruction error, and contact tangential speed was '
                         'never collected. shaft_integration is an accounting difference '
                         'between the integrated shaft rate and the emitted line count, not a '
                         'mechanical effect. An uncovered window or an unsolved ray is '
                         'reported, never given a value.'),
                  summary_mm=summary, intervals=rows)
    a.output.write_text(json.dumps(report, indent=2)+'\n')

    head = ('run', 'sim_t', 'mount', 'lever', 'remain', 'pitch', 'shaft_int', 'total')
    print('%-20s %-9s | %-9s %-9s %-9s %-9s %-10s | %-9s' % head)
    for r in rows:
        if not r['measured']:
            print('%-20s %-9.3f   UNMEASURED: %s' % (r['run'], r['simulation_time_s'], r['reason']))
            continue
        print('%-20s %-9.3f | %-9.3f %-9.3f %-9.3f %-9.3f %-10.3f | %-9.3f'
              % (r['run'], r['simulation_time_s'], *(r[k] for k in terms)))
    if summary:
        print()
        for k in terms:
            s = summary[k]
            print('%-22s mean %+7.3f  sd %5.3f  range %+7.3f .. %+7.3f mm'
                  % (k[:-3], s['mean'], s['stdev'], s['min'], s['max']))
    print('\n%d alerts, %d measured, %d unmeasured' % (len(rows), len(solved), len(rows)-len(solved)))


if __name__ == '__main__':
    main()
