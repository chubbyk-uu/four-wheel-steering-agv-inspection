#!/usr/bin/env python3
"""Where the scan line actually lands, from archived pose tags alone.

An earlier version of this took only the optical centre's height and turned it
into a scale with calibrated/actual - 1. That is a vertical-camera, level-ground
approximation: real pitch and roll turn the rays as well as move them, the
ground under the optical centre is not where the line lands, and roll makes the
two ends of one row scale differently, which no single number can carry.

So use the whole pose. Each tag stores the optical centre and its orientation
as the renderer used it. For a set of columns this casts the shipped ray model
through that pose, intersects the reference heightfield, and measures the
across-track coordinate the row really covers. Comparing that with the
calibration's own metric polynomial splits the error in two:

  * the affine part -- an offset and a scale -- which is what C(s) + q B(s) can
    represent, so it bounds how much B has to be allowed to move;
  * whatever is left, the in-row nonlinearity, which that model cannot express
    at all and which therefore lands in the seam.

Everything is reported per phase, because a number pooled over lead-in,
cruise and run-out describes no part of the run. Ground truth is used here for
diagnosis only; none of this goes into the stitching optimiser.
"""
import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/agv_linescan'))
from agv_linescan.heightfield import Heightfield   # noqa: E402

COLUMNS = np.array([-1., -.75, -.5, -.25, 0., .25, .5, .75, 1.])


def polynomial(coefficients, x):
    """Lowest order first, matching Polynomial() in sampling.hpp."""
    y = np.zeros_like(x)
    for c in reversed(coefficients):
        y = y*x+c
    return y


def intersect(origin, direction, field, steps=6):
    """Where a ray meets the reference surface. Nearly flat, so iterate."""
    t = origin[..., 2]/-direction[..., 2]
    for _ in range(steps):
        point = origin+t[..., None]*direction
        inside = ((point[..., 0] >= field.x[0]) & (point[..., 0] <= field.x[-1])
                  & (point[..., 1] >= field.y[0]) & (point[..., 1] <= field.y[-1]))
        if not inside.all():
            return None, inside
        t = (origin[..., 2]-field.sample(point[..., :2]))/-direction[..., 2]
    return origin+t[..., None]*direction, inside


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--session', type=Path, required=True)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--profile', type=Path, help='measured calibration profile')
    p.add_argument('--camera', type=Path, help='override; the session copy is used by default')
    p.add_argument('--cruise-fraction', type=float, default=.9,
                   help='fraction of the requested speed above which a tag counts as cruise')
    p.add_argument('--output', type=Path)
    a = p.parse_args()

    # The configuration the run used, not the one the repository has now: a
    # later focal-length change must not move the numbers for old data.
    camera_path = a.camera or a.session/'session/camera.yaml'
    if not camera_path.is_file():
        raise SystemExit('no archived camera config at %s; pass --camera only if you know '
                         'which one the run used' % camera_path)
    camera = yaml.safe_load(camera_path.read_text())
    scale = camera['width']*camera['pixel_pitch_m']/(2*camera['focal_length_m'])
    nominal_height = camera['nominal_width_m']/(2*scale)
    pixel = camera['nominal_width_m']/camera['width']

    manifest = json.loads(a.scene.read_text())
    field = Heightfield(a.scene.resolve().parent
                        / manifest['assets'][0]['collision_proxy']['heightfield']['mesh'])
    roi = manifest['inspection_bounds_xy_m']

    tags = []
    for block in sorted(glob.glob(str(a.session/'session/raw/*/block_*.json'))):
        meta = json.loads(Path(block).read_text())
        tags += [(g['time_s'], g['encoder_distance_m'], g['camera_position_world_m'],
                  g['camera_rotation_world']) for g in meta['pose_tags']]
    tags.sort()
    time = np.array([t[0] for t in tags])
    distance = np.array([t[1] for t in tags])
    centre = np.array([t[2] for t in tags])
    rotation = np.array([t[3] for t in tags])

    # Speed between neighbouring tags; a jump in time means a new pass, so the
    # pair straddling it is dropped rather than averaged across the gap.
    dt = np.diff(time, prepend=time[0])
    speed = np.where(dt > 0, np.abs(np.diff(distance, prepend=distance[0]))/np.where(dt > 0, dt, 1), np.nan)
    speed[0] = np.nan
    speed[dt > 5] = np.nan

    # Ray directions in the optical frame: lateral tangent from the shipped
    # polynomial, forward along +z, exactly as Optics builds them.
    tangent = scale*polynomial(camera['ray_polynomial'], COLUMNS)
    local = np.stack([tangent, np.zeros_like(tangent), np.ones_like(tangent)], axis=1)
    world = np.einsum('nij,kj->nki', rotation, local)
    origin = np.repeat(centre[:, None, :], len(COLUMNS), axis=1)
    hit, inside = intersect(origin, world, field)
    if hit is None:
        keep = inside.all(axis=1)
        origin, world = origin[keep], world[keep]
        centre, rotation, speed = centre[keep], rotation[keep], speed[keep]
        hit, _ = intersect(origin, world, field)
    middle = len(COLUMNS)//2

    # Across-track coordinate along the horizontal projection of the scan axis.
    axis = rotation@np.array([1., 0., 0.])
    axis[:, 2] = 0
    axis /= np.linalg.norm(axis, axis=1)[:, None]
    across = np.einsum('nki,ni->nk', hit-hit[:, middle][:, None, :], axis)

    reference = (polynomial(a.profile and json.loads(a.profile.read_text())['geometry']
                            ['metric_polynomial'] or [0., scale*nominal_height, 0., 0.], COLUMNS)
                 if a.profile else scale*nominal_height*polynomial(camera['ray_polynomial'], COLUMNS))
    reference = reference-reference[middle]

    # Affine fit per row: what C(s) + q B(s) can absorb, and what it cannot.
    design = np.stack([reference, np.ones_like(reference)], axis=1)
    solution, *_ = np.linalg.lstsq(design, across.T, rcond=None)
    fitted = (design@solution).T
    residual = across-fitted
    scale_error = solution[0]-1
    worst = np.abs(residual).max(axis=1)

    # The leftover is symmetric in the column, which is what a roll does: one
    # end of the row reaches further than the other end comes back. Fitting
    # that term separately says how much of it is a fixed mount angle, which a
    # calibration can remove, and how much moves row to row, which it cannot.
    curved = np.stack([reference, np.ones_like(reference), COLUMNS**2], axis=1)
    quadratic, *_ = np.linalg.lstsq(curved, across.T, rcond=None)
    after_roll = across-(curved@quadratic).T
    roll_term = quadratic[2]                      # metres at the column edge

    ground_under_line = field.sample(hit[:, middle, :2])
    height = hit[:, middle, 2]*0+centre[:, 2]-ground_under_line

    cruise_speed = a.cruise_fraction*np.nanmax(speed)
    phase = np.where(np.isnan(speed), 'unknown',
                     np.where(speed >= cruise_speed, 'cruise', 'transient'))
    in_roi = ((hit[:, middle, 0] >= roi[0]) & (hit[:, middle, 0] <= roi[1])
              & (hit[:, middle, 1] >= roi[2]) & (hit[:, middle, 1] <= roi[3]))

    def spread(v, factor=1.):
        v = np.asarray(v, float)
        if not len(v):
            return None
        return dict(n=int(len(v)), mean=float(v.mean()*factor), sd=float(v.std()*factor),
                    p5=float(np.percentile(v, 5)*factor), p95=float(np.percentile(v, 95)*factor),
                    min=float(v.min()*factor), max=float(v.max()*factor))

    groups = {'all': np.ones(len(scale_error), bool),
              'cruise_in_roi': (phase == 'cruise') & in_roi,
              'cruise_outside_roi': (phase == 'cruise') & ~in_roi,
              'transient': phase == 'transient'}
    report = dict(
        schema='agv.scan_line_geometry.v1', session=str(a.session),
        camera_config=str(camera_path), calibration=str(a.profile) if a.profile else None,
        pose_tags=int(len(scale_error)), columns=COLUMNS.tolist(),
        cruise_threshold_m_s=float(cruise_speed), pixel_m=pixel,
        reference=('the calibration metric polynomial' if a.profile
                   else 'the shipped ray model at the nominal height'),
        scope=('reference-heightfield intersection of the shipped ray model through the archived '
               'optical pose. It carries pitch, roll and mount deflection, which the height-only '
               'estimate it replaces did not. It is still the reference surface, not the optical '
               'mesh, and the tags are sparse, so it does not resolve anything faster than their '
               'spacing.'),
        by_phase={name: dict(
            affine_scale_error_percent=spread(scale_error[mask], 100.),
            in_row_nonlinearity_px=spread(worst[mask]/pixel),
            # Per column, because the seam is not at the edge of the swath: the
            # overlap sits near |q| = 0.67, where the residual is smaller than
            # the worst-column figure above.
            in_row_nonlinearity_rms_px_by_column=[
                float(np.sqrt((residual[mask, k]**2).mean())/pixel) for k in range(len(COLUMNS))],
            # Split again: a quadratic column term is what a roll produces. Its
            # mean is a fixed angle, which recalibration can take out; its
            # spread is not, and neither is what remains after removing it.
            roll_like_term_px=spread(roll_term[mask]/pixel),
            after_removing_roll_px=spread(np.abs(after_roll[mask]).max(axis=1)/pixel),
            optical_height_above_ground_m=spread(height[mask])) for name, mask in groups.items()
            if mask.any()})
    text = json.dumps(report, indent=2)+'\n'
    if a.output:
        a.output.write_text(text)
    print(text)


if __name__ == '__main__':
    main()
