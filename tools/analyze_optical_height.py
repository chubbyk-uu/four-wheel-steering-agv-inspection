#!/usr/bin/env python3
"""What height the line-scan camera actually images from, during a capture.

The offline contract fixes the lateral geometry at a calibrated height, so
whether that is the height the camera is really at decides the lateral scale.
Two ways of guessing it are both wrong:

  * the nominal 1.5*f/(width*pitch) is unloaded geometry -- the suspension is
    compressed under load before the vehicle has moved;
  * base_link z plus the URDF offset at zero joint angles ignores body pitch
    and the flexible mount's own angle.

Neither is needed. Every pose tag in the archive carries the optical centre's
world pose as the renderer used it, mount deflection and all. Subtracting the
reference heightfield under that point separates "the camera rose because the
road did", which changes no scale, from "the camera rose relative to the road",
which changes it directly.

The result is not a constant, and reporting only its mean would hide the part
that matters.
"""
import argparse
import glob
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/agv_linescan'))
from agv_linescan.heightfield import Heightfield   # noqa: E402


def optics_height(camera):
    """The height at which the model's nominal swath is exactly nominal_width_m."""
    scale = camera['width']*camera['pixel_pitch_m']/(2*camera['focal_length_m'])
    return camera['nominal_width_m']/(2*scale), scale


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--session', type=Path, required=True, help='a run directory holding session/raw')
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--profile', type=Path, help='measured calibration profile, for its metric polynomial')
    p.add_argument('--output', type=Path)
    a = p.parse_args()

    import yaml
    camera = yaml.safe_load((ROOT/'src/agv_description/config/linescan.yaml').read_text())
    nominal, scale = optics_height(camera)
    manifest = json.loads(a.scene.read_text())
    field = Heightfield(a.scene.resolve().parent
                        / manifest['assets'][0]['collision_proxy']['heightfield']['mesh'])

    blocks = sorted(glob.glob(str(a.session/'session/raw/*/block_*.json')))
    tags = [g['camera_position_world_m']
            for b in blocks for g in json.loads(Path(b).read_text())['pose_tags']]
    xyz = np.array(tags, float)
    on = ((xyz[:, 0] >= field.x[0]) & (xyz[:, 0] <= field.x[-1])
          & (xyz[:, 1] >= field.y[0]) & (xyz[:, 1] <= field.y[-1]))
    xyz = xyz[on]
    ground = field.sample(xyz[:, :2])
    above = xyz[:, 2]-ground

    def spread(v, factor=1.):
        return dict(mean=float(v.mean()*factor), sd=float(v.std()*factor),
                    p5=float(np.percentile(v, 5)*factor), p95=float(np.percentile(v, 95)*factor),
                    min=float(v.min()*factor), max=float(v.max()*factor))

    report = dict(schema='agv.optical_height_in_capture.v1', session=str(a.session),
                  blocks=len(blocks), pose_tags=int(len(xyz)),
                  nominal_unloaded_height_m=nominal,
                  height_above_local_ground_m=spread(above),
                  ground_under_camera_m=spread(ground),
                  note=('the ground column is how much of the variation is the camera following '
                        'the road, which changes no scale; the rest is attitude, which does'))
    report['lateral_scale_vs_nominal_percent'] = spread(nominal/above-1, 100.)

    if a.profile:
        profile = json.loads(a.profile.read_text())
        coefficients = profile['geometry']['metric_polynomial']
        # Lowest order first, as the ray polynomial is; the linear term is the
        # half-swath, which the model makes scale * height.
        implied = coefficients[1]/scale
        report['calibration'] = dict(
            path=str(a.profile), metric_polynomial=coefficients,
            output_width_m=profile['geometry']['output_width_m'],
            fit_error_px_max=profile['geometry']['fit_error_px_max'],
            implied_height_m=implied,
            note=('the correction resamples through this polynomial, not by spreading 4096 raw '
                  'columns over output_width_m, so output_width_m alone proves nothing about '
                  'scale. implied_height_m is inferred from the linear term under the shipped '
                  'optics model; the profile does not state the height it was taken at'))
        report['lateral_scale_vs_calibration_percent'] = spread(implied/above-1, 100.)

    pixel = camera['nominal_width_m']/camera['width']
    sd = report.get('lateral_scale_vs_calibration_percent',
                    report['lateral_scale_vs_nominal_percent'])['sd']
    report['at_half_metre_off_centre_px'] = dict(
        sd=sd/100*.5/pixel, basis='0.5 m * scale error / %.9f m per pixel' % pixel)
    text = json.dumps(report, indent=2)+'\n'
    if a.output:
        a.output.write_text(text)
    print(text)


if __name__ == '__main__':
    main()
