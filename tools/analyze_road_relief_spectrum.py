#!/usr/bin/env python3
"""What the road's own relief does to a two-dimensional strip model.

Two numbers decide whether P(s,q) = C(s) + q B(s) can carry this road, and
both come from the reference heightfield the optical mesh is built from, so
neither needs a run:

  * cross-strip lateral parallax. Adjacent passes view the same ground from
    positions one track spacing apart, so a point at height z back-projects to
    the z=0 plane at two places, b*z/h apart. That is geometry, not error.
  * the wavelength it lives at. A parallax that sits above the spline's node
    spacing is not left behind as a clean residual -- it is absorbed into C and
    B, which means the seam closes by deforming the road. Short-wavelength
    content would at least show up as residual; long-wavelength content is the
    dangerous kind.

The longitudinal column applies the pitch forcing a two-point wheelbase
difference produces, |2 sin(pi L / lambda)|, and the camera lever h. It is a
quasi-static estimate of the excitation, not a vehicle response: suspension,
speed and mount dynamics are not in it, and the comb has zeros at lambda = L/n,
so this does not license reading a node spacing straight off a wheelbase.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.heightfield import Heightfield   # noqa: E402

BANDS = ((0.4, 0.8), (0.8, 1.6), (1.6, 3.2), (3.2, 6.4), (6.4, float('inf')))


def spectrum(z, dx, wheelbase, height):
    rows = z - z.mean(axis=1, keepdims=True)
    n = rows.shape[1]
    power = (np.abs(np.fft.rfft(rows*np.hanning(n), axis=1))**2).mean(axis=0)
    freq = np.fft.rfftfreq(n, dx)
    lam = np.divide(1., freq, out=np.full_like(freq, np.inf), where=freq > 0)
    # Longitudinal displacement = camera lever x slope across the wheelbase.
    deformation = power*(np.abs(2*np.sin(np.pi*wheelbase*freq))/wheelbase*height)**2
    return power, deformation, lam


def fraction(power, lam, lo, hi):
    return float(power[(lam >= lo) & (lam < hi)].sum()/power[1:].sum())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--asset', default=None, help='asset name; default the first one')
    p.add_argument('--track-spacing-m', type=float, default=1.0)
    p.add_argument('--camera-height-m', type=float, default=1.046316964)
    p.add_argument('--wheelbase-m', type=float, default=1.3)
    p.add_argument('--pixel-m', type=float, default=1.5/4096)
    p.add_argument('--road-half-width-m', type=float, default=5.0)
    p.add_argument('--output', type=Path)
    a = p.parse_args()

    root = a.scene.resolve().parent
    manifest = json.loads(a.scene.read_text())
    asset = (next(v for v in manifest['assets'] if v['name'] == a.asset) if a.asset
             else manifest['assets'][0])
    field = Heightfield(root/asset['collision_proxy']['heightfield']['mesh'])
    on_road = np.abs(field.y) <= a.road_half_width_m
    z = field.z[on_road]
    dx = float(np.diff(field.x).mean())

    # Parallax is the height itself, scaled: two views one spacing apart.
    disparity = a.track_spacing_m*np.abs(z)/a.camera_height_m
    power, deformation, lam = spectrum(z, dx, a.wheelbase_m, a.camera_height_m)

    report = dict(
        schema='agv.road_relief_spectrum.v1', scene=str(a.scene), asset=asset['name'],
        geometry=dict(track_spacing_m=a.track_spacing_m, camera_height_m=a.camera_height_m,
                      wheelbase_m=a.wheelbase_m, pixel_m=a.pixel_m,
                      grid_m=dx, rows_on_road=int(on_road.sum())),
        height_mm=dict(rms=float(z.std()*1e3), peak=float(np.abs(z).max()*1e3)),
        cross_strip_parallax_px=dict(
            formula='track_spacing * z / camera_height',
            rms=float(np.sqrt((disparity**2).mean())/a.pixel_m),
            p95=float(np.percentile(disparity, 95)/a.pixel_m),
            peak=float(disparity.max()/a.pixel_m),
            note=('geometry, not error: a two-dimensional strip map cannot represent it, and at '
                  'any one station it is exactly collinear with the two passes\' differential '
                  'roll, so it does not by itself yield a height map')),
        power_by_wavelength=[dict(band_m=[lo, hi if hi != float('inf') else None],
                                  height=round(fraction(power, lam, lo, hi), 4),
                                  longitudinal_deformation=round(fraction(deformation, lam, lo, hi), 4))
                             for lo, hi in BANDS],
        node_spacing=[dict(spacing_m=d, represents_wavelength_from_m=2*d,
                           captures_longitudinal_deformation=round(
                               float(deformation[lam >= 2*d].sum()/deformation[1:].sum()), 4))
                      for d in (.25, .5, 1., 2.)],
        caveat=('the longitudinal column is quasi-static excitation, not vehicle response; the '
                'two-point wheelbase difference is a comb with zeros at lambda = wheelbase/n, so '
                'no node spacing follows from the wheelbase alone. Node spacing is decided by '
                'held-out error, with these fractions as the prior.'))
    text = json.dumps(report, indent=2)+'\n'
    if a.output:
        a.output.write_text(text)
    print(text)


if __name__ == '__main__':
    main()
