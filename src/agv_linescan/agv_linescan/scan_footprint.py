"""Where the centre pixel's ray actually meets the ground.

The camera/encoder geometry check divides ground travel by the travel of a
point on the vehicle. Using the optical centre compares the encoder against
something a metre above the road on a pitching body, so pitch alone moves it.
"Centre plus height times pitch" is only a small-angle stand-in: it assumes the
optical axis is vertical, ignores roll, and ignores the terrain the ray lands
on. This solves the ray against the surface instead.

Nothing here substitutes a value when it cannot solve. A ray that does not
descend, a solve that will not converge, or a landing outside the surface all
return no result, and the caller must treat that as unmeasured rather than as
zero travel or as a pass.
"""
import json
import math

import numpy as np


class Heightfield:
    """The deterministic road surface, sampled as it was generated."""

    def __init__(self, x, y, z):
        self.x = np.asarray(x, float)
        self.y = np.asarray(y, float)
        self.z = np.asarray(z, float)
        if self.z.shape != (self.y.size, self.x.size):
            raise ValueError('height grid does not match its axes')

    @classmethod
    def load(cls, path):
        d = json.loads(open(path).read())
        return cls(d['x'], d['y'], d['z'])

    @classmethod
    def flat(cls, height=0., extent=1e4):
        return cls([-extent, extent], [-extent, extent],
                   [[height, height], [height, height]])

    def contains(self, x, y):
        return bool(self.x[0] <= x <= self.x[-1] and self.y[0] <= y <= self.y[-1])

    def at(self, x, y):
        """Bilinear sample. Outside the grid there is no surface, not zero."""
        if not self.contains(x, y):
            return None
        i = float(np.interp(x, self.x, np.arange(self.x.size)))
        j = float(np.interp(y, self.y, np.arange(self.y.size)))
        i0, j0 = min(int(i), self.x.size-2), min(int(j), self.y.size-2)
        fi, fj = i-i0, j-j0
        a,b,c,d=self.z[j0,i0],self.z[j0,i0+1],self.z[j0+1,i0+1],self.z[j0+1,i0]
        return float(a*(1-fi)+b*(fi-fj)+c*fj if fj<=fi else a*(1-fj)+c*fi+d*(fj-fi))


def optical_axis(rotation):
    """The viewing direction of the archived camera rotation.

    Tag() stores the camera pose already turned into the optical convention, so
    the third column is where the centre pixel looks.
    """
    r = np.asarray(rotation, float)
    if r.shape != (3, 3) or not np.isfinite(r).all():
        return None
    axis = r[:, 2]
    norm = float(np.linalg.norm(axis))
    if norm == 0:
        return None
    return axis/norm


def ground_intersection(position, rotation, field, tolerance=1e-9, limit=32):
    """Solve the centre ray against the surface. None when it cannot be solved."""
    p = np.asarray(position, float)
    axis = optical_axis(rotation)
    if axis is None or not np.isfinite(p).all():
        return None
    # The ray has to descend to reach a road below it.
    if axis[2] >= -1e-6:
        return None
    height = field.at(p[0], p[1])
    if height is None:
        return None
    for _ in range(limit):
        t = (height - p[2])/axis[2]
        if not math.isfinite(t) or t <= 0:
            return None
        hit = p + t*axis
        sampled = field.at(hit[0], hit[1])
        if sampled is None:
            return None
        if abs(hit[2] - sampled) <= tolerance:
            return hit
        height = sampled
    return None


def interval_travel(first, last, field, spacing):
    """Encoder, optical-centre and footprint travel over one pose-tag interval.

    Travel is unsigned along the scan axis, so a reverse pass is not read as a
    negative distance. Returns None for the footprint when either end fails to
    solve, and the caller must not fall back to the centre.
    """
    lines = last['global_line'] - first['global_line']
    if lines <= 0 or not spacing or spacing <= 0:
        return None
    a = np.asarray(first['camera_position_world_m'], float)
    b = np.asarray(last['camera_position_world_m'], float)
    out = dict(lines=lines, encoder_m=lines*spacing,
               centre_m=float(abs(b[0]-a[0])), footprint_m=None,
               footprint_available=False)
    ga = ground_intersection(a, first['camera_rotation_world'], field)
    gb = ground_intersection(b, last['camera_rotation_world'], field)
    if ga is not None and gb is not None:
        out['footprint_m'] = float(abs(gb[0]-ga[0]))
        out['footprint_available'] = True
    out['centre_ratio'] = out['centre_m']/out['encoder_m']
    out['footprint_ratio'] = (out['footprint_m']/out['encoder_m']
                              if out['footprint_available'] else None)
    return out


def ground_verdict(encoder, travel, floor=.95, minimum=.30):
    if not math.isfinite(encoder) or encoder <= 0 or travel is None or not math.isfinite(travel):
        return 'unmeasured'
    if encoder < minimum:
        return 'short_interval'
    ratio=travel/encoder
    return 'alert' if ratio < floor or ratio > 1/floor else 'pass'
