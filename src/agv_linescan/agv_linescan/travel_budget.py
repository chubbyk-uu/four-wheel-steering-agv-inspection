"""Split footprint travel minus encoder travel into separately measured terms.

The split is a telescoping identity: the terms sum to the observed difference
by construction, so the sum proves nothing. What the data has to supply is each
intermediate position - the frozen-mount footprint, the wheel hub, the rolling
prediction - and the identity only guarantees that nothing is dropped between
them.

Two terms are kept apart on purpose. `body_pitch_m` is r*dtheta, the ground the
encoder cannot see because it counts shaft rotation relative to a body that
pitches. `shaft_integration_m` is how far the integrated shaft rate lands from
the line count the encoder actually emitted; it is an accounting difference
between two ways of reading the same encoder, not a mechanical effect.

`rolling_remainder_m` is deliberately not called slip. It is hub travel minus
an approximate rolling prediction, so it also carries steering, ground slope
and the error in rebuilding the hub from the body pose. Contact tangential
speed is the direct slip evidence and is gated behind probe_contact_kinematics.
"""
import numpy as np

TERMS = ('mount_deflection_m', 'lever_geometry_m', 'rolling_remainder_m',
         'body_pitch_m', 'shaft_integration_m')


def integrate_rate(t, rate, start, end):
    """Integrate a piecewise-linear rate over exactly [start, end].

    Snapping the limits to the nearest sample is worth a few tenths of a
    millimetre here, so the ends are interpolated instead. Returns None when
    the window is not inside the samples; the caller must not read that as
    zero travel.
    """
    t = np.asarray(t, float)
    rate = np.asarray(rate, float)
    if t.ndim != 1 or t.shape != rate.shape or t.size < 2:
        return None
    if not np.all(np.diff(t) > 0):
        return None
    if not (t[0] <= start < end <= t[-1]):
        return None
    grid = np.concatenate(([start], t[(t > start) & (t < end)], [end]))
    return float(np.trapz(np.interp(grid, t, rate), grid))


def travel_budget(encoder_m, footprint_m, rigid_mount_footprint_m, hub_travel_m,
                  shaft_rad, body_pitch_rad, radius):
    """The five terms and the observed difference they telescope to."""
    shaft_travel = radius*shaft_rad
    pitch = radius*body_pitch_rad
    out = {'mount_deflection_m': footprint_m - rigid_mount_footprint_m,
           'lever_geometry_m': rigid_mount_footprint_m - hub_travel_m,
           'rolling_remainder_m': hub_travel_m - (shaft_travel + pitch),
           'body_pitch_m': pitch,
           'shaft_integration_m': shaft_travel - encoder_m}
    out['total_m'] = footprint_m - encoder_m
    return out
