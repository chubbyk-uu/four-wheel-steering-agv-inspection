"""Exact-time integration, and terms that stay separate instead of absorbing each other."""
import numpy as np
import pytest

from agv_linescan.travel_budget import TERMS, integrate_rate, travel_budget

RADIUS = 0.2


def test_constant_rate_between_samples_is_exact():
    """Snapping the limits to the nearest sample is the error this removes."""
    t = np.arange(0., 2.001, .001)
    rate = np.full_like(t, 5.)
    start, end = 0.10049, 1.40071
    assert integrate_rate(t, rate, start, end) == pytest.approx(5*(end-start), abs=1e-12)
    snapped = 5*(round(end, 3) - round(start, 3))
    assert abs(snapped - 5*(end-start)) > 1e-4         # the shortcut really does differ


def test_linear_ramp_is_exact():
    t = np.arange(0., 1.0001, .001)
    rate = 3.*t
    start, end = 0.20025, 0.90075
    assert integrate_rate(t, rate, start, end) == pytest.approx(1.5*(end**2-start**2), rel=1e-9)


def test_window_outside_the_samples_is_unmeasured_not_zero():
    t = np.arange(0., 1.0001, .001)
    rate = np.ones_like(t)
    assert integrate_rate(t, rate, -0.5, 0.5) is None
    assert integrate_rate(t, rate, 0.5, 1.5) is None
    assert integrate_rate(t, rate, 0.5, 0.5) is None
    assert integrate_rate(t[::-1], rate, 0.1, 0.2) is None


def test_rigid_level_travel_leaves_every_term_zero():
    travel = 0.374
    b = travel_budget(encoder_m=travel, footprint_m=travel,
                      rigid_mount_footprint_m=travel, hub_travel_m=travel,
                      shaft_rad=travel/RADIUS, body_pitch_rad=0., radius=RADIUS)
    for name in TERMS:
        assert b[name] == pytest.approx(0., abs=1e-12)


def test_pitch_term_is_r_dtheta_and_does_not_absorb_the_shaft_difference():
    """The encoder cannot see r*dtheta; that is separate from how it was read."""
    dtheta = -0.0174533                                 # one degree nose-up
    shaft = 1.87
    encoder = RADIUS*shaft - 0.00069                    # line count lags the integral
    hub = RADIUS*(shaft + dtheta)                       # perfect rolling
    b = travel_budget(encoder_m=encoder, footprint_m=hub, rigid_mount_footprint_m=hub,
                      hub_travel_m=hub, shaft_rad=shaft, body_pitch_rad=dtheta,
                      radius=RADIUS)
    assert b['body_pitch_m'] == pytest.approx(RADIUS*dtheta, abs=1e-15)
    assert b['shaft_integration_m'] == pytest.approx(0.00069, abs=1e-12)
    assert b['rolling_remainder_m'] == pytest.approx(0., abs=1e-15)


def test_mount_and_lever_stay_apart():
    hub, mount, lever = 0.370, 0.0026, 0.0037
    rigid = hub + lever
    b = travel_budget(encoder_m=hub, footprint_m=rigid + mount,
                      rigid_mount_footprint_m=rigid, hub_travel_m=hub,
                      shaft_rad=hub/RADIUS, body_pitch_rad=0., radius=RADIUS)
    assert b['mount_deflection_m'] == pytest.approx(mount, abs=1e-15)
    assert b['lever_geometry_m'] == pytest.approx(lever, abs=1e-15)
    assert b['rolling_remainder_m'] == pytest.approx(0., abs=1e-15)


def test_rolling_remainder_is_not_attributed_to_slip():
    """Hub travel short of the rolling prediction lands in one named remainder."""
    shaft, dtheta, gap = 1.87, -0.0174533, -0.00047
    hub = RADIUS*(shaft + dtheta) + gap
    b = travel_budget(encoder_m=RADIUS*shaft, footprint_m=hub,
                      rigid_mount_footprint_m=hub, hub_travel_m=hub,
                      shaft_rad=shaft, body_pitch_rad=dtheta, radius=RADIUS)
    assert b['rolling_remainder_m'] == pytest.approx(gap, abs=1e-15)
    assert b['mount_deflection_m'] == pytest.approx(0., abs=1e-15)


def test_the_sum_is_an_identity_and_therefore_proves_nothing():
    """Deliberately inconsistent inputs still sum; the sum is not a check."""
    b = travel_budget(encoder_m=0.374, footprint_m=0.9, rigid_mount_footprint_m=0.1,
                      hub_travel_m=-0.4, shaft_rad=12., body_pitch_rad=3.,
                      radius=RADIUS)
    assert sum(b[name] for name in TERMS) == pytest.approx(b['total_m'], abs=1e-12)
    assert b['total_m'] == pytest.approx(0.9 - 0.374, abs=1e-12)
