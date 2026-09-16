"""The footprint solve must separate pitch from sliding, and refuse what it cannot see."""
import math

import numpy as np
import pytest

from agv_linescan.scan_footprint import (Heightfield, ground_intersection,
                                         interval_travel, optical_axis)

HEIGHT = 1.0416          # camera optical centre above the contact plane
SPACING = 0.0003657010198


def pitched(contact_x, theta, height=HEIGHT, mount=0.):
    """A camera rigidly above a contact point, with the body pitched by theta.

    Rotating the body about its contact point puts the centre at
    contact + R(theta)*(0,0,h); the optical axis tilts with it. A mount
    deflection adds rotation without moving the centre.
    """
    c, s = math.cos(theta), math.sin(theta)
    position = [contact_x + height*s, 0., height*c]
    total = theta + mount
    ct, st = math.cos(total), math.sin(total)
    # Columns of the optical frame; the third is the viewing direction.
    rotation = [[ct, 0., -st], [0., 1., 0.], [st, 0., -ct]]
    return dict(camera_position_world_m=position, camera_rotation_world=rotation)


def tag(line, contact_x, theta=0., mount=0.):
    d = pitched(contact_x, theta, mount=mount)
    d['global_line'] = line
    return d


def test_rigid_level_constant_speed_is_one_to_one():
    field = Heightfield.flat(0.)
    lines = 1024
    travel = lines*SPACING
    out = interval_travel(tag(0, 0.), tag(lines, travel), field, SPACING)
    assert out['footprint_available']
    assert out['footprint_ratio'] == pytest.approx(1., abs=1e-9)
    assert out['centre_ratio'] == pytest.approx(1., abs=1e-9)


def test_pitch_without_sliding_moves_the_centre_but_not_the_footprint():
    """The case that produced every alert: the ground kept up, the centre did not."""
    field = Heightfield.flat(0.)
    # Negative pitch is the observed direction: the body rotates so the camera
    # is carried backwards while the contact point keeps advancing. The real
    # alerts recorded -12.269 mrad over one such interval.
    lines, theta = 1024, -math.radians(1.2)
    travel = lines*SPACING
    out = interval_travel(tag(0, 0., 0.), tag(lines, travel, theta), field, SPACING)
    # The footprint tracks the contact point exactly, whatever the body does.
    assert out['footprint_ratio'] == pytest.approx(1., abs=1e-6)
    # The centre lags by the camera height times the pitch, which on this scale
    # crosses a 0.95 floor on its own with nothing sliding.
    assert out['centre_m'] < out['encoder_m']
    assert out['encoder_m'] - out['centre_m'] == pytest.approx(HEIGHT*math.sin(-theta), abs=1e-6)
    assert out['centre_ratio'] < .95 < out['footprint_ratio']
    # And the same pitch with the opposite sign pushes the centre ahead, which
    # a ratio floor would never notice at all.
    ahead = interval_travel(tag(0, 0., 0.), tag(lines, travel, -theta), field, SPACING)
    assert ahead['centre_ratio'] > 1. and ahead['footprint_ratio'] == pytest.approx(1., abs=1e-6)


def test_mount_deflection_moves_the_footprint_the_centre_cannot_see():
    """Rotation without translation: the centre says nothing happened."""
    field = Heightfield.flat(0.)
    mount = math.radians(.4)
    out = interval_travel(tag(0, 0.), tag(512, 0., mount=mount), field, SPACING)
    assert out['centre_m'] == pytest.approx(0., abs=1e-12)
    assert out['footprint_available'] and out['footprint_m'] > 0
    assert out['footprint_m'] == pytest.approx(HEIGHT*math.tan(mount), abs=1e-6)


def test_a_reverse_pass_is_not_negative_travel():
    field = Heightfield.flat(0.)
    lines = 1024
    travel = lines*SPACING
    forward = interval_travel(tag(0, 0.), tag(lines, travel), field, SPACING)
    reverse = interval_travel(tag(0, travel), tag(lines, 0.), field, SPACING)
    assert reverse['footprint_ratio'] == pytest.approx(forward['footprint_ratio'], abs=1e-12)
    assert reverse['centre_ratio'] == pytest.approx(forward['centre_ratio'], abs=1e-12)


def test_undulation_is_followed_rather_than_assumed_flat():
    field = Heightfield([0., 1., 2.], [-1., 1.], [[0., .002, -.002], [0., .002, -.002]])
    hit = ground_intersection([1., 0., HEIGHT], [[1., 0., 0.], [0., 1., 0.], [0., 0., -1.]], field)
    assert hit is not None and hit[2] == pytest.approx(.002, abs=1e-9)


def test_what_cannot_be_solved_is_not_reported_as_zero():
    field = Heightfield.flat(0.)
    up = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]           # ray points away from the road
    assert ground_intersection([0., 0., HEIGHT], up, field) is None
    sideways = [[1., 0., 0.], [0., 1., 1.], [0., 0., 0.]]      # no vertical component
    assert ground_intersection([0., 0., HEIGHT], sideways, field) is None
    small = Heightfield([0., 1.], [-1., 1.], [[0., 0.], [0., 0.]])
    assert ground_intersection([50., 0., HEIGHT], up, small) is None   # outside the surface
    nan = [[1., 0., 0.], [0., 1., 0.], [0., 0., float('nan')]]
    assert ground_intersection([0., 0., HEIGHT], nan, field) is None
    assert optical_axis([[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]) is None
    assert optical_axis(np.zeros((2, 2))) is None
    # An unsolved end leaves the footprint absent, never substituted.
    out = interval_travel(tag(0, 0.), dict(global_line=1024, camera_position_world_m=[1., 0., HEIGHT],
                                           camera_rotation_world=up), field, SPACING)
    assert out['footprint_available'] is False
    assert out['footprint_m'] is None and out['footprint_ratio'] is None


def test_real_travel_inconsistency_is_still_caught():
    """Construct sliding: the wheel turns further than the ground moves."""
    field = Heightfield.flat(0.)
    lines = 1024
    honest = lines*SPACING
    out = interval_travel(tag(0, 0.), tag(lines, .80*honest), field, SPACING)
    assert out['footprint_ratio'] == pytest.approx(.80, abs=1e-9)
    assert out['footprint_ratio'] < .95
    # Pitch cannot rescue it: adding a plausible body pitch leaves it failing.
    tilted = interval_travel(tag(0, 0.), tag(lines, .80*honest, math.radians(1.2)), field, SPACING)
    assert tilted['footprint_ratio'] == pytest.approx(.80, abs=1e-6)


def test_a_degenerate_interval_is_refused():
    field = Heightfield.flat(0.)
    assert interval_travel(tag(512, 0.), tag(512, 1.), field, SPACING) is None
    assert interval_travel(tag(512, 0.), tag(100, 1.), field, SPACING) is None
    assert interval_travel(tag(0, 0.), tag(1024, 1.), field, 0.) is None
