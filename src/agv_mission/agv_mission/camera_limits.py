"""Inspection motion gates follow physical wheel limits, not nominal pass speed."""
from copy import deepcopy
import math


def inspection_camera_config(camera,platform):
    limit=platform['max_speed'];tolerance=camera.get('encoder_speed_tolerance_m_s',.02)
    if not math.isfinite(limit) or limit<=0 or not math.isfinite(tolerance) or not 0<tolerance<=.1:
        raise ValueError('invalid platform wheel speed or encoder speed tolerance')
    result=deepcopy(camera)
    # The lateral gate must cover every body lateral velocity the tracker may
    # legally command, and that is not the position-feedback authority alone.
    # The controller caps its feedback in the track frame and then rotates the
    # whole command, feedforward included, into the body frame, so a heading
    # error psi leaks v*sin(psi) into the lateral axis: at the rated 2.778 m/s a
    # legal 0.04 rad contributes 0.111 m/s before any cross-track correction.
    # An earlier gate of 0.15 was set against the 0.12 feedback cap alone and
    # still did not cover a legal command of 0.209 m/s.
    #
    #   gate >= rated * sin(max_capture_heading_error_rad) + max_position_feedback
    #         = 2.778 * sin(0.05) + 0.12 = 0.259
    #
    # 0.28 leaves 8 percent over that, and tracking.yaml enforces the heading
    # term rather than assuming it. The relation is asserted in the tests, which
    # is the part that matters: the previous assertion compared the gate against
    # the feedback cap, a quantity the sensor does not measure.
    #
    # The gate does not bound blur. At a 20 us exposure 0.28 m/s moves the line
    # 5.6 um, a sixtieth of a pixel. What it bounds is lateral drift across one
    # block: 151 mm at the rated speed, which the per-line pose tags record and
    # rectification can undo.
    result.update(projected_encoder=True,max_yaw_rate_rad_s=.08,max_scan_lateral_m_s=.28,
                  max_scan_residual_m_s=.03,encoder_speed_tolerance_m_s=tolerance,
                  max_scan_speed_m_s=limit+tolerance)
    return result
