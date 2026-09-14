"""Inspection motion gates follow physical wheel limits, not nominal pass speed."""
from copy import deepcopy
import math


def inspection_camera_config(camera,platform):
    limit=platform['max_speed'];tolerance=camera.get('encoder_speed_tolerance_m_s',.02)
    if not math.isfinite(limit) or limit<=0 or not math.isfinite(tolerance) or not 0<tolerance<=.1:
        raise ValueError('invalid platform wheel speed or encoder speed tolerance')
    result=deepcopy(camera)
    # The lateral and yaw gates must sit above what the tracker is allowed to
    # command, or a mission is faulted by its own controller doing its job. That
    # is what happened on 2026-09-14: the gate was 0.10 against a 0.12 m/s
    # position-feedback authority, and a rated full-area run tripped it at
    # 0.10002 m/s, 335 s in, with three earlier runs having passed on peaks of
    # 0.084, 0.079 and 0.039. The gate does not bound blur: at a 20 us exposure,
    # 0.1 m/s moves the line by 2 um, a two-hundredth of a pixel. What it bounds
    # is the lateral drift across one block, which the per-line pose tags record
    # and rectification can undo. The relation is asserted in the tests.
    result.update(projected_encoder=True,max_yaw_rate_rad_s=.08,max_scan_lateral_m_s=.15,
                  max_scan_residual_m_s=.03,encoder_speed_tolerance_m_s=tolerance,
                  max_scan_speed_m_s=limit+tolerance)
    return result
