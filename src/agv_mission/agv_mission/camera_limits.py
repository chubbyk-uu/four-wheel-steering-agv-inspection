"""Inspection motion gates follow physical wheel limits, not nominal pass speed."""
from copy import deepcopy
import math


def inspection_camera_config(camera,platform):
    limit=platform['max_speed'];tolerance=camera.get('encoder_speed_tolerance_m_s',.02)
    if not math.isfinite(limit) or limit<=0 or not math.isfinite(tolerance) or not 0<tolerance<=.1:
        raise ValueError('invalid platform wheel speed or encoder speed tolerance')
    result=deepcopy(camera)
    result.update(projected_encoder=True,max_yaw_rate_rad_s=.08,max_scan_lateral_m_s=.10,
                  max_scan_residual_m_s=.03,encoder_speed_tolerance_m_s=tolerance,
                  max_scan_speed_m_s=limit+tolerance)
    return result
