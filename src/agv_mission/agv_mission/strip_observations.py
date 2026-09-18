"""The one observation per image the strip optimiser is allowed to see.

The production contract is deliberately thin: for each archived image, the
estimated horizontal position and heading of the scan line at the moment its
first row was exposed, with an uncertainty. Everything else an archive holds --
the per-row pose tags, the renderer's own camera pose, the real wheel radius,
the scene truth -- is diagnostic and must not reach the optimiser.

Two properties are what make this a contract rather than a dump.

Keyed by row, not by file. An observation names a global row, never a block or
a file name, so re-splitting the stored images at a different block size moves
bytes and changes nothing here. Written the other way round -- one entry per
file -- the requirement that re-chunking leave the geometry alone could not
even be expressed, because re-chunking would change how many entries there are.

Estimated, not true. The position comes from navigation.jsonl through the
calibrated lever arm, and the block metadata is read only for the first row's
number and exposure time. block['first'] also carries the renderer's own camera
pose; reading that would quietly turn the whole exercise into a truth replay,
so the extractor takes the two scalars it needs by name and never the pose.
"""
import glob
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .capture_audit import Navigation

SCHEMA = 'agv.strip_observations.v1'
# The two fields of block['first'] this may read. Anything else there describes
# where the simulator put the camera, which is not an estimate of anything.
FIRST_ROW_FIELDS = ('global_line', 'time_s')


def road_frame(request):
    road = request['road']
    return np.asarray(road['origin_xyz_m'], float), Rotation.from_euler('z', road['yaw_rad'])


def scan_centre(navigation, position, body, origin, rotation):
    """Where the middle of the scan line meets the declared road plane."""
    eye = position + body.apply(navigation.offset)
    ray = (body*navigation.mount).as_matrix()[:, 2]
    if ray[2] >= -1e-6:
        raise ValueError('camera ray does not face the declared road plane')
    distance = (origin[2]-eye[2])/ray[2]
    if distance <= 0:
        raise ValueError('camera below the declared road plane')
    point = rotation.inv().apply(eye+distance*ray-origin)
    heading = (body.as_euler('xyz')[2]-rotation.as_euler('xyz')[2]+math.pi) % (2*math.pi)-math.pi
    return point[:2], heading


def uncertainty(navigation, position, body, covariance, origin, rotation):
    """One sigma on the centre and heading, the pose covariance through the lever arm.

    The lever arm is why attitude enters a position sigma at all: an arm of
    1.15 m turns a milliradian of yaw into 1.15 mm on the ground, and the
    contract asks for that to be counted rather than assumed away.
    """
    rpy = body.as_euler('xyz')
    eps = 1e-5
    jac = np.zeros((3, 6))
    for j in range(6):
        delta = np.zeros(3)
        delta[j % 3] = eps
        if j < 3:
            plus = scan_centre(navigation, position+delta, body, origin, rotation)
            minus = scan_centre(navigation, position-delta, body, origin, rotation)
        else:
            plus = scan_centre(navigation, position, Rotation.from_euler('xyz', rpy+delta), origin, rotation)
            minus = scan_centre(navigation, position, Rotation.from_euler('xyz', rpy-delta), origin, rotation)
        jac[:2, j] = (plus[0]-minus[0])/(2*eps)
        jac[2, j] = ((plus[1]-minus[1]+math.pi) % (2*math.pi)-math.pi)/(2*eps)
    return np.sqrt(np.maximum(0, np.diag(jac@covariance@jac.T)))


def images(session):
    """First row and segment of every archived image, and nothing else from it."""
    out = []
    for path in sorted(glob.glob(str(Path(session)/'raw/*/block_*.json'))):
        block = json.loads(Path(path).read_text())
        first = {k: block['first'][k] for k in FIRST_ROW_FIELDS}
        out.append(dict(segment_id=block['segment_id'], rows=block['rows'], **first))
    if not out:
        raise ValueError('no archived images under '+str(session))
    return out


def extract(session, plan=None):
    session = Path(session)
    if plan is None:
        candidates = sorted(glob.glob(str(session/'tasks/*/mission/plan.json')))
        if len(candidates) != 1:
            raise ValueError('name the plan: %d task plans under this session' % len(candidates))
        plan = candidates[0]
    request = json.loads(Path(plan).read_text())['request']
    origin, rotation = road_frame(request)
    navigation = Navigation(session/'navigation')

    observations = []
    for image in images(session):
        time = float(image['time_s'])
        index = np.searchsorted(navigation.times, time)
        if not math.isfinite(time) or index == 0 or index == len(navigation.times):
            raise ValueError('no navigation bracket for row %d' % image['global_line'])
        if navigation.times[index]-navigation.times[index-1] > .06:
            raise ValueError('navigation gap at row %d' % image['global_line'])
        ratio = (time-navigation.times[index-1])/(navigation.times[index]-navigation.times[index-1])
        covariance = navigation.covariances[index-1]*(1-ratio)+navigation.covariances[index]*ratio
        covariance = (covariance+covariance.T)/2
        if not np.isfinite(covariance).all() or np.linalg.eigvalsh(covariance).min() < -1e-8:
            raise ValueError('invalid navigation covariance at row %d' % image['global_line'])
        position = navigation.positions[index-1]*(1-ratio)+navigation.positions[index]*ratio
        body = navigation.rotations([time])[0]
        centre, heading = scan_centre(navigation, position, body, origin, rotation)
        sigma = uncertainty(navigation, position, body, covariance, origin, rotation)
        observations.append(dict(
            segment_id=int(image['segment_id']), global_row=int(image['global_line']),
            time_s=time, road_x_m=float(centre[0]), road_y_m=float(centre[1]),
            heading_rad=float(heading), sigma_x_m=float(sigma[0]), sigma_y_m=float(sigma[1]),
            sigma_heading_rad=float(sigma[2])))

    rows = [v['global_row'] for v in observations]
    if len(set(rows)) != len(rows) or rows != sorted(rows):
        raise ValueError('observations must be unique and ordered by global row')
    return dict(
        schema=SCHEMA, session=str(session), plan=str(plan),
        navigation_sha256=navigation.digest, calibration_id=navigation.cal['calibration_id'],
        road=request['road'],
        reference_point=dict(
            what='the centre of the scan line where it meets the declared road plane',
            frame='road frame of the plan request, x along the road, y across it',
            heading='estimated body yaw in that frame; the scan line is perpendicular to it',
            lever_arm_m=list(map(float, navigation.offset)),
            lever_arm_source='camera_optical_calibrated in the session calibration, an estimate',
            attitude_policy=('pitch and roll enter only through the estimated body rotation at '
                             'this instant; the fixed-calibration policy for the rest of the '
                             'image is the consumer\'s business, not this table\'s')),
        sigma=dict(kind='one sigma', basis='navigation pose covariance through the lever arm'),
        excluded=('per-row pose tags, the renderer camera pose in block["first"], the real wheel '
                  'radius, and everything under navigation/evaluation'),
        observations=observations)


def load(path):
    """Read a table back and re-check what makes it usable, before anyone fits to it."""
    table = json.loads(Path(path).read_text())
    if table.get('schema') != SCHEMA:
        raise ValueError('unsupported observation schema')
    rows = [v['global_row'] for v in table['observations']]
    if not rows:
        raise ValueError('empty observation table')
    if len(set(rows)) != len(rows) or rows != sorted(rows):
        raise ValueError('observations must be unique and ordered by global row')
    for v in table['observations']:
        if any(not math.isfinite(v[k]) for k in ('road_x_m', 'road_y_m', 'heading_rad')):
            raise ValueError('non-finite observation at row %d' % v['global_row'])
        if any(not (math.isfinite(v[k]) and v[k] > 0) for k in ('sigma_x_m', 'sigma_y_m', 'sigma_heading_rad')):
            raise ValueError('observation at row %d carries no usable uncertainty' % v['global_row'])
    return table
