"""Pure geometry planner. Flat surface only; explicit 3-D output contract.

All feasibility checks are analytic in the road frame, independent of display
sampling. A swept disc conservatively contains every rigid body orientation.
It checks declared static bounds, not unknown obstacles or tire dynamics.
"""
from dataclasses import dataclass
import math
from .tracking import trajectory_deceleration


class PlanningError(ValueError):
    pass


def number(value, name, minimum=None, strictly=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlanningError(f'{name}: expected a finite number')
    value = float(value)
    if not math.isfinite(value) or (minimum is not None and
            (value <= minimum if strictly else value < minimum)):
        raise PlanningError(f'{name}: invalid value {value}')
    return value


def vector(value, size, name):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise PlanningError(f'{name}: expected {size} values')
    return [number(v, name) for v in value]


def fields(obj, required, optional=()):
    if not isinstance(obj, dict):
        raise PlanningError('expected a mapping')
    missing, extra = set(required) - obj.keys(), obj.keys() - set(required) - set(optional)
    if missing or extra:
        raise PlanningError(f'missing keys {sorted(missing)}; unknown keys {sorted(extra)}')


@dataclass(frozen=True)
class Vehicle:
    swath: float
    camera_x: float
    base_height: float
    accel: float
    decel: float
    max_speed: float
    rated_scan_speed: float
    discardable_tail_m: float = 0.0
    minimum_sweep_radius: float = 1.5
    raw_swath_factor: float = 1.04

    @classmethod
    def from_configs(cls, platform, camera):
        # Audited with the expanded URDF, including articulated wheel envelopes.
        # Config changes must not silently reuse the 1.5 m operating envelope.
        dimensions = {'body_length':1.9, 'body_width':1.1, 'wheelbase':1.3,
                      'track':.94, 'wheel_radius':.2, 'wheel_width':.16,
                      'base_height':.65, 'body_top_height':1.1,
                      'gnss_baseline':1.1, 'gnss_top_height':1.2,
                      'suspension_travel':.05}
        optics = {'camera_x_m':1.15, 'led_length_m':1.2, 'led_height_m':.30,
                  'led_forward_offset_m':-.0803847577293368,
                  'base_nominal_height_m':.65, 'focal_length_m':.020,
                  'nominal_width_m':1.5, 'pixel_pitch_m':.000007, 'width':4096}
        for source, expected in ((platform,dimensions),(camera,optics)):
            for key, value in expected.items():
                actual=source.get(key)
                if isinstance(actual,bool) or not isinstance(actual,(int,float)) or not math.isfinite(actual) or abs(actual-value)>1e-9:
                    raise PlanningError('vehicle envelope must be revalidated for changed '+key)
        polynomial = camera['ray_polynomial']
        # Current optical model must remain monotone, as required by the sensor.
        if polynomial != [0.0, 1.0, 0.0, 0.04]:
            raise PlanningError('raw optical envelope must be revalidated for changed distortion')
        from agv_linescan.encoder import line_spacing
        # The sensor drops a closing image shorter than min_tail_rows, so the last
        # archived row may trail wherever capture was closed by this much.
        tail = number(camera.get('min_tail_rows', 0), 'min tail rows', 0) * \
            number(line_spacing(camera, platform['wheel_radius']), 'line spacing', 0, True)
        # The vehicle's top speed and the fastest scan it may be asked for are
        # different numbers. Commanding a scan at the top speed would leave the
        # tracker no authority to accelerate, only to brake.
        top = number(platform.get('max_speed'), 'platform max speed', 0, True)
        rated = number(platform.get('rated_scan_speed'), 'rated scan speed', 0, True)
        if rated >= top:
            raise PlanningError('rated scan speed must stay below the vehicle top speed')
        return cls(camera['nominal_width_m'], camera['camera_x_m'], platform['base_height'],
                   platform['drive_accel'], trajectory_deceleration(platform), top, rated,
                   tail)


def plan(request, vehicle):
    fields(request, ['schema', 'mission_id', 'road', 'region', 'track_spacing_m',
                     'scan_speed_m_s', 'coverage_error_m', 'drivable_bounds_xy_m',
                     'optical_bounds_xy_m', 'longitudinal_margin_m',
                     'vehicle_sweep_radius_m', 'sample_step_m'],
                     ['heading_recovery_time_s'])
    if request['schema'] != 'agv.rectangle.request.v1':
        raise PlanningError('unsupported request schema')
    if not isinstance(request['mission_id'], str) or not request['mission_id'].strip():
        raise PlanningError('mission_id must be nonempty')
    road, region = request['road'], request['region']
    fields(road, ['frame_id', 'origin_xyz_m', 'yaw_rad', 'surface'])
    fields(region, ['start_xy_m', 'length_m', 'width_m'])
    if road['surface'] != 'flat':
        raise PlanningError('only flat surfaces are implemented; slopes need a surface adapter')
    if not isinstance(road['frame_id'], str) or not road['frame_id'] or road['frame_id'].startswith('/'):
        raise PlanningError('invalid target frame_id')
    origin = vector(road['origin_xyz_m'], 3, 'road origin')
    yaw = number(road['yaw_rad'], 'road yaw')
    x, y = vector(region['start_xy_m'], 2, 'region start')
    length = number(region['length_m'], 'length', 0, True)
    width = number(region['width_m'], 'width', 0, True)
    spacing = number(request['track_spacing_m'], 'spacing', 0, True)
    speed = number(request['scan_speed_m_s'], 'speed', 0, True)
    error = number(request['coverage_error_m'], 'coverage error', 0)
    margin = number(request['longitudinal_margin_m'], 'longitudinal margin', 0)
    # Re-steering after a lane change can yaw the stopped chassis through tyre
    # contact. Capture waits for the ordinary pass heading loop to recover, so a
    # low scan speed still needs time-based lead-in; acceleration distance alone
    # shrinks in exactly the wrong direction. Current request templates store the
    # budget explicitly. Missing means zero only to reproduce archived v1 plans;
    # changing their endpoints would invalidate capture-audit provenance.
    recovery_time = number(request.get('heading_recovery_time_s', 0.0),
                           'heading recovery time', 0)
    radius = number(request['vehicle_sweep_radius_m'], 'sweep radius', 0, True)
    step = number(request['sample_step_m'], 'sample step', 0, True)
    for name in ('swath', 'base_height', 'accel', 'decel', 'max_speed', 'rated_scan_speed',
                 'minimum_sweep_radius'):
        number(getattr(vehicle, name), name, 0, True)
    number(vehicle.camera_x, 'camera offset')
    if vehicle.rated_scan_speed >= vehicle.max_speed:
        raise PlanningError('rated scan speed must stay below the vehicle top speed')
    if radius < vehicle.minimum_sweep_radius:
        raise PlanningError('sweep radius is smaller than the audited 1.5 m vehicle envelope')
    if speed > vehicle.rated_scan_speed:
        raise PlanningError('scan speed exceeds the rated inspection speed')
    effective = vehicle.swath - 2 * error
    if effective <= 0 or spacing > effective:
        raise PlanningError('spacing/uncertainty permits an uncovered gap')
    bounds = vector(request['drivable_bounds_xy_m'], 4, 'drivable bounds')
    optical = vector(request['optical_bounds_xy_m'], 4, 'optical bounds')
    for b in (bounds, optical):
        if b[0] >= b[1] or b[2] >= b[3]:
            raise PlanningError('bounds must be [xmin, xmax, ymin, ymax]')
    n = max(1, math.ceil(max(0, width - effective) / spacing - 1e-12) + 1)
    if n > 10000:
        raise PlanningError('too many tracks')
    actual = spacing if n > 1 else 0.0
    centers = [y + width / 2 + (i - (n - 1) / 2) * actual for i in range(n)]
    # Capture cannot close at the region edge. What the sensor may still discard
    # trails behind the closing point, and the audit then shrinks each span by the
    # declared uncertainty and reads the span end at the nearest corner of the last
    # footprint, which trails the camera centre. Closing 0.10 m past a 13 m pass
    # against a 0.37 m discardable tail left 0.26-0.31 m of every pass unverified.
    overrun = vehicle.discardable_tail_m + 2 * error
    acceleration_time = speed / vehicle.accel
    recovery_distance = (.5 * vehicle.accel * recovery_time**2
                         if recovery_time <= acceleration_time else
                         speed * recovery_time - speed**2 / (2 * vehicle.accel))
    lead = max(speed**2 / (2 * vehicle.accel), recovery_distance, error) + margin
    # The margin also keeps the capture close ahead of the tracker's own stop, so
    # a short braking distance cannot race the over-run to the end of the pass.
    runout = max(speed**2 / (2 * vehicle.decel), overrun) + margin
    c, s = math.cos(yaw), math.sin(yaw)

    def world(px, py, z=0):
        return [origin[0] + c * px - s * py, origin[1] + s * px + c * py, origin[2] + z]

    def pose(px, py, heading):
        a = yaw + heading
        return {'position': world(px, py, vehicle.base_height),
                'orientation_xyzw': [0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]}

    def inside(px, py, b, inset=0):
        return b[0] + inset - 1e-9 <= px <= b[1] - inset + 1e-9 and b[2] + inset - 1e-9 <= py <= b[3] - inset + 1e-9

    # Conservative symmetric raw footprint for the audited v7 polynomial.
    raw_half = vehicle.swath * number(vehicle.raw_swath_factor, 'raw factor', 1) / 2
    tracks, segments = [], []
    total_s = 0.0
    point_count = 0

    def segment(kind, start, end, h0, h1, track_id, capture=False, stop=True):
        nonlocal total_s, point_count
        for px, py in (start, end):
            if not inside(px, py, bounds, radius):
                raise PlanningError(f'track {track_id} {kind}: swept vehicle leaves drivable bounds at ({px:.3f}, {py:.3f}); enlarge buffer or reduce region/speed')
        distance = math.dist(start, end)
        count = max(1, math.ceil(distance / step), math.ceil(abs(h1 - h0) / .1))
        point_count += count + 1
        if point_count > 200000:
            raise PlanningError('preview exceeds 200000 points; increase sample_step_m')
        pts = []
        tangent = ([c * (end[0]-start[0]) / distance - s * (end[1]-start[1]) / distance,
                    s * (end[0]-start[0]) / distance + c * (end[1]-start[1]) / distance, 0.0]
                   if distance else [0.0, 0.0, 0.0])
        for j in range(count + 1):
            t = j / count
            px, py = [a + t * (b-a) for a, b in zip(start, end)]
            h = h0 + t * (h1-h0)
            pts.append({'pose': pose(px, py, h), 'surface_normal': [0.0, 0.0, 1.0],
                        'tangent': tangent, 's_m': total_s + t * distance,
                        'scan_center_xyz_m': world(px + vehicle.camera_x * math.cos(h),
                                                   py + vehicle.camera_x * math.sin(h))})
        segments.append({'id': len(segments), 'track_id': track_id, 'kind': kind,
                         'capture': capture, 'stop_at_end': stop, 'length_m': distance,
                         'heading_delta_rad': h1-h0, 'points': pts})
        total_s += distance

    previous = None
    for i, cy in enumerate(centers):
        direction = 1 if i % 2 == 0 else -1
        heading = 0.0 if direction == 1 else math.pi
        begin, finish = (x, x + length) if direction == 1 else (x + length, x)
        # Include optical edge uncertainty; raw ray misses currently abort a batch.
        if any(not inside(px, py, optical) for px in (x-error, x+length+error)
               for py in (cy-raw_half-error, cy+raw_half+error)):
            raise PlanningError(f'track {i}: raw optical footprint leaves texture bounds')
        entry = (begin - direction * (vehicle.camera_x + lead), cy)
        scan0 = (begin - direction * vehicle.camera_x, cy)
        scan1 = (finish - direction * vehicle.camera_x, cy)
        turn_runout = max(runout, 2 * vehicle.camera_x + lead) if i < n-1 else runout
        exit_ = (scan1[0] + direction * turn_runout, cy)
        if previous:
            old_exit, old_heading = previous
            shifted = (old_exit[0], cy)
            if math.dist(shifted, entry) > 1e-8:
                raise PlanningError('forward-only turn geometry must meet the next entry')
            # The two expressions are algebraically identical but can differ by
            # one floating-point bit. Reuse the previous endpoint so adjacent
            # segments retain an exact continuous-pose contract.
            entry = shifted
            segment('SHIFT', old_exit, shifted, old_heading, old_heading, i)
            # Rotation sign is a PREVIEW candidate. Runtime must use actual wheel states.
            segment('ROTATE_180', shifted, shifted, old_heading, heading, i)
        segment('ACCELERATE', entry, scan0, heading, heading, i, stop=False)
        segment('SCAN', scan0, scan1, heading, heading, i, capture=True, stop=False)
        segment('RUNOUT_BRAKE', scan1, exit_, heading, heading, i)
        tracks.append({'id': i, 'direction': direction, 'center_road_y_m': cy,
                       'scan_start_xyz_m': world(begin, cy), 'scan_end_xyz_m': world(finish, cy),
                       'nominal_footprint_xyz_m': [world(px, py) for px, py in
                           [(x,cy-vehicle.swath/2),(x+length,cy-vehicle.swath/2),
                            (x+length,cy+vehicle.swath/2),(x,cy+vehicle.swath/2)]],
                       'base_entry_pose': pose(*entry, heading)})
        previous = (exit_, heading)
    return {'schema': 'agv.rectangle.plan.v1', 'mission_id': request['mission_id'],
            'frame_id': road['frame_id'], 'status': 'PREVIEW_ONLY', 'request': request,
            'assumptions': ['flat surface', 'nominal swath, not calibrated usable swath',
                            'static rectangular bounds, no obstacle avoidance',
                            'rotation signs provisional until wheel-state validation',
                            'approach from current vehicle pose is not planned'],
            'vehicle': dict(vars(vehicle)), 'scan_speed_m_s': speed,
            'track_count': n, 'actual_track_spacing_m': actual,
            'guaranteed_overlap_m': effective-actual if n > 1 else None,
            'lead_distance_m': lead, 'heading_recovery_time_s': recovery_time,
            'heading_recovery_distance_m': recovery_distance,
            'runout_distance_m': runout,
            'scan_overrun_distance_m': overrun,
            'turn_runout_distance_m': max(runout, 2*vehicle.camera_x+lead),
            'sweep_radius_m': radius, 'total_base_translation_m': total_s,
            'region_xyz_m': [world(px,py) for px,py in [(x,y),(x+length,y),(x+length,y+width),(x,y+width)]],
            'tracks': tracks, 'segments': segments}
