"""A sampled ray camera, not a cropped area-camera stream.

The analytic scene is deliberately limited to an unobstructed z=0 grid plane.
Ground truth poses belong to sensor generation only. Rectification consumes only
the calibrated raw-pixel-to-ray map; it never reads scene coordinates or poses.
"""
import math
import numpy as np
from numpy.polynomial.polynomial import polyval


def rotation(q):
    q = np.asarray(q, dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-9:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def interpolate_pose(a, b, fraction):
    """Shortest quaternion SLERP, linear translation; no extrapolation."""
    if not 0 <= fraction <= 1:
        raise ValueError('pose extrapolation is forbidden')
    pa, qa = np.asarray(a[0]), np.asarray(a[1], dtype=float)
    pb, qb = np.asarray(b[0]), np.asarray(b[1], dtype=float)
    qa, qb = qa / np.linalg.norm(qa), qb / np.linalg.norm(qb)
    dot = float(np.dot(qa, qb))
    if dot < 0:
        qb, dot = -qb, -dot
    if dot > .9995:
        q = qa + fraction*(qb-qa)
        q /= np.linalg.norm(q)
    else:
        theta = math.acos(np.clip(dot, -1, 1))
        q = (math.sin((1-fraction)*theta)*qa + math.sin(fraction*theta)*qb)/math.sin(theta)
    return (pa + fraction*(pb-pa)).tolist(), q.tolist()


class Camera:
    def __init__(self, config):
        self.config = config
        self.width = int(config['width'])
        self.rows = int(config['block_rows'])
        self.spacing = float(config['line_spacing_m'])
        self.exposure = float(config['exposure_s'])
        self.height = config['nominal_width_m']*config['focal_length_m']/(self.width*config['pixel_pitch_m'])
        if self.width < 2 or self.rows < 1 or min(self.spacing, self.exposure, self.height) <= 0:
            raise ValueError('invalid camera dimensions or timing')
        self.center = (self.width-1)/2
        self.scale = self.width*config['pixel_pitch_m']/(2*config['focal_length_m'])
        self.coefficients = np.asarray(config['ray_polynomial'], dtype=float)
        # Validate invertibility throughout the full physical sensor, including edges.
        q = np.linspace(-1, 1, max(self.width*2, 16385))
        derivative = np.arange(1, len(self.coefficients))*self.coefficients[1:]
        if not np.all(np.isfinite(self.coefficients)) or len(derivative) == 0 or np.min(polyval(q, derivative)) <= 1e-6:
            raise ValueError('pixel-to-ray mapping must be strictly increasing')
        self.raw_q = (np.arange(self.width)-self.center)/(self.width/2)
        self.ray_x = self.scale*polyval(self.raw_q, self.coefficients)
        # Optical +X = body +Y, optical +Y = body +X, optical +Z = body -Z.
        self.optical_rotation = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
        self.offset = np.array([config['camera_x_m'], 0., self.height-config['base_nominal_height_m']])
        self.lookup = np.interp(self.scale*self.raw_q, self.ray_x, np.arange(self.width), left=np.nan, right=np.nan)
        self.valid_columns = np.isfinite(self.lookup)

    def optical_pose(self, pose):
        p, q = pose
        return np.asarray(p)+rotation(q)@self.offset, rotation(q)@self.optical_rotation

    def grid_line(self, pose):
        origin, r = self.optical_pose(pose)
        rays = self.ray_x[:, None]*r[:, 0] + r[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = -origin[2]/rays[:, 2]
            points = origin + distance[:, None]*rays
        c = self.config
        valid = (origin[2] > 0) & (distance > 0) & np.all(np.isfinite(points), axis=1) & (np.max(np.abs(points[:, :2]), axis=1) <= c['grid_extent_m']/2)
        remainder = np.abs((points[:, :2]+c['grid_spacing_m']/2) % c['grid_spacing_m']-c['grid_spacing_m']/2)
        dark = np.min(remainder, axis=1) <= c['grid_line_width_m']/2
        intensity = np.where(dark, c['grid_dark'], c['grid_light']).astype(np.float32)
        intensity[~valid] = 0
        return intensity, valid

    def expose(self, start_pose, end_pose):
        # Three midpoint quadrature samples integrate geometry during exposure.
        values, masks = zip(*(self.grid_line(interpolate_pose(start_pose, end_pose, f)) for f in (1/6, .5, 5/6)))
        return np.rint(np.mean(values, axis=0)).astype(np.uint8), np.logical_and.reduce(masks)

    def rectify(self, image):
        """Inverse resampling onto nominal undistorted columns, preserving rows."""
        if image.ndim != 2 or image.shape[1] != self.width:
            raise ValueError('unexpected image shape')
        source = np.nan_to_num(self.lookup, nan=0.)
        left = np.floor(source).astype(int)
        right = np.minimum(left+1, self.width-1)
        weight = source-left
        result = np.rint(image[:, left]*(1-weight)+image[:, right]*weight).astype(np.uint8)
        result[:, ~self.valid_columns] = 0
        return result, self.valid_columns.copy()


class Trigger:
    """Signed cumulative encoder distance. Each new segment starts a fresh phase."""
    def __init__(self, spacing):
        self.spacing = spacing
        self.previous = None
        self.next_distance = None
        self.direction = 0

    def reset(self):
        self.previous = self.next_distance = None
        self.direction = 0

    def update(self, time_s, distance):
        if not math.isfinite(time_s) or not math.isfinite(distance):
            raise ValueError('nonfinite encoder sample')
        if self.previous is None:
            self.previous = (time_s, distance)
            return []
        t0, d0 = self.previous
        if time_s <= t0:
            raise ValueError('encoder time must increase')
        delta = distance-d0
        self.previous = (time_s, distance)
        if abs(delta) < 1e-12:
            return []
        direction = 1 if delta > 0 else -1
        if self.direction and direction != self.direction:
            raise ValueError('direction change requires a new segment')
        if not self.direction:
            self.direction = direction
            self.next_distance = d0+direction*self.spacing
        count = max(0, math.floor((direction*(distance-self.next_distance)+1e-12)/self.spacing)+1)
        events = []
        for _ in range(count):
            d = self.next_distance
            events.append((t0+(d-d0)/delta*(time_s-t0), d))
            self.next_distance += direction*self.spacing
        return events


class Blocks:
    def __init__(self, camera, emit, event=None):
        self.camera, self.emit = camera, emit
        self.event=event or (lambda event:None)
        self.min_tail_rows=camera.config.get("min_tail_rows",1000)
        if not isinstance(self.min_tail_rows,int) or not 0<=self.min_tail_rows<=16384:raise ValueError("invalid min_tail_rows")
        self.max_tag_gap=camera.config.get("pose_tag_time_gap_s",.1)
        if not math.isfinite(self.max_tag_gap) or self.max_tag_gap<=0:raise ValueError("invalid pose tag time gap")
        self.buffer = np.empty((camera.rows, camera.width), dtype=np.uint8)
        self.count = self.block = self.global_line = 0
        self.segment = 0
        self.tags = []
        self.invalid = 0

    def add(self, line, valid, tag):
        self.buffer[self.count] = line
        tag = dict(tag, global_line=self.global_line)
        if self.count == 0:
            self.first = tag
        gap=self.count>0 and tag["time_s"]-self.last["time_s"]>self.max_tag_gap
        if gap and self.tags[-1]["global_line"]!=self.last["global_line"]:
            self.tags.append(self.last)
        # Quarter-frame anchors plus both sides of actual long inter-line gaps.
        if self.count % max(1, self.camera.rows//4) == 0 or gap:
            self.tags.append(tag)
        self.last = tag
        self.invalid += int(np.count_nonzero(~valid))
        self.count += 1
        self.global_line += 1
        if self.count == self.camera.rows:
            self.flush('full')

    def flush(self, reason):
        if not self.count:
            return
        if self.tags[-1]['global_line'] != self.last['global_line']:
            self.tags.append(self.last)
        metadata = dict(schema='agv.linescan.grid.v1', block_id=self.block,
                        segment_id=self.segment, width=self.camera.width, rows=self.count,
                        encoding='mono8', first=self.first, last=self.last,
                        reference='last_line_exposure_midpoint', pose_tags=self.tags,
                        line_spacing_m=self.camera.spacing, exposure_s=self.camera.exposure,
                        calibration_id=self.camera.config['calibration_id'],
                        end_reason=reason, invalid_pixels=self.invalid,
                        pose_source='simulation_ground_truth_sensor_generation_only',
                        scene_backend='analytic_unobstructed_grid_plane')
        if reason!='full' and self.count<self.min_tail_rows:
            self.event(dict(reason='tail_discarded',end_reason=reason,rows=self.count,minimum_rows=self.min_tail_rows,block_id=self.block,segment_id=self.segment,first=self.first,last=self.last))
        else:self.emit(self.buffer[:self.count].copy(), metadata)
        self.block += 1
        self.count = self.invalid = 0
        self.tags = []

    def end_segment(self, reason):
        self.flush(reason)
        self.segment += 1
