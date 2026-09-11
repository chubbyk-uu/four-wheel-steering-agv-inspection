"""Numerical measurement models, calibration and bounded delivery queue."""
import heapq
import math
import numpy as np
from scipy.spatial.transform import Rotation


def finite(values, shape=None):
    a = np.asarray(values, dtype=float)
    if (shape is not None and a.shape != shape) or not np.isfinite(a).all():
        raise ValueError('invalid measurement/configuration shape or nonfinite value')
    return a


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


def nominal_antennas(platform):
    # Electrical phase centers provisionally coincide with antenna shell centers.
    return np.array([[-.85, platform['gnss_baseline']/2, platform['gnss_top_height']-platform['base_height']-.0175],
                     [-.85, -platform['gnss_baseline']/2, platform['gnss_top_height']-platform['base_height']-.0175]])


def calibrated_antennas(platform, calibration):
    true = nominal_antennas(platform)
    centre = true.mean(axis=0)
    r = Rotation.from_rotvec(finite(calibration['gnss_platform_rotvec_rad'], (3,))).as_matrix()
    phase = finite([calibration['gnss_left_phase_center_m'], calibration['gnss_right_phase_center_m']], (2,3))
    return centre + (true-centre+phase)@r.T + finite(calibration['gnss_platform_translation_m'], (3,))


class DeliveryQueue:
    def __init__(self, capacity=512):
        self.items, self.serial, self.capacity = [], 0, capacity
        self.peak = 0

    def push(self, due, kind, message):
        if not math.isfinite(due) or len(self.items) >= self.capacity:
            raise ValueError('invalid delivery deadline or bounded measurement queue overflow')
        self.serial += 1
        heapq.heappush(self.items, (due, self.serial, kind, message))
        self.peak = max(self.peak, len(self.items))

    def pop(self, now):
        while self.items and self.items[0][0] <= now:
            _, _, kind, message = heapq.heappop(self.items)
            yield kind, message


def delay_sample(config, channel, rng):
    if not config['delay_enabled']:
        return 0.0
    mean, sigma = finite(config[channel+'_delay_s'], (2,))
    if min(mean, sigma) < 0:
        raise ValueError('negative delivery delay')
    return max(0., mean+np.clip(rng.normal(0,sigma),-3*sigma,3*sigma))


class EncoderOdometry:
    def __init__(self, platform, config):
        self.previous = None
        self.config = config
        rng = np.random.default_rng(config['seed']+1)
        self.rng = np.random.default_rng(config['seed']+2)
        noise = bool(config['noise_enabled'])
        self.radius = platform['wheel_radius'] * (1+rng.normal(0, config['wheel_radius_relative_sigma'] if noise else 0, 4))
        xy = np.array([[platform['wheelbase']/2, platform['track']/2],
                       [platform['wheelbase']/2, -platform['track']/2],
                       [-platform['wheelbase']/2, platform['track']/2],
                       [-platform['wheelbase']/2, -platform['track']/2]])
        xy += finite(config['calibration']['wheel_xy_error_m'], (4,2))
        self.matrix = np.array([[1,0,-y] if axis==0 else [0,1,x] for x,y in xy for axis in (0,1)])
        self.inverse = np.linalg.pinv(self.matrix)
        self.zero = finite(config['calibration']['steer_zero_error_rad'], (4,))
        self.quantum = math.tau/config['encoder_counts_per_revolution'] if noise else 0
        self.sigma = config['steer_sigma_rad'] if noise else 0
        if np.linalg.matrix_rank(self.matrix) != 3 or min(self.radius) <= 0:
            raise ValueError('degenerate wheel geometry')

    def update(self, timestamp, drive, steer):
        drive, steer = finite(drive, (4,)), finite(steer, (4,))
        if not math.isfinite(timestamp):
            raise ValueError('nonfinite encoder timestamp')
        if self.previous is not None and timestamp <= self.previous[0]:
            return None
        if self.quantum:
            drive = np.rint(drive/self.quantum)*self.quantum
        steer = steer+self.zero+self.rng.normal(0,self.sigma,4)
        old = self.previous
        self.previous = (timestamp, drive, steer)
        if old is None:
            return None
        dt = timestamp-old[0]
        if dt > .2:
            # Rebase after missing feedback; never average across a long outage.
            return None
        distance = (drive-old[1])*self.radius
        angles = (steer+old[2])/2
        wheel_vectors = np.column_stack((distance*np.cos(angles),distance*np.sin(angles))).ravel()/dt
        twist = self.inverse@wheel_vectors
        residual = np.linalg.norm((self.matrix@twist-wheel_vectors).reshape(4,2),axis=1).max()
        sigma = finite(self.config['wheel_velocity_sigma'], (3,))
        covariance = np.diag(sigma**2 + residual**2)
        return twist, covariance, float(residual)


def gnss_covariance(config):
    sigma = finite(config['gnss_sigma_m'], (3,))
    rho = float(config['gnss_correlation'])
    if min(sigma) <= 0 or not -1 < rho < 1:
        raise ValueError('invalid GNSS covariance/correlation')
    single = np.diag(sigma*sigma)
    return np.block([[single, rho*single], [rho*single, single]])


def dual_gnss_pose(points, mounts, tilt, covariance):
    """Joint XYZ/yaw observation, retaining lever-arm-induced cross covariance.

    tilt=(roll,pitch) comes from IMU/local estimation. We do not observe roll/pitch
    with a single antenna baseline. Conditional covariance here is GNSS only;
    roll/pitch uncertainty is propagated separately by the adapter.
    """
    points, mounts = finite(points,(2,3)), finite(mounts,(2,3))
    covariance = finite(covariance,(6,6))
    tilt = finite(tilt,(2,))
    def solve(p):
        baseline = p[0]-p[1]
        expected = Rotation.from_euler('xyz',[*tilt,0]).apply(mounts[0]-mounts[1])
        if np.linalg.norm(baseline[:2]) < .2 or np.linalg.norm(expected[:2]) < .2:
            raise ValueError('GNSS baseline has insufficient horizontal span')
        yaw = wrap(math.atan2(baseline[1],baseline[0])-math.atan2(expected[1],expected[0]))
        rotation = Rotation.from_euler('xyz',[*tilt,yaw])
        return np.r_[p.mean(axis=0)-rotation.apply(mounts.mean(axis=0)),yaw]
    value = solve(points)
    jac = np.empty((4,6)); eps=1e-5
    for k in range(6):
        shift=np.zeros((2,3)); shift.flat[k]=eps
        a,b=solve(points+shift),solve(points-shift)
        jac[:,k]=(a-b)/(2*eps);jac[3,k]=wrap(a[3]-b[3])/(2*eps)
    return value, jac@covariance@jac.T


def body_acceleration(samples,window):
    """Least-squares planar kinematic acceleration in the body frame.

    samples: increasing (t, vx, vy, yaw_rate) measured body twist history.
    Returns (a_xyz, sigma, reference_time, span) or None when the window is not
    covered. a = dv/dt + omega x v; the vertical term is not observed by wheel
    twist. The slope describes the window mean time, NOT its latest sample, so
    the caller must pair it with accelerometer samples over the same span.
    sigma combines the slope standard error with a jerk-induced alignment bound
    taken from the half-window slope difference.
    """
    if not math.isfinite(window) or window<=0:raise ValueError('invalid acceleration window')
    if len(samples)<4:return None
    last=samples[-1][0]
    rows=[r for r in samples if last-r[0]<=window]
    if len(rows)<4 or last-rows[0][0]<window*.5:return None
    def fit(block):
        t=np.array([r[0] for r in block],dtype=float);centre=t.mean();t=t-centre
        spread=float(t@t)
        if spread<=0:return None
        v=np.array([[r[1],r[2]] for r in block],dtype=float)
        mean=v.mean(axis=0);slope=(t@(v-mean))/spread
        residual=v-mean-np.outer(t,slope)
        sigma=math.sqrt(float(np.sum(residual**2))/max(1,2*(len(block)-2))/spread)
        return slope,mean,centre,sigma,spread
    whole=fit(rows)
    if whole is None:return None
    slope,mean,centre,sigma,_=whole
    omega=float(np.mean([r[3] for r in rows]))
    a=np.array([slope[0]-omega*mean[1],slope[1]+omega*mean[0],0.])
    half=len(rows)//2
    early,late=fit(rows[:half]),fit(rows[half:])
    jerk=float(np.linalg.norm(late[0]-early[0]))/2 if early and late else 0.
    sigma=math.hypot(sigma,jerk)
    if not np.isfinite(a).all() or not math.isfinite(sigma):raise ValueError('nonfinite kinematic acceleration')
    return a,sigma,centre,(rows[0][0],rows[-1][0])


def gravity_tilt(acceleration):
    x,y,z=finite(acceleration,(3,))
    if not 9.3 < math.sqrt(x*x+y*y+z*z) < 10.3:
        raise ValueError('stationary acceleration magnitude inconsistent with gravity')
    return np.array([math.atan2(y,z), math.atan2(-x,math.hypot(y,z))])
