import copy
import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation
from agv_localization.core import (EncoderOdometry, DeliveryQueue, delay_sample, nominal_antennas,
    calibrated_antennas, dual_gnss_pose, gnss_covariance, gravity_tilt, body_acceleration)

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def config():return yaml.safe_load((ROOT/'config/measurements.yaml').read_text())


@pytest.fixture
def platform():return yaml.safe_load((ROOT.parent/'agv_description/config/platform.yaml').read_text())


@pytest.mark.parametrize('velocity', [[.5,0,0],[-.5,0,0],[0,.5,0],[.3,-.3,0],[0,0,.3]])
def test_encoders_preserve_omnidirectional_motion(config,platform,velocity):
    config['noise_enabled']=False
    odom=EncoderOdometry(platform,config)
    vectors=(odom.matrix@velocity).reshape(4,2)
    angle=np.arctan2(vectors[:,1],vectors[:,0]);rate=np.linalg.norm(vectors,axis=1)/platform['wheel_radius']
    assert odom.update(0,np.zeros(4),angle) is None
    value=odom.update(.01,rate*.01,angle)
    np.testing.assert_allclose(value[0],velocity,atol=1e-12)
    assert value[2]<1e-12
    assert odom.update(.01,rate*.01,angle) is None
    assert odom.update(1,rate,angle) is None


def test_quantized_cumulative_counts_do_not_lose_slow_motion(config,platform):
    config['wheel_radius_relative_sigma']=0;config['steer_sigma_rad']=0
    odom=EncoderOdometry(platform,config);total=0.
    for i in range(1001):
        out=odom.update(i*.01,np.ones(4)*i*.000001,np.zeros(4))
        if out is not None:total+=out[0][0]*.01
    assert abs(total-.0002)<platform['wheel_radius']*odom.quantum/2
    assert total>0


@pytest.mark.parametrize('rpy',[[0,0,0],[0,0,math.pi],[0,0,-2.1],[.15,-.1,1.2]])
def test_dual_gnss_pose_including_3d_tilt(config,platform,rpy):
    mounts=nominal_antennas(platform);p=np.array([13.,-4.,.65])
    points=Rotation.from_euler('xyz',rpy).apply(mounts)+p
    value,cov=dual_gnss_pose(points,mounts,rpy[:2],gnss_covariance(config))
    np.testing.assert_allclose(value[:3],p,atol=1e-12)
    assert abs(math.atan2(math.sin(value[3]-rpy[2]),math.cos(value[3]-rpy[2])))<1e-12
    assert np.linalg.eigvalsh(cov).min()>0
    assert abs(cov[1,3])>1e-6  # Rear-mounted lever arm couples yaw and base position.


def test_heading_uncertainty_comes_from_baseline(config,platform):
    mounts=nominal_antennas(platform)
    _,cov=dual_gnss_pose(mounts,mounts,[0,0],gnss_covariance(config))
    expected=math.sqrt(2)*.025/1.1
    assert math.sqrt(cov[3,3])==pytest.approx(expected,rel=1e-6)
    config['gnss_correlation']=.9
    _,correlated=dual_gnss_pose(mounts,mounts,[0,0],gnss_covariance(config))
    assert correlated[3,3]==pytest.approx(cov[3,3]*.1)


def test_joint_position_yaw_covariance_matches_monte_carlo(config,platform):
    mounts=nominal_antennas(platform)
    _,cov=dual_gnss_pose(mounts,mounts,[0,0],gnss_covariance(config))
    errors=np.random.default_rng(71).multivariate_normal(np.zeros(6),gnss_covariance(config),10000).reshape(-1,2,3)
    points=mounts+errors;baseline=points[:,0]-points[:,1]
    yaw=np.arctan2(baseline[:,1],baseline[:,0])-math.pi/2
    rotations=Rotation.from_euler('z',yaw)
    value=np.column_stack((points.mean(axis=1)-rotations.apply(np.tile(mounts.mean(axis=0),(len(points),1))),yaw))
    empirical=np.cov(value.T)
    np.testing.assert_allclose(np.diag(empirical),np.diag(cov),rtol=.05)
    assert empirical[1,3]==pytest.approx(cov[1,3],rel=.05)


def test_common_mount_rotation_is_not_extra_heading_noise(config,platform):
    config['calibration']['gnss_platform_rotvec_rad']=[0,0,.01]
    true=nominal_antennas(platform);hat=calibrated_antennas(platform,config['calibration'])
    pose,_=dual_gnss_pose(true,hat,[0,0],gnss_covariance(config))
    assert pose[3]==pytest.approx(-.01)
    np.testing.assert_allclose(hat.mean(axis=0),true.mean(axis=0))
    assert np.linalg.norm(hat[0]-hat[1])==pytest.approx(1.1)


def test_single_phase_center_error_changes_heading_and_baseline(config,platform):
    config['calibration']['gnss_left_phase_center_m']=[.003,.001,0]
    true=nominal_antennas(platform);hat=calibrated_antennas(platform,config['calibration'])
    value,_=dual_gnss_pose(true,hat,[0,0],gnss_covariance(config))
    assert value[3]==pytest.approx(math.atan2(.003,1.101))
    assert np.linalg.norm(hat[0]-hat[1])!=pytest.approx(1.1)


def test_delay_queue_retains_original_stamp_and_bounded_order(config):
    queue=DeliveryQueue(2)
    queue.push(1.05,'gnss',{'stamp':1.0});queue.push(1.01,'imu',{'stamp':1.005})
    with pytest.raises(ValueError):queue.push(2,'x',{})
    assert list(queue.pop(1.02))==[('imu',{'stamp':1.005})]
    assert list(queue.pop(1.05))==[('gnss',{'stamp':1.0})]
    r1=np.random.default_rng(1);r2=np.random.default_rng(1)
    a=[delay_sample(config,'gnss',r1) for _ in range(500)]
    assert a==[delay_sample(config,'gnss',r2) for _ in range(500)]
    assert min(a)>=.035 and max(a)<=.065


def test_gravity_tilt_does_not_supply_yaw():
    rpy=[.12,-.2,2.1]
    accel=Rotation.from_euler('xyz',rpy).inv().apply([0,0,9.81])
    np.testing.assert_allclose(gravity_tilt(accel),rpy[:2],atol=1e-12)
    with pytest.raises(ValueError):gravity_tilt([0,0,0])


def test_filters_fuse_3d_and_lateral_velocity_without_truth():
    for kind in ('local','global'):
        params=yaml.safe_load((ROOT/f'config/ekf_{kind}.yaml').read_text())['ekf_'+kind]['ros__parameters']
        assert params['two_d_mode'] is False
        assert params['odom0_config'][7] is True
        assert not any(params['imu0_config'][:9])
        assert params['imu1_config'][3:6]==[True,True,False]
        assert len(params['initial_estimate_covariance'])==225
        assert params['initial_estimate_covariance'][0]>=100
        assert params['world_frame']==('map' if kind=='global' else 'odom')
        assert 'ground_truth' not in str(params)
    assert 'ground_truth' not in (ROOT/'agv_localization/measurement_adapter.py').read_text()


def test_body_contact_constraint_does_not_lock_world_height():
    for kind in ('local','global'):
        params=yaml.safe_load((ROOT/f'config/ekf_{kind}.yaml').read_text())['ekf_'+kind]['ros__parameters']
        assert [i for i,v in enumerate(params['twist0_config']) if v]==[8]
        assert not params['two_d_mode']
    # Motion tangent to a tilted body still has a world-vertical component.
    world_velocity=Rotation.from_euler('y',-.1).apply([1.,0.,0.])
    assert world_velocity[2]==pytest.approx(math.sin(.1))


def test_body_acceleration_separates_longitudinal_and_centripetal():
    dt=.01
    straight=[(i*dt,.8*i*dt,0.,0.) for i in range(40)]
    a,sigma,centre,span=body_acceleration(straight,.2)
    assert a==pytest.approx([.8,0,0],abs=1e-9) and sigma==pytest.approx(0,abs=1e-9)
    # The slope describes the window mean time, not its newest sample.
    assert centre==pytest.approx((span[0]+span[1])/2,abs=1e-9)
    assert span[1]==pytest.approx(straight[-1][0]) and centre<span[1]-.04
    # Constant speed through a turn is pure omega x v, not a speed change.
    turning=[(i*dt,2.,0.,.3) for i in range(40)]
    assert body_acceleration(turning,.2)[0]==pytest.approx([0,.6,0],abs=1e-9)


def test_body_acceleration_refuses_an_uncovered_window():
    dt=.01
    assert body_acceleration([(0,0.,0.,0.),(dt,0.,0.,0.)],.2) is None
    assert body_acceleration([(i*dt,0.,0.,0.) for i in range(5)],.2) is None
    assert body_acceleration([(i*dt,0.,0.,0.) for i in range(40)],.2) is not None
    with pytest.raises(ValueError):body_acceleration([(0,0.,0.,0.)],0)


def test_body_acceleration_reports_slope_uncertainty_from_noisy_twist():
    rng=np.random.default_rng(7);dt=.01
    noisy=[(i*dt,.8*i*dt+rng.normal(0,.02),0.,0.) for i in range(20)]
    a,sigma,_,_=body_acceleration(noisy,.2)
    # A least-squares window must stay far better than a raw two-sample difference.
    assert sigma<.3 and abs(a[0]-.8)<3*sigma


def test_body_acceleration_inflates_uncertainty_under_jerk():
    dt=.01
    steady=[(i*dt,.8*i*dt,0.,0.) for i in range(40)]
    # Quadratic speed: acceleration changes across the window, so a single slope
    # cannot represent both halves and the reported sigma must say so.
    jerking=[(i*dt,.5*3.*(i*dt)**2,0.,0.) for i in range(40)]
    assert body_acceleration(jerking,.2)[1]>body_acceleration(steady,.2)[1]+.1


def test_motion_compensated_gravity_recovers_tilt_while_accelerating():
    roll,pitch,accel=math.radians(2.),math.radians(-1.5),.8
    body=Rotation.from_euler('xyz',[roll,pitch,0])
    measured=np.array([accel,0,0])+body.inv().apply([0,0,9.81])
    dt=.01;kinematic=body_acceleration([(i*dt,accel*i*dt,0.,0.) for i in range(40)],.2)[0]
    assert gravity_tilt(measured-kinematic)==pytest.approx([roll,pitch],abs=1e-9)
    # Without the kinematic term the same sample reads a large false pitch.
    assert abs(gravity_tilt(measured)[1]-pitch)>math.radians(4.)


def test_motion_tilt_configuration_is_validated(config):
    from agv_localization.common import validate
    assert validate(copy.deepcopy(config)) is not None
    for key,bad in [('motion_tilt_window_s',0.),('motion_tilt_interval_s',-1.),
                    ('motion_tilt_extra_sigma_m_s2',0.),('motion_tilt_reject_m_s2',float('nan')),
                    ('motion_tilt_accel_fraction',0.),
                    ('stationary_tilt_samples',1),('stationary_tilt_samples',True),
                    ('motion_tilt_enabled','yes')]:
        broken=copy.deepcopy(config);broken[key]=bad
        with pytest.raises(ValueError):validate(broken)


class TiltHarness:
    """Drive MeasurementAdapter.motion_tilt without a ROS graph."""
    from agv_localization.measurement_adapter import MeasurementAdapter as _A
    motion_tilt=_A.motion_tilt
    motion_rejects=_A.motion_rejects
    def __init__(self,config,twist):
        from collections import deque
        self.config=config;self.gravity=deque(maxlen=200)
        self.body_twist=twist;self.last_motion_tilt=-math.inf
        self.motion_tilts=0;self.published=[]
        self.motion_outcomes={k:0 for k in ('accepted','rate_limited','no_body_twist',
            'stale_body_twist','kinematic_too_large','window_too_short',
            'deviation_too_large','publish_error')}
    def publish_tilt(self,msg,vector,variance):self.published.append((vector,variance))


DT=.01;COUNT=60


def twist_history(accel=0.,speed=.5):
    return [(i*DT,speed+accel*i*DT,0.,0.) for i in range(COUNT)]


def drive(config,accel,measured,twist=None,span=20):
    """Feed accelerometer samples over the same span the twist fit covers.

    The accelerometer must be averaged over the twist window, so samples fed
    after it land outside [low, high] and leave through window_too_short.
    """
    h=TiltHarness(config,twist if twist is not None else twist_history(accel))
    start=(COUNT-span)*DT
    for i in range(span):h.motion_tilt(object(),start+i*DT,measured)
    return h


def test_every_motion_tilt_exit_is_counted_separately(config):
    config=copy.deepcopy(config);config['noise_enabled']=False
    level=np.array([0.,0.,9.81])

    h=drive(config,0.,level)
    assert h.motion_outcomes['accepted']>=1 and h.published
    # The first samples cannot fill the window yet; that is not a rejection.
    assert h.motion_outcomes['window_too_short']>=1
    assert h.motion_rejects==0
    # Once one is accepted the interval throttles the rest.
    assert h.motion_outcomes['rate_limited']>=1

    h=TiltHarness(config,[])
    for i in range(5):h.motion_tilt(object(),.4+i*DT,level)
    assert h.motion_outcomes['no_body_twist']==5 and not h.published

    h=TiltHarness(config,twist_history())
    h.motion_tilt(object(),2.0,level)
    assert h.motion_outcomes['stale_body_twist']==1

    # 2 m/s^2 is past motion_tilt_max_accel_m_s2: the wheel model is not trusted.
    h=drive(config,2.,level+np.array([2.,0,0]))
    assert h.motion_outcomes['kinematic_too_large']>=1
    assert h.motion_rejects==h.motion_outcomes['kinematic_too_large']
    assert not h.published

    h=drive(config,0.,level*2)
    assert h.motion_outcomes['deviation_too_large']>=1
    assert h.motion_rejects==h.motion_outcomes['deviation_too_large']
    assert not h.published


def test_a_closed_gate_keeps_counting_at_imu_rate_so_the_ratio_is_not_a_rate(config):
    config=copy.deepcopy(config);config['noise_enabled']=False
    # last_motion_tilt only advances on success, so a rejection never arms the
    # throttle: every later sample is evaluated and rejected again at IMU rate,
    # while accepted updates are capped at 1/motion_tilt_interval_s. Rejects
    # over accepts is therefore not an acceptance rate.
    calls=40
    h=drive(config,0.,np.array([0.,0.,19.62]),span=calls)
    assert sum(h.motion_outcomes.values())==calls,h.motion_outcomes
    assert h.motion_outcomes['rate_limited']==0 and h.motion_tilts==0
    # Every sample that reached the gravity check was rejected, one per sample.
    covered=calls-h.motion_outcomes['window_too_short']
    assert h.motion_outcomes['deviation_too_large']==covered>1
