from pathlib import Path
import numpy as np
import pytest
import yaml
from agv_mission.tracking import Profile, SegmentTracker, validate_tracking_config


@pytest.mark.parametrize('distance',[0.,.01,.5,4.,20.])
def test_profile_integral_limits_endpoints(distance):
    p=Profile(distance,.5,.8,1.)
    ts=np.unique(np.r_[np.linspace(0,p.duration,10001),p.ta,p.ta+p.tc]);samples=np.array([p.sample(t) for t in ts])
    assert p.sample(0)==(0.,0.)
    assert p.sample(p.duration)==(distance,0.)
    assert np.all(np.diff(samples[:,0])>=-1e-12)
    assert samples[:,1].max()<=.5+1e-12
    if distance:
        assert np.trapz(samples[:,1],ts)==pytest.approx(distance,abs=1e-6)
        acc=np.diff(samples[:,1])/np.diff(ts)
        assert acc.max()<=.8+1e-8 and acc.min()>=-1.-1e-8


def test_triangle_vs_trapezoid():
    assert Profile(.1,.5,.8,1).tc==0
    p=Profile(4,.5,.8,1);assert p.tc>0 and p.ta!=p.td


def tracker(kind='translate'):
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'config/tracking.yaml').read_text())
    platform=yaml.safe_load((root.parent/'agv_description/config/platform.yaml').read_text())
    return SegmentTracker([0,0,.65],[0,0,0,1],kind,[4,0],1,.5,platform,config)


def test_tracking_config_rejects_missing_unknown_and_non_mapping():
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'config/tracking.yaml').read_text())
    validate_tracking_config(config)
    missing=dict(config);del missing['heading_tolerance_rad']
    with pytest.raises(ValueError,match='missing tracking parameters: heading_tolerance_rad'):
        validate_tracking_config(missing)
    with pytest.raises(ValueError,match='unknown tracking parameters: heading_tolerence_rad'):
        validate_tracking_config(dict(config,heading_tolerence_rad=.025))
    with pytest.raises(ValueError,match='must be a mapping'):
        validate_tracking_config(None)


@pytest.mark.parametrize('key,value',[('control_phase_report_ticks',100.),
                                       ('max_terminal_trims',True),
                                       ('heading_tolerance_rad','0.025')])
def test_tracking_config_rejects_wrong_value_types(key,value):
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'config/tracking.yaml').read_text());config[key]=value
    with pytest.raises(ValueError,match=key):validate_tracking_config(config)


@pytest.mark.parametrize('band,expected',[(.01,0.),(.003,.0014),(0.,.0035)])
def test_scan_cross_deadband_is_independent(band,expected):
    c=tracker();c.cfg['scan_cross_track_deadband_m']=band
    c.forward_only=True;c.state='RUNNING';c.was_running=True;c.clock=.5
    distance,feedforward=c.profile.sample(.52)
    # A 5 mm lateral error, no longitudinal or heading error, already filtered.
    c.filtered[:]=[0,.005,0]
    command=c.update([distance,-.005,.65],[0,0,0,1],[feedforward,0,0],'DRIVE',.02)
    assert command[1]==pytest.approx(expected)
    assert command[0]==pytest.approx(feedforward)
    assert command[2]==0


def test_zero_scan_deadband_does_not_change_approach():
    c=tracker();c.cfg['scan_cross_track_deadband_m']=0
    c.state='RUNNING';c.was_running=True;c.clock=.5;c.filtered[:]=[0,.005,0]
    distance,speed=c.profile.sample(.52)
    command=c.update([distance,-.005,.65],[0,0,0,1],[speed,0,0],'DRIVE',.02)
    assert command[1]==0


@pytest.mark.parametrize('band',[0.,-.001,float('nan')])
def test_scan_deadband_validation(band):
    c=tracker();config=dict(c.cfg,scan_cross_track_deadband_m=band)
    def construct():
        return SegmentTracker([0,0,.65],[0,0,0,1],'translate',[4,0],0,.5,c.platform,config)
    if band==0:
        construct()
    else:
        with pytest.raises(ValueError,match='scan_cross_track_deadband_m'):construct()


def test_reference_pauses_and_retimes_after_alignment():
    c=tracker();args=([0,0,.65],[0,0,0,1],[0,0,0])
    for _ in range(100):c.update(*args,'ALIGN',.02)
    assert c.clock==0
    c.update(*args,'DRIVE',.02)
    previous=c.clock
    command=c.update(*args,'BRAKE',.02)
    for _ in range(30):np.testing.assert_equal(c.update(*args,'ALIGN',.02),command)
    assert c.clock==previous
    c.update([.2,0,.65],args[1],args[2],'DRIVE',.02)
    assert c.clock==pytest.approx(.02) and c.offset==pytest.approx(.2)
    assert c.reconfigurations==1


def test_unready_cancels_and_cannot_auto_resume():
    c=tracker();args=([0,0,.65],[0,0,0,1],[0,0,0],'DRIVE',.02)
    c.update(*args,healthy=False)
    assert c.state=='FAULT'
    np.testing.assert_equal(c.update(*args),np.zeros(3))


def test_completion_needs_pose_velocity_and_hold():
    c=tracker();c.state='RUNNING';c.was_running=True;c.clock=c.profile.duration
    c.update([4,0,.65],[0,0,0,1],[.1,0,0],'DRIVE',.02)
    assert c.state=='STOPPING'
    for _ in range(30):c.update([4,0,.65],[0,0,0,1],[0,0,0],'BRAKE',.02)
    assert c.state=='STOPPING'
    for _ in range(30):c.update([4,0,.65],[0,0,0,1],[0,0,0],'HOLD',.02)
    assert c.state=='COMPLETED'


def test_terminal_result_is_latched():
    c=tracker();c.state='COMPLETED';c.fault('later sensor outage')
    assert c.state=='COMPLETED'


def test_small_terminal_yaw_error_creates_time_profile_instead_of_sticking_in_hold():
    from scipy.spatial.transform import Rotation
    c=tracker();c.state='RUNNING';c.was_running=True;c.clock=c.profile.duration
    q=Rotation.from_euler('z',-.04).as_quat()
    for _ in range(22):c.update([4,0,.65],q,[0,0,0],'HOLD',.02)
    assert c.trims==1 and c.kind=='rotate'
    assert c.profile.distance==pytest.approx(.04)
    assert c.state=='ALIGNING'
    command=c.update([4,0,.65],q,[0,0,0],'ALIGN',.02)
    assert command[2]>=c.cfg['alignment_angular_probe_rad_s']


def test_terminal_heading_not_starved_by_smaller_position_error():
    from scipy.spatial.transform import Rotation
    c=tracker();c.state='RUNNING';c.was_running=True;c.clock=c.profile.duration
    q=Rotation.from_euler('z',-.1).as_quat()
    for _ in range(22):c.update([3.94,0,.65],q,[0,0,0],'HOLD',.02)
    assert c.trims==1 and c.kind=='rotate'


def test_alignment_probe_stable_despite_estimated_heading_noise():
    from scipy.spatial.transform import Rotation
    c=tracker();first=c.update([0,0,.65],[0,0,0,1],[0,0,0],'ALIGN',.02)
    noisy=Rotation.from_euler('z',.02).as_quat()
    np.testing.assert_equal(c.update([0,0,.65],noisy,[0,0,0],'ALIGN',.02),first)


def test_forward_only_pass_faults_instead_of_reversing_after_overshoot():
    c=tracker();c.forward_only=True;c.state='RUNNING';c.was_running=True;c.clock=c.profile.duration
    for _ in range(25):command=c.update([4.08,0,.65],[0,0,0,1],[0,0,0],'HOLD',.02)
    assert c.state=='FAULT' and c.reason=='FORWARD_ONLY_TERMINAL_OVERSHOOT'
    np.testing.assert_equal(command,np.zeros(3))


@pytest.mark.parametrize('offset',[0.,1e-12])
def test_stale_filtered_terminal_error_does_not_create_zero_trim(offset):
    c=tracker();c.state='STOPPING';c.was_running=True;c.clock=c.profile.duration
    c.terminal_filtered=np.array([.4,0.,0.])
    c.outside_time=c.cfg['terminal_dwell_s']
    with np.errstate(all='raise'):
        command=c.update([4-offset,0,.65],[0,0,0,1],[0,0,0],'HOLD',.02)
    assert c.trims==0 and c.state=='STOPPING'
    assert np.isfinite(c.axis).all() and np.isfinite(command).all()
    for _ in range(200):c.update([4,0,.65],[0,0,0,1],[0,0,0],'HOLD',.02)
    assert c.state=='COMPLETED'


def test_actual_heading_trim_at_goal_position():
    from scipy.spatial.transform import Rotation
    c=tracker();c.state='STOPPING'
    r=Rotation.from_euler('z',-.06)
    c.start_trim(np.array([4.,0.,.65]),r)
    assert c.kind=='rotate' and c.length==pytest.approx(.06)
    assert np.isfinite(c.axis).all()


def test_lateral_limit_follows_platform_configuration():
    c=tracker();c.platform=dict(c.platform,max_lateral_speed=.03)
    c.axis=np.array([0.,1.]);c.goal=c.start+np.array([0,4,0])
    c.final_goal=c.goal.copy()
    for _ in range(20):
        command=c.update([0,0,.65],[0,0,0,1],[0,0,0],'DRIVE',.02)
        assert abs(command[1])<=.03+1e-12
    assert command[1]==pytest.approx(.03)


def test_planner_and_tracker_reserve_physical_braking_authority():
    from pathlib import Path
    import yaml
    from agv_mission.planner import Vehicle
    from agv_mission.tracking import trajectory_deceleration
    root=Path(__file__).resolve().parents[2]
    platform=yaml.safe_load((root/'agv_description/config/platform.yaml').read_text())
    camera=yaml.safe_load((root/'agv_description/config/linescan.yaml').read_text())
    cfg=yaml.safe_load((root/'agv_mission/config/tracking.yaml').read_text())
    # The rated scan speed, not the vehicle top speed: a 15 m segment reaches
    # cruise at 10 km/h, so the profile really does have a braking phase to check.
    c=SegmentTracker([0,0,0],[0,0,0,1],'translate',[15,0],0,platform['rated_scan_speed'],platform,cfg)
    vehicle=Vehicle.from_configs(platform,camera)
    assert c.profile.decel==vehicle.decel==.8
    assert platform['drive_decel']==1.
    assert c.profile.dd==pytest.approx(platform['rated_scan_speed']**2/(2*vehicle.decel))
    # A segment may still be commanded up to the vehicle top speed.
    SegmentTracker([0,0,0],[0,0,0,1],'translate',[60,0],0,platform['max_speed'],platform,cfg)
    with pytest.raises(ValueError):
        SegmentTracker([0,0,0],[0,0,0,1],'translate',[60,0],0,platform['max_speed']*1.01,platform,cfg)
    with pytest.raises(ValueError):trajectory_deceleration(dict(platform,trajectory_decel=1.1))


def segment(distance,speed):
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'config/tracking.yaml').read_text())
    platform=yaml.safe_load((root.parent/'agv_description/config/platform.yaml').read_text())
    return SegmentTracker([0,0,.65],[0,0,0,1],'translate',[distance,0],1,speed,platform,config),config


def test_segment_watchdog_scales_with_the_planned_profile():
    short,config=segment(4.,.5)
    long,_=segment(103.,1.)
    for core in (short,long):
        assert core.budget>core.profile.duration
        assert core.budget==pytest.approx(core.profile.duration*config['segment_timeout_factor']
                                          +config['segment_timeout_margin_s'])
    # A 100 m pass must fit; the old fixed 90 s budget faulted it mid-scan.
    assert long.profile.duration>90. and long.budget>long.profile.duration
    # Scaling must not loosen a short segment: it stays well inside the old value.
    assert short.budget<90.


def test_segment_watchdog_still_faults_a_stalled_segment():
    # Never reaching DRIVE keeps the reference frozen, so the segment stalls
    # without the earlier tracking-error watchdog masking the timeout.
    core,_=segment(4.,.5)
    elapsed=0.
    while elapsed<core.budget+2. and core.state!='FAULT':
        core.update([0,0,.65],[0,0,0,1],[0,0,0],'HOLD',.05);elapsed+=.05
    assert core.state=='FAULT' and core.reason=='SEGMENT_TIMEOUT'
    assert elapsed==pytest.approx(core.budget,abs=.1)


def test_segment_watchdog_factor_may_not_shorten_the_plan():
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'config/tracking.yaml').read_text())
    platform=yaml.safe_load((root.parent/'agv_description/config/platform.yaml').read_text())
    config['segment_timeout_factor']=.9
    with pytest.raises(ValueError):
        SegmentTracker([0,0,.65],[0,0,0,1],'translate',[4,0],1,.5,platform,config)


def closed_loop_pass(distance,speed,delay_ticks,dt=.02):
    """Drive the tracker against a plant whose speed is its own command, delayed."""
    from collections import deque
    core,_=segment(distance,speed)
    core.forward_only=True
    lag=deque([0.]*delay_ticks,maxlen=delay_ticks)
    p=np.array([0.,0.,.65]);q=[0,0,0,1];v=0.;elapsed=0.
    while core.state not in ('COMPLETED','FAULT') and elapsed<core.budget:
        mode='HOLD' if core.state=='STOPPING' and v<core.cfg['stopped_speed_m_s'] else 'DRIVE'
        command=core.update(p,q,[v,0.,0.],mode,dt)
        lag.append(float(command[0]));v=lag[0]
        p=p+np.array([v*dt,0.,0.]);elapsed+=dt
    return core,float(p[0])


def test_terminal_overshoot_absorbs_the_actuation_delay():
    # Two ticks of pure delay is the measured 40 ms between command and motion.
    # Open loop that costs speed x delay = 40 mm of overshoot, four fifths of the
    # terminal tolerance, and a forward-only pass cannot take any of it back.
    core,x=closed_loop_pass(20.,1.,2)
    assert core.state=='COMPLETED',core.reason
    overshoot=x-20.
    assert 0<=overshoot<.01,overshoot


def test_stopping_distance_cap_does_not_slow_the_scan():
    # The cap is the profile's own braking curve, so it must be inactive while
    # there is distance left; a pass may not take longer than its plan allows.
    core,x=closed_loop_pass(20.,1.,2)
    assert core.elapsed<core.profile.duration+2.
