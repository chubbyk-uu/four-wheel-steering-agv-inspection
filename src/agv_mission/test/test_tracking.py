from pathlib import Path
import numpy as np
import pytest
import yaml
from agv_mission.tracking import Profile, SegmentTracker


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
