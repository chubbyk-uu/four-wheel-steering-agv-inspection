"""Exercise executor service/state transitions without a live ROS graph."""
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import yaml
from agv_mission.execution_node import Executor


class Harness:
    pause=Executor.pause
    resume=Executor.resume
    pause_tick=Executor.pause_tick
    stopped=Executor.stopped
    def __init__(self):
        self.cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tracking.yaml').read_text())
        self.state='RUNNING';self.now=1.;self.last_clock=1.;self.mode='DRIVE';self.events=[]
        self.capture=NS(error='',future=None,active=True)
        self.odom=NS(twist=NS(twist=NS(linear=NS(x=.5,y=0.),angular=NS(z=0.))))
        self.p=np.array([3.,0.,.65]);self.q=np.array([0.,0.,0.,1.]);self.index=0
        self.steps=[dict(kind='PASS',end=dict(position=[6.,0.,.65],orientation_xyzw=[0,0,0,1]),speed=.5)]
        self.plan=dict(request=dict(road=dict(origin_xyz_m=[0,0,0],yaw_rad=0),drivable_bounds_xy_m=[0,20,-5,5]),sweep_radius_m=1.5)
        self.healthy=True;self.reason='';self.core=object();self.step_attempts=3
    def get_clock(self):return NS(now=lambda:NS(nanoseconds=round(self.now*1e9)))
    def valid(self,now):return self.healthy
    def pose(self):return self.p.copy(),self.q.copy()
    def pause_record(self,event):self.events.append(event)
    def command(self,v):self.cmd=v
    def fault(self,reason):self.state='FAULT';self.reason=reason;self.command([0,0,0])


def park(h):
    assert h.pause(None,NS()).success
    h.mode='HOLD';h.odom.twist.twist.linear.x=0
    h.pause_tick(1.,True,.02,1.)
    h.pause_tick(1.6,True,.02,1.)
    assert h.state=='PAUSED'


def test_pause_keeps_camera_armed_and_resume_rebuilds_original_goal():
    h=Harness();goal=h.steps[0]['end'].copy();park(h)
    assert h.capture.active and h.cmd==[0,0,0]
    assert h.pause(None,NS()).success # Idempotent, no repeated events.
    assert h.resume(None,NS()).success
    assert h.state=='RUNNING' and h.core is None and h.steps[0]['end']==goal
    assert h.events==['pause_requested','stopped','resumed']
    assert h.capture.active and h.index==0


def test_resume_requires_settled_healthy_feedback_and_no_pending_handshake():
    h=Harness();h.pause(None,NS());assert not h.resume(None,NS()).success
    h=Harness();park(h);h.healthy=False;assert not h.resume(None,NS()).success
    h.healthy=True;h.capture.future=object();assert not h.resume(None,NS()).success
    assert h.state=='PAUSED'


def test_resume_rejects_moved_vehicle_or_reverse_pass():
    h=Harness();park(h);h.p[0]+=.3;assert not h.resume(None,NS()).success
    h=Harness();park(h);h.steps[0]['end']['position']=[2.,0.,.65]
    assert not h.resume(None,NS()).success and h.state=='PAUSED'


def test_fault_during_pause_never_resumes_on_health_recovery():
    h=Harness();park(h);h.pause_tick(2,False,.02,1)
    assert h.state=='FAULT' and h.cmd==[0,0,0]
    h.healthy=True;assert not h.resume(None,NS()).success


def test_stop_timeout_and_sim_clock_stall_are_faults():
    h=Harness();h.pause(None,NS());h.pause_tick(12,True,.02,1)
    assert h.reason=='PAUSE_STOP_TIMEOUT'
    h=Harness();park(h);h.pause_tick(2,True,0,2)
    assert h.reason=='SIM_CLOCK_STALLED'


def test_callback_gap_is_distinguished_from_localization_timeout():
    import io,json
    import pytest
    for dt,healthy,expected in ((.259,False,'INVALID_CONTROL_TIMESTEP'),
                                (.259,True,'INVALID_CONTROL_TIMESTEP'),
                                (.02,False,'STALE_OR_UNREADY_LOCALIZATION')):
        h=Harness();h.last_sim=h.now-dt;h.healthy=healthy
        h.core=None;h.steps=[];h.health={'state':'READY'};h.arrivals={}
        h.last_command=[.5,0,0];h.motion_reason='';h.log=io.StringIO()
        h.odom.header=NS(stamp=NS(sec=0,nanosec=900000000))
        h.capture=NS(error='',poll=lambda:None,enabled=True,active=True,
                     heartbeat={},close_failed=False)
        h.status=NS(publish=lambda message:None)
        Executor.tick(h)
        assert h.state=='FAULT' and h.reason==expected and h.cmd==[0,0,0]
        record=json.loads(h.log.getvalue())
        assert record['control_dt_s']==pytest.approx(dt)
        assert record['health']['state']=='READY'
        assert record['odom_age_s']==pytest.approx(.1)
