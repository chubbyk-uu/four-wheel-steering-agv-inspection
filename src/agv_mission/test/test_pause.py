"""Exercise executor service/state transitions without a live ROS graph."""
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import yaml
from agv_mission.execution_node import Executor
from agv_mission.callback_trace import PhaseTrace


class Harness:
    pause=Executor.pause
    resume=Executor.resume
    pause_tick=Executor.pause_tick
    stopped=Executor.stopped
    localization_reason=Executor.localization_reason
    def __init__(self):
        self.cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tracking.yaml').read_text())
        self.state='RUNNING';self.now=1.;self.last_clock=1.;self.mode='DRIVE';self.events=[]
        self.capture=NS(error='',future=None,active=True)
        self.odom=NS(twist=NS(twist=NS(linear=NS(x=.5,y=0.),angular=NS(z=0.))))
        self.p=np.array([3.,0.,.65]);self.q=np.array([0.,0.,0.,1.]);self.index=0
        self.steps=[dict(kind='PASS',end=dict(position=[6.,0.,.65],orientation_xyzw=[0,0,0,1]),speed=.5)]
        self.plan=dict(request=dict(road=dict(origin_xyz_m=[0,0,0],yaw_rad=0),drivable_bounds_xy_m=[0,20,-5,5]),sweep_radius_m=1.5)
        self.healthy=True;self.reason='';self.core=object();self.step_attempts=3
        self.health={}
        self.probe=PhaseTrace(self.cfg['control_stall_threshold_s']);self.ticks=0
        # Writes stay synchronous here so a test can read the record it just made.
        self.archive=NS(errors=0,peak=0,submit=lambda action:action(),idle=True,
                        sync=lambda:None,append=lambda handle,text:handle.write(text))
        self.archive_settled=False
    def get_clock(self):return NS(now=lambda:NS(nanoseconds=round(self.now*1e9)))
    def valid(self,now):return self.healthy
    def count_publishers(self,topic):return 1
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
        h.execution_id='test';h.ready_since=None
        h.last_command=[.5,0,0];h.motion_reason='';h.log=io.StringIO()
        h.odom.header=NS(stamp=NS(sec=0,nanosec=900000000))
        h.capture=NS(error='',poll=lambda:None,enabled=True,active=True,future=None,
                     heartbeat={},close_failed=False)
        h.published=[];h.status_stream=NS(offer=h.published.append,dropped=0,errors=0)
        Executor.tick(h)
        assert h.state=='FAULT' and h.reason==expected and h.cmd==[0,0,0]
        record=json.loads(h.log.getvalue())
        assert record['control_dt_s']==pytest.approx(dt)
        assert record['health']['state']=='READY'
        assert record['odom_age_s']==pytest.approx(.1)


def blocked_tick(gap,dt,healthy):
    """Drive one tick after the executor's own thread was blocked for `gap` seconds."""
    import io
    from time import monotonic
    h=Harness();h.last_sim=h.now-dt;h.healthy=healthy
    h.core=None;h.steps=[];h.health={'state':'READY'};h.execution_id='test';h.ready_since=None
    h.last_command=[.5,0,0];h.motion_reason='';h.log=io.StringIO()
    h.odom.header=NS(stamp=NS(sec=0,nanosec=900000000))
    # Feedback arrived before the block and was never dispatched during it.
    h.arrivals={k:monotonic()-gap for k in ('odom','mode','health')}
    h.capture=NS(error='',poll=lambda:None,enabled=True,active=True,future=None,
                 heartbeat={},close_failed=False)
    h.published=[];h.status_stream=NS(offer=h.published.append,dropped=0,errors=0)
    h.probe.previous_end=monotonic()-gap
    Executor.tick(h)
    return h


def test_executor_stall_is_not_reported_as_stale_localization():
    import json
    # The recorded 0.38 s block left the simulation clock undispatched, so the
    # timestep check could not fire and healthy feedback was blamed instead.
    h=blocked_tick(.38,0.,True)
    assert h.state=='FAULT' and h.reason=='CONTROL_LOOP_STALLED' and h.cmd==[0,0,0]
    record=json.loads(h.log.getvalue())
    assert record['health']['state']=='READY'
    assert min(record['feedback_wall_age_s'].values())>h.cfg['wall_timeout_s']
    stall=record['control_stalls'][-1]
    assert stall['kind']=='gap' and stall['gap_before_s']>=.38
    # Telemetry is handed over, not published from the control thread, and the
    # archived record and the broadcast one must stay the same object.
    assert h.published==[h.log.getvalue().rstrip('\n')]


def test_genuinely_stale_feedback_still_faults_as_localization():
    # Same stale arrival ages, but the control loop itself never stopped.
    h=blocked_tick(0.,.02,False)
    h.arrivals={}
    assert h.state=='FAULT' and h.reason=='STALE_OR_UNREADY_LOCALIZATION'


def test_paused_mission_also_names_its_own_stall():
    from time import monotonic
    h=Harness();h.state='PAUSED';h.probe.begin();h.probe.previous_end=monotonic()-.38
    h.probe.begin();h.pause_tick(1.,True,.02,1.)
    assert h.state=='FAULT' and h.reason=='CONTROL_LOOP_STALLED'


def test_a_lost_navigation_record_is_not_reported_as_stale_localization():
    # The adapter demotes itself when its archive write fails, which is what stops
    # the mission. Calling that staleness would send the search to the wrong node.
    import io
    h=Harness();h.last_sim=h.now-.02;h.healthy=False
    h.core=None;h.steps=[];h.arrivals={};h.execution_id='test';h.ready_since=None
    h.health={'state':'NOT_READY','archive_errors':1,'stop_required':True}
    h.last_command=[.5,0,0];h.motion_reason='';h.log=io.StringIO()
    h.odom.header=NS(stamp=NS(sec=0,nanosec=900000000))
    h.capture=NS(error='',poll=lambda:None,enabled=True,active=True,future=None,
                 heartbeat={},close_failed=False)
    h.published=[];h.status_stream=NS(offer=h.published.append,dropped=0,errors=0)
    Executor.tick(h)
    assert h.state=='FAULT' and h.reason=='NAVIGATION_ARCHIVE_WRITE_FAILED'
    # The same outage during a pause must name the same cause.
    p=Harness();park(p);p.health={'archive_errors':2}
    p.pause_tick(1.,False,.02,1.)
    assert p.reason=='NAVIGATION_ARCHIVE_WRITE_FAILED'
    # Without an archive failure the reason is unchanged.
    q=Harness();park(q);q.pause_tick(1.,False,.02,1.)
    assert q.reason=='STALE_OR_UNREADY_LOCALIZATION'


def test_a_cancel_or_fault_reports_its_archive_separately_from_its_state():
    # Mission end can wait for the queue before it claims success. A fault cannot:
    # it has to stop the vehicle on the tick it is raised. So the terminal state
    # stops meaning "the evidence is written", and a separate flag says that.
    import io
    def harness(state):
        h=Harness();h.state=state;h.reason='CANCELED' if state=='CANCELING' else ''
        h.last_sim=h.now-.02;h.core=None;h.steps=[];h.arrivals={};h.health={'state':'READY'}
        h.execution_id='test';h.ready_since=None;h.mode='HOLD'
        h.last_command=[0,0,0];h.motion_reason='';h.log=io.StringIO()
        h.odom.header=NS(stamp=NS(sec=0,nanosec=900000000))
        h.capture=NS(error='',poll=lambda:None,enabled=True,active=False,future=None,
                     heartbeat={},close_failed=False,request=lambda *a,**k:True)
        h.published=[];h.status_stream=NS(offer=h.published.append,dropped=0,errors=0)
        return h
    h=harness('CANCELING');h.archive.idle=False
    Executor.tick(h)
    assert h.state=='CANCELED','the state still goes terminal at once'
    assert h.archive_settled is False,'but it must not claim the archive is down'
    h.archive.idle=True;Executor.tick(h)
    assert h.archive_settled is True
    # A write that failed is not a settled archive either.
    f=harness('FAULT');f.archive.idle=True;f.archive.errors=1
    Executor.tick(f)
    assert f.state=='FAULT' and f.archive_settled is False
