"""When a pass opens its capture window, and what guarantees it reached the region."""
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
import yaml
from agv_mission.execution import camera_along_m, compile_steps
from agv_mission.execution_node import Executor
from agv_mission.planner import Vehicle, plan as make_plan

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT.parent/'agv_description/config'


class Capture:
    def __init__(self, enabled=True):
        self.enabled=enabled;self.active=False;self.future=None;self.target=False
        self.error='';self.heartbeat={};self.close_failed=False;self.opened=0;self.closed=[]
    def poll(self):pass
    def request(self, value, track=None, reason='requested'):
        self.target=value
        if value:self.opened+=1
        else:self.closed.append(reason)
        self.active=value;return True


class Pass:
    """Drive Executor.run_step over the first pass of a real plan."""
    run_step=Executor.run_step
    def __init__(self, capture=None):
        self.platform=yaml.safe_load((CONFIG/'platform.yaml').read_text())
        self.camera=yaml.safe_load((CONFIG/'linescan.yaml').read_text())
        self.cfg=yaml.safe_load((ROOT/'config/tracking.yaml').read_text())
        request=yaml.safe_load((ROOT/'config/rectangle_demo.yaml').read_text())
        request['road']['frame_id']='map';request['coverage_error_m']=.1
        self.plan=make_plan(request,Vehicle.from_configs(self.platform,self.camera))
        entry=np.array(self.plan['tracks'][0]['base_entry_pose']['position'])
        self.steps=[s for s in compile_steps(self.plan,entry,[0.,0.,0.,1.]) if s['kind']=='PASS'][:1]
        self.capture=capture if capture is not None else Capture()
        self.index=0;self.core=None;self.step_attempts=0;self.active_kind=None
        self.p=entry.copy();self.q=np.array([0.,0.,0.,1.])
        self.mode='HOLD';self.state='RUNNING';self.reason='';self.cmd=[0,0,0]
        self.odom=NS(twist=NS(twist=NS(linear=NS(x=0.,y=0.),angular=NS(z=0.))))
    def pose(self):return self.p.copy(),self.q.copy()
    def command(self,v):self.cmd=list(v)
    def fault(self,reason):self.state='FAULT';self.reason=reason;self.command([0,0,0])
    def along(self):return camera_along_m(self.plan,self.camera,self.steps[0],*self.pose())[0]
    def place(self,along):
        track=self.plan['tracks'][0]
        start=np.array(track['scan_start_xyz_m']);finish=np.array(track['scan_end_xyz_m'])
        axis=(finish-start)/np.linalg.norm(finish-start)
        self.p=start+axis*along-np.array([self.camera['camera_x_m'],0.,0.])
        self.p[2]=self.platform['base_height']


def started():
    """A pass whose tracker exists, still stopped at the entry with wheels unset."""
    h=Pass();h.run_step(.02)
    assert h.core is not None and h.active_kind=='translate'
    return h


def test_capture_opens_only_once_the_pass_has_finished_aligning():
    # Opening before the pass re-steers leaves the alignment's own creep in the
    # first image; a motion-quality window then excludes that whole 1.5 m block,
    # and 18 ms of overlap cost one measured pass 0.84 m inside the region.
    h=started()
    assert h.capture.opened==0
    for _ in range(5):
        h.mode='ALIGN';h.run_step(.02)
    assert h.capture.opened==0 and h.capture.active is False
    assert h.along()<-h.plan['request']['coverage_error_m']
    h.mode='DRIVE';h.run_step(.02)
    assert h.capture.opened==1 and h.capture.active is True
    # Re-requesting every tick is free and must not reopen a second interval.
    h.run_step(.02);assert h.capture.opened==2 and h.capture.active is True


def test_camera_may_not_reach_the_region_without_capture():
    # The guarantee that replaces "do not move before the sensor acknowledges".
    h=started();h.mode='ALIGN'
    h.place(-h.plan['request']['coverage_error_m']-.01);h.run_step(.02)
    assert h.state=='RUNNING'
    h.place(-h.plan['request']['coverage_error_m']+.01);h.run_step(.02)
    assert h.state=='FAULT' and h.reason=='CAPTURE_NOT_ACTIVE_AT_REGION'
    assert h.cmd==[0,0,0]


def test_mission_without_capture_integration_never_faults_at_the_region():
    h=started();h.capture=Capture(enabled=False);h.mode='ALIGN'
    h.place(1.);h.run_step(.02)
    assert h.state=='RUNNING' and h.capture.opened==0


def test_capture_closes_a_full_over_run_past_the_region_not_at_its_edge():
    h=started();h.mode='DRIVE';h.run_step(.02)
    length=camera_along_m(h.plan,h.camera,h.steps[0],*h.pose())[1]
    overrun=h.plan['scan_overrun_distance_m']
    h.place(length+overrun-.01);h.run_step(.02)
    assert h.capture.active is True and not h.capture.closed
    h.place(length+overrun+.01);h.run_step(.02)
    assert h.capture.active is False and h.capture.closed==['track_end']


class Queue:
    """The archive as the executor sees it: draining happens on another thread."""
    def __init__(self):
        self.errors=0;self.last_error='';self.drained=False;self.syncs=0
    def sync(self):self.syncs+=1
    @property
    def idle(self):return self.drained


class End(Pass):
    """Every step done: the tick on which the mission decides it has finished."""
    def __init__(self,queue):
        super().__init__()
        self.steps=[];self.index=0;self.archive=queue;self.archive_since=None


def test_the_mission_waits_for_the_archive_before_claiming_success():
    # ACQUIRED used to mean the camera shut. The final interval record is handed
    # over on that same tick, and the coverage audit drops a whole pass without it.
    queue=Queue();h=End(queue)
    h.run_step(.02)
    assert h.state=='RUNNING' and h.cmd==[0,0,0]
    assert queue.syncs>=1,'never asked the writer to drain'
    h.run_step(.02)
    assert h.state=='RUNNING'
    queue.drained=True
    h.run_step(.02)
    assert h.state=='ACQUIRED' and h.reason==''


def test_a_write_that_failed_while_draining_faults_instead_of_completing():
    queue=Queue();queue.drained=True;queue.errors=1;queue.last_error="OSError('no space left')"
    h=End(queue);h.run_step(.02)
    assert h.state=='FAULT' and h.reason=='ARCHIVE_WRITE_FAILED'


def test_a_queue_that_never_drains_faults_rather_than_hanging_in_running():
    import time
    queue=Queue();h=End(queue)
    h.run_step(.02)
    assert h.state=='RUNNING'
    h.archive_since=time.monotonic()-h.cfg['archive_drain_timeout_s']-.1
    h.run_step(.02)
    assert h.state=='FAULT' and h.reason=='ARCHIVE_DRAIN_TIMEOUT'


def test_a_mission_without_capture_completes_on_the_same_rule():
    queue=Queue();h=End(queue);h.capture=Capture(enabled=False)
    h.run_step(.02)
    assert h.state=='RUNNING'
    queue.drained=True;h.run_step(.02)
    assert h.state=='COMPLETED'


@pytest.mark.parametrize('sign',[-1,1])
def test_resteering_body_yaw_must_recover_before_shutter_opens(sign):
    from scipy.spatial.transform import Rotation
    h=started()
    h.mode='ALIGN';h.q=Rotation.from_euler('z',sign*.074).as_quat()
    h.run_step(.02)
    h.mode='DRIVE';h.run_step(.02)
    Executor.check_scan_heading(h)
    assert h.state=='RUNNING' and not h.capture.active
    h.run_step(.02)  # filtered heading feedback clears its deadband
    assert h.cmd[0]>0 and h.cmd[2]*sign<0  # lead-in feedback, no reverse or extra spin
    h.q=Rotation.from_euler('z',sign*.02).as_quat();h.run_step(.02)
    assert h.capture.active
    # Once armed, the tighter opening tolerance must not split a partial frame.
    h.q=Rotation.from_euler('z',sign*.03).as_quat();h.run_step(.02)
    Executor.check_scan_heading(h)
    assert h.capture.active and h.state=='RUNNING' and not h.capture.closed
    h.q=Rotation.from_euler('z',sign*.051).as_quat();h.run_step(.02)
    Executor.check_scan_heading(h)
    assert h.state=='FAULT' and h.reason=='SCAN_HEADING_EXCEEDS_GATE'


def test_heading_not_recovered_at_region_fails_without_opening():
    from scipy.spatial.transform import Rotation
    h=started();h.mode='DRIVE'
    h.q=Rotation.from_euler('z',-.074).as_quat()
    h.place(-h.plan['request']['coverage_error_m']+.02)
    # Keep the time reference close to the region to isolate the capture guard.
    h.core.start=h.p.copy();h.core.reference=h.p.copy()
    h.run_step(.02)
    assert not h.capture.active and h.capture.opened==0
    assert h.state=='FAULT' and h.reason=='CAPTURE_NOT_ACTIVE_AT_REGION'
