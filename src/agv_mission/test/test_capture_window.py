"""When a pass opens its capture window, and what guarantees it reached the region."""
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
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
