"""A capture handshake must not stop a pass that is already running.

Measured over five full-area runs (tools/analyze_capture_handshake.py): the
open handshake outlasted one control tick 50 times out of 50 and never braked,
because it was exempted; the close outlasted one tick 6 times out of 50 and
latched Mode::Brake all six times, stopping the vehicle from 10 km/h in the
run-out buffer. These pin the exemption to both directions while keeping the
one halt that the segment contract needs.
"""
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import yaml
from agv_mission.execution import compile_steps
from agv_mission.execution_node import Executor
from test_execution import fixture


class Core:
    """Stand-in tracker: records whether the executor advanced it this tick."""
    def __init__(self):
        self.state='RUNNING';self.diagnostic={'reference_heading_error_rad':0.};self.updates=0
        self.forward_only=False
    def update(self,p,q,v,mode,dt):
        self.updates+=1;return [2.7,0.,0.]


class Capture:
    def __init__(self,pending,target,active):
        self.future=object() if pending else None
        self.target=target;self.active=active;self.enabled=True;self.requests=[]
    def request(self,value,track=None,reason='requested'):
        self.requests.append((value,reason));return self.future is None


class Harness:
    run_step=Executor.run_step
    def __init__(self,*,pending,target,active=True,core=True,along_m=None):
        root=Path(__file__).resolve().parents[1]
        self.cfg=yaml.safe_load((root/'config/tracking.yaml').read_text())
        self.platform=yaml.safe_load(
            (root.parent/'agv_description/config/platform.yaml').read_text())
        self.camera={'camera_x_m':1.15}
        self.plan=fixture();self.plan['request']['coverage_error_m']=.1
        self.steps=compile_steps(self.plan,[3,0,.65],[0,0,0,1])
        # Stand on the first pass, the camera `along_m` past the region start.
        # A close is requested past the region plus the over-run, so that is
        # where the vehicle is unless a test says otherwise.
        self.index=self.steps.index(next(s for s in self.steps if s['kind']=='PASS'))
        track=self.plan['tracks'][0]
        start=np.array(track['scan_start_xyz_m']);end=np.array(track['scan_end_xyz_m'])
        length=float(np.linalg.norm(end-start));axis=(end-start)/length
        if along_m is None:along_m=length+self.plan['scan_overrun_distance_m']+.1
        self.p=start+along_m*axis-[self.camera['camera_x_m'],0,0];self.p[2]=.65
        self.q=np.array([0.,0.,0.,1.])
        self.capture=Capture(pending,target,active)
        self.core=Core() if core else None
        self.odom=NS(twist=NS(twist=NS(linear=NS(x=2.7,y=0.),angular=NS(z=0.))))
        self.mode='DRIVE';self.state='RUNNING';self.reason='';self.step_attempts=0
        self.active_kind='translate';self.cmd=None
    def pose(self):return self.p.copy(),self.q.copy()
    def command(self,v):self.cmd=list(v)
    def fault(self,reason):self.state='FAULT';self.reason=reason;self.command([0,0,0])


def test_a_pending_close_no_longer_halts_a_pass_that_is_already_running():
    h=Harness(pending=True,target=False);h.run_step(.02)
    assert h.core.updates==1 and h.cmd==[2.7,0.,0.] and h.state=='RUNNING'


def test_a_pending_open_still_does_not_halt_a_running_pass():
    # An open is requested in the lead-in, before the region begins.
    h=Harness(pending=True,target=True,active=False,along_m=-.5);h.run_step(.02)
    assert h.core.updates==1 and h.cmd==[2.7,0.,0.]


def test_no_pending_handshake_leaves_the_pass_untouched():
    h=Harness(pending=False,target=False);h.run_step(.02)
    assert h.core.updates==1 and h.cmd==[2.7,0.,0.]


def test_a_pending_handshake_still_halts_at_a_segment_boundary():
    """The contract that survives: a close must land before the next segment."""
    h=Harness(pending=True,target=False,core=False);h.run_step(.02)
    assert h.cmd==[0.,0.,0.] and h.core is None and h.step_attempts==0


def test_a_completing_segment_hands_over_to_the_boundary_halt():
    h=Harness(pending=True,target=False)
    h.core.state='COMPLETED';index=h.index
    h.run_step(.02)
    assert h.core is None and h.index==index+1        # the segment finished normally
    h.run_step(.02)
    assert h.cmd==[0.,0.,0.] and h.core is None       # and the next one waits
