"""Name the motion phase a telemetry record belongs to.

The acceptance matrix injects a pause or a fault in each phase, so the mapping
from a live status record to a phase name is itself part of the evidence: a
mislabelled injection would report a pass in a phase it never entered.
"""
from pathlib import Path
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from agv_mission.execution import phase_of,compile_steps
from test_execution import fixture


CAMERA=yaml.safe_load((Path(get_package_share_directory('agv_description'))/'config/linescan.yaml').read_text())


def record(plan,track_id,base_x,**extra):
    track=plan['tracks'][track_id]
    axis=np.array(track['scan_end_xyz_m'])-track['scan_start_xyz_m']
    axis=axis/np.linalg.norm(axis)
    position=np.array(track['scan_start_xyz_m'])+axis*base_x
    position[2]=.65
    fields=dict(kind='PASS',track_id=track_id,position_m=position.tolist(),segment_kind='translate')
    fields.update(extra);return fields


def test_pass_splits_at_the_camera_crossing_the_region_not_the_axle():
    p=fixture();length=np.linalg.norm(np.array(p['tracks'][0]['scan_end_xyz_m'])-p['tracks'][0]['scan_start_xyz_m'])
    offset=CAMERA['camera_x_m']
    # base_link one camera offset short of the region: the sensor is exactly on
    # the boundary, so anything earlier is still lead-in.
    assert phase_of(p,CAMERA,record(p,0,-offset-.01))=='accelerate'
    assert phase_of(p,CAMERA,record(p,0,-offset+.01))=='scan'
    assert phase_of(p,CAMERA,record(p,0,length-offset-.01))=='scan'
    assert phase_of(p,CAMERA,record(p,0,length-offset+.01))=='runout'


def test_reversed_track_uses_its_own_axis():
    p=fixture();first=p['tracks'][0];second=p['tracks'][1]
    assert second['direction']==-first['direction']
    # Same fractional progress along each track's own heading gives the same phase.
    for track_id in (0,1):
        assert phase_of(p,CAMERA,record(p,track_id,-3.))=='accelerate'
        assert phase_of(p,CAMERA,record(p,track_id,4.))=='scan'


def test_braking_is_runout_even_before_the_region_end():
    p=fixture()
    assert phase_of(p,CAMERA,record(p,0,2.,tracker_state='STOPPING'))=='runout'
    assert phase_of(p,CAMERA,record(p,0,2.,tracker_state='RUNNING'))=='scan'


def test_transfer_steps_and_in_pass_alignment_are_named_separately():
    p=fixture()
    assert phase_of(p,CAMERA,dict(kind='SHIFT',track_id=0,segment_kind='translate'))=='shift'
    assert phase_of(p,CAMERA,dict(kind='SHIFT',track_id=0,segment_kind='rotate'))=='rotate'
    assert phase_of(p,CAMERA,dict(kind='ROTATE_180',track_id=0,segment_kind='rotate'))=='rotate'
    assert phase_of(p,CAMERA,dict(kind='APPROACH',track_id=0,segment_kind='translate'))=='approach'
    # A pass whose heading has not converged is turning, not accelerating.
    assert phase_of(p,CAMERA,record(p,0,-3.,segment_kind='rotate'))=='rotate'


def test_records_without_a_step_or_tracker_claim_no_phase():
    p=fixture()
    assert phase_of(p,CAMERA,{})=='none'
    assert phase_of(p,CAMERA,dict(kind='PASS',track_id=0,position_m=[3,0,.65]))=='none'
    assert phase_of(p,CAMERA,dict(kind='PASS',track_id=0,segment_kind='translate'))=='none'


def test_every_compiled_step_kind_is_covered():
    p=fixture();kinds={s['kind'] for s in compile_steps(p,[3,0,.65],[0,0,0,1])}
    named={'APPROACH':'approach','SHIFT':'shift','ROTATE_180':'rotate'}
    for kind in kinds-{'PASS'}:
        assert phase_of(p,CAMERA,dict(kind=kind,track_id=0,segment_kind='translate'))==named[kind]


def test_region_along_measures_from_the_region_start_including_the_camera_offset():
    from agv_mission.execution import region_along_m
    p=fixture();length=float(np.linalg.norm(
        np.array(p['tracks'][0]['scan_end_xyz_m'])-p['tracks'][0]['scan_start_xyz_m']))
    along,reported=region_along_m(p,CAMERA,record(p,0,2.))
    assert reported==length
    assert along==2.+CAMERA['camera_x_m']
    # Both reversed and forward tracks measure along their own heading.
    assert region_along_m(p,CAMERA,record(p,1,2.))[0]==2.+CAMERA['camera_x_m']
    assert region_along_m(p,CAMERA,{}) is None
