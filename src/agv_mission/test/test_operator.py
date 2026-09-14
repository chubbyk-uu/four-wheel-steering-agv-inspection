from copy import deepcopy
import math
import json
from pathlib import Path
from types import SimpleNamespace as NS
import yaml
from agv_mission.operator_node import edited_request,Operator


def test_editor_retains_map_registration_and_bounds():
    request=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/rectangle_demo.yaml').read_text());before=deepcopy(request)
    updated=edited_request(request,dict(start_x=7.,length=2.,spacing=1.))
    assert request==before and updated['region']['start_xy_m'][0]==7
    assert updated['road']==before['road'] and updated['drivable_bounds_xy_m']==before['drivable_bounds_xy_m']


class Harness:
    receive=Operator.receive
    def __init__(self):
        self.ids=set();self.pending=None;self.job=None;self.child=NS(state='RUNNING');self.control_clients={};self.ok=True
    def editable(self):return False


def test_running_rejects_task_replacement_and_duplicate_request():
    h=Harness();m=NS(data=json.dumps(dict(id='one',action='load',path='/must/not/be/read')))
    h.receive(m);assert not h.ok and '运行中' in h.message
    h.message='unchanged';h.receive(m);assert h.message=='unchanged'


def test_other_cmd_publisher_prevents_start():
    h=Harness();h.control_clients={'start':None};h.count_publishers=lambda topic:2
    h.receive(NS(data=json.dumps(dict(id='two',action='start'))))
    assert not h.ok and '其他速度' in h.message


def test_real_broker_initializes_without_motion_publisher():
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init();executor=SingleThreadedExecutor();node=None
    try:
        node=Operator(executor)
        assert node.child is None and node.preview is None
        assert all(p.topic_name!='/cmd_vel' for p in node.publishers)
    finally:
        if node:node.close()
        executor.shutdown();rclpy.try_shutdown()


def test_inspection_gate_uses_physical_wheel_limit_not_nominal_pass_speed():
    from agv_mission.camera_limits import inspection_camera_config
    p=Path(__file__).resolve().parents[2]/'agv_description/config'
    camera=yaml.safe_load((p/'linescan.yaml').read_text());platform=yaml.safe_load((p/'platform.yaml').read_text())
    result=inspection_camera_config(camera,platform)
    assert result['max_scan_speed_m_s']==platform['max_speed']+.02
    assert result['max_scan_speed_m_s']>.8+.12
    assert result['width']==4096 and result['block_rows']==4096
    assert result['focal_length_m']==camera['focal_length_m']


def test_loaded_scene_rejects_forged_bounds_and_wrong_registration():
    import pytest
    from agv_mission.scene_bounds import scene_bounds,bind_request,check_request_scene
    base=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/rectangle_demo.yaml').read_text());base['road']['frame_id']='map'
    road=scene_bounds(dict(transform='identity_world_baked',frame='world',inspection_bounds_xy_m=[0,100,-5,5],drivable_bounds_xy_m=[-8,108,-6.5,6.5],optical_valid_bounds_xy_m=[-9,109,-8,8]))
    request=bind_request(base,road);check_request_scene(request,road)
    assert base['drivable_bounds_xy_m']==[0,20,-5,5]
    wrong=deepcopy(request);wrong['drivable_bounds_xy_m']=[-100,200,-20,20]
    with pytest.raises(ValueError,match='exceed'):check_request_scene(wrong,road)
    wrong=deepcopy(request);wrong['road']['origin_xyz_m']=[1,0,0]
    with pytest.raises(ValueError,match='registration'):check_request_scene(wrong,road)
    wrong=deepcopy(request);wrong['region']['length_m']=100
    with pytest.raises(ValueError,match='region'):check_request_scene(wrong,road)
    small=scene_bounds(dict(transform='identity_world_baked',frame='world',inspection_bounds_xy_m=[0,20,-5,5],optical_valid_bounds_xy_m=[-1.024,21.504,-6.144,6.144]))
    with pytest.raises(ValueError,match='exceed'):check_request_scene(request,small)


def test_static_preview_is_not_rebuilt_by_runtime_ticks(monkeypatch):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    rclpy.init();executor=SingleThreadedExecutor();node=Operator(executor)
    try:
        node.preview={'sent_already':True}
        monkeypatch.setattr(node,'show_preview',lambda: (_ for _ in ()).throw(AssertionError('runtime static preview rebuild')))
        for _ in range(12):node.tick()
    finally:
        node.close();executor.shutdown();rclpy.try_shutdown()


def test_process_status_cannot_cross_task_instance():
    from agv_mission.execution_process import ExecutionProcess
    child=ExecutionProcess.__new__(ExecutionProcess);child.execution_id='new';child.snapshot={};child.failure=''
    child.capture=NS(active=None,pending=False,future=None)
    assert not child.receive(dict(execution_id='old',state='READY',capture_active=False,capture_pending=False))
    assert child.state=='STARTING'
    assert child.receive(dict(execution_id='new',state='READY',capture_active=False,capture_pending=True))
    assert child.state=='READY' and child.capture.active is False and child.capture.pending and child.capture.future is None


def test_dead_executor_camera_close_failure_is_latched():
    from agv_mission.execution_process import ExecutionProcess
    child=ExecutionProcess.__new__(ExecutionProcess);child.failure='';child.process=NS(poll=lambda:1)
    child.capture=NS(active=True,future=None,close_failed=False);calls=[]
    def request(value):
        calls.append(value);return NS(done=lambda:True,result=lambda:NS(success=False))
    child.close_client=NS(service_is_ready=lambda:True,call_async=request)
    child.poll();child.poll();child.poll()
    assert child.state=='FAULT' and child.capture.close_failed and child.capture.active is None
    assert len(calls)==1


def test_navigation_tail_decides_when_the_audit_may_read(tmp_path):
    from agv_mission.operator_node import archived_through
    path=tmp_path/'navigation.jsonl'
    assert not archived_through(path,10.),'a missing archive is not ready'
    path.write_text(''.join(json.dumps(dict(time_s=t,x=0))+'\n' for t in (8.,9.,9.5)))
    assert archived_through(path,9.5)
    assert not archived_through(path,9.6),'the mission ends after the last record'
    # The writer appends, so the final line can be half a record.
    with path.open('a') as handle:handle.write('{"time_s": 10.')
    assert archived_through(path,9.5),'a torn last line must not hide a complete one'
    assert not archived_through(path,9.6)


def test_the_audit_waits_for_navigation_then_runs(tmp_path, monkeypatch):
    # Reading early is not a loud failure: the last block falls outside the
    # execution trace and the report grows a gap, and a rescan request, from
    # nothing. So the wait is on evidence, not on a guessed number of seconds.
    import agv_mission.operator_node as node
    navigation=tmp_path/'nav';navigation.mkdir()
    path=navigation/'navigation.jsonl'
    path.write_text(json.dumps(dict(time_s=5.))+'\n')
    calls=[]
    monkeypatch.setattr(node,'audit_capture',lambda *a:calls.append(a) or 'report')
    sleeps=[]
    def advance(seconds):
        sleeps.append(seconds)
        if len(sleeps)==3:
            with path.open('a') as handle:handle.write(json.dumps(dict(time_s=12.))+'\n')
    monkeypatch.setattr(node.time,'sleep',advance)
    out=node.audit_when_archived('mission',navigation,'out','platform','camera',.1,12.)
    assert out=='report' and len(calls)==1
    assert len(sleeps)==3,'it should stop waiting as soon as the record lands'


def test_the_audit_still_runs_if_navigation_never_catches_up(tmp_path, monkeypatch):
    import agv_mission.operator_node as node
    navigation=tmp_path/'nav';navigation.mkdir()
    (navigation/'navigation.jsonl').write_text(json.dumps(dict(time_s=1.))+'\n')
    monkeypatch.setattr(node,'audit_capture',lambda *a:'report')
    monkeypatch.setattr(node.time,'sleep',lambda seconds:None)
    # A missing tail must not strand the operator with a job that never returns;
    # the audit reports whatever evidence exists and its own issues say so.
    assert node.audit_when_archived('m',navigation,'o','p','c',.1,99.,timeout=.2)=='report'


def test_the_scan_gates_sit_above_what_the_tracker_may_command():
    # A gate below the controller's authority faults the mission for doing its
    # job. On 2026-09-14 the lateral pair was 0.10 against 0.12 and a rated
    # full-area run tripped it at 0.10002 m/s. max_position_feedback_m_s caps the
    # norm of the two-dimensional feedback, so it bounds the lateral command too.
    from agv_mission.camera_limits import inspection_camera_config
    p=Path(__file__).resolve().parents[2]/'agv_description/config'
    camera=yaml.safe_load((p/'linescan.yaml').read_text());platform=yaml.safe_load((p/'platform.yaml').read_text())
    tracking=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/tracking.yaml').read_text())
    result=inspection_camera_config(camera,platform)
    # The controller caps its feedback in the track frame and rotates the whole
    # command, feedforward included, into the body frame. A heading error psi
    # therefore leaks rated*sin(psi) into the axis the sensor gates, and at the
    # rated speed that term is larger than the entire feedback authority. An
    # assertion against the feedback cap alone -- which is what this test used to
    # do -- passes while a legal command of 0.209 m/s trips the gate.
    budget=(platform['rated_scan_speed']*math.sin(tracking['max_capture_heading_error_rad'])
            +tracking['max_position_feedback_m_s'])
    assert budget<=result['max_scan_lateral_m_s'],(budget,result['max_scan_lateral_m_s'])
    assert result['max_scan_lateral_m_s']>=1.05*budget,'keep margin over the legal envelope'
    # Yaw has no feedforward to project: an in-place rotation is its own segment
    # and never scans, so ordering the two rates is enough there.
    assert tracking['max_heading_feedback_rad_s']<result['max_yaw_rate_rad_s'],(
        tracking['max_heading_feedback_rad_s'],result['max_yaw_rate_rad_s'])
    assert result['max_yaw_rate_rad_s']>=1.2*tracking['max_heading_feedback_rad_s']
