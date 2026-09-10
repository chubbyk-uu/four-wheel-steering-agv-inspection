from copy import deepcopy
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
