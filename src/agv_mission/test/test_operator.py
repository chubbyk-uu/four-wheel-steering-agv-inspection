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
