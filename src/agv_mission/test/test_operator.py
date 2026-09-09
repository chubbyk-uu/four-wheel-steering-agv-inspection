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
