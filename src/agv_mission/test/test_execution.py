from pathlib import Path
import numpy as np
import pytest
import yaml
from agv_mission.planner import plan,Vehicle,PlanningError
from agv_mission.execution import compile_steps,segment_arguments,check_position


def fixture():
    root=Path(__file__).resolve().parents[1]
    req=yaml.safe_load((root/'config/rectangle_demo.yaml').read_text());req['road']['frame_id']='map'
    v=Vehicle(1.5,1.15,.65,.8,1.,2.778)
    return plan(req,v)


def test_pass_merges_acceleration_scan_runout_without_intermediate_stops():
    p=fixture();steps=compile_steps(p,[3,0,.65],[0,0,0,1])
    passes=[s for s in steps if s['kind']=='PASS']
    assert len(passes)==4
    assert [s['kind'] for s in steps]==['APPROACH','PASS','SHIFT','ROTATE_180','ENTRY','PASS','SHIFT','ROTATE_180','ENTRY','PASS','SHIFT','ROTATE_180','ENTRY','PASS']
    for s in passes:
        distance=np.linalg.norm(np.array(s['end']['position'])-s['start']['position'])
        assert distance==pytest.approx(8+p['lead_distance_m']+p['runout_distance_m'])


def test_no_implicit_world_to_map_or_unsafe_approach():
    p=fixture()
    with pytest.raises(PlanningError):compile_steps(p,[.5,0,.65],[0,0,0,1])
    p['frame_id']='world'
    with pytest.raises(PlanningError):compile_steps(p,[3,0,.65],[0,0,0,1])


def test_rotates_to_planned_heading_before_translation():
    p=fixture();step=compile_steps(p,[3,0,.65],[0,0,0,1])[1]
    args=segment_arguments(step,[3,0,.65],[0,0,1,0])
    assert args['kind']=='rotate' and abs(args['angle'])==pytest.approx(np.pi)
    args=segment_arguments(step,[3,0,.65],[0,0,0,1])
    assert args['kind']=='translate'
