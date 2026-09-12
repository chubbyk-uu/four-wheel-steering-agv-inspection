from pathlib import Path
import numpy as np
import pytest
import yaml
from agv_mission.planner import plan,Vehicle,PlanningError
from agv_mission.execution import compile_steps,segment_arguments,check_position


def fixture():
    root=Path(__file__).resolve().parents[1]
    req=yaml.safe_load((root/'config/rectangle_demo.yaml').read_text());req['road']['frame_id']='map'
    req['coverage_error_m']=.1
    # The shipped sensor discards a closing image below 1000 lines; the capture
    # window has to reach past the region by at least that, so say so here.
    v=Vehicle(1.5,1.15,.65,.8,1.,max_speed=15/3.6,rated_scan_speed=10/3.6,
              discardable_tail_m=1000*.0003662109375)
    return plan(req,v)


def test_pass_merges_acceleration_scan_runout_without_intermediate_stops():
    p=fixture();steps=compile_steps(p,[3,0,.65],[0,0,0,1])
    passes=[s for s in steps if s['kind']=='PASS']
    assert len(passes)==4
    assert [s['kind'] for s in steps]==['APPROACH','PASS','SHIFT','ROTATE_180','PASS','SHIFT','ROTATE_180','PASS','SHIFT','ROTATE_180','PASS']
    for s in passes:
        distance=np.linalg.norm(np.array(s['end']['position'])-s['start']['position'])
        assert distance==pytest.approx(8+p['lead_distance_m']+(p['turn_runout_distance_m'] if s['track_id']<3 else p['runout_distance_m']))


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


def test_capture_stays_closed_when_resuming_in_runout_in_both_directions():
    from agv_mission.execution import scan_end_reached
    p=fixture();steps=compile_steps(p,[3,0,.65],[0,0,0,1])
    import numpy as np
    from scipy.spatial.transform import Rotation
    camera={'camera_x_m':1.15}
    overrun=p['scan_overrun_distance_m']
    assert overrun>0
    for step in (s for s in steps if s['kind']=='PASS'):
        assert not scan_end_reached(p,camera,step,step['start']['position'],step['start']['orientation_xyzw'])
        assert scan_end_reached(p,camera,step,step['end']['position'],step['end']['orientation_xyzw'])
        # Capture must stay open the whole planned over-run past the region end,
        # or the sensor's discardable closing image comes out of the region.
        track=p['tracks'][step['track_id']]
        start=np.array(track['scan_start_xyz_m']);finish=np.array(track['scan_end_xyz_m'])
        axis=(finish-start)/np.linalg.norm(finish-start)
        quaternion=step['end']['orientation_xyzw']
        offset=Rotation.from_quat(quaternion).apply([camera['camera_x_m'],0.,0.])
        for fraction,expected in ((.99,False),(1.01,True)):
            base=finish+axis*overrun*fraction-offset
            assert bool(scan_end_reached(p,camera,step,base,quaternion)) is expected
