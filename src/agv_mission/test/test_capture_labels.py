import importlib.util
import json
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('label_mission_capture',Path(__file__).resolve().parents[3]/'tools/label_mission_capture.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def write(path,value):path.write_text(json.dumps(value))


def fixture(tmp_path):
    mission=tmp_path/'mission';nav=tmp_path/'nav';raw=tmp_path/'raw'
    for p in (mission,nav,raw):p.mkdir()
    write(mission/'capture_intervals.json',[{'track_id':1,'enabled_ack_time_s':0.,'disabled_ack_time_s':.1,'archive':str(raw)}])
    write(mission/'plan.json',{'mission_id':'test'})
    write(nav/'calibration.json',{'calibration_id':'cal','estimated_frames':[{'frame_id':'camera_optical_calibrated','translation_m':[1,0,0],'orientation_xyzw':[0,0,0,1]}]})
    values=[dict(time_s=t,position_m=[t,0,1],orientation_xyzw=[0,0,0,1],frame_id='map',child_frame_id='base_link',calibration_id='cal') for t in (0.,.02,.04)]
    (nav/'navigation.jsonl').write_text('\n'.join(json.dumps(v) for v in values))
    tags=[dict(time_s=t,global_line=i,camera_position_world_m=[999,999,999]) for i,t in enumerate((.01,.02,.03))]
    write(raw/'block_000000.json',{'block_id':0,'first':tags[0],'last':tags[-1],'pose_tags':tags,'rows':3,'width':4096,'segment_id':2})
    return mission,nav


def test_sparse_labels_use_navigation_and_calibration_never_render_truth(tmp_path):
    mission,nav=fixture(tmp_path);r=module.label(mission,nav);block=r['blocks'][0]
    assert block['track_id']==1 and block['reference']=='last_line_exposure_midpoint'
    assert block['pose_tags'][-1]['camera_position_map_m']==pytest.approx([1.03,0,1])
    assert block['last_global_line']==2


def test_labels_reject_pose_extrapolation(tmp_path):
    mission,nav=fixture(tmp_path)
    rows=(nav/'navigation.jsonl').read_text().splitlines();(nav/'navigation.jsonl').write_text('\n'.join(rows[:2]))
    with pytest.raises(ValueError,match='bracket'):module.label(mission,nav)
