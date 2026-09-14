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


@pytest.mark.parametrize('yaw',[0.,3.141592653589793])
def test_fixed_camera_residual_rotates_with_vehicle_and_ignores_truth(tmp_path,yaw):
    import numpy as np
    from scipy.spatial.transform import Rotation
    mission,nav=fixture(tmp_path)
    path=nav/'navigation.jsonl';rows=[json.loads(v) for v in path.read_text().splitlines()]
    for row in rows:row['orientation_xyzw']=Rotation.from_euler('z',yaw).as_quat().tolist()
    path.write_text('\n'.join(json.dumps(v) for v in rows))
    before=module.label(mission,nav)['blocks'][0]['pose_tags']
    calpath=nav/'calibration.json';cal=json.loads(calpath.read_text())
    cal['estimated_frames'][0]['translation_m'][0]+=.003;write(calpath,cal)
    after=module.label(mission,nav)['blocks'][0]['pose_tags']
    for a,b in zip(before,after):
        np.testing.assert_allclose(np.array(b['camera_position_map_m'])-a['camera_position_map_m'],
                                   [.003 if yaw==0 else -.003,0,0],atol=1e-12)
    raw=tmp_path/'raw/block_000000.json';metadata=json.loads(raw.read_text())
    for tag in metadata['pose_tags']:tag['camera_position_world_m']=[-12345,67890,-999]
    write(raw,metadata)
    assert module.label(mission,nav)['blocks'][0]['pose_tags']==after


def test_cross_pause_tags_use_actual_exposure_times_not_uniform_block_time(tmp_path):
    mission,nav=fixture(tmp_path)
    intervals=json.loads((mission/'capture_intervals.json').read_text());intervals[0]['disabled_ack_time_s']=3.1
    write(mission/'capture_intervals.json',intervals)
    path=Path(intervals[0]['archive'])/'block_000000.json';m=json.loads(path.read_text())
    m['last']['time_s']=3.03;m['pose_tags'][-1]['time_s']=3.03;write(path,m)
    values=[json.loads(v) for v in (nav/'navigation.jsonl').read_text().splitlines()]
    for t in (3.02,3.04):
        values.append(dict(values[-1],time_s=t,position_m=[t-3,0,1]))
    (nav/'navigation.jsonl').write_text('\n'.join(json.dumps(v) for v in values))
    b=module.label(mission,nav)['blocks'][0]
    assert [t['time_s'] for t in b['pose_tags']]==[.01,.02,3.03]
    assert b['pose_tags'][-1]['camera_position_map_m']==pytest.approx([1.03,0,1])
