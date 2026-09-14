import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from PIL import Image
from scipy.spatial.transform import Rotation
from agv_mission.coverage import estimate,rescan_requests,merge,complement
from agv_mission.capture_audit import audit_capture
from agv_mission.planner import plan,Vehicle


def config():
    root=Path(__file__).resolve().parents[2]
    platform=yaml.safe_load((root/'agv_description/config/platform.yaml').read_text())
    camera=yaml.safe_load((root/'agv_description/config/linescan.yaml').read_text())
    request=yaml.safe_load((root/'agv_mission/config/rectangle_demo.yaml').read_text())
    request['road']['frame_id']='map'
    request['region']=dict(start_xy_m=[6.,-.5],length_m=1.,width_m=1.)
    return platform,camera,request


def test_gaps_union_and_reverse_direction_rescan_geometry():
    platform,camera,request=config();vehicle=Vehicle.from_configs(platform,camera)
    request['region'].update(length_m=3.,width_m=2.,start_xy_m=[6.,-1.]);source=plan(request,vehicle)
    points=[dict(road_x=[9.2,9.2],road_y=[-.25,1.25]),dict(road_x=[8.,8.],road_y=[-.25,1.25])]
    audit=estimate(source,[dict(track_id=1,points=points)],.1)
    assert audit['tracks'][0]['unverified_along_m']==[[0.,3.]]
    assert audit['tracks'][1]['estimated_covered_along_m'][0]==pytest.approx([0.,.9])
    candidates=rescan_requests(source,audit,vehicle)
    assert all(c['status']=='PREVIEW_ONLY' for c in candidates)
    reverse=next(c for c in candidates if c['source_track_id']==1)
    assert reverse['request']['region']['start_xy_m'][0]==pytest.approx(6.)
    assert reverse['request']['region']['length_m']==pytest.approx(2.2)
    assert merge([[0,1],[.5,2]])==[[0,2]]
    assert complement([[0,1],[2,3]],3)==[[1,2]]


def test_lateral_uncertainty_prevents_false_complete():
    p,c,r=config();source=plan(r,Vehicle.from_configs(p,c))
    points=[dict(road_x=[5.8,5.8],road_y=[-.75,.75],lateral_sigma3_m=.3),dict(road_x=[7.2,7.2],road_y=[-.75,.75],lateral_sigma3_m=.3)]
    assert estimate(source,[dict(track_id=0,points=points)])['status']=='NEEDS_RESCAN'
    for point in points:point['lateral_sigma3_m']=.1
    assert estimate(source,[dict(track_id=0,points=points)])['status']=='ESTIMATED_COMPLETE'


def archive(tmp_path):
    platform,camera,request=config();source=plan(request,Vehicle.from_configs(platform,camera))
    mission=tmp_path/'mission';nav=tmp_path/'nav';raw=tmp_path/'raw'
    for p in (mission,nav,raw):p.mkdir()
    (mission/'plan.json').write_text(json.dumps(source))
    (mission/'execution.jsonl').write_text('\n'.join(json.dumps(dict(time_s=float(t),kind='PASS',capture_active=True,motion_state='DRIVE',motion_reason='NONE',track_id=0,control_dt_s=.02)) for t in np.arange(0,4.001,.02))+'\n')
    (mission/'capture_intervals.json').write_text(json.dumps([dict(track_id=0,archive=str(raw),enabled_ack_time_s=0.,disabled_ack_time_s=4.)]))
    height=camera['nominal_width_m']*camera['focal_length_m']/(camera['width']*camera['pixel_pitch_m'])
    cal=dict(calibration_id='nav',optical_intrinsic_id=camera['calibration_id'],estimated_frames=[dict(frame_id='camera_optical_calibrated',translation_m=[camera['camera_x_m'],0,height-.65],orientation_xyzw=Rotation.from_euler('xyz',[np.pi,0,np.pi/2]).as_quat().tolist())])
    (nav/'calibration.json').write_text(json.dumps(cal));rows=[]
    for t in np.arange(0,4.001,.02):
        rows.append(dict(time_s=float(t),position_m=[6-.25-camera['camera_x_m']+.5*(t-.1),0,.65],orientation_xyzw=[0,0,0,1],pose_covariance=[0.]*36,frame_id='map',child_frame_id='base_link',calibration_id='nav'))
    (nav/'navigation.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    tags=[dict(global_line=i,time_s=.1+i*camera['line_spacing_m']/.5,camera_position_world_m=[999,999,999]) for i in (0,1024,2048,3072,4095)]
    metadata=dict(block_id=0,segment_id=0,rows=4096,width=4096,calibration_id=camera['calibration_id'],first=tags[0],last=tags[-1],pose_tags=tags,line_spacing_m=camera['line_spacing_m'])
    (raw/'block_000000.json').write_text(json.dumps(metadata));(raw/'calibration.yaml').write_text(yaml.safe_dump(camera))
    Image.new('L',(4096,4096),100).save(raw/'block_000000.pgm')
    return mission,nav,raw,platform,camera


def test_real_archive_uses_fused_pose_and_preserves_source(tmp_path):
    mission,nav,raw,p,c=archive(tmp_path);before=(raw/'block_000000.pgm').read_bytes()
    report=audit_capture(mission,nav,tmp_path/'audit',p,c)
    assert report['status']=='ESTIMATED_COMPLETE' and not report['issues'] and not report['rescan_candidates']
    assert before==(raw/'block_000000.pgm').read_bytes()
    with pytest.raises(FileExistsError):audit_capture(mission,nav,tmp_path/'audit',p,c)


def test_steering_during_capture_keeps_pixels_but_requires_rescan(tmp_path):
    mission,nav,raw,p,c=archive(tmp_path);before=(raw/'block_000000.pgm').read_bytes()
    row=dict(time_s=1.,kind='PASS',capture_active=True,motion_state='ALIGN',motion_reason='LIMIT_RECONFIGURE',track_id=0,control_dt_s=.02)
    records=[json.loads(v) for v in (mission/'execution.jsonl').read_text().splitlines()]+[row]
    (mission/'execution.jsonl').write_text('\n'.join(json.dumps(v) for v in sorted(records,key=lambda r:r['time_s'])))
    report=audit_capture(mission,nav,tmp_path/'quality',p,c)
    assert report['status']=='NEEDS_RESCAN' and report['rescan_candidates']
    assert any('STEERING_DURING_CAPTURE' in issue['reason'] for issue in report['issues'])
    assert report['inputs'][0]['motion_quality_excluded'] and report['motion_quality_windows']
    assert before==(raw/'block_000000.pgm').read_bytes()


def test_ordinary_pause_and_non_capture_turn_do_not_reject_image(tmp_path):
    mission,nav,raw,p,c=archive(tmp_path)
    rows=[dict(time_s=1.,kind='PASS',capture_active=True,motion_state='HOLD',motion_reason='STOP_REQUEST',track_id=0,control_dt_s=.02),
          dict(time_s=2.,kind='PASS',capture_active=True,motion_state='ALIGN',motion_reason='STOP_REQUEST',track_id=0,control_dt_s=.02),
          dict(time_s=3.5,kind='ROTATE_180',capture_active=False,motion_state='ALIGN',motion_reason='LARGE_STEER_CHANGE',track_id=0,control_dt_s=.02)]
    records=[json.loads(v) for v in (mission/'execution.jsonl').read_text().splitlines()]+rows
    (mission/'execution.jsonl').write_text('\n'.join(json.dumps(v) for v in sorted(records,key=lambda r:r['time_s'])))
    report=audit_capture(mission,nav,tmp_path/'pause_quality',p,c)
    assert report['status']=='ESTIMATED_COMPLETE' and not report['motion_quality_windows']


def test_truncated_execution_trace_cannot_certify_motion_quality(tmp_path):
    mission,nav,raw,p,c=archive(tmp_path)
    lines=(mission/'execution.jsonl').read_text().splitlines()
    (mission/'execution.jsonl').write_text(lines[0]+'\n')
    report=audit_capture(mission,nav,tmp_path/'missing_motion',p,c)
    assert report['status']=='NEEDS_RESCAN' and report['inputs'][0]['motion_quality_excluded']
    assert any('EXECUTION_TRACE_INCOMPLETE' in x['reason'] for x in report['issues'])


def test_a_hole_inside_the_execution_trace_cannot_certify_motion_quality(tmp_path):
    # Keeping only the first and last record leaves the bounds intact, so the
    # old first/last check passed while seconds of vehicle behaviour went
    # unobserved. A steering reconfiguration inside that hole would have been
    # missed in silence and the block certified anyway.
    mission,nav,raw,p,c=archive(tmp_path)
    lines=(mission/'execution.jsonl').read_text().splitlines()
    (mission/'execution.jsonl').write_text(lines[0]+'\n'+lines[-1]+'\n')
    report=audit_capture(mission,nav,tmp_path/'trace_hole',p,c)
    assert report['status']=='NEEDS_RESCAN' and report['inputs'][0]['motion_quality_excluded']
    assert any('EXECUTION_TRACE_GAP' in x['reason'] for x in report['issues'])
    assert report['execution_trace_gaps'] and report['execution_trace_gaps'][0]['span_s']>3


def test_a_slow_control_step_is_evidence_not_a_hole(tmp_path):
    # Every record declares how long its own step took, so a slow tick still
    # proves what happened. accept_rated_5 contained exactly one 0.352 s step
    # whose record declared 0.352 s; a fixed threshold would have rejected that
    # run's blocks for being thorough rather than for missing anything.
    mission,nav,raw,p,c=archive(tmp_path)
    records=[json.loads(v) for v in (mission/'execution.jsonl').read_text().splitlines()]
    dropped=records.pop(40)
    records[40]['control_dt_s']=round(records[40]['time_s']-records[39]['time_s'],6)
    assert records[40]['control_dt_s']>.03 and dropped
    (mission/'execution.jsonl').write_text('\n'.join(json.dumps(v) for v in records)+'\n')
    report=audit_capture(mission,nav,tmp_path/'slow_step',p,c)
    assert report['status']=='ESTIMATED_COMPLETE' and not report['execution_trace_gaps']
    assert not report['inputs'][0]['motion_quality_excluded']


def test_missing_pixels_and_navigation_gap_do_not_bridge_coverage(tmp_path):
    mission,nav,raw,p,c=archive(tmp_path)
    rows=[json.loads(s) for s in (nav/'navigation.jsonl').read_text().splitlines()]
    (nav/'navigation.jsonl').write_text('\n'.join(json.dumps(r) for r in rows if not .8<r['time_s']<1.8))
    report=audit_capture(mission,nav,tmp_path/'gap',p,c)
    assert report['status']=='NEEDS_RESCAN' and report['issues'] and report['rescan_candidates']
    (raw/'block_000000.pgm').unlink()
    report=audit_capture(mission,nav,tmp_path/'missing',p,c)
    assert report['tracks'][0]['unverified_along_m']==[[0.,1.]]


def test_covariance_expands_the_footprint_allowance(tmp_path):
    mission,nav,raw,p,c=archive(tmp_path)
    rows=[json.loads(s) for s in (nav/'navigation.jsonl').read_text().splitlines()]
    for r in rows:r['pose_covariance'][7]=.04
    (nav/'navigation.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    report=audit_capture(mission,nav,tmp_path/'uncertain',p,c)
    assert report['status']=='NEEDS_RESCAN'


def test_unclosed_interval_is_unverified_and_ambiguous_intervals_fail(tmp_path):
    mission,nav,raw,p,c=archive(tmp_path)
    path=mission/'capture_intervals.json';intervals=json.loads(path.read_text())
    del intervals[0]['disabled_ack_time_s'];path.write_text(json.dumps(intervals))
    report=audit_capture(mission,nav,tmp_path/'unclosed',p,c)
    assert report['status']=='NEEDS_RESCAN' and not report['inputs']
    intervals[0]['disabled_ack_time_s']=4.;path.write_text(json.dumps(intervals*2))
    with pytest.raises(RuntimeError,match='ambiguous'):audit_capture(mission,nav,tmp_path/'ambiguous',p,c)


def test_uncertain_final_tag_preserves_a_valid_prefix():
    p,c,r=config();r['region']['length_m']=3.;source=plan(r,Vehicle.from_configs(p,c))
    points=[dict(road_x=[x,x],road_y=[-.75,.75]) for x in (5.8,7.,8.)]
    points[-1]['lateral_sigma3_m']=.4
    report=estimate(source,[dict(track_id=0,points=points)])
    assert report['tracks'][0]['estimated_covered_along_m'][0]==pytest.approx([0.,.9])
    assert report['tracks'][0]['unverified_along_m'][0]==pytest.approx([.9,3.])


def test_cross_session_union_and_identity_guards():
    from copy import deepcopy
    from agv_mission.coverage import combine
    p,c,r=config();v=Vehicle.from_configs(p,c);source=plan(r,v)
    parent=estimate(source,[]);parent.update(request=r,scene_contract_sha256=['scene'],optical_intrinsic_id='lens',provenance={'navigation_sha256':'a'})
    candidates=rescan_requests(source,parent,v);parent['rescan_candidates']=candidates
    child=deepcopy(parent);child['request']=candidates[0]['request'];child['provenance']={'navigation_sha256':'b'}
    child['tracks'][0]['estimated_covered_along_m']=[[0.,1.]]
    assert combine(parent,[child])['status']=='ESTIMATED_COMPLETE'
    child['scene_contract_sha256']=['other']
    with pytest.raises(ValueError,match='incompatible'):combine(parent,[child])
    child['scene_contract_sha256']=['scene'];child['provenance']={'navigation_sha256':'a'}
    with pytest.raises(ValueError,match='duplicate'):combine(parent,[child])


def test_incremental_rescans_keep_remaining_candidates_and_reject_replays():
    from copy import deepcopy
    from agv_mission.coverage import combine
    p,c,r=config();r['region'].update(start_xy_m=[6.,-1.],length_m=3.,width_m=2.)
    vehicle=Vehicle.from_configs(p,c);source=plan(r,vehicle)
    parent=estimate(source,[]);parent.update(request=r,scene_contract_sha256=['scene'],optical_intrinsic_id='lens',provenance={'navigation_sha256':'parent'})
    parent['rescan_candidates']=rescan_requests(source,parent,vehicle)
    children=[]
    for i,candidate in enumerate(parent['rescan_candidates']):
        request=candidate['request'];child=estimate(plan(request,vehicle),[])
        child.update(request=request,scene_contract_sha256=['scene'],optical_intrinsic_id='lens',provenance={'navigation_sha256':str(i)})
        child['tracks'][0]['estimated_covered_along_m']=[[0.,3.]];child['tracks'][0]['unverified_along_m']=[]
        children.append(child)
    first=combine(parent,children[:1]);assert len(first['rescan_candidates'])==1
    assert first['rescan_candidates'][0]['source_track_id']==1
    with pytest.raises(ValueError,match='duplicate'):combine(first,children[:1])
    final=combine(first,children[1:]);assert final['status']=='ESTIMATED_COMPLETE' and not final['rescan_candidates']
    assert len(final['merged_navigation_sha256'])==3


def test_no_saved_tail_keeps_scene_identity_without_claiming_pixels(tmp_path):
    import hashlib
    mission,nav,raw,p,c=archive(tmp_path)
    for path in raw.glob('block_*'):path.unlink()
    scene={'schema':'test.scene','identity':'unchanged'}
    (raw/'scene_manifest.json').write_text(json.dumps(scene))
    report=audit_capture(mission,nav,tmp_path/'empty',p,c)
    assert report['scene_contract_sha256']==[hashlib.sha256(json.dumps(scene,sort_keys=True,separators=(',',':')).encode()).hexdigest()]
    assert not report['inputs'] and report['status']=='NEEDS_RESCAN'


def test_navigation_tolerates_a_torn_last_line_but_not_a_corrupt_middle(tmp_path):
    # The readiness check the operator uses tolerates a half-written final line,
    # because the adapter appends from another process. If the parser does not
    # agree, the audit is refused for a race the caller was told was over.
    import json as _json
    from agv_mission.capture_audit import Navigation
    d=tmp_path/'nav';d.mkdir()
    (d/'calibration.json').write_text(_json.dumps(dict(calibration_id='c',optical_intrinsic_id='o',
        estimated_frames=[dict(frame_id='camera_optical_calibrated',translation_m=[0.,0.,0.],orientation_xyzw=[0.,0.,0.,1.])])))
    def row(t):
        return _json.dumps(dict(time_s=t,frame_id='map',child_frame_id='base_link',calibration_id='c',
            position_m=[t,0.,1.],orientation_xyzw=[0.,0.,0.,1.],pose_covariance=[0.]*36))
    (d/'navigation.jsonl').write_text(row(1.)+'\n'+row(2.)+'\n'+row(3.)+'\n'+'{"time_s": 4.')
    nav=Navigation(d)
    assert nav.torn_tail is True and len(nav.times)==3
    (d/'navigation.jsonl').write_text(row(1.)+'\n'+'{"time_s": 2.'+'\n'+row(3.)+'\n')
    try:
        Navigation(d);assert False,'a corrupt line that is not the last is not a race'
    except ValueError as exc:
        assert 'corrupt navigation record 1' in str(exc)
