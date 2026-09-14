from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agv_linescan.calibration import flat_field, geometry, make_profile, Correction


def fixture_profile():
    width = 64
    dark = np.full((256, width), 4, np.uint8)
    flat = np.tile(np.arange(width, dtype=np.uint8)+100, (256, 1))
    f = flat_field(dark, flat)
    x = np.linspace(0, 63, 9)
    g = geometry(x, (x-31.5)/64, width, 1.)
    return make_profile(f, g, dict(exposure_s=.00002, source_calibration_id='source'), {})


def test_flat_strip_projection_analytic_reverse_and_partition():
    from scipy.spatial.transform import Rotation
    from agv_linescan.strip_projection import project_flat
    p=np.column_stack([np.linspace(0,1,9),np.zeros(9),np.ones(9)])
    body=Rotation.from_euler('z',np.zeros(9));mount=Rotation.from_euler('x',np.pi)
    rays=np.array([[-.5,0,1],[0,0,1],[.5,0,1]])
    actual=project_flat(p,body,[0,0,0],mount,rays)
    np.testing.assert_allclose(actual[:,:,0],p[:,0,None]+rays[None,:,0],atol=1e-12)
    np.testing.assert_allclose(actual[:,:,1],0,atol=1e-12)
    split=np.concatenate([project_flat(p[:4],body[:4],[0,0,0],mount,rays),project_flat(p[4:],body[4:],[0,0,0],mount,rays)])
    np.testing.assert_array_equal(actual,split)
    for yaw,sign in ((0,1),(np.pi,-1)):
        r=Rotation.from_euler('z',np.full(9,yaw))
        before=project_flat(p,r,[0,0,0],mount,rays)
        after=project_flat(p,r,[.003,0,0],mount,rays)
        np.testing.assert_allclose(after-before,np.broadcast_to([sign*.003,0],after.shape),atol=1e-12)


def test_row_times_keep_pause_anchors_and_reject_bad_order():
    from agv_linescan.strip_projection import line_times
    tags=[dict(global_line=n,time_s=t) for n,t in ((10,1.),(12,1.2),(13,5.),(15,5.2))]
    m=dict(first=tags[0],last=tags[-1],pose_tags=tags,rows=6)
    np.testing.assert_allclose(line_times(m),[1,1.1,1.2,5,5.1,5.2])
    tags[2]['time_s']=.5
    with pytest.raises(ValueError):line_times(m)


def test_flat_rejects_saturated_and_insufficient_reference():
    with pytest.raises(ValueError):
        flat_field(np.zeros((256, 64), np.uint8), np.full((256, 64), 255, np.uint8))
    with pytest.raises(ValueError):
        flat_field(np.zeros((10, 64), np.uint8), np.full((256, 64), 100, np.uint8))


def test_measured_geometry_independent_polynomial_and_no_extrapolation():
    x = np.linspace(10, 990, 21)
    q = (x-499.5)/500
    g = geometry(x, .6*(q+.03*q**3), 1000, 1.2)
    assert g['metric_polynomial'] == pytest.approx([0, .6, 0, .018], abs=1e-12)
    assert g['fit_error_px_max'] < 1e-8
    narrow = geometry(x[3:-3], .6*(q[3:-3]+.03*q[3:-3]**3), 1000, 1.2)
    assert not narrow['valid'][0] and not narrow['valid'][-1]
    with pytest.raises(ValueError):
        geometry(x, -q, 1000, 1.2)


def test_correction_partition_and_saturation_mask():
    c = Correction(fixture_profile())
    raw = np.tile(np.arange(64, dtype=np.uint8)+100, (270, 1))
    result, quality = c.apply(raw)
    assert np.ptp(result[:, c.valid]) <= 1
    assert np.array_equal(result, np.concatenate([c.apply(raw[:131])[0], c.apply(raw[131:])[0]]))
    raw[1, 20] = 255
    result, quality = c.apply(raw)
    assert result[1, 20] == 0 and quality['raw_saturated_pixels'] == 1
    with pytest.raises(ValueError):
        c.apply(raw.astype(np.float32))


def test_metadata_rejects_other_exposure_preserves_last_line():
    c = Correction(fixture_profile())
    m = dict(width=64, rows=270, reference='last_line_exposure_midpoint', exposure_s=.00002,
             calibration_id='source', first={'global_line': 0}, last={'global_line': 269},
             pose_tags=[{'global_line': 0}, {'global_line': 269}])
    result = c.metadata(m)
    assert all(result[k] == m[k] for k in ('rows', 'first', 'last', 'pose_tags'))
    with pytest.raises(ValueError):
        c.metadata(dict(m, exposure_s=.00001))


def test_changed_led_invalidates_flat_profile():
    import yaml
    from agv_linescan.calibration import capture_signature
    cfg = yaml.safe_load((Path(__file__).resolve().parents[2]/'agv_description/config/linescan.yaml').read_text())
    profile = fixture_profile()
    profile['conditions']['capture_signature'] = capture_signature(cfg)
    correction = Correction(profile)
    correction.check_capture(cfg)
    cfg['radiometry']['led_peak_relative'] *= .5
    with pytest.raises(ValueError):
        correction.check_capture(cfg)


def test_same_optics_other_backend_or_robot_rejected():
    profile=fixture_profile();profile['conditions'].update(scene_backend='optix',robot_source_sha256='mount-a')
    correction=Correction(profile)
    meta=dict(width=64,reference='last_line_exposure_midpoint',exposure_s=.00002,calibration_id='source',
              scene_backend='optix',robot_contract={'source_sha256':'mount-a'})
    correction.metadata(meta)
    with pytest.raises(ValueError,match='backend'):correction.metadata(dict(meta,scene_backend='cuda'))
    with pytest.raises(ValueError,match='geometry'):correction.metadata(dict(meta,robot_contract={'source_sha256':'mount-b'}))


def test_measured_cli_json_scientific_exposure(tmp_path):
    import json
    import subprocess
    import yaml
    from PIL import Image
    from agv_linescan.calibration import capture_signature
    root = Path(__file__).resolve().parents[3]
    config = yaml.safe_load((root/'src/agv_description/config/linescan.yaml').read_text())
    config['width'] = 64
    profile = fixture_profile()
    profile['conditions']['capture_signature'] = capture_signature(config)
    (tmp_path/'profile.json').write_text(json.dumps(profile))
    (tmp_path/'camera.yaml').write_text(yaml.safe_dump(config))
    raw = np.full((8, 64), 120, np.uint8)
    Image.fromarray(raw).save(tmp_path/'raw.pgm')
    meta = dict(width=64, rows=8, reference='last_line_exposure_midpoint', exposure_s=2e-5,
                calibration_id='source', last={'global_line': 7}, pose_tags=[{'global_line': 7}])
    (tmp_path/'raw.json').write_text(json.dumps(meta))
    result = subprocess.run([sys.executable, str(root/'tools/rectify_linescan.py'),
        '--calibration', str(tmp_path/'profile.json'), '--capture-config', str(tmp_path/'camera.yaml'),
        '--input', str(tmp_path/'raw.pgm'), '--output', str(tmp_path/'fixed.pgm')], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    with Image.open(tmp_path/'fixed.pgm') as im:
        assert np.array_equal(np.array(im), Correction(profile).apply(raw)[0])
    assert json.loads((tmp_path/'fixed.json').read_text())['last'] == meta['last']


@pytest.mark.parametrize('metadata_first',[False,True])
def test_live_pairing_and_duplicate_failure(tmp_path,monkeypatch,metadata_first):
    import json
    import yaml
    import rclpy
    from PIL import Image as PILImage
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from agv_linescan.correction_node import CorrectionNode
    from agv_linescan.calibration import capture_signature
    monkeypatch.setenv('ROS_DOMAIN_ID','97')
    config=yaml.safe_load((Path(__file__).resolve().parents[2]/'agv_description/config/linescan.yaml').read_text())
    config['width']=64;profile=fixture_profile();profile['conditions']['capture_signature']=capture_signature(config)
    (tmp_path/'profile.json').write_text(json.dumps(profile));(tmp_path/'config.yaml').write_text(yaml.safe_dump(config))
    rclpy.init(args=['--ros-args','-p',f'profile:={tmp_path}/profile.json','-p',f'capture_config:={tmp_path}/config.yaml','-p',f'output_dir:={tmp_path}/out'])
    node=CorrectionNode()
    try:
        im=Image();im.width=64;im.height=8;im.step=64;im.encoding='mono8';im.header.stamp.sec=2
        raw=np.full((8,64),120,np.uint8);im.data=raw.tobytes()
        meta=dict(width=64,rows=8,reference='last_line_exposure_midpoint',exposure_s=.00002,
                  calibration_id='source',block_id=0,segment_id=0,first={'global_line':0},
                  last={'global_line':7,'time_s':2.},pose_tags=[{'global_line':7}])
        import copy
        future=copy.deepcopy(im);future.header.stamp.sec=3
        future_meta=dict(meta,block_id=1,first={'global_line':8},last={'global_line':15,'time_s':3.})
        node.receive('image',future);node.receive('metadata',String(data=json.dumps(future_meta)))
        node.process();assert node.next_block==0 and not node.error
        order=[('image',im),('metadata',String(data=json.dumps(meta)))]
        if metadata_first:order.reverse()
        node.receive(*order[0]);node.process();assert node.next_block==0
        node.receive(*order[1]);node.process();assert node.next_block==1 and not node.error
        actual=np.array(PILImage.open(tmp_path/'out/block_000000.pgm'))
        assert np.array_equal(actual,Correction(profile).apply(raw)[0])
        node.process();assert node.next_block==2 and not node.error
        node.receive('image',im);assert 'duplicate' in node.error
    finally:node.destroy_node();rclpy.shutdown()


def offline_fixture(tmp_path):
    import json,yaml
    from PIL import Image
    from agv_linescan.calibration import capture_signature
    source=tmp_path/'raw';source.mkdir()
    config=yaml.safe_load((Path(__file__).resolve().parents[2]/'agv_description/config/linescan.yaml').read_text())
    config.update(width=64,block_rows=17,calibration_id='source')
    (source/'calibration.yaml').write_text(yaml.safe_dump(config))
    profile=fixture_profile();profile['conditions']['capture_signature']=capture_signature(config)
    path=tmp_path/'profile.json';path.write_text(json.dumps(profile))
    for block,first,rows in ((0,0,17),(1,17,3)):
        m=dict(block_id=block,segment_id=0,width=64,rows=rows,encoding='mono8',reference='last_line_exposure_midpoint',
            exposure_s=.00002,calibration_id='source',first=dict(global_line=first,time_s=first*.001),
            last=dict(global_line=first+rows-1,time_s=(first+rows-1)*.001),pose_tags=[dict(global_line=first),dict(global_line=first+rows-1)])
        (source/f'block_{block:06d}.json').write_text(json.dumps(m))
        Image.fromarray(np.tile(np.arange(64,dtype=np.uint8)+100,(rows,1))).save(source/f'block_{block:06d}.pgm')
    return source,path


def test_offline_session_configurable_rows_short_tail_and_source_preservation(tmp_path):
    import json
    from PIL import Image
    from agv_linescan.offline_correction import process_session
    source,profile=offline_fixture(tmp_path);before={p.name:p.read_bytes() for p in source.iterdir()}
    out=tmp_path/'corrected';r=process_session(source,profile,out)
    assert r['rows']==20 and r['blocks']==2 and r['configured_block_rows']==17 and r['tail_block_rows']==3
    assert r['rows_overlap_added']==0
    c=Correction(json.loads(profile.read_text()))
    for block in range(2):
        name=f'block_{block:06d}';raw=np.array(Image.open(source/(name+'.pgm')))
        assert np.array_equal(np.array(Image.open(out/(name+'.pgm'))),c.apply(raw)[0])
        a=json.loads((source/(name+'.json')).read_text());b=json.loads((out/(name+'.json')).read_text())
        assert all(a[k]==b[k] for k in ('first','last','pose_tags','rows','reference'))
    assert before=={p.name:p.read_bytes() for p in source.iterdir()}
    with pytest.raises(FileExistsError):process_session(source,profile,out)


@pytest.mark.parametrize('fault',['missing_image','missing_block','line_gap'])
def test_offline_rejects_incomplete_or_discontinuous_session(tmp_path,fault):
    import json
    from agv_linescan.offline_correction import process_session
    source,profile=offline_fixture(tmp_path)
    if fault=='missing_image':(source/'block_000001.pgm').unlink()
    elif fault=='missing_block':
        (source/'block_000000.pgm').unlink();(source/'block_000000.json').unlink()
    else:
        path=source/'block_000001.json';m=json.loads(path.read_text())
        m['first']['global_line']+=1;m['last']['global_line']+=1;path.write_text(json.dumps(m))
    with pytest.raises(ValueError):process_session(source,profile,tmp_path/'corrected')
    assert not (tmp_path/'corrected').exists()


def discarded_tail_fixture(tmp_path,rows=3,minimum=1000,block_id=1):
    """A session shaped like the real one: the short tail spent id 1 and was dropped.

    Taken from the 0.5 m/s full-area run, where block 66 (segment 2) is followed by
    block 68 (segment 4) and the sensor logged id 67 as a 806-row tail against a
    1000-row minimum. The kept block after the gap carries a new segment id, which
    is why the intra-segment line-continuity rule does not apply across it.
    """
    import json
    source,profile=offline_fixture(tmp_path)
    old=source/'block_000001.json';m=json.loads(old.read_text())
    m['block_id']=2;m['segment_id']=2
    for key in ('first','last'):
        m[key]['global_line']+=rows;m[key]['time_s']+=rows*.001
    for tag in m['pose_tags']:tag['global_line']+=rows
    (source/'block_000002.json').write_text(json.dumps(m));old.unlink()
    (source/'block_000001.pgm').rename(source/'block_000002.pgm')
    (source/'events.jsonl').write_text(json.dumps(dict(reason='tail_discarded',end_reason='capture_toggle',
        rows=rows,minimum_rows=minimum,block_id=block_id,segment_id=0,simulation_time_s=(16+rows)*.001,
        first=dict(global_line=17,time_s=.017),last=dict(global_line=16+rows,time_s=(16+rows)*.001)))+'\n')
    return source,profile


def test_offline_accepts_block_ids_spent_by_a_discarded_short_tail(tmp_path):
    # Discarding a tail under the row threshold is policy, not data loss, and it
    # consumes a block id. Before this, a legal full-area capture could not be
    # corrected at all: the 0.5 m/s run had nine such gaps, the first 66 -> 68.
    from agv_linescan.offline_correction import process_session
    source,profile=discarded_tail_fixture(tmp_path)
    r=process_session(source,profile,tmp_path/'corrected')
    assert r['blocks']==2 and r['rows']==20
    assert r['block_id_gaps']==[1]
    assert [d['block_id'] for d in r['discarded_tail_blocks']]==[1]
    assert r['discarded_tail_blocks'][0]['rows']==3
    import json
    assert r['discarded_tail_blocks']==[json.loads((source/'events.jsonl').read_text())]


@pytest.mark.parametrize('fault',['no_evidence','tail_not_short','discarded_block_present'])
def test_offline_still_rejects_gaps_without_honest_evidence(tmp_path,fault):
    import json
    from agv_linescan.offline_correction import process_session
    if fault=='no_evidence':
        source,profile=discarded_tail_fixture(tmp_path);(source/'events.jsonl').unlink()
    elif fault=='tail_not_short':
        # A block as long as the threshold was never a short tail, so it cannot
        # excuse the gap; otherwise any lost block could be waved through.
        source,profile=discarded_tail_fixture(tmp_path,rows=1000)
    else:
        # Claiming a block was discarded while its file is present is a contradiction.
        source,profile=discarded_tail_fixture(tmp_path,block_id=2)
    with pytest.raises(ValueError):process_session(source,profile,tmp_path/'corrected')
    assert not (tmp_path/'corrected').exists()


@pytest.mark.parametrize('fault',['missing_range','wrong_rows','overlap','line_gap',
                                 'time_overlap','nan_time','reversed_segment','terminal_id_gap'])
def test_discard_evidence_range_and_neighbors_are_checked(tmp_path,fault):
    import json
    from agv_linescan.offline_correction import process_session
    source,profile=discarded_tail_fixture(tmp_path)
    path=source/'events.jsonl';event=json.loads(path.read_text())
    if fault=='missing_range':del event['first']
    elif fault=='wrong_rows':event['last']['global_line']=9999
    elif fault in ('overlap','line_gap'):
        shift=-1 if fault=='overlap' else 1
        for key in ('first','last'):event[key]['global_line']+=shift
    elif fault=='time_overlap':event['first']['time_s']=.016
    elif fault=='nan_time':event['last']['time_s']=float('nan')
    elif fault=='reversed_segment':event['segment_id']=3
    else:
        extra=dict(event,block_id=4)
        path.write_text(json.dumps(event)+'\n'+json.dumps(extra)+'\n')
    if fault!='terminal_id_gap':path.write_text(json.dumps(event)+'\n')
    with pytest.raises(ValueError):process_session(source,profile,tmp_path/'corrected')
    assert not (tmp_path/'corrected').exists()


def test_final_discard_keeps_full_range_and_checks_previous_image(tmp_path):
    import json
    from agv_linescan.offline_correction import process_session
    source,profile=discarded_tail_fixture(tmp_path)
    event=json.loads((source/'events.jsonl').read_text())
    final=dict(event,block_id=3,segment_id=2,first=dict(global_line=23,time_s=.023),
               last=dict(global_line=25,time_s=.025),simulation_time_s=.025)
    (source/'events.jsonl').write_text(json.dumps(event)+'\n'+json.dumps(final)+'\n')
    out=tmp_path/'corrected';process_session(source,profile,out)
    report=json.loads((out/'summary.json').read_text())
    assert report['discarded_tail_blocks']==[event,final]
    assert report['block_id_gaps']==[1]  # Final discard is retained even without a following image.
    final['first']['global_line']+=1;final['last']['global_line']+=1
    (source/'events.jsonl').write_text(json.dumps(event)+'\n'+json.dumps(final)+'\n')
    with pytest.raises(ValueError):process_session(source,profile,tmp_path/'bad_final')
    assert not (tmp_path/'bad_final').exists()


def test_offline_preserves_extra_sparse_tags_across_pause(tmp_path):
    import json
    from agv_linescan.offline_correction import process_session
    source,profile=offline_fixture(tmp_path)
    path=source/'block_000000.json';m=json.loads(path.read_text())
    m['pose_tags']=[dict(global_line=i,time_s=i*.001+(3 if i>=8 else 0)) for i in (0,4,7,8,9,12,16)]
    m['last']['time_s']+=3;path.write_text(json.dumps(m))
    next_path=source/'block_000001.json';n=json.loads(next_path.read_text())
    n['first']['time_s']+=3;n['last']['time_s']+=3;next_path.write_text(json.dumps(n))
    process_session(source,profile,tmp_path/'corrected')
    corrected=json.loads((tmp_path/'corrected'/path.name).read_text())
    assert corrected['pose_tags']==m['pose_tags'] and corrected['rows']==17
