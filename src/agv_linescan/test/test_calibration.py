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
