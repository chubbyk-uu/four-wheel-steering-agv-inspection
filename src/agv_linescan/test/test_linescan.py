from pathlib import Path
import sys
import math
import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from agv_linescan.core import Camera, Trigger, Blocks, interpolate_pose


@pytest.fixture
def config():
    return yaml.safe_load((Path(__file__).resolve().parents[2]/'agv_description/config/linescan.yaml').read_text())


def test_nominal_optics(config):
    config['ray_polynomial'] = [0, 1]
    camera = Camera(config)
    origin, r = camera.optical_pose(([0, 0, config['base_nominal_height_m']], [0, 0, 0, 1]))
    assert origin[2] == pytest.approx(1.0463169642857143)
    assert np.linalg.det(r) == pytest.approx(1)
    assert config['focal_length_m'] == .020
    assert config['width'] == 4096
    assert config['nominal_width_m'] == 1.5
    assert config['line_spacing_m'] == pytest.approx(1.5/4096)
    assert camera.height*(camera.ray_x[-1]-camera.ray_x[0]) == pytest.approx(1.5-1.5/4096)


def test_grid_spacing_in_pixels(config):
    config['ray_polynomial'] = [0, 1]
    camera = Camera(config)
    pixels, valid = camera.grid_line(([config['grid_spacing_m']/2-config['camera_x_m'], 0, config['base_nominal_height_m']], [0, 0, 0, 1]))
    dark = np.flatnonzero(pixels == config['grid_dark'])
    groups = np.split(dark, np.flatnonzero(np.diff(dark) > 1)+1)
    pitch = config['nominal_width_m']/config['width']
    minimum_pixels = max(1, math.floor(config['grid_line_width_m']/pitch))
    centers = np.array([g.mean() for g in groups if len(g) >= minimum_pixels])
    # Check each rasterized center against its independent metric position;
    # differencing two rounded centers can accumulate almost one pixel.
    last = math.floor((config['nominal_width_m']-config['grid_line_width_m'])/2/config['grid_spacing_m'])
    expected = np.arange(-last, last+1)*config['grid_spacing_m']/pitch+(config['width']-1)/2
    assert len(centers) == len(expected)
    assert np.max(np.abs(centers-expected)) <= .5
    assert valid.all()


@pytest.mark.parametrize('direction', [1, -1])
def test_encoder_chunking_stop_and_resume(config, direction):
    tr = Trigger(config['line_spacing_m'])
    events = tr.update(0, 0)
    for t, d in [(1, .17), (2, .17), (3, .63), (4, 1.001)]:
        events += tr.update(t, direction*d)
    assert len(events) == math.floor(1.001/tr.spacing)
    assert np.allclose(np.diff([e[1] for e in events]), direction*tr.spacing)
    assert np.all(np.diff([e[0] for e in events]) > 0)
    with pytest.raises(ValueError):
        tr.update(5, 0)
    tr.reset()
    assert tr.update(6, 0) == []


def test_full_speed_trigger_count(config):
    tr = Trigger(config['line_spacing_m'])
    tr.update(0, 0)
    platform = yaml.safe_load((Path(__file__).resolve().parents[2]/'agv_description/config/platform.yaml').read_text())
    speed = platform['max_speed']
    assert speed == pytest.approx(10/3.6)
    events = tr.update(1, speed)
    assert len(events) == math.floor(speed/config['line_spacing_m'])
    assert np.diff([e[0] for e in events]).mean() == pytest.approx(config['line_spacing_m']/speed)


def test_rectification_against_independent_signal(config):
    camera = Camera(config)
    # Smooth physical signal sampled by raw rays; expected ideal coordinates
    # are independently derived from pixel pitch / height, not the lookup.
    world_y = camera.height*camera.ray_x
    raw = np.rint(128+90*np.sin(2*np.pi*world_y/.1)).astype(np.uint8)
    corrected, valid = camera.rectify(np.stack([raw, raw//2]))
    y = (np.arange(config['width'])-(config['width']-1)/2)*config['nominal_width_m']/config['width']
    expected = 128+90*np.sin(2*np.pi*y/.1)
    assert np.mean(np.abs(corrected[0].astype(float)-expected)) < .5
    assert np.mean(np.abs(raw.astype(float)-expected)) > 15
    assert np.all(valid)
    assert np.max(np.abs(corrected[1].astype(float)-corrected[0]/2)) <= 1


def test_invalid_distortion_and_uncaptured_edges(config):
    config['ray_polynomial'] = [0, 1, 0, -1]
    with pytest.raises(ValueError):
        Camera(config)
    config['ray_polynomial'] = [0, .9]
    camera = Camera(config)
    _, valid = camera.rectify(np.full((1, 4096), 200, dtype=np.uint8))
    assert not valid[0] and not valid[-1] and valid[2048]


def test_pose_slerp_and_plane_visibility(config):
    a = ([0, 0, config['base_nominal_height_m']], [0, 0, 0, 1])
    b = ([1, 0, config['base_nominal_height_m']], [0, 0, 1, 0])
    mid = interpolate_pose(a, b, .5)
    assert mid[0][0] == .5
    assert mid[1][2] == pytest.approx(math.sqrt(.5))
    camera = Camera(config)
    _, visible = camera.grid_line(a)
    assert visible.all()
    _, visible = camera.grid_line(([0, 0, config['base_nominal_height_m']], [1, 0, 0, 0]))
    assert not visible.any()


def test_blocks_no_duplicate_and_last_tag(config):
    config['block_rows'] = 8
    camera = Camera(config)
    emitted = []
    blocks = Blocks(camera, lambda im, meta: emitted.append((im, meta)))
    for i in range(19):
        blocks.add(np.full(4096, i, dtype=np.uint8), np.ones(4096, dtype=bool), {'time_s': float(i)})
    blocks.end_segment('stop')
    assert [m['rows'] for _, m in emitted] == [8, 8, 3]
    assert np.array_equal(np.concatenate([im[:, 0] for im, _ in emitted]), np.arange(19))
    assert [m['last']['global_line'] for _, m in emitted] == [7, 15, 18]
    assert all(m['pose_tags'][-1] == m['last'] for _, m in emitted)


def test_exposure_integrates_motion(config):
    camera = Camera(config)
    # Cross a 2 mm grid line during an artificially long exposure.
    a, b = ([.048, 0, config['base_nominal_height_m']], [0, 0, 0, 1]), ([.052, 0, config['base_nominal_height_m']], [0, 0, 0, 1])
    # Cross a known grid line independently of the configured camera offset.
    a[0][0], b[0][0] = 1-config['camera_x_m']-.002, 1-config['camera_x_m']+.002
    pixels, _ = camera.expose(a, b)
    assert np.any((pixels > config['grid_dark']) & (pixels < config['grid_light']))
