"""The optimiser's input table: one estimate per image, keyed by row, no truth.

What these pin down is not the arithmetic -- that is bound to the audited
projection in capture_audit -- but the two properties that make the table a
contract. It is keyed by global row rather than by file, so re-chunking the
stored images cannot move the geometry. And it is built from the estimate,
never from the renderer pose that sits in the same block metadata.
"""
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

from agv_mission.capture_audit import Navigation
from agv_mission.strip_observations import SCHEMA, extract, load, road_frame, scan_centre

CALIBRATION = 'agv-cal-test'
LEVER = [1.15, 0.0, 0.39631696428571417]


def write_session(tmp_path, blocks=4, rows=4096, gap_s=None, gap_at=10., truth=True):
    """A session with straight motion: navigation, a plan, and archived images."""
    session = tmp_path/'session'
    (session/'navigation').mkdir(parents=True)
    (session/'raw/session_cpp_1').mkdir(parents=True)
    (session/'tasks/t0/mission').mkdir(parents=True)

    (session/'navigation/calibration.json').write_text(json.dumps(dict(
        calibration_id=CALIBRATION, parent='base_link', convention='xyzw',
        wheel_calibration=dict(radii_m=[.2]*4),
        estimated_frames=[dict(frame_id='camera_optical_calibrated', translation_m=LEVER,
                               orientation_xyzw=list(Rotation.from_euler(
                                   'xyz', [math.pi, 0, math.pi/2]).as_quat()))])))

    covariance = np.diag([4e-4, 9e-4, 1e-4, 1e-6, 1e-6, 4e-6]).ravel().tolist()
    times = np.arange(0, 60, .02)
    records = []
    for t in times:
        if gap_s and gap_at < t < gap_at+gap_s:
            continue
        records.append(json.dumps(dict(
            time_s=float(t), frame_id='map', child_frame_id='base_link',
            position_m=[float(2.777778*t), 0.25, .6474], orientation_xyzw=[0., 0., 0., 1.],
            linear_velocity_m_s=[2.777778, 0., 0.], angular_velocity_rad_s=[0., 0., 0.],
            pose_covariance=covariance, twist_covariance=covariance,
            calibration_id=CALIBRATION)))
    (session/'navigation/navigation.jsonl').write_text('\n'.join(records)+'\n')

    (session/'tasks/t0/mission/plan.json').write_text(json.dumps(dict(
        request=dict(road=dict(frame_id='map', origin_xyz_m=[0., 0., 0.], yaw_rad=0., surface='flat')))))

    for i in range(blocks):
        first = dict(global_line=i*rows, time_s=20.+i*1.5)
        if truth:                      # exactly what the extractor must not read
            first.update(camera_position_world_m=[999., 999., 999.],
                         camera_rotation_world=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                         encoder_distance_m=float(i), scan_direction=1)
        (session/('raw/session_cpp_1/block_%06d.json' % i)).write_text(json.dumps(dict(
            block_id=i, segment_id=1+i//2, rows=rows, first=first,
            last=dict(global_line=(i+1)*rows-1, time_s=20.+i*1.5+1.4),
            pose_tags=[], calibration_id='nominal')))
    return session


def test_one_observation_per_image_ordered_and_keyed_by_row(tmp_path):
    table = extract(write_session(tmp_path, blocks=6))
    rows = [v['global_row'] for v in table['observations']]
    assert table['schema'] == SCHEMA and len(rows) == 6
    assert rows == sorted(set(rows)) == [i*4096 for i in range(6)]
    assert {v['segment_id'] for v in table['observations']} == {1, 2, 3}


def test_no_entry_names_a_file_or_a_block(tmp_path):
    """Keyed by row is the point; a file name in here would undo it."""
    table = extract(write_session(tmp_path))
    for entry in table['observations']:
        assert set(entry) == {'segment_id', 'global_row', 'time_s', 'road_x_m', 'road_y_m',
                              'heading_rad', 'sigma_x_m', 'sigma_y_m', 'sigma_heading_rad'}
    assert 'block' not in json.dumps(table['observations'])


def test_the_renderer_pose_in_the_block_is_never_read(tmp_path):
    """block['first'] also carries where the simulator put the camera."""
    honest = extract(write_session(tmp_path/'a', truth=False))['observations']
    poisoned = extract(write_session(tmp_path/'b', truth=True))['observations']
    assert honest == poisoned


def test_the_table_stands_alone_once_written(tmp_path):
    """Re-splitting stored images cannot move geometry that no longer reads them."""
    session = write_session(tmp_path)
    path = tmp_path/'observations.json'
    path.write_text(json.dumps(extract(session))+'\n')
    import shutil
    shutil.rmtree(session/'raw')
    assert len(load(path)['observations']) == 4


def test_the_estimate_is_used_not_the_truth_position(tmp_path):
    """Straight motion at a known speed: the table has to land on it."""
    table = extract(write_session(tmp_path))
    for entry in table['observations']:
        # base_link at 2.777778 t, camera 1.15 m ahead, nadir on a flat road.
        assert entry['road_x_m'] == pytest.approx(2.777778*entry['time_s']+1.15, abs=1e-6)
        assert entry['road_y_m'] == pytest.approx(0.25, abs=1e-6)
        assert entry['heading_rad'] == pytest.approx(0., abs=1e-9)


def test_uncertainty_carries_the_lever_arm_not_just_the_position(tmp_path):
    """A 1.15 m arm turns yaw error into ground error; the contract asks for that."""
    table = extract(write_session(tmp_path))
    entry = table['observations'][0]
    # sigma_y grows over the raw 0.03 m position sigma by the yaw term, 1.15*0.002.
    assert entry['sigma_y_m'] > 0.03
    assert entry['sigma_y_m'] == pytest.approx(math.hypot(.03, 1.15*.002), rel=.02)
    assert entry['sigma_x_m'] == pytest.approx(.02, rel=.05)
    assert entry['sigma_heading_rad'] == pytest.approx(.002, rel=.02)


def test_a_navigation_gap_over_an_observation_is_refused(tmp_path):
    """A gap elsewhere is harmless; one spanning an exposure is not, and the
    difference is the point -- the table interpolates, it does not extrapolate."""
    assert extract(write_session(tmp_path/'far', gap_s=.5, gap_at=10.))   # no exposure there
    with pytest.raises(ValueError, match='navigation gap|bracket'):
        extract(write_session(tmp_path/'over', gap_s=.5, gap_at=21.4))  # spans row 4096


def test_load_refuses_a_table_a_fit_could_not_use(tmp_path):
    session = write_session(tmp_path)
    good = extract(session)
    path = tmp_path/'t.json'

    def store(table):
        path.write_text(json.dumps(table));return path

    assert load(store(good))
    for mutate, message in (
            (lambda t: t.update(schema='something.else'), 'schema'),
            (lambda t: t.update(observations=[]), 'empty'),
            (lambda t: t['observations'].append(dict(t['observations'][0])), 'unique and ordered'),
            (lambda t: t['observations'][1].update(global_row=0), 'unique and ordered'),
            (lambda t: t['observations'][0].update(road_x_m=float('nan')), 'non-finite'),
            (lambda t: t['observations'][0].update(sigma_y_m=0.), 'no usable uncertainty')):
        table = json.loads(json.dumps(good))
        mutate(table)
        with pytest.raises(ValueError, match=message):
            load(store(table))


def test_the_projection_is_the_audited_one(tmp_path):
    """Bound to capture_audit rather than reimplemented beside it.

    footprint projects the two ends of the row; with the pixel pitch collapsed
    its two ends fall onto the centre ray, which is what this table stores. If
    the frame, the lever arm, the mount or a sign ever diverge, this fails.
    """
    session = write_session(tmp_path)
    navigation = Navigation(session/'navigation')
    request = json.loads((session/'tasks/t0/mission/plan.json').read_text())['request']
    origin, rotation = road_frame(request)
    camera = dict(width=4096, pixel_pitch_m=1e-12, focal_length_m=.02)
    time = 20.
    index = int(np.searchsorted(navigation.times, time))
    ratio = (time-navigation.times[index-1])/(navigation.times[index]-navigation.times[index-1])
    position = navigation.positions[index-1]*(1-ratio)+navigation.positions[index]*ratio
    body = navigation.rotations([time])[0]
    centre, _ = scan_centre(navigation, position, body, origin, rotation)
    edges = navigation.footprint(dict(global_line=0, time_s=time), camera,
                                 dict(request=request), None)
    assert edges['road_x'][0] == pytest.approx(centre[0], abs=1e-6)
    assert edges['road_y'][0] == pytest.approx(centre[1], abs=1e-6)
