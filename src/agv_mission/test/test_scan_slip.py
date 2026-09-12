"""The duplicate-texture detectors must catch the artifact they were written for."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('audit_scan_slip', ROOT/'tools/audit_scan_slip.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)

SPACING = .0003662109375


def block(pairs, block_id=0):
    """One archived block whose tags advance by (lines, ground metres) each."""
    tags = [dict(global_line=0, camera_position_world_m=[0., 0., 1.], time_s=0.)]
    for lines, ground in pairs:
        last = tags[-1]
        tags.append(dict(global_line=last['global_line']+lines,
                         camera_position_world_m=[last['camera_position_world_m'][0]+ground, 0., 1.],
                         time_s=last['time_s']+1.))
    return dict(block_id=block_id, segment_id=0, line_spacing_m=SPACING, pose_tags=tags)


def test_wheels_turning_under_a_stopped_vehicle_are_reported():
    # The measured artifact: 8.057 mm of encoder against 0.968 mm of ground over
    # 23 lines, while a healthy interval tracks the encoder to within a part in
    # a thousand. 23 lines is 8.42 mm, so the recorded encoder distance is that.
    healthy = (1024, 1024*SPACING)
    stalled = (23, .000968)
    intervals = list(audit.tag_intervals([block([healthy, stalled, healthy])]))
    assert [round(v['ratio'], 3) for v in intervals] == [1.0, 0.115, 1.0]
    caught = [v for v in intervals if v['ratio'] < .95]
    assert len(caught) == 1 and caught[0]['lines'] == 23


def test_a_healthy_interval_is_not_reported():
    # Tyre slip during acceleration was measured at about 1% over 375 mm and
    # must not be confused with a stall.
    slipping = (1024, 1024*SPACING*.99)
    assert all(v['ratio'] > .95 for v in audit.tag_intervals([block([slipping])]))


def test_duplicate_rows_are_found_in_the_pixels(tmp_path):
    from PIL import Image
    rng = np.random.default_rng(0)
    pixels = rng.integers(40, 200, size=(256, 64), dtype=np.uint8)
    # Twenty-three lines imaging the same ground repeat one row exactly.
    pixels[100:123] = pixels[100]
    path = tmp_path/'block_000000.pgm'
    Image.fromarray(pixels, mode='L').save(path)
    difference = audit.row_differences(path, 1)
    assert difference.size == 255
    duplicated = np.flatnonzero(difference == 0)
    assert duplicated.tolist() == list(range(100, 122))
    # The threshold is read from the run's own texture, not assumed.
    threshold = float(np.median(difference))*.4
    assert threshold > 0 and (difference < threshold).sum() == duplicated.size


def test_ground_distance_ignores_suspension_travel():
    # During a stop the body rises millimetres on the suspension. A 3-D distance
    # would read that as travel; the scan axis is the only honest measure.
    tags = [dict(global_line=0, camera_position_world_m=[0., 0., 1.0411], time_s=0.),
            dict(global_line=1, camera_position_world_m=[0., 0., 1.0446], time_s=1.)]
    interval, = audit.tag_intervals([dict(block_id=0, segment_id=0,
                                          line_spacing_m=SPACING, pose_tags=tags)])
    assert interval['ground_m'] == pytest.approx(0.)
    assert interval['ratio'] == pytest.approx(0.)
