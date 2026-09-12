"""The duplicate-texture detectors must catch the artifact they were written for."""
import importlib.util
import json
import sys
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


def archived(tmp_path, with_image=True, rows=256):
    """A one-block mission whose archive may or may not still hold its pixels."""
    mission = tmp_path/'mission'; mission.mkdir()
    archive = tmp_path/'raw'/'session'; archive.mkdir(parents=True)
    tags = [dict(global_line=0, camera_position_world_m=[0., 0., 1.], time_s=0.),
            dict(global_line=rows-1, camera_position_world_m=[(rows-1)*SPACING, 0., 1.], time_s=1.)]
    record = dict(block_id=0, segment_id=0, rows=rows, line_spacing_m=SPACING, pose_tags=tags,
                  first=dict(global_line=0), last=dict(global_line=rows-1))
    (mission/'capture_blocks.jsonl').write_text(json.dumps(record)+'\n')
    (mission/'capture_intervals.json').write_text(json.dumps(
        [dict(track_id=0, archive=str(archive), enabled_ack_time_s=0., disabled_ack_time_s=1.)]))
    (archive/'block_000000.json').write_text(json.dumps(record))
    if with_image:
        from PIL import Image
        pixels = np.random.default_rng(1).integers(40, 200, size=(rows, 64), dtype=np.uint8)
        Image.fromarray(pixels, mode='L').save(archive/'block_000000.pgm')
    return mission


def run(mission, output, monkeypatch):
    monkeypatch.setattr(sys, 'argv',
                        ['audit_scan_slip', '--mission', str(mission), '--output', str(output)])
    with pytest.raises(SystemExit) as exit:
        audit.main()
    return exit.value.code, json.loads(output.read_text()) if output.is_file() else None


def test_metadata_without_pixels_is_a_failure_not_a_pass(tmp_path, monkeypatch):
    # Deleting or moving an archive used to leave the pixel half with nothing to
    # test, and "no duplicate rows found" then passed the audit vacuously.
    code, report = run(archived(tmp_path, with_image=False), tmp_path/'r.json', monkeypatch)
    assert report['blocks'] == 1 and report['images'] == 0
    assert report['pixel_evidence']['missing'] == 1
    assert report['pixel_evidence']['first_missing'][0]['image'] == 'block_000000.pgm'
    assert report['pixel_evidence']['first_missing'][0]['metadata_located'] is True
    assert report['passed'] is False and code == 1


def test_a_block_paired_with_its_own_image_passes(tmp_path, monkeypatch):
    code, report = run(archived(tmp_path), tmp_path/'r.json', monkeypatch)
    assert report['blocks'] == 1 and report['images'] == 1
    assert report['pixel_evidence']['missing'] == 0
    assert report['passed'] is True and code == 0
