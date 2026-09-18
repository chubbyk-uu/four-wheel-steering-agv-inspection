"""Reading a track by the row, and refusing to paper over a hole in it.

The tool this replaces built the whole track in memory, so the properties
worth pinning are that a window costs the window and that a discarded tail
stays a discontinuity. Closing that hole silently would shift every row after
it, which is the one error a stitched mosaic cannot show you.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from agv_linescan.strip_reader import Strip, index, read_header

WIDTH = 64


def write_block(directory, block_id, segment, first, rows, seed):
    data = np.random.default_rng(seed).integers(0, 256, (rows, WIDTH), dtype=np.uint8)
    path = directory/('block_%06d.pgm' % block_id)
    path.write_bytes(b'P5\n%d %d\n255\n' % (WIDTH, rows)+data.tobytes())
    (directory/('block_%06d.json' % block_id)).write_text(json.dumps(dict(
        block_id=block_id, segment_id=segment, rows=rows,
        first=dict(global_line=first, time_s=10.+first*1e-4),
        last=dict(global_line=first+rows-1, time_s=10.+(first+rows-1)*1e-4))))
    return data


def archive(tmp_path, layout, discard=None):
    """layout: (block_id, segment, first_row, rows). discard: a block id to drop."""
    directory = tmp_path/'raw'
    directory.mkdir()
    truth = {}
    for block, segment, first, rows in layout:
        if discard is not None and block == discard:
            (directory/'events.jsonl').write_text(json.dumps(dict(
                reason='tail_discarded', block_id=block, segment_id=segment, rows=rows,
                minimum_rows=1000, first=dict(global_line=first, time_s=10.+first*1e-4),
                last=dict(global_line=first+rows-1, time_s=10.+(first+rows-1)*1e-4)))+'\n')
            continue
        truth[block] = write_block(directory, block, segment, first, rows, block)
    return directory, truth


def straight(tmp_path, blocks=4, rows=500):
    layout = [(i, 1, i*rows, rows) for i in range(blocks)]
    return archive(tmp_path, layout)


def test_a_window_equals_the_same_rows_of_the_whole_strip(tmp_path):
    directory, truth = straight(tmp_path)
    strip = index(directory)[1]
    whole = np.concatenate([truth[i] for i in sorted(truth)])
    assert strip.rows == len(whole) and strip.width == WIDTH
    for start, stop in ((0, 10), (499, 501), (450, 1050), (1500, 2000), (0, 2000)):
        assert np.array_equal(strip.read(start, stop), whole[start:stop])


def test_a_window_reads_only_the_blocks_it_spans(tmp_path):
    """The point of the exercise: cost follows the window, not the track."""
    directory, _ = straight(tmp_path, blocks=8)
    strip = index(directory)[1]
    opened = []
    original = Path.open

    def counting(self, *args, **kwargs):
        opened.append(self.name)
        return original(self, *args, **kwargs)

    Path.open = counting
    try:
        strip.read(1200, 1300)          # wholly inside block 2
        assert opened == ['block_000002.pgm']
        opened.clear()
        strip.read(990, 1010)           # straddles blocks 1 and 2
        assert opened == ['block_000001.pgm', 'block_000002.pgm']
    finally:
        Path.open = original


def test_a_discarded_tail_stays_a_hole(tmp_path):
    """Closing it would move every row after it, and nothing downstream could tell."""
    directory, truth = archive(tmp_path, [(0, 1, 0, 500), (1, 1, 500, 1), (2, 2, 501, 500)],
                               discard=1)
    strip = index(directory)[1]
    assert strip.spans == [(0, 500)]
    other = index(directory)[2]
    assert other.spans == [(501, 1001)]
    assert np.array_equal(strip.read(0, 500), truth[0])
    with pytest.raises(ValueError, match='cross a gap'):
        strip.read(400, 600)


def test_reads_outside_the_strip_are_refused(tmp_path):
    directory, _ = straight(tmp_path)
    strip = index(directory)[1]
    for start, stop, message in ((5000, 5100, 'not in this strip'), (10, 10, 'empty row range')):
        with pytest.raises(ValueError, match=message):
            strip.read(start, stop)


def test_the_archive_rules_are_the_correction_stage_rules(tmp_path):
    """Bound to validate_capture_sequence, not to a looser copy of it."""
    directory, _ = archive(tmp_path, [(0, 1, 0, 500), (1, 1, 700, 500)])   # unexplained hole
    with pytest.raises(ValueError, match='discontinuity'):
        index(directory)


def test_metadata_and_image_must_agree(tmp_path):
    directory, _ = straight(tmp_path)
    record = json.loads((directory/'block_000001.json').read_text())
    record['rows'] = 499
    record['last']['global_line'] = 500+498
    (directory/'block_000001.json').write_text(json.dumps(record))
    with pytest.raises(ValueError, match='disagrees with metadata'):
        index(directory)


def test_a_missing_image_is_not_silently_skipped(tmp_path):
    directory, _ = straight(tmp_path)
    (directory/'block_000002.pgm').unlink()
    with pytest.raises(ValueError, match='pairs are incomplete'):
        index(directory)


def test_header_parsing_is_not_an_assumed_offset(tmp_path):
    directory, _ = straight(tmp_path, blocks=1, rows=7)
    path = directory/'block_000000.pgm'
    assert read_header(path) == (WIDTH, 7, len(b'P5\n%d %d\n255\n' % (WIDTH, 7)))
    path.write_bytes(b'P6\n1 1\n255\nxxx')
    with pytest.raises(ValueError, match='not a binary PGM'):
        read_header(path)


def test_the_strip_never_materialises_the_track(tmp_path):
    """A window of 100 rows must allocate a window, whatever the track holds."""
    directory, _ = straight(tmp_path, blocks=20, rows=500)
    strip = index(directory)[1]
    assert strip.rows == 10000
    window = strip.read(0, 100)
    assert window.nbytes == 100*WIDTH
