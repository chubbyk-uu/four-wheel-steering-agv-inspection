"""A track read by the row, without ever holding the track.

The first concatenation tool built a whole track in memory, wrote it as one
PNG and read it back to verify: three copies of 1.13 Gpx for a 100 m pass, and
over Pillow's decompression-bomb limit besides. Twenty metres already exceeds
that limit, so this is not a hundred-metre problem to solve later.

Nothing about the archive needs it, though. Every image is P5 with a fixed
header and a fixed row stride, so a row is a seek, and the block metadata
already says which global rows each file holds. What this adds is the index
and the honesty about gaps: a discarded short tail leaves a real hole in the
row numbering, and a reader that silently closed it would move every row after
it. Runs are therefore exposed as contiguous spans and a read may not cross
one.

Sequence validation is the archive's own -- validate_capture_sequence and
discarded_tails from offline_correction -- so a strip cannot be read under
looser rules than the correction stage applies.
"""
import json
from pathlib import Path

import numpy as np

from .offline_correction import discarded_tails, validate_block_range, validate_capture_sequence


def read_header(path):
    """P5 width, height and the offset its pixel data starts at."""
    with path.open('rb') as f:
        head = f.read(64)
    if not head.startswith(b'P5'):
        raise ValueError('not a binary PGM: '+path.name)
    fields, index = [], 2
    while len(fields) < 3:
        while index < len(head) and head[index:index+1].isspace():
            index += 1
        if index < len(head) and head[index:index+1] == b'#':
            raise ValueError('commented PGM header is not written by this project')
        start = index
        while index < len(head) and not head[index:index+1].isspace():
            index += 1
        if index >= len(head):
            raise ValueError('truncated PGM header: '+path.name)
        fields.append(int(head[start:index]))
    if fields[2] != 255:
        raise ValueError('only 8 bit images are archived')
    return fields[0], fields[1], index+1


class Strip:
    """One capture segment, addressable by global row."""

    def __init__(self, blocks, width):
        self.blocks = blocks                      # (first_row, rows, path, offset), ordered
        self.width = width
        self.rows = sum(b[1] for b in blocks)
        self.spans = []                           # contiguous global-row runs
        for first, rows, _, _ in blocks:
            if self.spans and self.spans[-1][1] == first:
                self.spans[-1] = (self.spans[-1][0], first+rows)
            else:
                self.spans.append((first, first+rows))
        self.spans = [tuple(v) for v in self.spans]

    @property
    def first_row(self):
        return self.blocks[0][0]

    @property
    def last_row(self):
        return self.blocks[-1][0]+self.blocks[-1][1]-1

    def span_of(self, row):
        for span in self.spans:
            if span[0] <= row < span[1]:
                return span
        raise ValueError('row %d is not in this strip' % row)

    def read(self, start, stop):
        """Rows [start, stop) as (n, width) uint8, touching only those bytes."""
        if stop <= start:
            raise ValueError('empty row range')
        span = self.span_of(start)
        if stop > span[1]:
            raise ValueError('rows %d..%d cross a gap at %d; read each run separately'
                             % (start, stop, span[1]))
        out = np.empty((stop-start, self.width), np.uint8)
        filled = 0
        for first, rows, path, offset in self.blocks:
            lo, hi = max(start, first), min(stop, first+rows)
            if lo >= hi:
                continue
            with path.open('rb') as f:
                f.seek(offset+(lo-first)*self.width)
                chunk = f.read((hi-lo)*self.width)
            if len(chunk) != (hi-lo)*self.width:
                raise ValueError('short image file: '+path.name)
            out[filled:filled+hi-lo] = np.frombuffer(chunk, np.uint8).reshape(hi-lo, self.width)
            filled += hi-lo
        if filled != stop-start:
            raise ValueError('rows %d..%d are not all archived' % (start, stop))
        return out


def index(source):
    """Validated strips of one raw or corrected capture directory, by segment."""
    source = Path(source)
    paths = sorted(source.glob('block_*.json'))
    if not paths:
        raise ValueError('no image metadata under '+str(source))
    if {p.stem for p in paths} != {p.stem for p in source.glob('block_*.pgm')}:
        raise ValueError('image / metadata pairs are incomplete')
    records, width = [], None
    for path in paths:
        record = json.loads(path.read_text())
        validate_block_range(record)
        if path.stem != 'block_%06d' % record['block_id']:
            raise ValueError('misnumbered image block')
        image = path.with_suffix('.pgm')
        w, h, offset = read_header(image)
        if h != record['rows'] or (record.get('width') not in (None, w)):
            raise ValueError('image shape disagrees with metadata: '+image.name)
        if width is None:
            width = w
        elif w != width:
            raise ValueError('image width changes inside a session')
        records.append((record, image, offset))
    validate_capture_sequence([r for r, _, _ in records], discarded_tails(source))

    strips = {}
    for record, image, offset in records:
        strips.setdefault(record['segment_id'], []).append(
            (record['first']['global_line'], record['rows'], image, offset))
    return {k: Strip(sorted(v), width) for k, v in sorted(strips.items())}
