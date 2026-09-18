"""Post-process a complete raw capture session without ROS or a running simulator."""
import hashlib
import json
import math
import os
from pathlib import Path
import time
import numpy as np
from PIL import Image
import yaml
from agv_linescan.calibration import Correction


def _write(path, data):
    # A final filename denotes a complete file. Session completion is recorded separately.
    temporary=path.with_name(path.name+'.tmp')
    with temporary.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
    temporary.rename(path)


def validate_block_range(record):
    """Validate saved and discarded ranges with the same units and ordering."""
    try:
        for key in ('block_id','segment_id','rows'):
            if type(record[key]) is not int or record[key] < (1 if key=='rows' else 0):
                raise ValueError('invalid block identity or row count')
        first,last=record['first'],record['last']
        for tag in (first,last):
            if type(tag['global_line']) is not int or tag['global_line']<0:
                raise ValueError('invalid global line')
            if type(tag['time_s']) not in (int,float) or not math.isfinite(tag['time_s']):
                raise ValueError('invalid line timestamp')
        if last['global_line']-first['global_line']+1!=record['rows']:
            raise ValueError('line count differs from metadata')
        if first['time_s']>last['time_s']:
            raise ValueError('reversed timestamps')
    except (KeyError,TypeError) as exc:
        raise ValueError('missing or invalid block range') from exc


def validate_capture_sequence(saved,discarded):
    """Include terminal discards, not just events between two saved images."""
    records={}
    for record in [*saved,*discarded.values()]:
        validate_block_range(record)
        key=record['block_id']
        if key in records:raise ValueError('saved and discarded block ids overlap')
        records[key]=record
    previous=None
    for expected,key in enumerate(sorted(records)):
        if key!=expected:raise ValueError('missing or misnumbered image block')
        current=records[key]
        if previous:
            if current['segment_id']<previous['segment_id']:raise ValueError('reversed segment ordering')
            if current['first']['time_s']<=previous['last']['time_s']:raise ValueError('nonmonotonic block timestamp')
            line=current['first']['global_line'];end=previous['last']['global_line']
            if line<=end:raise ValueError('overlapping block line ranges')
            if (key in discarded or previous['block_id'] in discarded or
                    current['segment_id']==previous['segment_id']) and line!=end+1:
                raise ValueError('line discontinuity around block or discarded tail')
        previous=current


def discarded_tails(source):
    """Block ids the sensor discarded as short tails, from its own event log.

    A gap in the numbering is only acceptable with evidence, and the evidence is
    checked rather than trusted: the event must declare fewer rows than the
    threshold it was measured against, which is what makes it a short tail.
    """
    path=source/'events.jsonl';found={}
    if not path.is_file():return found
    for index,line in enumerate(path.read_text().splitlines()):
        if not line.strip():continue
        try:event=json.loads(line)
        except ValueError:raise ValueError('corrupt capture event record %d'%index)
        if not isinstance(event,dict):raise ValueError('invalid capture event record')
        if event.get('reason')!='tail_discarded':continue
        validate_block_range(event)
        rows,minimum=event.get('rows',0),event.get('minimum_rows',0)
        if type(minimum) is not int or not 0<rows<minimum:raise ValueError('tail discard event does not justify a missing block')
        if event['block_id'] in found:raise ValueError('duplicate tail discard for one block id')
        found[event['block_id']]=event
    return found


def process_session(source, profile, output, accept_robot_change=None):
    source,profile,output=map(lambda p:Path(p).resolve(),(source,profile,output))
    config=yaml.safe_load((source/'calibration.yaml').read_text())
    correction=Correction(json.loads(profile.read_text()),accept_robot_change);correction.check_capture(config)
    paths=sorted(source.glob('block_*.json'))
    if not paths:raise ValueError('raw capture contains no image metadata')
    if {p.stem for p in paths}!={p.stem for p in source.glob('block_*.pgm')}:raise ValueError('raw image / metadata pairs are incomplete')
    discarded=discarded_tails(source)
    entries=[]
    for path in paths:
        m=json.loads(path.read_text());correction.metadata(m)
        validate_block_range(m)
        if path.stem!=f"block_{m['block_id']:06d}":raise ValueError('misnumbered image block')
        if m.get('encoding')!='mono8' or not 1<=m['rows']<=config['block_rows']:raise ValueError('invalid raw block dimensions/encoding')
        first,last=m['first'],m['last']
        tags=m['pose_tags']
        if not 1<=len(tags)<=m['rows'] or any(not first['global_line']<=t['global_line']<=last['global_line'] for t in tags):raise ValueError('invalid sparse pose tags')
        entries.append((path,m))
    validate_capture_sequence([m for _,m in entries],discarded)
    output.mkdir(parents=True,exist_ok=False)
    (output/'calibration.json').write_bytes(profile.read_bytes())
    start=time.monotonic();rows=0;raw_saturated=0;clipped=0;per_block=[]
    try:
        for path,m in entries:
            t=time.monotonic();raw=np.array(Image.open(path.with_suffix('.pgm')))
            if raw.shape!=(m['rows'],m['width']) or raw.dtype!=np.uint8:raise ValueError('raw pixels differ from declared dimensions/encoding')
            pixels,quality=correction.apply(raw);metadata=correction.metadata(m);metadata.update(quality)
            metadata.update(source_pixel_sha256=hashlib.sha256(raw).hexdigest(),corrected_pixel_sha256=hashlib.sha256(pixels).hexdigest(),
                corrected_image_fsync=True,processing_mode='offline',rows_preserved=True)
            _write(output/path.with_suffix('.pgm').name,f'P5\n{m["width"]} {m["rows"]}\n255\n'.encode()+pixels.tobytes())
            _write(output/path.name,json.dumps(metadata).encode()+b'\n')
            rows+=m['rows'];raw_saturated+=quality['raw_saturated_pixels'];clipped+=quality['corrected_clipped_pixels'];per_block.append(time.monotonic()-t)
        elapsed=time.monotonic()-start
        report=dict(passed=True,processing_mode='offline',blocks=len(entries),rows=rows,width=correction.width,
            configured_block_rows=config['block_rows'],tail_block_rows=entries[-1][1]['rows'],
            calibration_id=correction.profile['calibration_id'],raw_saturated_pixels=raw_saturated,corrected_clipped_pixels=clipped,
            elapsed_seconds=elapsed,lines_per_second=rows/elapsed,max_block_seconds=max(per_block),
            metadata_preserved=True,rows_overlap_added=0,image_and_metadata_fsync=True,
            discarded_tail_blocks=[discarded[k] for k in sorted(discarded)],
            block_id_gaps=sorted(k for k in discarded if k<entries[-1][1]['block_id']),
            note='Source images untouched; no ROS/GZ required, no real-time throughput requirement. Directory entries are not fsynced.')
        _write(output/'summary.json',json.dumps(report,indent=2).encode()+b'\n');return report
    except Exception as e:
        (output/'FAILED.json').write_text(json.dumps(dict(error=str(e),completed_blocks=len(per_block))))
        raise
