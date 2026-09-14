"""Post-process a complete raw capture session without ROS or a running simulator."""
import hashlib
import json
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
        if event.get('reason')!='tail_discarded' or 'block_id' not in event:continue
        rows,minimum=event.get('rows',0),event.get('minimum_rows',0)
        if not 0<rows<minimum:raise ValueError('tail discard event does not justify a missing block')
        if event['block_id'] in found:raise ValueError('duplicate tail discard for one block id')
        found[event['block_id']]=dict(block_id=event['block_id'],rows=rows,minimum_rows=minimum,
                                      segment_id=event.get('segment_id'),end_reason=event.get('end_reason'))
    return found


def process_session(source, profile, output):
    source,profile,output=map(lambda p:Path(p).resolve(),(source,profile,output))
    config=yaml.safe_load((source/'calibration.yaml').read_text())
    correction=Correction(json.loads(profile.read_text()));correction.check_capture(config)
    paths=sorted(source.glob('block_*.json'))
    if not paths:raise ValueError('raw capture contains no image metadata')
    if {p.stem for p in paths}!={p.stem for p in source.glob('block_*.pgm')}:raise ValueError('raw image / metadata pairs are incomplete')
    discarded=discarded_tails(source)
    entries=[];previous=None;expected=0
    for path in paths:
        m=json.loads(path.read_text());correction.metadata(m)
        # A short tail is discarded by policy and still consumes its block id, so the
        # numbering is allowed to skip exactly those ids and nothing else.
        while expected in discarded:expected+=1
        if m['block_id']!=expected or path.stem!=f'block_{expected:06d}':raise ValueError('missing or misnumbered image block')
        expected+=1
        if m.get('encoding')!='mono8' or not 1<=m['rows']<=config['block_rows']:raise ValueError('invalid raw block dimensions/encoding')
        first,last=m['first'],m['last']
        if last['global_line']-first['global_line']+1!=m['rows']:raise ValueError('line count differs from metadata')
        if first['time_s']>last['time_s']:raise ValueError('reversed timestamps')
        if previous:
            if first['time_s']<=previous['last']['time_s']:raise ValueError('nonmonotonic block timestamp')
            if m['segment_id']==previous['segment_id'] and first['global_line']!=previous['last']['global_line']+1:raise ValueError('line discontinuity within segment')
        tags=m['pose_tags']
        if not 1<=len(tags)<=m['rows'] or any(not first['global_line']<=t['global_line']<=last['global_line'] for t in tags):raise ValueError('invalid sparse pose tags')
        entries.append((path,m));previous=m
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
