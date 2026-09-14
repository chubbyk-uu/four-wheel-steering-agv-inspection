"""Conservative motion-quality exclusion from control records, never rendered pose truth."""
import math


def reconfiguration_windows(records):
    windows=[];previous=None
    for row in records:
        t=row['time_s']
        if not math.isfinite(t) or (previous is not None and t<previous):
            raise ValueError('invalid execution time ordering')
        previous=t
        if (row.get('kind')!='PASS' or row.get('capture_active') is not True
            or row.get('motion_state') not in ('BRAKE','ALIGN')
            or row.get('motion_reason') not in ('LIMIT_RECONFIGURE','ALIGN_WHEEL_MOTION','LARGE_STEER_CHANGE','LATERAL_MISMATCH','DRIVE_REVERSAL')):
            continue
        # Cover telemetry latency around transitions; this is not pixel timing.
        start,end=t-.05,t+.05;track=row['track_id']
        if windows and windows[-1]['track_id']==track and start<=windows[-1]['end_s']:
            windows[-1]['end_s']=end
        else:windows.append(dict(track_id=track,start_s=start,end_s=end))
    return windows


def affected(windows,track,first,last):
    return any(w['track_id']==track and first<=w['end_s'] and last>=w['start_s'] for w in windows)

def trace_gaps(records, tolerance=.002):
    """Intervals where execution records are missing, as opposed to merely slow.

    Every record reports the duration of its own control step, so a slow tick still
    leaves evidence and its gap equals the duration it declares; a gap materially
    longer than that means records between the two are absent and nothing can be
    said about what the vehicle did in between. Measured across 193,730 record
    pairs in the 2026-09-14 acceptance runs the two agree exactly, including one
    0.352 s step that declared 0.352 s, so a fixed threshold would have been both
    arbitrary and wrong about that step.
    """
    gaps=[]
    for previous,row in zip(records,records[1:]):
        span=row['time_s']-previous['time_s'];declared=row.get('control_dt_s')
        if declared is None or span>declared+tolerance:
            gaps.append(dict(start_s=previous['time_s'],end_s=row['time_s'],span_s=span,declared_step_s=declared))
    return gaps
