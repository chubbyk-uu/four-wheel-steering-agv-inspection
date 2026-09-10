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
