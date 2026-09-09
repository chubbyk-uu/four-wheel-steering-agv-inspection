"""Conservative interval estimates on a declared flat road; never pixel-level proof."""
from copy import deepcopy
import math
from .planner import plan as make_plan


def merge(intervals):
    out=[]
    for lo,hi in sorted(intervals):
        if hi<=lo:continue
        if out and lo<=out[-1][1]+1e-9:out[-1][1]=max(hi,out[-1][1])
        else:out.append([lo,hi])
    return out


def complement(intervals,length):
    gaps=[];cursor=0.
    for lo,hi in merge(intervals):
        lo=max(0.,lo);hi=min(length,hi)
        if lo>cursor+1e-9:gaps.append([cursor,lo])
        cursor=max(cursor,hi)
    if cursor<length-1e-9:gaps.append([cursor,length])
    return gaps


def estimate(source,spans,uncertainty=.10):
    if not math.isfinite(uncertainty) or uncertainty<=0:raise ValueError('positive spatial uncertainty required')
    region=source['request']['region'];x,y=region['start_xy_m'];length=region['length_m']
    centers=[t['center_road_y_m'] for t in source['tracks']];tracks=[]
    for i,track in enumerate(source['tracks']):
        lo=y if i==0 else (centers[i-1]+centers[i])/2
        hi=y+region['width_m'] if i+1==len(centers) else (centers[i]+centers[i+1])/2
        accepted=[];rejected=0;max_lateral=0.;flags=set();point_count=0
        direction=track['direction']
        def include(points):
            if len(points)<2:return
            def along(p):return [v-x if direction==1 else x+length-v for v in p['road_x']]
            start=max(along(points[0]))+max(uncertainty,points[0].get('longitudinal_sigma3_m',0))
            end=min(along(points[-1]))-max(uncertainty,points[-1].get('longitudinal_sigma3_m',0))
            if end>start:accepted.append([max(0.,start),min(length,end)])
        for span in spans:
            if span['track_id']!=track['id']:continue
            # A bad end tag must not erase the verified prefix of the same image.
            sub=[]
            for point in span['points']:
                point_count+=1
                pad=max(uncertainty,point.get('lateral_sigma3_m',0));max_lateral=max(max_lateral,pad)
                if min(point['road_y'])+pad>lo or max(point['road_y'])-pad<hi:
                    include(sub);sub=[];rejected+=1
                    flags.add('UNCERTAINTY_EXCEEDS_OVERLAP' if min(point['road_y'])<=lo and max(point['road_y'])>=hi else 'FOOTPRINT_OUTSIDE_TARGET')
                else:sub.append(point)
            include(sub)
        if not point_count:flags.add('NO_CAPTURE_EVIDENCE')
        elif point_count<2:flags.add('INSUFFICIENT_POSE_TAGS')
        accepted=merge(accepted)
        tracks.append(dict(track_id=track['id'],direction=track['direction'],assigned_road_y_m=[lo,hi],
            estimated_covered_along_m=accepted,unverified_along_m=complement(accepted,length),
            excluded_footprint_tags=rejected,max_lateral_allowance_m=max_lateral if point_count else None,
            quality_flags=sorted(flags)))
    return dict(schema='agv.mission.coverage.v1',mission_id=source['mission_id'],uncertainty_m=uncertainty,
        status='NEEDS_RESCAN' if any(t['unverified_along_m'] for t in tracks) else 'ESTIMATED_COMPLETE',tracks=tracks,
        scope='Sparse fused/calibrated footprints with a declared uncertainty allowance on a flat road. Not ground-truth or pixel-level coverage certification.')


def rescan_requests(source,audit,vehicle):
    """Return independently validated rectangle candidates; never command a faulted robot."""
    result=[];region=source['request']['region'];x=region['start_xy_m'][0];length=region['length_m']
    pad=audit['uncertainty_m']
    for track in audit['tracks']:
        # Expand into known coverage, then merge, so small gaps do not fragment missions.
        intervals=merge([[max(0.,a-pad),min(length,b+pad)] for a,b in track['unverified_along_m']])
        for a,b in intervals:
            request=deepcopy(source['request']);lo,hi=track['assigned_road_y_m']
            start=x+a if track['direction']==1 else x+length-b
            request['mission_id']=f"{source['mission_id']}_rescan_{len(result):03d}"
            request['region']=dict(start_xy_m=[start,lo],length_m=b-a,width_m=hi-lo)
            request['coverage_error_m']=max(request['coverage_error_m'],pad)
            entry=dict(source_track_id=track['track_id'],source_along_m=[a,b],request=request)
            try:
                candidate=make_plan(request,vehicle)
                entry.update(status='PREVIEW_ONLY',plan=candidate)
            except ValueError as exc:entry.update(status='BLOCKED_GEOMETRY',reason=str(exc))
            result.append(entry)
    return result
