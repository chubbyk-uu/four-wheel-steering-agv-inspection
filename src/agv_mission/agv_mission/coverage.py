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


def combine(parent, children):
    """Union conservative road rectangles; require explicit parent candidate association."""
    result=deepcopy(parent)
    if not parent.get('scene_contract_sha256') or len(parent['scene_contract_sha256'])!=1:
        raise ValueError('one archived scene identity required for cross-session coverage')
    rectangles=[]
    def collect(report):
        region=report['request']['region'];x=region['start_xy_m'][0];length=region['length_m']
        for track in report['tracks']:
            lo,hi=track['assigned_road_y_m']
            for a,b in track['estimated_covered_along_m']:
                rectangles.append((x+a,x+b,lo,hi) if track['direction']==1 else (x+length-b,x+length-a,lo,hi))
    collect(parent);identities=set(parent.get('merged_navigation_sha256',[parent['provenance']['navigation_sha256']]))
    original_candidates=parent.get('original_rescan_candidates',parent.get('rescan_candidates',[]))
    candidates=[c['request'] for c in original_candidates if c['status']=='PREVIEW_ONLY']
    for child in children:
        if child.get('request') not in candidates:raise ValueError('child is not a validated parent rescan candidate')
        for key in ('scene_contract_sha256','optical_intrinsic_id'):
            if child.get(key)!=parent.get(key):raise ValueError('incompatible '+key)
        if child['uncertainty_m']<parent['uncertainty_m']:raise ValueError('child weakens uncertainty floor')
        identity=child['provenance']['navigation_sha256']
        if identity in identities:raise ValueError('duplicate navigation session')
        identities.add(identity);collect(child)
    region=parent['request']['region'];x=region['start_xy_m'][0];length=region['length_m']
    for track in result['tracks']:
        lo,hi=track['assigned_road_y_m'];edges=sorted({x,x+length,*[max(x,min(x+length,v)) for r in rectangles for v in r[:2]]})
        covered=[]
        for a,b in zip(edges,edges[1:]):
            ys=merge([[r[2],r[3]] for r in rectangles if r[0]<=a+1e-9 and r[1]>=b-1e-9])
            if any(c<=lo+1e-9 and d>=hi-1e-9 for c,d in ys):
                covered.append([a-x,b-x] if track['direction']==1 else [x+length-b,x+length-a])
        track['estimated_covered_along_m']=merge(covered)
        track['unverified_along_m']=complement(covered,length)
        # Original quality reasons remain evidence, not claims about all later sessions.
        track['source_quality_flags']=track.pop('quality_flags',track.get('source_quality_flags',[]))
    result['status']='NEEDS_RESCAN' if any(t['unverified_along_m'] for t in result['tracks']) else 'ESTIMATED_COMPLETE'
    result['merged_navigation_sha256']=sorted(identities)
    result['original_rescan_candidates']=original_candidates
    remaining={t['track_id']:t['unverified_along_m'] for t in result['tracks']}
    result['rescan_candidates']=[c for c in original_candidates if any(max(a,c['source_along_m'][0])<min(b,c['source_along_m'][1]) for a,b in remaining[c['source_track_id']])]
    result['scope']='Union of independently audited flat-road footprint rectangles; original quality reasons retained. No image stitching or pixel-level proof.'
    return result
