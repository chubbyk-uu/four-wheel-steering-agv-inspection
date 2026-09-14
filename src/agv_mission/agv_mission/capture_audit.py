"""Read archived pixels and estimated navigation; do not read rendered pose truth."""
from agv_linescan.encoder import line_spacing
import math
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from scipy.spatial.transform import Rotation,Slerp
from .coverage import estimate,rescan_requests
from .planner import plan as make_plan,Vehicle
from .capture_quality import reconfiguration_windows,affected,trace_gaps


class Navigation:
    def __init__(self,directory):
        self.cal=json.loads((directory/'calibration.json').read_text())
        content=(directory/'navigation.jsonl').read_text();self.digest=hashlib.sha256(content.encode()).hexdigest()
        lines=[v for v in content.splitlines() if v.strip()];rows=[];self.torn_tail=False
        for index,line in enumerate(lines):
            try:rows.append(json.loads(line))
            except ValueError:
                # The adapter appends to this file from another process while the
                # audit reads it, so the final line can be half a record. The
                # readiness check already tolerates that; parsing has to agree, or
                # the audit is refused for a race the caller was told was over.
                # Any earlier line failing is corruption, not a race.
                if index!=len(lines)-1:raise ValueError('corrupt navigation record %d'%index)
                self.torn_tail=True
        if len(rows)<2:raise ValueError('not enough navigation samples')
        if any(r['frame_id']!='map' or r['child_frame_id']!='base_link' or r['calibration_id']!=self.cal['calibration_id'] for r in rows):raise ValueError('navigation frame/calibration changes')
        self.times=np.array([r['time_s'] for r in rows]);self.positions=np.array([r['position_m'] for r in rows])
        if not np.isfinite(self.times).all() or not np.isfinite(self.positions).all() or np.any(np.diff(self.times)<=0):raise ValueError('invalid navigation ordering/positions')
        self.rotations=Slerp(self.times,Rotation.from_quat([r['orientation_xyzw'] for r in rows]))
        self.covariances=np.array([r['pose_covariance'] for r in rows]).reshape(-1,6,6)
        frame=next(v for v in self.cal['estimated_frames'] if v['frame_id']=='camera_optical_calibrated')
        self.offset=np.array(frame['translation_m']);self.mount=Rotation.from_quat(frame['orientation_xyzw'])

    def footprint(self,tag,camera,source,uncertainty):
        t=tag['time_s'];i=np.searchsorted(self.times,t)
        if not np.isfinite(t) or i==0 or i==len(self.times) or self.times[i]-self.times[i-1]>.06:raise ValueError('navigation bracket missing')
        ratio=(t-self.times[i-1])/(self.times[i]-self.times[i-1])
        cov=self.covariances[i-1]*(1-ratio)+self.covariances[i]*ratio
        if not np.isfinite(cov).all() or np.any(np.diag(cov)<0):raise ValueError('invalid navigation covariance')
        cov=(cov+cov.T)/2
        if np.linalg.eigvalsh(cov).min()<-1e-8:raise ValueError('non-positive navigation covariance')
        body=self.rotations([t])[0];position=self.positions[i-1]*(1-ratio)+self.positions[i]*ratio
        road=source['request']['road'];road_rotation=Rotation.from_euler('z',road['yaw_rad']).inv()
        slope=camera['width']*camera['pixel_pitch_m']/(2*camera['focal_length_m'])
        def project(p,r,side):
            origin=p+r.apply(self.offset);matrix=(r*self.mount).as_matrix()
            ray=matrix[:,2]+side*slope*matrix[:,0]
            if ray[2]>=-1e-6:raise ValueError('camera ray does not face the flat road')
            distance=(road['origin_xyz_m'][2]-origin[2])/ray[2]
            if distance<=0:raise ValueError('camera below declared surface')
            return road_rotation.apply(origin+distance*ray-road['origin_xyz_m'])[:2]
        points=[];sigma=[];rpy=body.as_euler('xyz');eps=1e-5
        for side in (-1,1):
            points.append(project(position,body,side));jac=np.zeros((2,6))
            for j in range(6):
                d=np.zeros(3);d[j%3]=eps
                if j<3:plus=project(position+d,body,side);minus=project(position-d,body,side)
                else:plus=project(position,Rotation.from_euler('xyz',rpy+d),side);minus=project(position,Rotation.from_euler('xyz',rpy-d),side)
                jac[:,j]=(plus-minus)/(2*eps)
            sigma.append(3*np.sqrt(np.maximum(0,np.diag(jac@cov@jac.T))))
        return dict(global_line=int(tag['global_line']),time_s=t,road_x=[float(p[0]) for p in points],road_y=[float(p[1]) for p in points],
            longitudinal_sigma3_m=float(max(v[0] for v in sigma)),lateral_sigma3_m=float(max(v[1] for v in sigma)))



def audit_capture(mission,navigation,output,platform,camera,uncertainty=.10):
    mission,navigation,output=map(Path,(mission,navigation,output))
    if output.exists():raise FileExistsError(output)
    source=json.loads((mission/'plan.json').read_text());vehicle=Vehicle.from_configs(platform,camera)
    checked=make_plan(source['request'],vehicle)
    if source['frame_id']!='map' or source['tracks']!=checked['tracks'] or source['vehicle']!=checked['vehicle']:raise ValueError('source plan incompatible with current validated vehicle')
    nav=Navigation(navigation)
    intervals=json.loads((mission/'capture_intervals.json').read_text())
    execution=(mission/'execution.jsonl').read_bytes()
    records=[json.loads(line) for line in execution.splitlines()]
    if not records:raise ValueError('missing execution quality evidence')
    quality_windows=reconfiguration_windows(records)
    # The first and last record only bound the trace; a hole inside it hides whatever
    # happened there, including a steering reconfiguration during an open shutter.
    missing=trace_gaps(records)
    spans=[];issues=[];inputs=[];seen=set();scenes=set()
    for interval in intervals:
        if 'disabled_ack_time_s' not in interval:
            issues.append(dict(track_id=interval['track_id'],reason='capture close not acknowledged; interval excluded'));continue
        track_id=interval['track_id'];track=source['tracks'][track_id];run=[];previous=None
        def finish():
            nonlocal run
            if run:spans.append(dict(track_id=track_id,points=run));run=[]
        archive=Path(interval['archive'])
        try:
            raw_camera=yaml.safe_load((archive/'calibration.yaml').read_text())
            if raw_camera['calibration_id'] not in (nav.cal['optical_intrinsic_id'],nav.cal['optical_intrinsic_id']+'-optix-strip-v1'):raise ValueError('sensor/navigation intrinsic ID mismatch')
            # A session with only discarded tails still has a known scene identity.
            manifest=archive/'scene_manifest.json'
            if manifest.exists():
                scene=json.loads(manifest.read_text())
                scenes.add(hashlib.sha256(json.dumps(scene,sort_keys=True,separators=(',',':')).encode()).hexdigest())
            for key in ('width','pixel_pitch_m','focal_length_m','nominal_width_m','ray_polynomial'):
                if raw_camera[key]!=camera[key]:raise ValueError('optical configuration changed: '+key)
        except (OSError,ValueError,KeyError) as exc:
            issues.append(dict(track_id=track_id,reason=str(exc)));continue
        for path in sorted(archive.glob('block_*.json')):
            try:
                raw=json.loads(path.read_text());first=raw['first'];last=raw['last']
                if not interval['enabled_ack_time_s']-.05<=first['time_s']<=last['time_s']<=interval.get('disabled_ack_time_s',float('inf'))+.05:continue
                identity=str(path.resolve())
                if identity in seen:raise RuntimeError('ambiguous capture intervals; cannot assign a block to two passes')
                seen.add(identity)
                im=path.with_suffix('.pgm')
                with Image.open(im) as pixels:
                    if pixels.mode!='L' or pixels.size!=(camera['width'],raw['rows']):raise ValueError('pixel dimensions or encoding mismatch')
                    pixels.load()
                if raw['calibration_id']!=raw_camera['calibration_id'] or raw.get('invalid_pixels',0)!=0:raise ValueError('invalid pixels/calibration')
                if not 1<=raw['rows']<=raw_camera['block_rows'] or last['global_line']-first['global_line']+1!=raw['rows']:raise ValueError('invalid block line range')
                if not math.isclose(raw['line_spacing_m'],line_spacing(camera,platform['wheel_radius']),rel_tol=0,abs_tol=1e-12) or raw.get('encoding','mono8')!='mono8':raise ValueError('trigger spacing or encoding changed')
                if any(not first['time_s']<=t['time_s']<=last['time_s'] for t in raw['pose_tags']):raise ValueError('tag time outside frame')
                tags={t['global_line']:t for t in raw['pose_tags']};tags[first['global_line']]=first;tags[last['global_line']]=last
                tags=[tags[k] for k in sorted(tags)]
                if any(not first['global_line']<=t['global_line']<=last['global_line'] for t in tags):raise ValueError('tag outside block')
                if previous is None or raw['segment_id']!=previous['segment_id'] or first['global_line']!=previous['last']['global_line']+1:finish()
                if raw.get('scene_contract'):
                    scenes.add(hashlib.sha256(json.dumps(raw['scene_contract'],sort_keys=True,separators=(',',':')).encode()).hexdigest())
                inputs.append(dict(track_id=track_id,block_id=raw['block_id'],image=im.name,image_sha256=hashlib.sha256(im.read_bytes()).hexdigest(),metadata_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
                inputs[-1]['motion_quality_excluded']=affected(quality_windows,track_id,first['time_s'],last['time_s'])
                if inputs[-1]['motion_quality_excluded']:
                    raise ValueError('STEERING_DURING_CAPTURE: raw block retained; excluded from verified coverage')
                if records[0]['time_s']>first['time_s'] or records[-1]['time_s']<last['time_s']:
                    inputs[-1]['motion_quality_excluded']=True
                    raise ValueError('EXECUTION_TRACE_INCOMPLETE: raw block retained; motion quality unknown')
                if any(g['start_s']<last['time_s'] and g['end_s']>first['time_s'] for g in missing):
                    inputs[-1]['motion_quality_excluded']=True
                    raise ValueError('EXECUTION_TRACE_GAP: raw block retained; motion quality unknown')
                for tag in tags:
                    try:
                        point=nav.footprint(tag,camera,source,uncertainty)
                        if run:
                            along=(np.mean(point['road_x'])-np.mean(run[-1]['road_x']))*track['direction']
                            encoder=(point['global_line']-run[-1]['global_line'])*raw['line_spacing_m']
                            if point['time_s']<=run[-1]['time_s'] or abs(along-encoder)>2*uncertainty+.05*encoder:raise ValueError('pose/encoder span inconsistent')
                        run.append(point)
                    except ValueError as exc:
                        finish();issues.append(dict(track_id=track_id,block_id=raw['block_id'],global_line=tag['global_line'],reason=str(exc)))
                previous=raw
            except (OSError,ValueError,KeyError) as exc:
                finish();previous=None;issues.append(dict(track_id=track_id,block=path.name,reason=str(exc)))
        finish()
    report=estimate(source,spans,uncertainty);report.update(request=source['request'],scene_contract_sha256=sorted(scenes),optical_intrinsic_id=nav.cal['optical_intrinsic_id'],issues=issues,inputs=inputs,navigation_calibration_id=nav.cal['calibration_id'],
        provenance=dict(plan_sha256=hashlib.sha256((mission/'plan.json').read_bytes()).hexdigest(),navigation_sha256=nav.digest,
            navigation_calibration_sha256=hashlib.sha256((navigation/'calibration.json').read_bytes()).hexdigest()))
    report['motion_quality_windows']=quality_windows
    report['execution_trace_gaps']=missing
    report['provenance']['execution_sha256']=hashlib.sha256(execution).hexdigest()
    candidates=rescan_requests(source,report,vehicle)
    report['rescan_candidates']=[{k:v for k,v in c.items() if k!='plan'} for c in candidates]
    report['assumptions']=['current validated flat-road model','minimum total footprint allowance, enlarged by propagated 3-sigma marginal EKF uncertainty; unknown systematic errors are not certified','continuous valid sensor segment between sparse tags; cannot prove arbitrary unobserved motion']
    output.mkdir(parents=True)
    (output/'coverage.json').write_text(json.dumps(report,indent=2)+'\n')
    for i,candidate in enumerate(candidates):
        if candidate['status']!='PREVIEW_ONLY':continue
        (output/f'rescan_{i:03d}.yaml').write_text(yaml.safe_dump(candidate['request'],sort_keys=False))
        (output/f'rescan_{i:03d}.plan.json').write_text(json.dumps(candidate['plan'],indent=2)+'\n')
    return report
