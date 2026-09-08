#!/usr/bin/env python3
"""Associate raw blocks with tracks and sparse fused/calibrated camera pose tags.

Rendering truth remains in original sensor metadata and is never a navigation
input to these production labels. No image correction or resampling occurs.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp


def label(mission,navigation):
    intervals=json.loads((mission/'capture_intervals.json').read_text())
    plan=json.loads((mission/'plan.json').read_text())
    nav=[json.loads(line) for line in (navigation/'navigation.jsonl').read_text().splitlines()]
    cal=json.loads((navigation/'calibration.json').read_text())
    frame=next(v for v in cal['estimated_frames'] if v['frame_id']=='camera_optical_calibrated')
    offset=np.array(frame['translation_m']);mount=Rotation.from_quat(frame['orientation_xyzw'])
    times=np.array([v['time_s'] for v in nav]);positions=np.array([v['position_m'] for v in nav])
    rotations=Rotation.from_quat([v['orientation_xyzw'] for v in nav]);slerp=Slerp(times,rotations)
    if any(v['frame_id']!='map' or v['child_frame_id']!='base_link' or v['calibration_id']!=cal['calibration_id'] for v in nav):raise ValueError('navigation frame/calibration changes')
    blocks=[]
    for interval in intervals:
        if 'disabled_ack_time_s' not in interval:raise ValueError('capture was not acknowledged closed')
        archive=Path(interval['archive'])
        for path in archive.glob('block_*.json'):
            raw=json.loads(path.read_text());first=raw['first']['time_s'];last=raw['last']['time_s']
            if not interval['enabled_ack_time_s']-.05<=first<=last<=interval['disabled_ack_time_s']+.05:continue
            tags=[]
            for tag in raw['pose_tags']:
                t=tag['time_s'];i=np.searchsorted(times,t)
                if i==0 or i==len(times) or times[i]-times[i-1]>.06:raise ValueError('pose tag lacks bounded interpolation bracket')
                ratio=(t-times[i-1])/(times[i]-times[i-1]);r=slerp([t])[0]
                p=positions[i-1]*(1-ratio)+positions[i]*ratio
                tags.append({'time_s':t,'global_line':tag['global_line'],'camera_position_map_m':(p+r.apply(offset)).tolist(),
                    'camera_orientation_map_xyzw':(r*mount).as_quat().tolist(),'navigation_bracket_s':[float(times[i-1]),float(times[i])]})
            blocks.append({'block_id':raw['block_id'],'track_id':interval['track_id'],'mission_id':plan['mission_id'],
                'image':str(archive/path.with_suffix('.pgm').name),'rows':raw['rows'],'width':raw['width'],
                'first_global_line':raw['first']['global_line'],'last_global_line':raw['last']['global_line'],
                'sensor_segment_id':raw['segment_id'],'pose_tags':tags,'reference':'last_line_exposure_midpoint',
                'pose_source':'fused_navigation_with_estimated_camera_extrinsic','calibration_id':cal['calibration_id']})
    if len({v['block_id'] for v in blocks})!=len(blocks):raise ValueError('ambiguous block/track intervals')
    result={'schema':'agv.mission.capture.v1','mission_id':plan['mission_id'],'blocks':sorted(blocks,key=lambda v:v['block_id']),
            'scope':'raw pass images include run-up/runout; sparse estimated tags; spatial coverage requires separate audit'}
    (mission/'capture_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--mission',type=Path,required=True);p.add_argument('--navigation',type=Path,required=True);a=p.parse_args()
    result=label(a.mission,a.navigation);print(json.dumps({'blocks':len(result['blocks']),'rows':sum(v['rows'] for v in result['blocks'])}))


if __name__=='__main__':main()
