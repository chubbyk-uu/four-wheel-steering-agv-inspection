#!/usr/bin/env python3
"""Independent navigation, motion-time and raw scan geometry comparison."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def metrics(root):
    mission=root/'mission';rows=[json.loads(x) for x in (mission/'execution.jsonl').read_text().splitlines()]
    running=[r for r in rows if r['state']=='RUNNING'];dt=np.diff([r['time_s'] for r in rows],append=rows[-1]['time_s'])
    duration={}
    for r,d in zip(rows,dt):
        if r['state']=='RUNNING':
            key=r.get('tracker_state','PREP');duration[key]=duration.get(key,0)+float(d)
    truth=np.load(root/'truth_evaluation.npy')
    nav=[json.loads(x) for x in (root/'navigation/navigation.jsonl').read_text().splitlines()]
    nav=[r for r in nav if max(truth[0,0],running[0]['time_s'])<=r['time_s']<=min(truth[-1,0],running[-1]['time_s'])];ts=np.array([r['time_s'] for r in nav])
    actual=np.column_stack([np.interp(ts,truth[:,0],truth[:,i]) for i in (1,2,3)])
    error=np.array([r['position_m'] for r in nav])-actual
    actual_yaw=np.interp(ts,truth[:,0],np.unwrap(Rotation.from_quat(truth[:,4:]).as_euler('xyz')[:,2]))
    yaw=np.unwrap(Rotation.from_quat([r['orientation_xyzw'] for r in nav]).as_euler('xyz')[:,2]);ye=np.arctan2(np.sin(yaw-actual_yaw),np.cos(yaw-actual_yaw))
    blocks=json.loads((mission/'capture_manifest.json').read_text())['blocks'];geometry=[]
    for b in blocks:
        if b['rows']!=4096:continue
        m=json.loads(Path(b['image']).with_suffix('.json').read_text());tags=m['pose_tags']
        p=np.array([t['camera_position_world_m'] for t in tags]);r=np.array([t['camera_rotation_world'] for t in tags]);heading=np.unwrap(np.arctan2(r[:,1,1],r[:,0,1]))
        detrended=p[:,1]-np.polyval(np.polyfit(p[:,0],p[:,1],1),p[:,0])
        geometry.append({'lateral_nonlinearity_mm':float(np.ptp(detrended)*1000),'block_id':b['block_id'],'track_id':b['track_id'],'lateral_span_mm':float(np.ptp(p[:,1])*1000),'heading_span_deg':float(np.degrees(np.ptp(heading)))})
    result={'elapsed_motion_sim_s':running[-1]['time_s']-running[0]['time_s'],
        'state_time_sim_s':duration,'xy_truth_rmse_m':float(np.sqrt(np.mean(np.sum(error[:,:2]**2,axis=1)))),
        'xy_truth_error_max_m':float(np.linalg.norm(error[:,:2],axis=1).max()),'yaw_truth_rmse_deg':float(np.degrees(np.sqrt(np.mean(ye**2)))),
        'global_xy_update_jump_p99_mm':float(np.percentile(np.linalg.norm(np.diff(np.array([r['position_m'][:2] for r in nav]),axis=0),axis=1),99)*1000),
        'full_block_geometry':geometry,'full_block_lateral_span_median_mm':float(np.median([g['lateral_span_mm'] for g in geometry])),
        'full_block_lateral_nonlinearity_median_mm':float(np.median([g['lateral_nonlinearity_mm'] for g in geometry])),
        'full_block_heading_span_median_deg':float(np.median([g['heading_span_deg'] for g in geometry])),
        'pass_reverse_command_samples':sum(r.get('kind')=='PASS' and r.get('active_kind')=='translate' and r.get('command_body',[0])[0]<-.001 for r in rows) if all('command_body' in r for r in rows) else None,
        'entry_steps':len([s for s in json.loads((mission/'steps.json').read_text()) if s['kind']=='ENTRY'])}
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--before',type=Path,required=True);p.add_argument('--after',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result={'before':metrics(a.before),'after':metrics(a.after),'scope':'observed runs with same noise settings; not identical noise realizations or guaranteed error bounds'}
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:{x:y for x,y in v.items() if x!='full_block_geometry'} for k,v in result.items() if isinstance(v,dict)},indent=2))
