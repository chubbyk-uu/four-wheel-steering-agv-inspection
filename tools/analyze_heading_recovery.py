#!/usr/bin/env python3
"""Audit body yaw during wheel alignment and subsequent capture admission."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image


def analyze(path):
    records=[json.loads(line) for line in (path/'mission/execution.jsonl').read_text().splitlines()]
    truth=np.load(path/'truth_evaluation.npy');joints=np.load(path/'joint_evaluation.npy')
    result=json.loads((path/'results.json').read_text())
    output={'passed':result['passed'],'final_state':result['final']['state'],
            'reason':result['final']['reason'],'final_motion':result['final']['motion_state'],
            'passes':[],'capture':result.get('capture'),
            'pause_resume':result.get('pause_resume'),'wheel_order':['fl','fr','rl','rr']}
    for track in sorted({r['track_id'] for r in records if r.get('kind')=='PASS'}):
        rows=[r for r in records if r.get('kind')=='PASS' and r.get('track_id')==track
              and r.get('segment_kind')=='translate']
        drive=next((r for r in rows if r.get('motion_state')=='DRIVE'),None)
        if drive is None:continue
        times=[rows[0]['time_s'],drive['time_s']]
        poses=np.array([truth[np.argmin(abs(truth[:,0]-t))] for t in times])
        yaw=Rotation.from_quat(poses[:,4:8]).as_euler('xyz')[:,2]
        angles=np.array([joints[np.argmin(abs(joints[:,0]-t)),1:] for t in times])
        active=[r for r in rows if r.get('capture_active') and r.get('reference_heading_error_rad') is not None]
        first=active[0] if active else None
        output['passes'].append(dict(track_id=track,align_seconds=times[1]-times[0],
            align_truth_yaw_change_deg=float(np.rad2deg(np.arctan2(np.sin(yaw[1]-yaw[0]),np.cos(yaw[1]-yaw[0])))),
            align_truth_translation_m=float(np.linalg.norm(poses[1,1:4]-poses[0,1:4])),
            wheel_start_end_deg=np.rad2deg(angles).tolist(),wheel_travel_deg=np.rad2deg(angles[1]-angles[0]).tolist(),
            drive_start_heading_error_rad=drive.get('reference_heading_error_rad'),
            drive_start_time_s=times[1],
            capture_delay_after_drive_s=first['time_s']-times[1] if first else None,
            first_capture_heading_error_rad=first.get('reference_heading_error_rad') if first else None,
            max_active_heading_error_rad=max((abs(r['reference_heading_error_rad']) for r in active),default=None),
            reverse_command_samples=sum(r['command_body'][0]<-1e-9 for r in rows)))
    return output


def main():
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,action='append',required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--preview',type=Path)
    a=p.parse_args();report={path.name:analyze(path) for path in a.run}
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    if a.preview:
        # Actual raw images, fixed brightness and equal pixel aspect; no pose warp.
        files=sorted(a.run[-1].glob('raw/*/block_*.pgm'))
        thumbs=[]
        for file in files:
            with Image.open(file) as im:
                im.thumbnail((256,256));thumbs.append(im.convert('RGB').copy())
        canvas=Image.new('RGB',(256*min(6,len(thumbs)),280*((len(thumbs)+5)//6)),(35,35,35))
        from PIL import ImageDraw
        d=ImageDraw.Draw(canvas)
        for i,im in enumerate(thumbs):
            x=i%6*256;y=i//6*280;canvas.paste(im,(x,y));d.text((x+4,y+257),files[i].stem,fill='white')
        canvas.save(a.preview)
    print(json.dumps(report))


if __name__=='__main__':main()
