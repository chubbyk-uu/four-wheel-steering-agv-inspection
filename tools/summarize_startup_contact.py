#!/usr/bin/env python3
"""Summarise matched startup probes; do not infer tyre slip from camera travel."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def summarize(path):
    info=json.loads((path/'probe.json').read_text())
    docs=list(path.rglob('*startup_probe.json'));assert len(docs)==1
    records=json.loads(docs[0].read_text())['records']
    t=np.array([r['t'] for r in records]);xyz=np.array([r['body_xyz'] for r in records]);cam=np.array([r['camera_xyz'] for r in records])
    pitch=Rotation.from_quat([r['body_quat_xyzw'] for r in records]).as_euler('xyz')[:,1]
    rates=np.array([r['wheel_speed_m_s'][0] for r in records]);start=info['commands'][0]['t'];brake=info['commands'][1]['t']
    windows={}
    for name,begin,end in [('startup_first_second',start,start+1),('acceleration',start,start+3.6),('cruise',start+5,brake),('brake',brake,brake+3.6)]:
        idx=np.flatnonzero((t>=begin)&(t<=end));assert len(idx)>200
        i,j=idx[0],idx[-1];enc=float(np.trapz(rates[idx],t[idx]));body=float(xyz[j,0]-xyz[i,0]);camera=float(cam[j,0]-cam[i,0]);delta=float(pitch[j]-pitch[i])
        # Small-angle diagnostic only: a body origin above the rolling reference
        # moves horizontally as it pitches, while the axle encoder is relative
        # to that pitching body. The contact velocity is the primary slip test.
        rocking=float(-np.mean(xyz[idx,2])*delta)
        wheels=[]
        for w in range(4):
            peaks=[];means=[];available=total=0;missing=0
            for k in idx:
                contact=records[k]['wheel_contact'][w];points=contact['points'];vv=[p['tangential_speed_m_s'] for p in points if p['velocity_available']]
                total+=len(points);available+=len(vv)
                if vv:peaks.append(max(vv));means.append(float(np.mean(vv)))
                else:missing+=1;peaks.append(0);means.append(0)
            assert available==total and total>0,'contact velocity evidence incomplete'
            wheels.append({'wheel':['fl','fr','rl','rr'][w], 'points':total,'velocity_points':available,'no_contact_samples':missing,
                           'tangential_speed_max_m_s':max(peaks),'tangential_speed_p99_m_s':float(np.percentile(peaks,99)),
                           'observed_absolute_tangent_travel_m':float(np.trapz(means,t[idx])),
                           'travel_excludes_no_contact_samples':True})
        windows[name]={'samples':len(idx),'encoder_m':enc,'body_reference_x_m':body,'camera_centre_x_m':camera,
                       'body_over_encoder':body/enc if abs(enc)>1e-6 else None,'camera_over_encoder':camera/enc if abs(enc)>1e-6 else None,
                       'encoder_minus_body_m':enc-body,'body_pitch_delta_deg':delta*180/np.pi,
                       'small_angle_rocking_estimate_m':rocking,'unexplained_after_rocking_m':enc-body-rocking,'wheel_contact':wheels}
    return {'passed':info['passed'],'max_physics_gap_s':float(np.diff(t).max()),'windows':windows,'final_state':info['final_state']}

def main():
    p=argparse.ArgumentParser();p.add_argument('--runs',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    results={}
    for kind in ('mesh','heightmap'):
        for n in (1,2,3):results[f'{kind}_{n}']=summarize(a.runs/f'{kind}_formal_{n}')
    assert all(r['passed'] and r['max_physics_gap_s']<.00101 for r in results.values())
    report={'schema':'agv.startup_contact_ab.v1','scope':'Six independent headless straight probes at x=6,y=0,yaw=0; same 550kg/1200 damping/300 mount, aligned stationary capture enable, stamped body commands, unchanged 0.8 m/s² controller acceleration, 10km/h cruise, braking to HOLD. Same OptiX visual road; only road collision representation differs.',
            'encoder_distance_definition':'Integral of archived fl shaft peripheral speed using the plugin nominal radius; continuous kinematic diagnostic, not a recount of AB edges or emitted image rows.',
            'contact_velocity_definition':'Native GZ wheel world linear velocity + angular velocity cross (contact point - wheel link origin), projected tangent to contact normal. Other collision must belong to a static road model. This is the primary mechanical slip evidence.',
            'limitations':['No force aggregation is used.','Small-angle height*pitch estimate is explanatory, not an exact terrain/rolling reconstruction.','Absolute tangent travel covers observed contact samples only; no-contact gaps are reported separately.','This is a startup diagnostic, not full-area repeat acceptance or a real-tyre validation.'], 'runs':results}
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    for name,r in results.items():
        w=r['windows']['startup_first_second'];print(name,'encoder-body mm %.3f / rocking %.3f / remainder %.3f'%(w['encoder_minus_body_m']*1000,w['small_angle_rocking_estimate_m']*1000,w['unexplained_after_rocking_m']*1000),'max tangent mm/s %.3f'%(1000*max(x['tangential_speed_max_m_s'] for x in w['wheel_contact'])))
if __name__=='__main__':main()
