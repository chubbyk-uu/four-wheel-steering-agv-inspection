#!/usr/bin/env python3
"""Summarize the matched mesh/FCL/native-heightmap contact experiment."""
import argparse,json
from pathlib import Path

import numpy as np


def last_block(root):
    files=sorted(root.glob('raw/session_cpp_*/block_*.json'))
    return files,json.loads(files[-1].read_text()) if files else None


def run(root):
    result=json.loads((root/'results.json').read_text());clock=np.load(root/'clock_evaluation.npy')
    files,block=last_block(root)
    out={'passed':result['passed'],'end_reason':result['final']['reason'],
         'mission_simulation_time_s':result['final']['time_s'],'blocks':len(files),
         'rtf':float((clock[-1,0]-clock[0,0])/(clock[-1,1]-clock[0,1])),
         'capture':result.get('capture',[])}
    if block:
        out['residual_statistics']={k:block['scan_residual_statistics'][k] for k in
          ('samples','max_m_s','p95_m_s','p99_m_s','over_half_limit','over_limit')}
        wheels=[]
        for value in block['capture_contact_statistics']:
            samples=value['contact_available_samples']
            wheels.append({
              'wheel':value['wheel'],'capturing_samples':value['capturing_samples'],
              'no_contact_samples':value['no_contact_samples'],
              'no_contact_fraction':value['no_contact_samples']/samples,
              'no_contact_episodes':value['no_contact_episodes'],
              'longest_no_contact_samples':value['longest_no_contact_samples'],
              'max_normal_tilt_deg':float(np.degrees(value['max_normal_tilt_rad'])),
              'max_depth_m':value['max_depth_m'],
              'max_suspension_rate_m_s':value['max_suspension_rate_m_s']})
        out['capture_contact_statistics']=wheels
    flights=list(root.glob('raw/session_cpp_*/flight_*_unsupported_scan_motion.json'))
    if flights:
        doc=json.loads(flights[-1].read_text());last=doc['records'][-1]
        out['fault_evidence']={'simulation_time_s':doc['fault_simulation_time_s'],
          'residual_m_s':last['residual'],'wheel_residual_m_s':last['wheel_residual_m_s'],
          'contact_points':[w['point_count'] for w in last['wheel_contact']],
          # Diagnostic only: the force field's aggregation semantics are not a load metric.
          'max_reported_point_force_magnitude_n':max(w['max_force_magnitude_n'] for w in last['wheel_contact']),
          'max_depth_m':max(w['max_depth_m'] for w in last['wheel_contact'])}
    return out


def resampling_error(source,samples):
    field=json.loads(source.read_text());x=np.asarray(field['x']);y=np.asarray(field['y']);z=np.asarray(field['z'])
    tx=np.linspace(x[0],x[-1],samples);ty=np.linspace(y[0],y[-1],samples)
    along_x=np.asarray([np.interp(tx,x,row) for row in z])
    raster=np.asarray([np.interp(ty,y,along_x[:,i]) for i in range(samples)]).T
    along_y=np.asarray([np.interp(y,ty,raster[:,i]) for i in range(samples)]).T
    reconstructed=np.asarray([np.interp(x,tx,row) for row in along_y])
    mx=(x>=-7)&(x<=22);my=(y>=2)&(y<=5);error=(reconstructed-z)[np.ix_(my,mx)]
    gx0=np.max(np.abs(np.diff(z,axis=1)/np.diff(x)))
    gy0=np.max(np.abs(np.diff(z,axis=0)/np.diff(y)[:,None]))
    gx1=np.max(np.abs(np.diff(reconstructed,axis=1)/np.diff(x)))
    gy1=np.max(np.abs(np.diff(reconstructed,axis=0)/np.diff(y)[:,None]))
    return {'samples_xy':[samples,samples],
      'cell_size_m':[(x[-1]-x[0])/(samples-1),(y[-1]-y[0])/(samples-1)],
      'mission_window_height_error_max_m':float(np.max(np.abs(error))),
      'mission_window_height_error_rms_m':float(np.sqrt(np.mean(error*error))),
      'source_max_abs_slope_xy':[float(gx0),float(gy0)],
      'reconstructed_max_abs_slope_xy':[float(gx1),float(gy1)]}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    cases={name:run(args.root/directory) for name,directory in {
      'ode_triangle_mesh':'run_ode_mesh_100m','fcl_triangle_mesh':'run_fcl_mesh_100m',
      'ode_heightmap_2049':'run_ode_heightmap_100m','ode_heightmap_1025':'run_ode_heightmap_100m_1025'}.items()}
    source=Path('assets/road/runtime_fullwidth_100m_rough_marked_3mm_v2/road_heightfield.json')
    report={'schema':'agv.contact_heightmap_ab.v1','date':'2026-09-16',
      'scope':'Matched 20 x 2 m, two-track, 10 km/h zero-noise missions on one 100 m visual/OptiX road and one +/-3 mm source heightfield; WheelSlip and tyre compliance disabled.',
      'sampling_note':'The successful-run contact counters cover camera-active physical steps. Different lead-in readiness changes their counts, so compare fractions and consecutive duration, not raw totals.',
      'force_note':'Contact force magnitudes are retained only as fault diagnostics; their point/substep/tangential aggregation is not defined and they are not compared with vehicle weight.',
      'source_heightfield_resampling':{'ode_heightmap_1025':resampling_error(source,1025),'ode_heightmap_2049':resampling_error(source,2049)},
      'cases':cases,
      'conclusion':('FCL is rejected: it faults before the first full block with a 0.318 m/s residual and a severely overconstrained manifold. '
        'A single 2049-square ODE heightmap improves contact but reduces RTF to about 0.555. The 1025-square ODE heightmap keeps RTF near 0.999, '
        'reduces no-contact samples from about 2 percent to about 0.3 percent, bounds gaps to one 1 ms step, and removes the near-horizontal mesh normal. '
        'Its 11.6 cm longitudinal cell size introduces at most 0.070 mm height error in the tested mission window. It is the recommended collision representation pending repeated and full-area regression.')}
    args.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
