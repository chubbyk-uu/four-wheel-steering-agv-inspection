#!/usr/bin/env python3
"""Summarize real mission shifts and isolated post-brake roll traces."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def mission_case(root,damping):
    path=root/f'run_{damping}'
    result=json.loads((path/'results.json').read_text())
    truth=np.load(path/'truth_evaluation.npy');t=truth[:,0]
    roll=np.degrees(Rotation.from_quat(truth[:,4:8]).as_euler('xyz')[:,0])
    records=[json.loads(line) for line in (path/'mission/execution.jsonl').read_text().splitlines()]
    shift=[row for row in records if row.get('kind')=='SHIFT']
    align=next(row['time_s'] for row in shift if row.get('tracker_state')=='ALIGNING')
    moving=next(row['time_s'] for row in shift if row.get('tracker_state')=='RUNNING')
    stopping=next(row['time_s'] for row in shift if row.get('tracker_state')=='STOPPING')
    rotate=next(row['time_s'] for row in records if row.get('kind')=='ROTATE_180')
    baseline=float(np.mean(roll[(t>=align-.5)&(t<align)]));relative=roll-baseline
    def values(lo,hi):return relative[(t>=lo)&(t<hi)]
    move=values(moving,stopping);stop=values(stopping,rotate)
    return dict(passed=result['passed'],align_duration_s=moving-align,move_duration_s=stopping-moving,
        stop_to_rotate_s=rotate-stopping,move_roll_peak_to_peak_deg=float(np.ptp(move)),
        stop_roll_peak_to_peak_deg=float(np.ptp(stop)),stop_end_roll_offset_deg=float(np.mean(stop[-5:])),
        track_reference_error_max_m=[track['reference_error_max_m'] for track in result['tracks']])


def isolated_case(root,damping,axis):
    path=root/f'direct_{damping}'
    summary=json.loads((path/'summary.json').read_text())
    captured=json.loads((path/'samples.json').read_text());rows=captured['rows']
    for run in summary['runs']:
        numeric=np.asarray([row[:11] for row in rows if row[10]==run['index']],float)
        prior=np.asarray([row[:11] for row in rows if row[10]<run['index'] and row[11]=='rest'],float)
        baseline=float(np.mean(np.degrees(prior[-50:,6])))
        axis.plot(numeric[:,0]-run['brake_time_s'],np.degrees(numeric[:,6])-baseline,
                  label=f'{damping}, direction {int(run["direction"]):+d}',alpha=.82)
    keys=('mean_move_roll_peak_to_peak_deg','mean_post_hold_peak_abs_roll_deg',
          'mean_post_hold_extrema_count','mean_settle_within_0_05deg_s')
    result={key:summary[key] for key in keys}
    result['max_suspension_abs_m']=max(run['max_suspension_abs_m'] for run in summary['runs'])
    result['repeat_spread_peak_deg']=float(np.ptp([run['post_hold_peak_abs_roll_deg'] for run in summary['runs']]))
    return result


def acceleration_case(root):
    report=json.loads((root/'accel_1200/summary.json').read_text())
    cases=report['cases']
    return dict(
        passed=report['passed'],
        repeats=len(cases),
        sustained_accel_pitch_abs_mean_deg=float(np.mean([
            abs(case['phases']['accel']['active_pitch_median_deg']) for case in cases])),
        sustained_brake_pitch_abs_mean_deg=float(np.mean([
            abs(case['phases']['brake']['active_pitch_median_deg']) for case in cases])),
        accel_peak_abs_max_deg=max(abs(case['phases']['accel']['pitch_offset_min_deg']) for case in cases),
        brake_peak_abs_max_deg=max(abs(case['phases']['brake']['pitch_offset_max_deg']) for case in cases),
        settle_to_0_01deg_max_s=max(case['settle_after_vx_below_0_02_to_0_01deg_s'] for case in cases),
        maximum_suspension_abs_m=max(case['max_suspension_abs_m'] for case in cases))


def gui_case(path):
    result=json.loads((path/'results.json').read_text())
    final=result['final']
    log=(path/'simulation.log').read_text(errors='replace')
    return dict(profile=result['profile'],gui_ready='Gazebo GUI readiness confirmed' in log,
        motion_observed=any(track['running_samples'] for track in result['tracks']),
        final_state=final['state'],final_reason=final['reason'],final_motion_state=final['motion_state'],
        second_track_heading_error_deg=float(np.degrees(final.get('reference_heading_error_rad',float('nan')))))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--figure',type=Path,required=True);a=parser.parse_args()
    fig,ax=plt.subplots(figsize=(9,5.2));report=dict(schema='agv.lateral_damping_comparison.v1',date='2026-09-15',
        constants=dict(mass_kg=550,suspension_stiffness_n_m=8000,travel_m=.05,
            drive_accel_m_s2=.8,drive_decel_m_s2=1.,track_shift_m=1.),cases={})
    for damping in (750,1000,1200):
        report['cases'][str(damping)]=dict(isolated=isolated_case(a.input,damping,ax),mission=mission_case(a.input,damping))
    baseline=report['cases']['750']['isolated'];candidate=report['cases']['1200']['isolated']
    report['candidate_1200_vs_750_percent']=dict(
        post_hold_peak_abs_roll=100*(candidate['mean_post_hold_peak_abs_roll_deg']/baseline['mean_post_hold_peak_abs_roll_deg']-1),
        move_roll_peak_to_peak=100*(candidate['mean_move_roll_peak_to_peak_deg']/baseline['mean_move_roll_peak_to_peak_deg']-1),
        max_suspension_travel=100*(candidate['max_suspension_abs_m']/baseline['max_suspension_abs_m']-1))
    report['candidate_1200_acceleration_check']=acceleration_case(a.input)
    report['candidate_1200_gui_checks']=[gui_case(a.input/name) for name in ('gui_1200','gui_1200_zero_fixed')]
    report['recommendation']='1200 N s/m was selected as the default after user review: one prominent post-HOLD extremum instead of three, 18% lower post-HOLD peak and 19% lower maximum suspension travel. It retains the requested 1-2 degree sustained acceleration and braking pitch. GUI/RViz motion was displayed, but both two-track capture attempts stopped loudly at the second-track heading gate; that separate heading-entry regression remains open.'
    report['limitations']=['One zero-noise rectangle mission per damping value; two opposite repeats in the isolated test.',
        'The isolated brake trigger uses controller velocity commands and a fixed six-second observation; it is diagnostic rather than the mission position tracker.',
        'The mission starts ROTATE_180 after different STOPPING durations, so stop-to-rotate duration is reported and is not treated as pure damping response.',
        'Both 1200 N s/m GUI checks reached visible motion but faulted on CAPTURE_NOT_ACTIVE_AT_REGION after the turn; they are visual-motion evidence, not successful two-track capture acceptance.']
    ax.axvline(0,color='black',lw=1,label='zero command');ax.axhline(0,color='0.5',lw=.7)
    ax.set(xlabel='Simulation time from brake command (s)',ylabel='Body roll relative to rest (deg)',xlim=(-1.4,4.0))
    ax.grid(alpha=.25);ax.legend(ncol=2,fontsize=8);fig.tight_layout();a.figure.parent.mkdir(parents=True,exist_ok=True);fig.savefig(a.figure,dpi=160);plt.close(fig)
    a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['candidate_1200_vs_750_percent']))


if __name__=='__main__':main()
