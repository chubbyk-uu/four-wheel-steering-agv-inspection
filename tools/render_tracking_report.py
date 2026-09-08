#!/usr/bin/env python3
"""Plot archived references, fused tracking and independent physical attitude."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('archive',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    fig,axes=plt.subplots(3,2,figsize=(12,9),layout='constrained')
    for col,name in enumerate(('forward','short_triangle')):
        directory=next(args.archive.glob('*'+name))
        rows=[json.loads(line) for line in (directory/'tracking.jsonl').read_text().splitlines()]
        t=np.array([r['time_s'] for r in rows]);origin=t[0];t-=origin
        request=json.loads((directory/'request.json').read_text())
        start=np.array(request['start_position_m']);goal=np.array(request['goal_position_m'])
        axis=(goal-start)/np.linalg.norm(goal-start)
        for key,label in (('reference_position_m','Time reference'),('position_m','Fused position')):
            axes[0,col].plot(t,(np.array([r[key] for r in rows])-start)@axis,label=label)
        axes[0,col].set_title(name.replace('_',' '));axes[0,col].set_ylabel('Along initial segment (m)')
        speed=[r.get('reference_speed',0) if r['state']=='RUNNING' and r['terminal_trims']==0 else np.nan for r in rows]
        axes[1,col].plot(t,speed,label='Primary reference speed (m/s)')
        axes[1,col].plot(t,[np.linalg.norm(r['command'][:2]) for r in rows],label='Command translation (m/s)',alpha=.7)
        axes[1,col].plot(t,[r['command'][2] for r in rows],label='Command yaw (rad/s)',alpha=.7)
        truth=np.load(directory/'truth_evaluation.npy')
        rp=np.degrees(Rotation.from_quat(truth[:,4:]).as_euler('xyz')[:,:2])
        axes[2,col].plot(truth[:,0]-origin,rp[:,0],label='Physical roll')
        axes[2,col].plot(truth[:,0]-origin,rp[:,1],label='Physical pitch')
        axes[2,col].set_ylabel('Attitude (degrees)');axes[2,col].set_xlabel('Elapsed simulation time (s)')
        for ax in axes[:,col]:
            ax.set_xlim(0,t[-1]);ax.grid(alpha=.25);ax.legend(fontsize=8)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=150)


if __name__=='__main__':main()
