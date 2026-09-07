#!/usr/bin/env python3
"""Run an isolated simulator and evaluator; keep complete logs outside the repo."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

p=argparse.ArgumentParser()
p.add_argument('--output', default='results/stage1_motion.json')
p.add_argument('--domain', type=int, default=72)
p.add_argument('--suite', choices=['basic','steering'], default='basic')
a=p.parse_args()
root=Path(__file__).resolve().parents[1]
tag=f'agv_stage1_{os.getpid()}'
env=dict(os.environ,ROS_DOMAIN_ID=str(a.domain),GZ_PARTITION=tag,ROS_LOG_DIR=f'/tmp/{tag}_ros')
logpath=Path('/tmp')/(tag+'_sim.log')
evalpath=Path('/tmp')/(tag+'_evaluation.log')
with logpath.open('w') as log, evalpath.open('w') as eval_log:
    sim=subprocess.Popen(['ros2','launch','agv_bringup','sim.launch.py','headless:=true'],env=env,stdout=log,stderr=log,start_new_session=True)
    try:
        evaluator='validate_motion.py' if a.suite=='basic' else 'validate_steering_sequences.py'
        result=subprocess.run(['python3',str(root/'tools'/evaluator),'--output',a.output],env=env,stdout=eval_log,stderr=eval_log,timeout=600)
        print(f'Evaluation exit: {result.returncode}; simulator log: {logpath}; evaluator log: {evalpath}')
        if result.returncode:raise SystemExit(result.returncode)
    finally:
        if sim.poll() is None:
            os.killpg(sim.pid,signal.SIGINT)
            try:sim.wait(timeout=20)
            except subprocess.TimeoutExpired:os.killpg(sim.pid,signal.SIGKILL);sim.wait()
