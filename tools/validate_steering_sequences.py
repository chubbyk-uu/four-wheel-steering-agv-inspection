#!/usr/bin/env python3
"""Additional GZ regression for diagonal reversal and sustained steering travel."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from std_msgs.msg import String
from validate_motion import Evaluator


class SteeringEvaluator(Evaluator):
    def __init__(self):
        super().__init__()
        self.reason = ''
        self.create_subscription(String, '/motion_transition_reason', self.on_reason, 10)
        self.recording = False
        self.sequence_max_angle = 0.0
        self.sequence_max_step = 0.0
        self.sequence_brakes = 0
        self.limit_reasons = False
        self.last_recorded = None
        self.steering = [0.0]*4
        self.reference = [0.0]*4
        self.deviation = 0.0

    def on_reason(self, m):
        self.reason = m.data
        if self.recording and m.data == 'LIMIT_RECONFIGURE':
            self.limit_reasons = True

    def on_state(self, m):
        if self.recording and m.data == 'BRAKE' and self.state != 'BRAKE':
            self.sequence_brakes += 1
        super().on_state(m)

    def on_joints(self, m):
        super().on_joints(m)
        self.steering = [p for n,p in zip(m.name,m.position) if n.endswith('_steer_joint')]
        if self.recording:
            self.sequence_max_angle = max(self.sequence_max_angle, max(map(abs,self.steering)))
            self.deviation = max(self.deviation,max(abs(a-b) for a,b in zip(self.steering,self.reference)))
            if self.last_recorded is not None:
                self.sequence_max_step = max(self.sequence_max_step,max(abs(a-b) for a,b in zip(self.steering,self.last_recorded)))
            self.last_recorded = self.steering.copy()

    def begin_recording(self):
        self.sequence_max_angle = 0.0
        self.sequence_max_step = 0.0
        self.sequence_brakes = 0
        self.limit_reasons = False
        self.last_recorded = None
        self.reference = self.steering.copy()
        self.deviation = 0.0
        self.recording = True

    def evaluate_sequences(self):
        deadline = time.monotonic()+60
        while not (self.odom and self.joints and self.state):
            if time.monotonic()>deadline:raise RuntimeError('simulation not ready')
            rclpy.spin_once(self,timeout_sec=.1)
        result={'passed':False,'diagonal_reversal':[],'continuous_sweeps':[]}
        self.run_for(1,(0,0,0))
        for degrees in [45,-60]:
            h=math.radians(degrees)
            forward=(.35*math.cos(h),.35*math.sin(h),0)
            self.run_for(5,forward)
            assert self.state=='DRIVE',self.state
            self.begin_recording()
            self.run_for(3,tuple(-x for x in forward))
            self.recording=False
            assert self.deviation<.035,self.deviation
            v=self.odom.twist.twist.linear
            assert v.x*forward[0]+v.y*forward[1]<-.08,(v,forward)
            result['diagonal_reversal'].append({'heading_deg':degrees,'max_steering_change_rad':self.deviation})
        for direction in [1,-1]:
            self.run_for(5,(.25,0,0))
            assert self.state=='DRIVE',self.state
            self.begin_recording()
            start=self.get_clock().now().nanoseconds/1e9
            deadline=time.monotonic()+240
            while True:
                elapsed=self.get_clock().now().nanoseconds/1e9-start
                if elapsed>=22:break
                if time.monotonic()>deadline:raise RuntimeError('sweep timed out')
                h=direction*.2*elapsed
                self.command((.25*math.cos(h),.25*math.sin(h),0))
                rclpy.spin_once(self,timeout_sec=.01)
            self.recording=False
            row={'direction':direction,'brake_events':self.sequence_brakes,
                 'limit_reconfiguration_seen':self.limit_reasons,
                 'max_abs_steering_rad':self.sequence_max_angle,
                 'max_sample_steering_step_rad':self.sequence_max_step,'end_state':self.state}
            assert self.limit_reasons and self.sequence_brakes>=1,row
            assert self.sequence_max_angle<math.radians(185),row
            assert self.sequence_max_step<.10,row
            assert self.state=='DRIVE',row
            result['continuous_sweeps'].append(row)
        self.run_for(2,(0,0,0))
        result['passed']=True
        return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);a=p.parse_args()
    rclpy.init();n=SteeringEvaluator()
    try:
        result=n.evaluate_sequences()
        Path(a.output).write_text(json.dumps(result,indent=2)+'\n')
        print('PASS: diagonal reversals and both continuous steering sweeps.')
    finally:
        n.command((0,0,0));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
