#!/usr/bin/env python3
"""Observe actual steering through stopped lateral/spin/straight transitions."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from std_msgs.msg import String
from validate_motion import Evaluator


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    platform = yaml.safe_load(Path('src/agv_description/config/platform.yaml').read_text())
    policy = yaml.safe_load(Path('src/agv_bringup/config/motion.yaml').read_text())['swerve_controller']['ros__parameters']
    soft = platform['steer_soft_limit']; reserve = policy['steering_limit_reserve']
    rclpy.init(); node = Evaluator(); reasons = set(); report = {'passed': False, 'cases': []}
    node.create_subscription(String, '/motion_transition_reason', lambda m: reasons.add(m.data), 20)

    def angles():
        values = dict(zip(node.joints.name, node.joints.position))
        return [values[n+'_steer_joint'] for n in ('fl', 'fr', 'rl', 'rr')]

    def settle(command):
        deadline = time.monotonic()+45; stable = None
        expected = 'HOLD' if not any(command) else 'DRIVE'
        while True:
            node.command(command); rclpy.spin_once(node, timeout_sec=.02)
            now = node.get_clock().now().nanoseconds/1e9
            if node.state == expected:
                if stable is None: stable = now
                if now-stable >= .5: return
            else: stable = None
            assert time.monotonic() < deadline, (command, node.state)

    try:
        deadline = time.monotonic()+60
        while not (node.joints and node.odom and node.state):
            rclpy.spin_once(node, timeout_sec=.05)
            assert time.monotonic() < deadline
        settle((0,0,0))
        for label, lateral, spin in [('straight_spin',0.,.3), ('left_shift_spin',.25,.3), ('right_shift_spin',-.25,-.3)]:
            settle((.25,0,0)); settle((0,0,0))
            if lateral: settle((0,lateral,0)); settle((0,0,0))
            start = angles()
            settle((0,0,spin)); node.run_for(math.pi/abs(spin), (0,0,spin)); settle((0,0,0))
            before = angles(); reasons.clear()
            settle((.25,0,0)); after = angles(); observed = sorted(reasons)
            settle((0,0,0))
            wheels = []
            for old, new in zip(before, after):
                candidates = [k*math.pi for k in range(-2,3) if abs(k*math.pi) <= soft]
                nearest = min(candidates, key=lambda a: abs(a-old))
                allowed = [a for a in candidates if abs(a) <= soft-reserve]
                expected = min(allowed, key=lambda a: abs(a-old))
                wheels.append(dict(before_deg=math.degrees(old), after_deg=math.degrees(new),
                    travel_deg=math.degrees(abs(new-old)), nearest_without_reserve_deg=math.degrees(nearest),
                    nearest_reserve_deg=math.degrees(soft-abs(nearest)),
                    nearest_rejected_by_reserve=abs(nearest)>soft-reserve))
                assert abs(new-expected)<.035, wheels[-1]
                if abs(new-old)>math.pi/2:
                    assert abs(nearest)>soft-reserve, 'unexplained nonminimal steering'
            if not lateral: assert all(w['travel_deg']<90 for w in wheels)
            else: assert sum(w['travel_deg']>90 for w in wheels)==2
            report['cases'].append(dict(case=label, before_spin_deg=list(map(math.degrees,start)),
                                        wheels=wheels, recovery_reasons=observed))
        report.update(passed=True, final_motion_state=node.state,
                      soft_limit_deg=math.degrees(soft), stationary_reserve_deg=math.degrees(reserve),
                      wheel_order=['fl','fr','rl','rr'])
    finally:
        try: settle((0,0,0))
        finally:
            node.destroy_node(); rclpy.shutdown()
            args.output.write_text(json.dumps(report,indent=2)+'\n')


if __name__ == '__main__':
    main()
