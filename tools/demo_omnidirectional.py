#!/usr/bin/env python3
"""Ten commands, each for ten simulation seconds, followed by verified stop."""
import json
import math
import time
from pathlib import Path
import rclpy
from validate_motion import Evaluator


def main():
    rclpy.init()
    node = Evaluator()
    rows = []
    output = Path('results/omnidirectional_demo.json')
    try:
        deadline = time.monotonic() + 60
        while not (node.odom and node.joints and node.state):
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation feedback not ready')
            rclpy.spin_once(node, timeout_sec=.1)
        node.run_for(1, (0, 0, 0))
        d = .5 / math.sqrt(2)
        cases = [('前', (.5, 0, 0)), ('左前', (d, d, 0)),
                 ('左', (0, .5, 0)), ('左后', (-d, d, 0)),
                 ('后', (-.5, 0, 0)), ('右后', (-d, -d, 0)),
                 ('右', (0, -.5, 0)), ('右前', (d, -d, 0)),
                 ('原地正转（逆时针）', (0, 0, .3)),
                 ('原地反转（顺时针）', (0, 0, -.3))]
        for name, cmd in cases:
            Path('/tmp/agv_demo_progress.json').write_text(json.dumps(
                {'current': name, 'completed': len(rows)}, ensure_ascii=False))
            node.states.clear()
            before = node.pose()
            start = node.get_clock().now().nanoseconds
            node.run_for(10, cmd)
            if node.state != 'DRIVE':
                raise RuntimeError(f'{name}: expected DRIVE, got {node.state}')
            rows.append(dict(name=name, command=cmd, start_pose=before,
                             end_pose=node.pose(), states=sorted(node.states),
                             command_duration_s=(node.get_clock().now().nanoseconds-start)/1e9))
        node.run_for(3, (0, 0, 0))
        wheel_rates = [v for n, v in zip(node.joints.name, node.joints.velocity)
                       if n.endswith('_drive_joint')]
        if node.state != 'HOLD' or len(wheel_rates) != 4 or max(map(abs, wheel_rates)) > .1:
            raise RuntimeError('Final stop was not confirmed')
        output.parent.mkdir(exist_ok=True)
        output.write_text(json.dumps({'passed': True, 'duration_includes_transitions': True,
                                     'motions': rows, 'final_state': node.state,
                                     'final_wheel_rates': wheel_rates}, ensure_ascii=False, indent=2)+'\n')
        Path('/tmp/agv_demo_progress.json').write_text('{"completed": 10, "current": "停车完成"}')
        print(f'PASS: ten motions, ten simulation seconds each; stopped; {output}')
    finally:
        node.command((0, 0, 0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
