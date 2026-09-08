#!/usr/bin/env python3
"""Bounded ROS integration check for the planner CLI and latched preview topics."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path as RosPath
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    out = root/'local_data/mission_demo'
    out.mkdir(parents=True, exist_ok=True)
    log = (out/'publisher.log').open('w')
    process = subprocess.Popen(['ros2', 'run', 'agv_mission', 'plan_rectangle',
                                str(root/'src/agv_mission/config/rectangle_demo.yaml'),
                                '--output', str(out/'plan.json'), '--publish'],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('validate_rectangle_preview')
    received = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    try:
        # Deliberately join after publisher creation, exercising retained plan QoS.
        time.sleep(2)
        subscriptions = [node.create_subscription(cls, topic,
                         lambda msg, key=key: received.update({key: msg}), qos)
                         for cls, topic, key in [(RosPath, '/mission/preview/base_path', 'path'),
                             (MarkerArray, '/mission/preview/markers', 'markers')]]
        deadline = time.monotonic()+12
        while len(received) < 2 and time.monotonic() < deadline:
            assert process.poll() is None, 'preview exited before messages arrived'
            rclpy.spin_once(node, timeout_sec=.1)
        assert len(received) == 2, 'preview topics missing'
        plan = json.loads((out/'plan.json').read_text())
        path, markers = received['path'], received['markers']
        assert len(path.poses) == sum(len(s['points']) for s in plan['segments'])
        assert path.header.frame_id == plan['frame_id'] == 'world'
        assert sum(m.ns == 'scan_footprint' for m in markers.markers) == 4
        assert plan['track_count'] == 4 and plan['actual_track_spacing_m'] == 1
        assert not any(info.node_name == 'rectangle_plan_preview'
                       for info in node.get_publishers_info_by_topic('/cmd_vel'))
        result = {'schema': 'agv.mission.preview_validation.v1', 'passed': True,
                  'track_count': 4, 'fixed_track_spacing_m': 1.0,
                  'segments': len(plan['segments']), 'path_poses': len(path.poses),
                  'markers': len(markers.markers), 'frame': path.header.frame_id,
                  'preview_publishes_cmd_vel': False, 'execution_validated': False}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
        log.close()


if __name__ == '__main__':
    main()
