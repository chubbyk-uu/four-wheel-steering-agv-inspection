#!/usr/bin/env python3
"""Isolated GZ smoke/continuity test; commands go through the AGV controller."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def evaluate(archive):
    import rclpy
    from std_srvs.srv import SetBool
    from validate_motion import Evaluator
    from PIL import Image
    rclpy.init()
    node = Evaluator()
    client = node.create_client(SetBool, '/linescan/set_enabled')
    def enable(value):
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        deadline = time.monotonic()+20
        while not future.done():
            node.command((.05, 0, 0) if value else (0, 0, 0))
            rclpy.spin_once(node, timeout_sec=.01)
            if time.monotonic() > deadline:
                raise RuntimeError('capture service timeout')
        assert future.result().success
    try:
        assert client.wait_for_service(timeout_sec=60)
        deadline = time.monotonic()+60
        while not (node.odom and node.joints and node.state):
            rclpy.spin_once(node, timeout_sec=.1)
            assert time.monotonic() < deadline
        node.run_for(2, (.05, 0, 0))
        enable(True)
        node.run_for(26, (.05, 0, 0))
        # Stop and allow the sensor to flush its partial block.
        node.run_for(2, (0, 0, 0))
        enable(False)
        assert node.state == 'HOLD'
        session = next(Path(archive).glob('session_*'))
        metadata = [json.loads(p.read_text()) for p in sorted(session.glob('block_*.json'))]
        assert metadata and any(m['rows'] == 4096 for m in metadata), metadata
        assert sum(m['rows'] for m in metadata) > 4300
        assert len({m['segment_id'] for m in metadata}) == 1, metadata
        assert all(m['end_reason'] in ('full', 'motion_BRAKE', 'motion_HOLD', 'capture_toggle') for m in metadata)
        for m in metadata:
            assert m['width'] == 4096 and m['invalid_pixels'] == 0
            with Image.open(session/f'block_{m["block_id"]:06d}.png') as im:
                assert im.size == (4096, m['rows'])
            assert m['last']['time_s'] >= m['first']['time_s']
        for a, b in zip(metadata, metadata[1:]):
            assert b['first']['global_line'] == a['last']['global_line']+1
            if a['segment_id'] == b['segment_id']:
                assert abs(b['first']['encoder_distance_m']-a['last']['encoder_distance_m']-1.2/4096) < 1e-9
        report = dict(passed=True, backend='GZ_feedback_analytic_grid_plane',
                      rows=[m['rows'] for m in metadata], archive=str(session),
                      final_motion_state=node.state,
                      first_last_x=[metadata[0]['first']['camera_position_world_m'][0], metadata[-1]['last']['camera_position_world_m'][0]])
        (Path(archive)/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report))
    finally:
        node.command((0, 0, 0))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        evaluate(sys.argv[1])
    else:
        tag = f'agv_linescan_gz_{os.getpid()}'
        archive = '/tmp/'+tag
        env = dict(os.environ, ROS_DOMAIN_ID='79', GZ_PARTITION=tag, ROS_LOG_DIR=archive+'_ros')
        with open(archive+'_sim.log', 'w') as log:
            sim = subprocess.Popen(['ros2', 'launch', 'agv_bringup', 'sim.launch.py',
                                    'headless:=true', 'linescan:=true', 'linescan_backend:=analytic', 'capture_dir:='+archive],
                                   env=env, stdout=log, stderr=log, start_new_session=True)
            try:
                result = subprocess.run([sys.executable, __file__, archive], env=env, timeout=240)
                print(f'Simulation log: {archive}_sim.log')
                raise SystemExit(result.returncode)
            finally:
                if sim.poll() is None:
                    os.killpg(sim.pid, signal.SIGINT)
                    try:
                        sim.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(sim.pid, signal.SIGKILL)
                        sim.wait()
