#!/usr/bin/env python3
"""Isolated CUDA static-grid + separate ROS receiver + PGM archive benchmark."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def receive(directory, blocks, calibration=None, config=None):
    import numpy as np
    import rclpy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from PIL import Image as PilImage
    correction = None
    correction_times = []
    if calibration:
        import yaml
        sys.path.append(str(Path(__file__).resolve().parents[1]/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
        from agv_linescan.calibration import Correction
        correction = Correction(json.loads(Path(calibration).read_text()))
        correction.check_capture(yaml.safe_load(Path(config).read_text()))
    rclpy.init()
    node = rclpy.create_node('cuda_grid_benchmark_receiver')
    pub = node.create_publisher(String, '/benchmark/ack', 10)
    received = 0
    last_time = -1
    def callback(message):
        nonlocal received, last_time
        block = int(message.header.frame_id.removeprefix('cuda_grid_block_'))
        assert block == received, ('missing / duplicate block', block, received)
        assert message.width == 4096 and message.height == 4096
        assert message.step == 4096 and message.encoding == 'mono8'
        assert len(message.data) == message.width*message.height
        stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        meta = json.loads((Path(directory)/f'block_{block}.json').read_text())
        assert abs(stamp-meta['last']['time_s']) < 1e-8
        assert meta['first']['global_line'] == block*4096
        assert meta['last']['global_line'] == (block+1)*4096-1
        assert len(meta['pose_tags']) == 5
        assert stamp > last_time
        # Compare every received byte against the independently read archive.
        with PilImage.open(Path(directory)/f'block_{block}.pgm') as im:
            assert np.array_equal(np.asarray(im).ravel(), np.frombuffer(message.data, dtype=np.uint8))
        if correction:
            start = time.perf_counter()
            pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width)
            corrected, quality = correction.apply(pixels)
            destination = Path(directory)/f'corrected_{block}.pgm'
            with destination.open('wb') as stream:
                stream.write(f'P5\n{message.width} {message.height}\n255\n'.encode())
                stream.write(corrected.tobytes()); stream.flush(); os.fsync(stream.fileno())
            corrected_meta = correction.metadata(meta); corrected_meta.update(quality)
            destination.with_suffix('.json').write_text(json.dumps(corrected_meta))
            correction_times.append(time.perf_counter()-start)
        last_time = stamp
        received += 1
        ack = String(); ack.data = str(block); pub.publish(ack)
    node.create_subscription(Image, '/benchmark/image', callback, rclpy.qos.QoSProfile(depth=1))
    deadline = time.monotonic()+180
    while received < blocks:
        rclpy.spin_once(node, timeout_sec=.1)
        if time.monotonic() > deadline:
            raise TimeoutError('benchmark receiver timeout')
    # Allow final acknowledgement to leave DDS before destroying the publisher.
    finish = time.monotonic()+.5
    while time.monotonic() < finish:
        rclpy.spin_once(node, timeout_sec=.02)
    if correction:
        (Path(directory)/'correction_summary.json').write_text(json.dumps(dict(
            blocks=received, calibration_id=correction.profile['calibration_id'],
            correction_and_archive_seconds_max=max(correction_times),
            correction_and_archive_seconds_mean=float(np.mean(correction_times)),
            corrected_image_fsync=True, corrected_ros_published=False)))
    node.destroy_node(); rclpy.shutdown()


def validate_summary(report, blocks, minimum_rate):
    assert report['width'] == 4096 and report['rows_per_block'] == 4096
    assert report['ros_acknowledged_blocks'] == blocks
    assert report['invalid_pixels'] == 0
    assert report['wall_seconds'] >= 30, 'sustained test must last at least 30 seconds'
    assert report['effective_lines_per_wall_second'] >= minimum_rate, 'sustained grid throughput below threshold'
    if report['backend'] == 'cuda_tiled_grid_testbench':
        assert report['tiles']['required_tile_misses'] == 0
        assert report['tiles']['tile_loads'] > report['tiles']['cache_slots']
        assert report['batch_seconds_max'] <= report['continuous_delay_budget']['batch_seconds'], 'batch delay budget exceeded'
        assert report['block_receive_interval_seconds_max'] <= report['continuous_delay_budget']['block_interval_seconds'], 'block reception jitter budget exceeded'
    report['grid_subtest_minimum_rate'] = minimum_rate
    report['grid_throughput_subtest_passed'] = True
    # This flag intentionally remains false until illumination/occlusion exist.
    assert report['full_acceptance_passed'] is False


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output', required=True)
    p.add_argument('--blocks', type=int, default=180)
    p.add_argument('--rate', type=float, default=23000)
    p.add_argument('--minimum-rate', type=float, default=22000)
    p.add_argument('--domain', type=int, default=83)
    p.add_argument('--terrain')
    p.add_argument('--config', help='Optional camera YAML, e.g. radiometry-disabled geometry regression')
    p.add_argument('--calibration', help='Measured profile; correction and durable corrected archive before each ACK')
    p.add_argument('--receiver', action='store_true')
    args = p.parse_args()
    if args.receiver:
        receive(args.output, args.blocks, args.calibration, args.config); return
    root = Path(__file__).resolve().parents[1]
    config = str(Path(args.config).resolve() if args.config else root/'src/agv_description/config/linescan.yaml')
    receiver_args = ['--config', config]
    if args.calibration:
        receiver_args += ['--calibration', str(Path(args.calibration).resolve())]
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
    with open(args.output+'_receiver.log', 'w') as log:
        receiver = subprocess.Popen([sys.executable, __file__, '--receiver', '--output', args.output,
                                     '--blocks', str(args.blocks)]+receiver_args, env=env, stdout=log, stderr=log)
        try:
            command = ['ros2', 'run', 'agv_linescan', 'benchmark_cuda_grid',
                str(Path(args.config).resolve() if args.config else root/'src/agv_description/config/linescan.yaml'), args.output,
                str(args.blocks), str(args.rate), '1', '1']
            if args.terrain:
                command.append(args.terrain)
            result = subprocess.run(command, env=env, timeout=180)
            if result.returncode:
                raise RuntimeError(f'benchmark failed: {result.returncode}; receiver log {log.name}')
            assert receiver.wait(timeout=10) == 0, log.name
            report = json.loads((Path(args.output)/'summary.json').read_text())
            if args.calibration:
                report['correction'] = json.loads((Path(args.output)/'correction_summary.json').read_text())
            validate_summary(report, args.blocks, args.minimum_rate)
            (Path(args.output)/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
            print('Separate ROS receiver matched every archived pixel; report: '+args.output+'/summary.json')
        finally:
            if receiver.poll() is None:
                receiver.terminate()
                try: receiver.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    receiver.kill(); receiver.wait()


if __name__ == '__main__':
    main()
