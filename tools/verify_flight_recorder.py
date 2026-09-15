#!/usr/bin/env python3
"""Verify the scan flight recorder: it must fire, be complete, and not delay stopping.

The residual gate is lowered so a rejection is certain. This proves the recorder
works; it does not reproduce the intermittent fault and is no evidence about its
cause. Every check below is an assertion, so a regression fails the exit code
rather than printing a discouraging line into a log nobody reads.
"""
import argparse
import glob
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SPAN_S = 2.0
STEP_S = 0.001


def log(*a):
    print(*a, flush=True)


class Failure(Exception):
    pass


def require(condition, message):
    if not condition:
        raise Failure(message)


def build_scene(workdir):
    """A generatable scene, so a clean checkout can run this without local data."""
    scene = os.path.join(workdir, 'scene')
    subprocess.run([sys.executable, os.path.join(ROOT, 'tools/generate_shared_scene.py'),
                    '--output', scene, '--length', '40', '--width', '10'],
                   check=True, cwd=ROOT, capture_output=True, text=True)
    return os.path.join(scene, 'manifest.json')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene', help='shared scene manifest; generated when omitted')
    p.add_argument('--output', help='working directory; a temporary one when omitted')
    p.add_argument('--max-dumps', type=int, default=3)
    p.add_argument('--stop-latency-limit-s', type=float, default=5.0)
    a = p.parse_args()

    work = a.output or tempfile.mkdtemp(prefix='agv_flight_verify_')
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)
    scene = a.scene or build_scene(work)
    log('scene', scene)

    camera = yaml.safe_load(open(os.path.join(ROOT, 'src/agv_description/config/linescan.yaml')))
    camera.update(projected_encoder=True, max_scan_residual_m_s=1e-9,
                  max_scan_lateral_m_s=.28, max_yaw_rate_rad_s=.08)
    camera['flight_recorder'] = {'span_s': SPAN_S, 'max_samples': 4000,
                                 'half_limit_cooldown_s': 10.0, 'max_dumps': a.max_dumps}
    camera_path = os.path.join(work, 'camera.yaml')
    yaml.safe_dump(camera, open(camera_path, 'w'))

    # Isolate the bus: a leftover simulator from another run otherwise collides on
    # the controller manager and this reads as "the simulator did not start".
    isolated = dict(os.environ,
                    ROS_DOMAIN_ID=str(100 + os.getpid() % 80),
                    GZ_PARTITION='agv_flight_verify_%d' % os.getpid())
    launch = subprocess.Popen(['bash', '-c',
        'cd %s && source /opt/ros/jazzy/setup.bash && source install/setup.bash && '
        'exec python3 tools/with_mesa_runtime.py -- ros2 launch agv_bringup sim.launch.py '
        'headless:=true linescan:=true linescan_backend:=optix scene_manifest:=%s spawn_x:=2 '
        'camera_config:=%s capture_dir:=%s' % (ROOT, scene, camera_path, work)],
        stdout=open(os.path.join(work, 'sim.log'), 'w'), stderr=subprocess.STDOUT,
        preexec_fn=os.setsid, env=isolated)
    os.environ.update(ROS_DOMAIN_ID=isolated['ROS_DOMAIN_ID'],
                      GZ_PARTITION=isolated['GZ_PARTITION'])

    def dead():
        return launch.poll() is not None

    try:
        deadline = time.time() + 300
        while time.time() < deadline and not dead():
            if open(os.path.join(work, 'sim.log'), errors='replace').read().count(
                    'Successfully switched controllers') >= 3:
                break
            time.sleep(2)
        else:
            raise Failure('simulator did not start; see ' + os.path.join(work, 'sim.log'))
        log('controllers up')

        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from geometry_msgs.msg import TwistStamped
        from std_srvs.srv import SetBool
        from nav_msgs.msg import Odometry
        rclpy.init()

        class Driver(Node):
            def __init__(self):
                super().__init__('flight_verify',
                                 parameter_overrides=[Parameter('use_sim_time', value=True)])
                self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
                self.enable = self.create_client(SetBool, '/linescan/set_enabled')
                self.x = None
                self.speed = 0.
                self.create_subscription(Odometry, '/ground_truth/odom', self.odom, 20)

            def odom(self, m):
                self.x = m.pose.pose.position.x
                self.speed = m.twist.twist.linear.x

            def drive(self, vx, seconds):
                end = time.time() + seconds
                while time.time() < end:
                    m = TwistStamped()
                    m.header.frame_id = 'base_link'
                    m.header.stamp = self.get_clock().now().to_msg()
                    m.twist.linear.x = float(vx)
                    self.pub.publish(m)
                    rclpy.spin_once(self, timeout_sec=.02)
                    time.sleep(.02)
                    if dead():
                        raise Failure('simulator exited during the run')

        node = Driver()
        start = time.time()
        while node.x is None and time.time() - start < 60:
            rclpy.spin_once(node, timeout_sec=.1)
        require(node.x is not None, 'no odometry')
        require(node.enable.wait_for_service(timeout_sec=30), '/linescan/set_enabled never appeared')
        future = node.enable.call_async(SetBool.Request(data=True))
        while not future.done():
            rclpy.spin_once(node, timeout_sec=.1)
        require(future.result().success, 'capture could not be enabled')
        session = future.result().message
        log('capture enabled in', session)

        def events():
            path = os.path.join(session, 'events.jsonl')
            if not os.path.isfile(path):
                return []
            return [json.loads(l) for l in open(path) if l.strip()]

        node.drive(1.0, 6)
        fault_wall = None
        for _ in range(200):
            node.drive(1.0, .5)
            if any(e.get('reason') == 'unsupported_scan_motion' for e in events()):
                fault_wall = time.time()
                break
        require(fault_wall is not None, 'the lowered gate never rejected a scan')
        log('rejection observed at x=%.2f' % node.x)

        node.drive(0., .2)
        stop_wall = None
        for _ in range(100):
            node.drive(0., .2)
            if abs(node.speed) < .02:
                stop_wall = time.time()
                break
        require(stop_wall is not None, 'the vehicle never stopped after the rejection')
        latency = stop_wall - fault_wall
        log('stop latency %.2f s' % latency)
        require(latency <= a.stop_latency_limit_s,
                'stopping took %.2f s, beyond the %.2f s limit: the async write may be holding it'
                % (latency, a.stop_latency_limit_s))

        # Let the capped dumps accumulate so suppression can be checked too.
        for _ in range(60):
            node.drive(1.0, .5)
            if len(glob.glob(os.path.join(session, 'flight_*.json'))) >= a.max_dumps:
                break
    finally:
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=40)
        except Exception:
            try:
                os.killpg(launch.pid, signal.SIGKILL)
            except OSError:
                pass

    files = sorted(glob.glob(os.path.join(work, 'session_cpp_*', 'flight_*.json')))
    require(files, 'no flight recorder file was written')
    log('flight files:', len(files))
    require(len(files) <= a.max_dumps,
            'wrote %d files against a cap of %d' % (len(files), a.max_dumps))

    doc = json.load(open(files[0]))
    require(doc['schema'] == 'agv.linescan.flight_recorder.v1', 'unexpected schema ' + doc['schema'])
    require(doc['reason'] == 'unsupported_scan_motion', 'unexpected reason ' + doc['reason'])
    expected = int(SPAN_S / STEP_S) + 1
    require(doc['samples'] == expected,
            'expected %d samples for a %.1f s window at %.0f ms, got %d'
            % (expected, SPAN_S, STEP_S * 1000, doc['samples']))
    require(abs(doc['window_s'] - SPAN_S) < 1e-6,
            'window was %.6f s, expected %.1f s' % (doc['window_s'], SPAN_S))

    last = doc['records'][-1]
    for field, count in (('drive_rate_rad_s', 4), ('wheel_speed_m_s', 4), ('steer_rad', 4),
                         ('suspension_m', 4), ('suspension_rate_m_s', 4),
                         ('wheel_residual_m_s', 4), ('body_xyz', 3),
                         ('body_quat_xyzw', 4), ('body_velocity_base_link_m_s', 3)):
        require(field in last, 'missing field ' + field)
        require(len(last[field]) == count,
                '%s had %d entries, expected %d' % (field, len(last[field]), count))
    require(len(set(last['wheel_residual_m_s'])) > 1,
            'per-wheel residuals are identical; a single bad wheel could not be told apart')

    guards = last['pass']
    require(guards['residual'] is False, 'the residual guard should be the one that failed')
    for name in ('speed', 'lateral', 'yaw', 'data', 'polarity'):
        require(guards[name] is True, 'guard %s unexpectedly failed' % name)

    stats = doc['residual_statistics']
    for key in ('samples', 'limit_m_s', 'max_m_s', 'p95_m_s', 'p99_m_s', 'over_limit',
                'histogram', 'flight_dumps', 'flight_dumps_suppressed', 'flight_write_failures'):
        require(key in stats, 'missing statistic ' + key)
    require(stats['flight_write_failures'] == 0,
            'the recorder reported %d failed writes' % stats['flight_write_failures'])

    suppressed = [e.get('flight_recorder_suppressed') for e in
                  [json.loads(l) for l in
                   open(glob.glob(os.path.join(work, 'session_cpp_*', 'events.jsonl'))[0])
                   if l.strip()]
                  if 'flight_recorder_suppressed' in e]
    if len(files) >= a.max_dumps:
        require(suppressed, 'the cap was reached but no suppression was reported')
        log('suppressed reported, last =', suppressed[-1])

    log('OK: recorder fires, window and fields are complete, stopping is not delayed')
    log('    this verifies the recorder only; it is not a reproduction of the real fault')


if __name__ == '__main__':
    try:
        main()
    except Failure as error:
        print('FAIL:', error, file=sys.stderr)
        sys.exit(1)
