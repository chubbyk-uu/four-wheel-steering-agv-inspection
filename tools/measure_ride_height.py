#!/usr/bin/env python3
"""Resting ride height against physical tyre diameter.

The 39/41 cm pair is meant to inject a longitudinal scale error, but a tyre
that is 5 mm smaller in radius also sits the body 5 mm lower if the suspension
is a spring at equilibrium under an unchanged load -- and the camera goes with
it, moving lateral magnification by roughly the same fraction. That is a
prediction about this model's suspension, not a fact, so measure it before
quoting a lateral figure in an acceptance table.

Reads ground-truth odometry at rest on the empty default world. Reports the
base height and what it implies for the imaged swath, given that the pipeline
keeps using the calibrated height.
"""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node

ROOT = Path(__file__).resolve().parents[1]


def optical_frame_above_base():
    """Height of camera_optical_frame over base_link, summed from the URDF.

    Not camera_link: the optical frame hangs 40 mm below it, which cancels the
    +0.04 in the mount expression. Reading the wrong one of those two puts the
    lens 40 mm out and the lateral scale off by an order of magnitude.
    """
    import xacro
    import xml.etree.ElementTree as ET
    desc = ROOT/'src/agv_description'
    robot = ET.fromstring(xacro.process_file(str(desc/'urdf/agv.urdf.xacro'), mappings={
        'platform': str(desc/'config/platform.yaml'), 'actual_wheel_diameter': '0.40',
        'camera_config': str(desc/'config/linescan.yaml'),
        'controllers': str(desc/'config/controllers.yaml')}).toxml())
    up = {}
    for joint in robot.findall('joint'):
        origin = joint.find('origin')
        xyz = [0., 0., 0.] if origin is None or not origin.get('xyz') else [
            float(v) for v in origin.get('xyz').split()]
        up[joint.find('child').get('link')] = (joint.find('parent').get('link'), xyz[2])
    link, z = 'camera_optical_frame', 0.
    while link in up:
        link, rise = up[link];z += rise
    assert link == 'base_link', link
    return z


def sample(diameter, settle, samples, timeout, log):
    # The reader has to sit in the same ROS domain as the simulation it starts,
    # so set it on this process rather than only on the child's environment.
    sim = subprocess.Popen(
        ['ros2', 'launch', 'agv_bringup', 'sim.launch.py', 'headless:=true', 'rviz:=false',
         'linescan:=false', 'actual_wheel_diameter:='+('%.3f' % diameter)],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('ride_height')
    seen = []
    node.create_subscription(Odometry, '/ground_truth/odom',
                             lambda m: seen.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                                                    m.pose.pose.position.z)), 20)
    try:
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline and (not seen or seen[-1][0]-seen[0][0] < settle):
            assert sim.poll() is None, 'simulation exited'
            rclpy.spin_once(node, timeout_sec=.05)
        assert seen and seen[-1][0]-seen[0][0] >= settle, 'no settled odometry within the timeout'
        tail = [z for t, z in seen if t >= seen[-1][0]-1.0][-samples:]
        return dict(diameter_m=diameter, base_z_m=float(np.mean(tail)),
                    spread_m=float(np.ptp(tail)), samples=len(tail),
                    settled_sim_s=float(seen[-1][0]-seen[0][0]))
    finally:
        node.destroy_node();rclpy.shutdown()
        os.killpg(os.getpgid(sim.pid), signal.SIGINT)
        try:sim.wait(timeout=30)
        except subprocess.TimeoutExpired:os.killpg(os.getpgid(sim.pid), signal.SIGKILL)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--diameters', type=float, nargs='+', default=[.40, .39, .41])
    p.add_argument('--settle', type=float, default=6.)
    p.add_argument('--samples', type=int, default=40)
    p.add_argument('--timeout', type=float, default=300.)
    p.add_argument('--output', type=Path)
    a = p.parse_args()

    os.environ.update(ROS_DOMAIN_ID=str(100+os.getpid() % 80),
                      GZ_PARTITION='agv_ride_'+str(os.getpid()))
    camera = yaml.safe_load((ROOT/'src/agv_description/config/linescan.yaml').read_text())
    # Calibrated working distance: what the pipeline keeps assuming whatever the
    # tyres do. Swath scales with it, so a ride-height change is a lateral scale
    # change of the same fraction.
    nominal_h = (camera['nominal_width_m']*camera['focal_length_m']
                 / (camera['width']*camera['pixel_pitch_m']))
    optical_offset = optical_frame_above_base()
    runs = []
    with (ROOT/'local_data/ride_height.log').open('w') as log:
        for d in a.diameters:
            runs.append(sample(d, a.settle, a.samples, a.timeout, log))
            print(json.dumps(runs[-1]), flush=True)
    base = next(r for r in runs if abs(r['diameter_m']-.40) < 1e-9)
    for r in runs:
        drop = r['base_z_m']-base['base_z_m']
        height = r['base_z_m']+optical_offset
        r.update(ride_height_change_m=drop,
                 predicted_from_radius_m=(r['diameter_m']-.40)/2,
                 optical_centre_height_m=height,
                 # What the pipeline is wrong by in absolute terms: it maps 4096
                 # columns onto the calibrated 1.5 m whatever height the camera
                 # is really at, so an object comes out this much too large.
                 lateral_scale_percent=100*(nominal_h/height-1),
                 longitudinal_scale_percent=100*(.40/r['diameter_m']-1))
    for r in runs:
        r['lateral_scale_vs_040_percent'] = (r['lateral_scale_percent']
                                             - base['lateral_scale_percent'])
    report = dict(schema='agv.ride_height_vs_tyre.v1', calibrated_camera_height_m=nominal_h,
                  optical_frame_above_base_link_m=optical_offset,
                  note=('lateral_scale_percent is absolute, against the calibrated height; '
                        'lateral_scale_vs_040_percent is the part the tyre experiment adds. '
                        'They differ because the suspension already compresses under load, so '
                        'the optical centre never sits at the calibrated height even at 0.40 m.'),
                  runs=runs)
    text = json.dumps(report, indent=2)+'\n'
    if a.output:
        a.output.write_text(text)
    print(text)


if __name__ == '__main__':
    main()
