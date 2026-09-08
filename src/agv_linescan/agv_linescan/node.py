"""ROS adapter for the analytic grid sensor, using measured GZ wheel positions."""
from collections import deque
from bisect import bisect_left
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time
import numpy as np
import yaml
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from .core import Camera, Trigger, Blocks, interpolate_pose


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec*1e-9


class LineScanNode(Node):
    def __init__(self):
        super().__init__('linescan_camera')
        self.declare_parameter('config', '')
        self.declare_parameter('platform', '')
        self.declare_parameter('output_dir', '/tmp/agv_linescan')
        self.c = yaml.safe_load(Path(self.get_parameter('config').value).read_text())
        absolute=self.c.get('wheel_speed_spread_absolute_m_s',.01);relative=self.c.get('wheel_speed_spread_relative',0)
        if not np.isfinite([absolute,relative]).all() or absolute<0 or not 0<=relative<=.05:
            raise ValueError('invalid wheel speed consistency tolerance')
        platform = yaml.safe_load(Path(self.get_parameter('platform').value).read_text())
        self.radius = platform['wheel_radius']
        self.track = platform['track']
        self.camera = Camera(self.c)
        self.output = Path(self.get_parameter('output_dir').value)
        # A fresh session directory prevents overwriting prior raw captures.
        self.output = self.output / f'session_{time.time_ns()}'
        self.output.mkdir(parents=True, exist_ok=False)
        (self.output/'calibration.yaml').write_text(yaml.safe_dump(self.c))
        self.pub = self.create_publisher(Image, '/linescan/image_raw', 2)
        self.tags_pub = self.create_publisher(String, '/linescan/block_metadata', 10)
        self.status_pub = self.create_publisher(String, '/linescan/status', 10)
        self.blocks = Blocks(self.camera, self.emit)
        self.writer = ThreadPoolExecutor(max_workers=1, thread_name_prefix='linescan_archive')
        self.writes = deque()
        self.trigger = Trigger(self.camera.spacing)
        self.poses = deque(maxlen=300)
        self.pending = deque()
        self.enabled = False
        self.state = ''
        self.last_encoder = None
        self.last_stop_reason = None
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom, 100)
        self.create_subscription(JointState, '/joint_states', self.joints,
                                 QoSProfile(depth=512, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, '/motion_state', self.motion, 10)
        self.create_service(SetBool, '/linescan/set_enabled', self.enable)
        self.create_timer(.1, self.watchdog)
        self.get_logger().info(f'{self.camera.width}-pixel grid-plane sensor ready, initially disabled. Archive: {self.output}')

    def enable(self, request, response):
        self.stop('capture_toggle')
        self.enabled = request.data
        if not self.enabled:
            for future in self.writes:
                future.result()
        response.success = True
        response.message = f'capture enabled={self.enabled}; {self.output}'
        return response

    def stop(self, reason):
        # Pending lines without a bracketed exposure cannot be fabricated.
        discarded = len(self.pending)
        record = (discarded or self.blocks.count or self.trigger.previous is not None
                  or reason != self.last_stop_reason)
        self.pending.clear()
        self.blocks.end_segment(reason)
        self.trigger.reset()
        self.last_encoder = None
        self.last_stop_reason = reason
        if record:
            event = dict(reason=reason, discarded_pending_lines=discarded,
                         simulation_time_s=self.get_clock().now().nanoseconds*1e-9,
                         next_global_line=self.blocks.global_line)
            self.status_pub.publish(String(data=json.dumps(event)))
            with (self.output/'events.jsonl').open('a') as f:
                f.write(json.dumps(event)+'\n')

    def motion(self, message):
        if message.data != self.state and message.data != 'DRIVE':
            self.process()
            self.stop('motion_'+message.data)
        self.state = message.data

    def odom(self, m):
        t = stamp_seconds(m.header.stamp)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        pose = ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        if self.poses and t <= self.poses[-1][0]:
            self.stop('pose_time_reset')
            self.poses.clear()
        self.poses.append((t, pose))
        # Truth is used to form rays only; acquisition gating uses wheel feedback.
        self.process()

    def joints(self, m):
        if not self.enabled or self.state != 'DRIVE':
            return
        names = [f'{w}_drive_joint' for w in ('fl', 'fr', 'rl', 'rr')]
        try:
            p = dict(zip(m.name, m.position))
            v = dict(zip(m.name, m.velocity))
            steering = [p[f'{w}_steer_joint'] for w in ('fl', 'fr', 'rl', 'rr')]
            speed = np.array([v[n]*self.radius for n in names])
            yaw_rate = (speed[1]+speed[3]-speed[0]-speed[2])/(2*self.track)
            distance = float(np.mean([p[n]*self.radius for n in names]))
            if not np.all(np.isfinite(steering+speed.tolist()+[distance])):
                raise ValueError('nonfinite feedback')
            # Conservative straight +/-X gate, including wheel disagreement.
            spread_limit=max(self.c.get('wheel_speed_spread_absolute_m_s',.01),self.c.get('wheel_speed_spread_relative',0)*abs(float(np.mean(speed))))
            if (max(map(abs, steering)) > self.c['max_steer_rad'] or np.ptp(speed) > spread_limit
                    or abs(yaw_rate) > self.c['max_yaw_rate_rad_s']
                    or max(abs(speed)) > self.c['max_scan_speed_m_s']):
                self.stop('unsupported_scan_motion')
                return
            t = stamp_seconds(m.header.stamp)
            if self.last_encoder is not None and t-self.last_encoder > self.c['max_sample_gap_s']:
                self.stop('encoder_gap')
            self.last_encoder = t
            self.pending.extend(self.trigger.update(t, distance))
            if len(self.pending) > 8192:
                self.stop('pending_overflow')
            else:
                self.process()
        except (KeyError, ValueError) as exc:
            self.stop('encoder_rejected_'+str(exc))

    def pose_at(self, t):
        times = [row[0] for row in self.poses]
        i = bisect_left(times, t)
        if i < len(times) and abs(times[i]-t) < 1e-10:
            return self.poses[i][1]
        if i == 0 or i == len(times):
            raise ValueError('pose_outside_buffer')
        a, b = self.poses[i-1], self.poses[i]
        if b[0]-a[0] > self.c['max_sample_gap_s']:
            raise ValueError('pose_gap')
        return interpolate_pose(a[1], b[1], (t-a[0])/(b[0]-a[0]))

    def process(self):
        while self.pending and len(self.poses) >= 2:
            t, distance = self.pending[0]
            # Trigger starts exposure; tags use its midpoint, not reception time.
            end = t+self.camera.exposure
            if end > self.poses[-1][0]:
                return
            self.pending.popleft()
            try:
                start_pose, end_pose = self.pose_at(t), self.pose_at(end)
                mid = self.pose_at(t+self.camera.exposure/2)
            except ValueError as exc:
                self.stop(str(exc))
                return
            line, valid = self.camera.expose(start_pose, end_pose)
            optical_position, optical_rotation = self.camera.optical_pose(mid)
            self.blocks.add(line, valid, dict(time_s=t+self.camera.exposure/2,
                            encoder_distance_m=distance, scan_direction=self.trigger.direction,
                            camera_position_world_m=optical_position.tolist(),
                            camera_rotation_world=optical_rotation.tolist()))

    def watchdog(self):
        while self.writes and self.writes[0].done():
            self.writes.popleft().result()  # Surface archive failures, never silently lose blocks.
        if self.last_encoder is not None and self.get_clock().now().nanoseconds*1e-9-self.last_encoder > self.c['max_sample_gap_s']:
            self.stop('encoder_timeout')

    def emit(self, pixels, metadata):
        while self.writes and self.writes[0].done():
            self.writes.popleft().result()
        if len(self.writes) >= 2:
            raise RuntimeError('archive backpressure: reduce scanning speed')
        self.writes.append(self.writer.submit(self.write_block, pixels, metadata))

    def write_block(self, pixels, metadata):
        basename = f'block_{metadata["block_id"]:06d}'
        PILImage.fromarray(pixels).save(self.output/(basename+'.png'))
        (self.output/(basename+'.json')).write_text(json.dumps(metadata, indent=2)+'\n')
        msg = Image()
        ns = round(metadata['last']['time_s']*1e9)
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 10**9)
        msg.header.frame_id = 'camera_optical_frame'
        msg.height, msg.width = pixels.shape
        msg.encoding, msg.step = 'mono8', msg.width
        msg.data = pixels.tobytes()
        if rclpy.ok():
            self.pub.publish(msg)
            self.tags_pub.publish(String(data=json.dumps(metadata)))
            self.get_logger().info(f'{basename}: {msg.width}x{msg.height}, {metadata["end_reason"]}')


def main():
    rclpy.init()
    node = LineScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Explicit set_enabled(false) is the reliable flush path before shutdown.
        if rclpy.ok():
            node.stop('shutdown')
        node.writer.shutdown(wait=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
