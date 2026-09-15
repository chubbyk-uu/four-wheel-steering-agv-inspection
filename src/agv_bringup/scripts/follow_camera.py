#!/usr/bin/env python3
"""Follow the AGV and keep it in view; the GUI plugin retains wheel zoom."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def acceptable(status, ready_only=False):
    if ready_only:
        return isinstance(status, dict)
    return (isinstance(status, dict)
            and status.get('trackMode') == 'FOLLOW_LOOK_AT'
            and status.get('followTarget', {}).get('name') == 'agv'
            and status.get('trackTarget', {}).get('name') == 'agv')


def write_ready(path, ready_only):
    if not path:
        return
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + '.tmp.' + str(os.getpid()))
    temporary.write_text(json.dumps({
        'ready': True,
        'mode': 'gui_only' if ready_only else 'follow_look_at',
        'pid': os.getpid(),
        'wall_monotonic_s': time.monotonic()
    }, indent=2) + '\n')
    os.replace(temporary, target)


def own_arguments(argv):
    """Discard arguments launch_ros appends for a helper that does not use rclpy."""
    return argv[:argv.index('--ros-args')] if '--ros-args' in argv else argv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ready-only', action='store_true',
                        help='wait for the GUI plugin without changing camera tracking')
    parser.add_argument('--ready-file', default='',
                        help='atomically write readiness evidence after three stable seconds')
    parser.add_argument('--timeout', type=float, default=90.)
    # launch_ros.Node appends ROS remapping arguments even though this helper does
    # not initialize rclpy. Keep the helper's own arguments strict before that marker.
    args = parser.parse_args(own_arguments(sys.argv[1:]))
    if not 1 <= args.timeout <= 300:
        parser.error('timeout must be within [1, 300] seconds')
    deadline = time.monotonic() + args.timeout
    confirmed_since = None
    while time.monotonic() < deadline:
        # CameraTracking retains the configured follow_offset. Do not resend
        # while acknowledged: user zoom changes must remain intact.
        # A first acknowledgement may precede completion of scene loading.
        try:
            if confirmed_since is None and not args.ready_only:
                subprocess.run([
                    'gz', 'topic', '-t', '/gui/track', '-m', 'gz.msgs.CameraTrack',
                    '-p', 'track_mode: FOLLOW_LOOK_AT follow_target { name: "agv" } track_target { name: "agv" } track_pgain: 1.0'],
                    capture_output=True, text=True, timeout=5, check=True)
            result = subprocess.run([
                'gz', 'topic', '-t', '/gui/currently_tracked', '-e', '-n', '1',
                '--json-output'], capture_output=True, text=True, timeout=3, check=True)
            # Transport shutdown can race another publication despite -n 1.
            status = json.JSONDecoder().raw_decode(result.stdout.lstrip())[0]
            if acceptable(status, args.ready_only):
                if confirmed_since is None:
                    confirmed_since = time.monotonic()
                elif time.monotonic() - confirmed_since >= 3:
                    write_ready(args.ready_file, args.ready_only)
                    print(('Gazebo GUI ready; free camera remains enabled.' if args.ready_only else
                           'AGV locked follow enabled; mouse wheel zoom remains available.'), flush=True)
                    return
            else:
                confirmed_since = None
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError,
                json.JSONDecodeError, OSError):
            confirmed_since = None
        time.sleep(.5)
    mode = 'readiness' if args.ready_only else 'locked follow'
    raise RuntimeError(f'Gazebo GUI {mode} did not become ready within {args.timeout:g} seconds')


if __name__ == '__main__':
    main()
