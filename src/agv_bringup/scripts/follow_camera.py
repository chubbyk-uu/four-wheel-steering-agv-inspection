#!/usr/bin/env python3
"""Follow the AGV and keep it in view; the GUI plugin retains wheel zoom."""
import json
import subprocess
import time


def main():
    deadline = time.monotonic() + 90
    confirmed_since = None
    while time.monotonic() < deadline:
        # CameraTracking retains the configured follow_offset. Do not resend
        # while acknowledged: user zoom changes must remain intact.
        # A first acknowledgement may precede completion of scene loading.
        if confirmed_since is None:
            subprocess.run([
                'gz', 'topic', '-t', '/gui/track', '-m', 'gz.msgs.CameraTrack',
                '-p', 'track_mode: FOLLOW_LOOK_AT follow_target { name: "agv" } track_target { name: "agv" } track_pgain: 1.0'],
                capture_output=True, text=True, timeout=5, check=True)
        try:
            result = subprocess.run([
                'gz', 'topic', '-t', '/gui/currently_tracked', '-e', '-n', '1',
                '--json-output'], capture_output=True, text=True, timeout=3, check=True)
            # Transport shutdown can race another publication despite -n 1.
            status = json.JSONDecoder().raw_decode(result.stdout.lstrip())[0]
            if (status.get('trackMode') == 'FOLLOW_LOOK_AT'
                    and status.get('followTarget', {}).get('name') == 'agv'
                    and status.get('trackTarget', {}).get('name') == 'agv'):
                if confirmed_since is None:
                    confirmed_since = time.monotonic()
                elif time.monotonic() - confirmed_since >= 3:
                    print('AGV locked follow enabled; mouse wheel zoom remains available.', flush=True)
                    return
            else:
                confirmed_since = None
        except (subprocess.TimeoutExpired, json.JSONDecodeError):
            confirmed_since = None
        time.sleep(.5)
    raise RuntimeError('Gazebo GUI locked follow did not become ready within 90 seconds')


if __name__ == '__main__':
    main()
