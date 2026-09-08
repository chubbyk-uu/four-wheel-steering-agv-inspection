#!/usr/bin/env python3
"""Enable position following without forcing the GUI camera to look at the AGV."""
import json
import subprocess
import time


def main():
    deadline = time.monotonic() + 90
    while time.monotonic() < deadline:
        # CameraTracking retains the configured follow_offset. Do not resend
        # after acknowledgement: user orientation changes must remain intact.
        subprocess.run([
            'gz', 'topic', '-t', '/gui/track', '-m', 'gz.msgs.CameraTrack',
            '-p', 'track_mode: FOLLOW_FREE_LOOK follow_target { name: "agv" }'],
            capture_output=True, text=True, timeout=5, check=True)
        try:
            result = subprocess.run([
                'gz', 'topic', '-t', '/gui/currently_tracked', '-e', '-n', '1',
                '--json-output'], capture_output=True, text=True, timeout=3, check=True)
            status = json.loads(result.stdout)
            if (status.get('trackMode') == 'FOLLOW_FREE_LOOK'
                    and status.get('followTarget', {}).get('name') == 'agv'):
                print('AGV position follow enabled; mouse orientation remains free.', flush=True)
                return
        except (subprocess.TimeoutExpired, json.JSONDecodeError):
            pass
        time.sleep(.5)
    raise RuntimeError('Gazebo GUI free-look follow did not become ready within 90 seconds')


if __name__ == '__main__':
    main()
