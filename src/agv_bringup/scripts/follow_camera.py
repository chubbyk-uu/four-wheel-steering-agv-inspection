#!/usr/bin/env python3
"""Enable position following without forcing the GUI camera to look at the AGV."""
import json
import subprocess
import time


def main():
    deadline = time.monotonic() + 90
    confirmed_since = None
    while time.monotonic() < deadline:
        # CameraTracking retains the configured follow_offset. Do not resend
        # while acknowledged: user orientation changes must remain intact.
        # A first acknowledgement may precede completion of scene loading.
        if confirmed_since is None:
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
                if confirmed_since is None:
                    confirmed_since = time.monotonic()
                elif time.monotonic() - confirmed_since >= 3:
                    print('AGV position follow enabled; mouse orientation remains free.', flush=True)
                    return
            else:
                confirmed_since = None
        except (subprocess.TimeoutExpired, json.JSONDecodeError):
            confirmed_since = None
        time.sleep(.5)
    raise RuntimeError('Gazebo GUI free-look follow did not become ready within 90 seconds')


if __name__ == '__main__':
    main()
