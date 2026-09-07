#!/usr/bin/env python3
"""Enable the configured follow target once Gazebo GUI transport is ready."""
import subprocess
import time

deadline = time.monotonic() + 90
while time.monotonic() < deadline:
    result = subprocess.run([
        'gz', 'service', '-s', '/gui/follow', '--reqtype', 'gz.msgs.StringMsg',
        '--reptype', 'gz.msgs.Boolean', '--timeout', '2000',
        '--req', 'data: "agv"'], capture_output=True, text=True, timeout=5)
    if result.returncode == 0 and 'data: true' in result.stdout:
        print('AGV camera follow enabled.', flush=True)
        break
    time.sleep(1)
else:
    raise RuntimeError('Gazebo GUI follow service did not become ready within 90 seconds')
