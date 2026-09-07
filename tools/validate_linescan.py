#!/usr/bin/env python3
"""Full-width synthetic encoder testbench; no ROS or GZ required."""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/agv_linescan'))
from agv_linescan.core import Camera, Trigger, Blocks


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='/tmp/agv_linescan_bench')
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    c = yaml.safe_load((ROOT/'src/agv_description/config/linescan.yaml').read_text())
    camera = Camera(c)
    emitted = []
    def emit(pixels, meta):
        name = f'block_{meta["block_id"]:03d}'
        Image.fromarray(pixels).save(out/(name+'_raw.png'))
        corrected, mask = camera.rectify(pixels)
        Image.fromarray(corrected).save(out/(name+'_rectified.png'))
        (out/(name+'.json')).write_text(json.dumps(meta, indent=2)+'\n')
        emitted.append(meta)
        if meta['block_id'] == 0:
            Image.fromarray(pixels).resize((1024, round(pixels.shape[0]*1024/camera.width))).save(out/'raw_preview.png')
            Image.fromarray(corrected).resize((1024, round(pixels.shape[0]*1024/camera.width))).save(out/'rectified_preview.png')
    blocks = Blocks(camera, emit)
    trigger = Trigger(camera.spacing)
    trigger.update(0, 0)
    start = time.perf_counter()
    # Two full blocks plus a tail at 0.1 m/s. Encoder and true movement are independent inputs.
    speed = .1
    count = 0
    distance = camera.spacing*(2*camera.rows+204)
    duration = distance/speed
    steps = int(np.ceil(duration/.01))
    for step in range(1, steps+1):
        t = min(step*.01, duration)
        for trigger_t, distance in trigger.update(t, speed*t):
            a = ([speed*trigger_t, 0, .36], [0, 0, 0, 1])
            b = ([speed*(trigger_t+camera.exposure), 0, .36], [0, 0, 0, 1])
            im, valid = camera.expose(a, b)
            mid_t = trigger_t+camera.exposure/2
            origin, r = camera.optical_pose(([speed*mid_t, 0, .36], a[1]))
            blocks.add(im, valid, dict(time_s=mid_t, encoder_distance_m=distance,
                       scan_direction=1, camera_position_world_m=origin.tolist(), camera_rotation_world=r.tolist()))
            count += 1
    blocks.end_segment('test_end')
    elapsed = time.perf_counter()-start
    assert count == 2*camera.rows+204, count
    assert [m['rows'] for m in emitted] == [camera.rows, camera.rows, 204]
    assert all(m['invalid_pixels'] == 0 for m in emitted)
    assert all(b['first']['global_line'] == a['last']['global_line']+1 for a, b in zip(emitted, emitted[1:]))
    result = dict(passed=True, backend='analytic_grid_testbench', lines=count,
                  rows=[m['rows'] for m in emitted], wall_seconds=elapsed,
                  effective_lines_per_wall_second=count/elapsed,
                  scan_speed_m_s=speed, simulated_seconds=duration,
                  note='Includes PNG output and rectification; not a GZ/GPU benchmark.')
    (out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
