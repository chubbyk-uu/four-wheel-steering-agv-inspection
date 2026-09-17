#!/usr/bin/env python3
"""Matched production OptiX captures for the local branching-crack trial."""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image
import xacro

from bake_concrete_road import sha
from check_concrete_correction import display_srgb

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT/'src/agv_linescan'))  # after the installed package, so the compiled _obj_arrays is used
from agv_linescan.robot_scene import export, split_visual_links


def stress_capture(scenes, out, robot):
    dest = out/'branched_stress'
    with (out/'branched_stress.log').open('w') as log:
        subprocess.run([str(ROOT/'build/agv_linescan/benchmark_grooves'),
            str(scenes/'branched/manifest.json'),str(robot),str(ROOT/'build/agv_linescan/agv_optix_scan.ptx'),
            str(ROOT/'src/agv_description/config/linescan.yaml'),str(dest),'8192','16'],
            stdout=log,stderr=subprocess.STDOUT,check=True,timeout=240)
    report = json.loads((dest/'timing.json').read_text())
    assert report['measurements'][0]['active_lines_per_second'] >= 11000
    return report


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scene-root', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    scenes = Path(a.scene_root).resolve()
    out = Path(a.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    robot = split_visual_links(xacro.process_file(str(ROOT/'src/agv_description/urdf/agv.urdf.xacro')).toxml())
    rp = export(robot, out/'robot')
    r = json.loads(rp.read_text())
    r['groups'] = [g for g in r['groups'] if g['name'] == r['led_emitters']['link']]
    rp.write_text(json.dumps(r))
    build = json.loads((scenes/'build.json').read_text())
    report = dict(scope='local production OptixScene + synchronous readback; no GZ/physics/ROS/archive timing; noise and PRNU disabled in matched fixture',
                  full_11khz_acceptance=False, target_hz=11000, build=build, cases={},
                  orders=[['old','branched'],['branched','old']], images_deterministic=True)
    for repeat, order in enumerate(report['orders']):
        for name in order:
            dest = out/f'{name}_{repeat}'
            command = [str(ROOT/'build/agv_linescan/benchmark_grooves'),
                       str(scenes/name/'manifest.json'), str(rp),
                       str(ROOT/'build/agv_linescan/agv_optix_scan.ptx'),
                       str(ROOT/'src/agv_description/config/linescan.yaml'), str(dest)]
            with (out/f'{name}_{repeat}.log').open('w') as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=180)
            run = json.loads((dest/'timing.json').read_text())
            assert all(m['active_lines_per_second'] >= 11000 for m in run['measurements']), 'local active sampling below 11 kHz'
            report['cases'].setdefault(name, []).append(run)
    arrays = {}
    for name in ('old','branched'):
        path = out/f'{name}_0/samples_16.pgm'
        arrays[name] = np.array(Image.open(path))
        assert np.array_equal(arrays[name], np.array(Image.open(out/f'{name}_1/samples_16.pgm')))
        report[name+'_raw_sha256'] = sha(path)
    delta = np.abs(arrays['branched'].astype('int16')-arrays['old'].astype('int16'))
    report['image_difference'] = dict(changed_pixels=int((delta>2).sum()), max_dn=int(delta.max()))
    report['added_triangles'] = build['cases']['branched']['triangles']-build['cases']['old']['triangles']
    old = [m for run in report['cases']['old'] for m in run['measurements'] if m['samples']==16]
    new = [m for run in report['cases']['branched'] for m in run['measurements'] if m['samples']==16]
    report['comparison_16_samples'] = dict(
        old_rate_range=[min(m['active_lines_per_second'] for m in old), max(m['active_lines_per_second'] for m in old)],
        new_rate_range=[min(m['active_lines_per_second'] for m in new), max(m['active_lines_per_second'] for m in new)],
        extra_explicit_device_MiB=(new[0]['allocated_device_bytes']-old[0]['allocated_device_bytes'])/2**20,
        old_mean_batch_ms=float(np.mean([256000/m['active_lines_per_second'] for m in old])),
        new_mean_batch_ms=float(np.mean([256000/m['active_lines_per_second'] for m in new])))
    report['sampling_stress'] = stress_capture(scenes,out,rp)
    assert np.array_equal(arrays['branched'],np.array(Image.open(out/'branched_stress/samples_16.pgm')))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    panels = {name:display_srgb(pixels,4) for name,pixels in arrays.items()}
    fig, axes = plt.subplots(1,2,figsize=(9,9))
    for ax, name in zip(axes, ('old','branched')):
        ax.imshow(panels[name],cmap='gray',vmin=0,vmax=255,extent=[-.624,.624,3.7,1.3])
        ax.set_title(name+' / actual OptiX scan')
        ax.set_xlabel('across camera (m, approximate before correction)')
        ax.set_ylabel('along scan (m)')
    fig.tight_layout()
    fig.savefig(out/'overview.png',dpi=160)
    plt.close(fig)
    # About 0.3 x 0.3 m at a prominent fork; crop actual raw sensor pixels, no drawing.
    row = round((2.43-1.3)/(.00029296875))
    col = 2048
    fig, axes = plt.subplots(1,2,figsize=(12,6))
    for ax, name in zip(axes, ('old','branched')):
        ax.imshow(panels[name][row-512:row+512,col-512:col+512],cmap='gray',vmin=0,vmax=255,interpolation='nearest')
        ax.set_title(name+' / same 1024 x 1024 pixel crop')
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(out/'fork_crop.png',dpi=180)
    plt.close(fig)
    report['figures'] = dict(transfer='sRGB display only, pedestal 4 DN removed', exposure_us=20,
                            rows=8192,width=4096,calibrated=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report['comparison_16_samples']),flush=True)


if __name__ == '__main__':
    main()
