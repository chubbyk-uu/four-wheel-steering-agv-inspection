#!/usr/bin/env python3
"""Did this build keep the backends this machine is configured for?

A build that drops CUDA or OptiX succeeds and leaves every workspace test
green, because none of them need a GPU. The first thing that notices is a
capture, where the gz plugin throws in Configure and the server aborts. That
happened on 2026-09-18 from reusing the README's GPU-free build line on a
machine with the OptiX SDK installed.

So check the artefact rather than the build log: the installed plugin either
carries the symbols or it does not. Run it after any rebuild on a machine that
is supposed to capture.
"""
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT/'install/agv_linescan/lib/libagv_gz_linescan.so'
LIBRARIES = ('libagv_cuda_grid.a', 'libagv_runtime_material.a', 'libagv_optix_scene.a')


def symbols(path, pattern):
    out = subprocess.run(['nm', '-D', '--defined-only', str(path)],
                         capture_output=True, text=True, check=True).stdout
    return sum(1 for line in out.splitlines() if pattern in line.lower())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--expect-gpu', action='store_true',
                   help='fail if the accelerated backends are missing; default is to report only')
    p.add_argument('--output', type=Path)
    a = p.parse_args()

    if not PLUGIN.is_file():
        raise SystemExit('no installed plugin at %s; build the workspace first' % PLUGIN)
    cache = ROOT/'build/agv_linescan/CMakeCache.txt'
    flags = {}
    if cache.is_file():
        for line in cache.read_text().splitlines():
            for key in ('AGV_ENABLE_CUDA', 'AGV_ENABLE_OPTIX', 'AGV_OPTIX_SDK'):
                if line.startswith(key+':'):
                    flags[key] = line.split('=', 1)[1]
    report = dict(
        schema='agv.accelerated_backends.v1', plugin=str(PLUGIN), cmake=flags,
        cuda_symbols=symbols(PLUGIN, 'cuda'), optix_symbols=symbols(PLUGIN, 'optix'),
        static_libraries={name: (ROOT/'build/agv_linescan'/name).is_file() for name in LIBRARIES},
        sdk_present=bool(os.environ.get('AGV_OPTIX_SDK')) or (Path.home()/'opt').glob('optix-sdk-*') is not None)
    report['accelerated'] = (report['cuda_symbols'] > 0 and report['optix_symbols'] > 0
                             and all(report['static_libraries'].values()))
    report['rebuild_with'] = ('colcon build --symlink-install --cmake-args -DAGV_ENABLE_CUDA=ON '
                              '-DAGV_ENABLE_OPTIX=ON -DAGV_OPTIX_SDK="$AGV_OPTIX_SDK"')
    text = json.dumps(report, indent=2)+'\n'
    if a.output:
        a.output.write_text(text)
    print(text)
    if a.expect_gpu and not report['accelerated']:
        sys.exit('accelerated backends are missing from the installed plugin; '
                 'a capture would abort in the gz plugin. Rebuild with:\n  '+report['rebuild_with'])


if __name__ == '__main__':
    main()
