#!/usr/bin/env python3
"""Select project-private Mesa for a child process; never edit system libraries."""
import argparse
import json
import os
from pathlib import Path
import sys


KEYS = ('LD_LIBRARY_PATH', 'LD_PRELOAD', 'LIBGL_DRIVERS_PATH',
        'GBM_BACKENDS_PATH', '__EGL_VENDOR_LIBRARY_FILENAMES',
        '__GLX_VENDOR_LIBRARY_NAME')
SNAPSHOT = 'AGV_MESA_ORIGINAL_ENV'


def environment(mode, prefix, inherited):
    env = dict(inherited)
    # Allow nested invocation to switch back without retaining our overrides.
    saved = env.pop(SNAPSHOT, None)
    if saved is not None:
        original = json.loads(saved)
        if set(original) != set(KEYS):
            raise ValueError('Invalid inherited Mesa environment snapshot')
        for key, value in original.items():
            if value is None:
                env.pop(key, None)
            elif isinstance(value, str):
                env[key] = value
            else:
                raise ValueError('Invalid inherited Mesa environment value')
    if mode == 'system':
        return env

    prefix = prefix.resolve()
    if any(c.isspace() or c == ':' for c in str(prefix)):
        raise ValueError('Mesa prefix cannot contain whitespace or colon (LD_PRELOAD syntax)')
    lib = prefix / 'lib'
    libraries = [lib / name for name in ('libgallium-25.2.8.so', 'libgbm.so.1',
                                         'libGLX_mesa.so.0', 'libEGL_mesa.so.0')]
    vendor = prefix / 'share/glvnd/egl_vendor.d/50_mesa.json'
    for item in [*libraries, vendor, lib / 'dri', lib / 'gbm']:
        if not item.exists():
            raise ValueError(f'Missing private Mesa component: {item}; use --system to revert')
    env[SNAPSHOT] = json.dumps({key: env.get(key) for key in KEYS})
    env['LD_LIBRARY_PATH'] = str(lib) + (':' + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    # Ubuntu Gallium has a distro-suffixed SONAME; our source build does not.
    # LD_LIBRARY_PATH alone cannot substitute differently named libraries and may
    # silently retain system Mesa. Preload the matching GLX/EGL/GBM frontends too.
    # Absolute preloads also survive with_optix_runtime.sh resetting library paths.
    # This is inherited by ALL subprocesses, including non-rendering Python nodes.
    # Do not simplify to library-path-only selection without checking actual maps.
    env['LD_PRELOAD'] = ':'.join(map(str, libraries)) + (':' + env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
    # D3D12-only build: no swrast/llvmpipe fallback in this private directory.
    env['LIBGL_DRIVERS_PATH'] = str(lib / 'dri')
    env['GBM_BACKENDS_PATH'] = str(lib / 'gbm')
    env['__EGL_VENDOR_LIBRARY_FILENAMES'] = str(vendor)
    env['__GLX_VENDOR_LIBRARY_NAME'] = 'mesa'
    return env


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--system', action='store_true', help='Use inherited system Mesa instead of the private build')
    parser.add_argument('--prefix', type=Path, default=Path(__file__).resolve().parents[1] / 'local_data/mesa-source-build/install')
    parser.add_argument('command', nargs=argparse.REMAINDER, help='COMMAND [ARG...] (optionally after --)')
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('A command is required')
    try:
        env = environment('system' if args.system else 'patched', args.prefix, os.environ)
        os.execvpe(command[0], command, env)
    except (ValueError, OSError) as error:
        print(f'Mesa launcher: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
