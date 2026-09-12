#!/usr/bin/env python3
"""Fixed single-threaded work, timed, to tell a slow guest from a slow GPU path.

The simulation's real-time factor decays with the WSL instance rather than with
the code: six runs over 75 minutes fell from 0.9588 to 0.8669 and a
wsl --shutdown restored the identical run to 0.9467. Two candidates survive --
the guest losing single-thread speed (Windows 11 demotes background work to
efficiency cores on hybrid CPUs) and the GPU submission path getting slower --
and they are told apart by whether plain CPU work slows down alongside it.

Run it when a session has been up for hours and the real-time factor has sagged,
and compare against a figure taken right after a restart. It touches no GPU, no
disk and no network, so a change in it is the guest itself.
"""
import sys
import time


def burn(n):
    x = 0.
    for i in range(n):
        x += (i*i) % 7 + x*1e-12
    return x


N = 4_000_000
best = min(( (lambda t: (burn(N), time.perf_counter()-t)[1])(time.perf_counter()) ) for _ in range(5))
print('%s best of 5: %.4f s (%.2f Mops/s)' % (sys.argv[1]+' ' if len(sys.argv) > 1 else '', best, N/best/1e6))
