#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
sdk="${AGV_OPTIX_SDK:-${HOME}/opt/optix-sdk-9.1.0}"
out="${1:-/tmp/agv_optix_scan_build}"
mkdir -p "$out"
/usr/local/cuda/bin/nvcc -ptx -O3 -std=c++17 -arch=compute_89 \
 -I"$sdk/include" -I"$root/src/agv_linescan/include" \
 "$root/tools/optix_scan/scan.cu" -o "$out/scan.ptx"
c++ -O3 -std=c++17 -I"$sdk/include" -I/usr/local/cuda/include \
 -I"$root/src/agv_linescan/include" "$root/tools/optix_scan/bench.cpp" \
 -L/usr/local/cuda/lib64 -lcudart -ldl -lcrypto -o "$out/bench"
