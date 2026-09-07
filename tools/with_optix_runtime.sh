#!/usr/bin/env bash
# Experimental WSL runtime, isolated to the child process. No system driver edits.
set -euo pipefail
runtime="${AGV_OPTIX_RUNTIME:-${HOME}/opt/optix-runtime-610.57.04}"
sdk="${AGV_OPTIX_SDK:-${HOME}/opt/optix-sdk-9.1.0}"
if [[ $# -eq 0 ]]; then
  echo "Usage: bash tools/with_optix_runtime.sh COMMAND [ARG...]" >&2
  exit 2
fi
for library in libnvoptix.so.1 libnvidia-rtcore.so.610.57.04 libnvidia-gpucomp.so.610.57.04; do
  [[ -r "$runtime/$library" ]] || { echo "Missing runtime library: $runtime/$library" >&2; exit 1; }
done
# Deliberately exclude inherited paths, especially CUDA stub libraries.
exec env LD_LIBRARY_PATH="$runtime:/usr/lib/wsl/lib:$sdk/build/lib:/usr/local/cuda/lib64" "$@"
