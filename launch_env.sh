#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.9"
fi

export STAGING_ROOT="/data/safe_staging"

# BluePilot: connect backend (BPConnectBackend). openpilot already reads API_HOST / ATHENA_HOST
# everywhere it talks to the backend (common/api/comma_connect.py, system/athena/athenad.py,
# registration and the uploader), so setting them here redirects all of it with no other code
# changes. Takes effect on reboot.
#   0 / unset = Comma Connect (stock defaults)
#   1         = Konik Stable (api.konik.ai / athena.konik.ai)
#   2         = Offline Mode (bogus hosts — uploads can never succeed)
# Legacy: migrate BPUseKonik=1 -> BPConnectBackend=1 when the new param file is missing.
BP_CONNECT_BACKEND="$(cat /data/params/d/BPConnectBackend 2>/dev/null)"
if [ -z "$BP_CONNECT_BACKEND" ] && [ "$(cat /data/params/d/BPUseKonik 2>/dev/null)" = "1" ]; then
  BP_CONNECT_BACKEND="1"
  mkdir -p /data/params/d
  printf '%s' "1" > /data/params/d/BPConnectBackend
fi
if [ "$BP_CONNECT_BACKEND" = "1" ]; then
  export API_HOST="https://api.konik.ai"
  export ATHENA_HOST="wss://athena.konik.ai"
elif [ "$BP_CONNECT_BACKEND" = "2" ]; then
  # RFC 2606 .invalid never resolves, so offline mode can never egress
  export API_HOST="https://api.invalid"
  export ATHENA_HOST="wss://athena.invalid"
fi
# End BluePilot
