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
  export AGNOS_VERSION="18.4"
fi

export STAGING_ROOT="/data/safe_staging"

# BluePilot: when the "Use Konik instead of comma connect" toggle (BPUseKonik) is on, point the
# device's connectivity at Konik (stable.konik.ai) instead of comma connect. openpilot already
# reads API_HOST / ATHENA_HOST everywhere it talks to the backend (common/api/comma_connect.py,
# system/athena/athenad.py, registration and the uploader), so setting them here redirects all of
# it with no other code changes. Only applied when the toggle is set; unset -> comma defaults.
# Takes effect on reboot; the user must re-pair the device at https://stable.konik.ai once enabled.
if [ "$(cat /data/params/d/BPUseKonik 2>/dev/null)" = "1" ]; then
  export API_HOST="https://api.konik.ai"
  export ATHENA_HOST="wss://athena.konik.ai"
fi
# End BluePilot
