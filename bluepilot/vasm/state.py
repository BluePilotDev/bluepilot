from __future__ import annotations

import platform
import time

from openpilot.common.params import Params, UnknownKeyName


VASM_STATE_MAX_AGE = 4.0


def get_memory_params(persistent_params: Params | None = None) -> Params:
  """Return the shared-memory Params store used for short-lived V-ASM state."""
  if platform.system() == "Darwin":
    return persistent_params or Params()
  return Params("/dev/shm/params")


def get_vasm_blindspots(params_memory: Params, now: float | None = None) -> tuple[bool, bool]:
  """Read fresh V-ASM detections, ignoring stale state after a daemon failure."""
  now = time.monotonic() if now is None else now
  try:
    heartbeat = float(params_memory.get("VASMHeartbeat") or 0.0)
    if heartbeat <= 0.0 or now - heartbeat > VASM_STATE_MAX_AGE:
      return False, False
    return params_memory.get_bool("VASMLeftActive"), params_memory.get_bool("VASMRightActive")
  except (TypeError, ValueError, UnknownKeyName):
    return False, False


def combined_blindspots(car_state, params_memory: Params, now: float | None = None) -> tuple[bool, bool]:
  vision_left, vision_right = get_vasm_blindspots(params_memory, now)
  return bool(car_state.leftBlindspot or vision_left), bool(car_state.rightBlindspot or vision_right)
