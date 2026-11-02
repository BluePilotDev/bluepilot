#!/usr/bin/env python3
from __future__ import annotations

import os
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import time

import cereal.messaging as messaging
from openpilot.bluepilot.vasm.cpu_throttle import CpuThrottle
from openpilot.bluepilot.vasm.config import normalize_annotation_config
from openpilot.bluepilot.vasm.state import get_memory_params
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper, set_core_affinity
from openpilot.system.hardware import PC


BASE_INTERVAL = 0.500
FOLLOWUP_INTERVAL = 0.200
FOLLOWUP_WINDOW = 1.5
PARAM_REFRESH_INTERVAL = 2.0
STATUS_LOG_INTERVAL = 10.0


def _run_disabled(reason: str) -> None:
  """Keep this optional managed process alive while its outputs fail closed."""
  print(f"[VASM] disabled: {reason}")
  while True:
    time.sleep(60.0)


class VASMDaemon:
  def __init__(self, cv2_module, numpy_module, inference_class):
    from msgq.visionipc import VisionIpcClient, VisionStreamType

    self.params = Params()
    self.params_memory = get_memory_params(self.params)
    self.sm = messaging.SubMaster(["deviceState", "managerState"])
    self.VisionIpcClient = VisionIpcClient
    self.stream_type = VisionStreamType.VISION_STREAM_DRIVER
    self.client = None

    self.cv2 = cv2_module
    self.np = numpy_module
    self.inference = inference_class()
    self.throttle = CpuThrottle()
    self.enabled = False
    self.annotation_loaded = False
    self.confidence_threshold = 0.85
    self.smooth_seconds = 0.2
    self._cache_params(force=True)

    self.last_inference_at = 0.0
    self.last_inference_at_side = {"left": 0.0, "right": 0.0}
    self.current_side = "left"
    self.followup_until = 0.0
    self._last_param_refresh = 0.0
    self._last_status_log = 0.0
    self._inference_count = 0
    self._last_state = (False, False, -1.0, -1.0)
    self._last_heartbeat = 0.0
    self._affinity_set = False
    self._other_daemon_running_prev: bool | None = None

    self._publish(False, False, 0.0, 0.0, 0, force=True)
    print(f"[VASM] Started (enabled={self.enabled}, model_valid={self.inference.valid}, annotation={self.annotation_loaded})")

  def _cache_params(self, force: bool = False) -> None:
    was_enabled = self.enabled
    self.enabled = self.params.get_bool("VASMEnabled")
    self.confidence_threshold = float(self.params.get("VASMConfidenceThreshold", return_default=True) or 0.85)
    self.smooth_seconds = float(self.params.get("VASMSmoothSeconds", return_default=True) or 0.2)
    if self.enabled and (force or not was_enabled):
      if not self.inference.valid:
        self.inference.load()
      self._load_annotation_config()

  def _load_annotation_config(self) -> None:
    try:
      config = normalize_annotation_config(self.params.get("VASMAnnotationConfig", return_default=True) or {})
      self.annotation_loaded = self.inference.load_config(config)
    except (TypeError, ValueError):
      self.annotation_loaded = False
    if not self.annotation_loaded:
      print("[VASM] No valid window annotation; configure V-ASM in BluePilot Portal")

  def _connect_camera(self) -> bool:
    if self.client is not None and self.client.is_connected():
      return True
    try:
      available = self.VisionIpcClient.available_streams("camerad", block=False)
    except Exception:
      available = []
    if self.stream_type not in available:
      return False
    if self.client is None:
      self.client = self.VisionIpcClient("camerad", self.stream_type, True)
    if not self.client.is_connected():
      self.client.connect(True)
    return self.client.is_connected()

  def _other_vision_daemon_running(self) -> bool:
    if not self.sm.valid.get("managerState", False):
      return False
    return any(process.name == "speed_limit_vision" and process.running for process in self.sm["managerState"].processes)

  def _update_core_affinity(self) -> None:
    other_running = self._other_vision_daemon_running()
    if other_running != self._other_daemon_running_prev:
      self._affinity_set = False
      self._other_daemon_running_prev = other_running
    if not self._affinity_set:
      set_core_affinity([2] if other_running else [0, 1, 2])
      self._affinity_set = True

  def _inference_interval(self, now: float) -> tuple[float, float]:
    base = FOLLOWUP_INTERVAL if now < self.followup_until else BASE_INTERVAL
    cpu_usage = list(self.sm["deviceState"].cpuUsagePercent) if self.sm.valid.get("deviceState", False) else []
    factor = self.throttle.update(cpu_usage)
    return base * factor, factor

  def _publish(self, left_active: bool, right_active: bool, left_confidence: float,
               right_confidence: float, timestamp_sof: int, force: bool = False) -> None:
    # Driver-camera frames are mirrored relative to the road view.
    ui_left_active, ui_right_active = right_active, left_active
    ui_left_confidence, ui_right_confidence = right_confidence, left_confidence
    state = (ui_left_active, ui_right_active, round(ui_left_confidence, 3), round(ui_right_confidence, 3))
    now = time.monotonic()
    state_changed = force or state != self._last_state
    if state_changed:
      self._last_state = state
      self.params_memory.put_bool("VASMLeftActive", ui_left_active)
      self.params_memory.put_bool("VASMRightActive", ui_right_active)
      self.params_memory.put("VASMLeftConfidence", float(state[2]))
      self.params_memory.put("VASMRightConfidence", float(state[3]))
      self.params_memory.put("VASMTimestampEof", int(timestamp_sof))
    if state_changed or now - self._last_heartbeat >= 1.0:
      # Publish freshness last so readers never accept an older state as new.
      self.params_memory.put("VASMHeartbeat", float(now))
      self._last_heartbeat = now

  def _publish_inactive(self, reset_inference: bool = False) -> None:
    if reset_inference:
      self.inference.reset_state()
    self._publish(False, False, 0.0, 0.0, 0)

  def run(self) -> None:
    ratekeeper = Ratekeeper(10, None)
    while True:
      try:
        now = time.monotonic()
        self.sm.update(0)
        if not PC:
          self._update_core_affinity()
        if now - self._last_param_refresh >= PARAM_REFRESH_INTERVAL:
          self._last_param_refresh = now
          self._cache_params()

        onroad = self.sm["deviceState"].started if self.sm.valid.get("deviceState", False) else False
        if not onroad or not self.enabled or not self.inference.valid or not self.annotation_loaded:
          self._publish_inactive(reset_inference=not onroad or not self.enabled)
          ratekeeper.keep_time()
          continue
        if not self._connect_camera():
          self._publish_inactive()
          ratekeeper.keep_time()
          continue

        interval, throttle_factor = self._inference_interval(now)
        if self.last_inference_at and now - self.last_inference_at < interval - 0.015:
          ratekeeper.keep_time()
          continue

        buffer = None
        while True:
          next_buffer = self.client.recv(timeout_ms=0)
          if next_buffer is None:
            break
          buffer = next_buffer
        if buffer is None:
          ratekeeper.keep_time()
          continue

        last_side_time = self.last_inference_at_side[self.current_side]
        dt = now - last_side_time if last_side_time else interval
        self.last_inference_at = now
        self.last_inference_at_side[self.current_side] = now

        image = self.np.frombuffer(buffer.data, dtype=self.np.uint8).reshape((len(buffer.data) // self.client.stride, self.client.stride))
        if self.client.stride != self.client.width:
          image = image[:, :self.client.width]

        left_active, right_active = self.inference.update(
          image, self.client.width, self.client.height, dt,
          self.confidence_threshold, self.smooth_seconds, self.current_side,
        )
        self._publish(left_active, right_active, self.inference.left_confidence,
                      self.inference.right_confidence, int(self.client.timestamp_sof))
        if left_active or right_active:
          self.followup_until = now + FOLLOWUP_WINDOW
        self.current_side = "right" if self.current_side == "left" else "left"

        self._inference_count += 1
        if now - self._last_status_log >= STATUS_LOG_INTERVAL:
          status = f"[VASM] inference_count={self._inference_count} throttle={throttle_factor:.1f}x left={self.inference.left_confidence:.3f}"
          print(f"{status} right={self.inference.right_confidence:.3f}")
          self._last_status_log = now
          self._inference_count = 0
        ratekeeper.keep_time()
      except Exception as exc:
        print(f"[VASM] daemon error: {exc}")
        self._publish_inactive(reset_inference=True)
        time.sleep(1.0)


def main() -> None:
  try:
    import cv2
    import numpy as np
    from openpilot.bluepilot.vasm.inference import VASMInference
    try:
      os.nice(19)
    except OSError:
      pass
    cv2.setNumThreads(1)
    VASMDaemon(cv2, np, VASMInference).run()
  except (ImportError, OSError) as exc:
    # Optional native dependencies can be absent in a stale Quickboot environment.
    # Remain alive and fail closed so this experimental aid never blocks engagement.
    _run_disabled(f"runtime dependency unavailable: {exc}")
  except Exception as exc:
    # Initialization errors happen before the daemon's guarded run loop. Treat them as
    # a disabled optional aid rather than a mandatory driving-process failure.
    _run_disabled(f"startup failed: {exc}")


if __name__ == "__main__":
  main()
