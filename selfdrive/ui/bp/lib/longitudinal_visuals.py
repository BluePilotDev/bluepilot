"""Shared engagement and animation state for BluePilot longitudinal visuals."""

from __future__ import annotations

import numpy as np


RAINBOW_SPEED_CAP_MS = 35.0
RAINBOW_RATE_DIVISOR = 30.0
TESLA_BLUE_CYCLES_PER_SECOND = 0.25
TESLA_GEOMETRY_MIN_SPEED_MPS = 2.0
TESLA_GEOMETRY_MIN_DEPTH_M = 38.0
TESLA_GEOMETRY_FADE_SECONDS = 0.32


def longitudinal_control_active(sm, status) -> bool:
  """True while longitudinal control is represented as active in the UI.

  carControl.longActive covers openpilot longitudinal. BluePilot's ENGAGED and
  LONG_ONLY states also cover platforms using stock/OEM longitudinal control,
  where carControl is valid but longActive intentionally remains false.
  """
  if sm.valid.get("carControl", False) and sm["carControl"].longActive:
    return True

  status_value = getattr(status, "value", status)
  return status_value in ("engaged", "long_only")


def rainbow_cycle_rate(sm) -> float:
  """Scale rainbow animation rate with actual ego speed, stopping at standstill."""
  if not sm.valid.get("carState", False):
    return 0.0

  v_ego = max(0.0, min(float(sm["carState"].vEgo), RAINBOW_SPEED_CAP_MS))
  return v_ego / RAINBOW_RATE_DIVISOR


def legacy_rainbow_cycle_rate(sm) -> float:
  """Preserve BluePilot's pre-theme rainbow animation floor outside Tesla themes."""
  if not sm.valid.get("carState", False):
    return 1.0

  v_ego = max(2.5, min(float(sm["carState"].vEgo), RAINBOW_SPEED_CAP_MS))
  return v_ego / RAINBOW_RATE_DIVISOR


def advance_tesla_blue_phase(phase: float, speed_rate: float, fps: float) -> float:
  """Advance the Tesla-blue shade cycle, stopping naturally at zero speed."""
  if fps <= 0.0:
    return phase % 1.0
  return (phase + max(0.0, speed_rate) * TESLA_BLUE_CYCLES_PER_SECOND / fps) % 1.0


def tesla_path_mode(rainbow_enabled: bool, longitudinal_active: bool) -> str:
  if rainbow_enabled:
    return "rainbow"
  return "blue_cycle" if longitudinal_active else "blue_static"


def tesla_geometry_reliable(path_points, sm) -> bool:
  """Require useful forward reach before presenting model-backed Tesla geometry."""
  if not sm.valid.get("carState", False):
    return False
  if hasattr(sm, "alive") and not sm.alive.get("carState", False):
    return False

  speed = float(sm["carState"].vEgo)
  points = np.asarray(path_points)
  if not np.isfinite(speed) or speed < TESLA_GEOMETRY_MIN_SPEED_MPS:
    return False
  if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] < 1:
    return False
  forward = points[:, 0]
  return bool(np.all(np.isfinite(forward)) and np.max(forward) >= TESLA_GEOMETRY_MIN_DEPTH_M)


def approach_tesla_geometry_alpha(current: float, reliable: bool, fps: float) -> float:
  """Ease path visibility so parking-lot and sharp-turn dropouts never flash."""
  target = 1.0 if reliable else 0.0
  dt = 1.0 / max(1.0, float(fps))
  alpha = 1.0 - float(np.exp(-dt / TESLA_GEOMETRY_FADE_SECONDS))
  return float(np.clip(current + (target - current) * alpha, 0.0, 1.0))
