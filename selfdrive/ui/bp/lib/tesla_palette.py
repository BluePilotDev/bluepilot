"""Shared Light/Dark palettes for BluePilot's Tesla-inspired code themes."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import pyray as rl


# Sampled from two Tesla driving-visualization references. The newer filled
# path centers near #0071FE; the classic thin path centers near #4495EC.
TESLA_PATH_BLUE_DEEP = (0, 113, 254)
TESLA_PATH_BLUE_MID = (42, 121, 248)
TESLA_PATH_BLUE_LIGHT = (68, 149, 236)
TESLA_PATH_BLUE_CYCLE = (
  TESLA_PATH_BLUE_DEEP,
  TESLA_PATH_BLUE_MID,
  TESLA_PATH_BLUE_LIGHT,
  TESLA_PATH_BLUE_MID,
)

# ui_state.light_sensor is 100 - wide-camera exposure percentage. Require a
# sustained low reading before darkening, then a materially brighter reading
# before returning to Light so tunnels and shadows cannot chatter the palette.
TESLA_DARK_ENTER_LIGHT = 35.0
TESLA_LIGHT_ENTER_LIGHT = 50.0
TESLA_PALETTE_DWELL_S = 3.0
TESLA_PALETTE_TRANSITION_S = 0.8


@dataclass(frozen=True)
class TeslaPalette:
  sky_top: rl.Color
  sky_horizon: rl.Color
  ground_horizon: rl.Color
  ground_near: rl.Color
  road_surface: rl.Color
  road_shoulder: rl.Color
  lane_inner: rl.Color
  lane_outer: rl.Color
  lane_change: rl.Color
  road_edge: rl.Color
  path_blue: rl.Color
  path_cyan: rl.Color
  path_disengaged: rl.Color
  blindspot: rl.Color
  set_speed: rl.Color
  max_active: rl.Color
  max_inactive: rl.Color
  wheel_active: rl.Color


LIGHT_PALETTE = TeslaPalette(
  sky_top=rl.Color(213, 218, 221, 255),
  sky_horizon=rl.Color(235, 237, 238, 255),
  ground_horizon=rl.Color(211, 215, 217, 255),
  ground_near=rl.Color(190, 196, 199, 255),
  road_surface=rl.Color(169, 175, 179, 255),
  road_shoulder=rl.Color(126, 134, 139, 190),
  lane_inner=rl.Color(245, 247, 248, 210),
  lane_outer=rl.Color(222, 226, 228, 125),
  lane_change=rl.Color(40, 145, 238, 220),
  road_edge=rl.Color(103, 112, 118, 175),
  path_blue=rl.Color(*TESLA_PATH_BLUE_MID, 185),
  path_cyan=rl.Color(*TESLA_PATH_BLUE_LIGHT, 115),
  path_disengaged=rl.Color(92, 108, 119, 80),
  blindspot=rl.Color(224, 76, 66, 255),
  set_speed=rl.Color(174, 180, 184, 255),
  max_active=rl.Color(40, 145, 238, 255),
  max_inactive=rl.Color(135, 142, 147, 255),
  wheel_active=rl.Color(25, 105, 190, 255),
)


DARK_PALETTE = TeslaPalette(
  sky_top=rl.Color(20, 25, 34, 255),
  sky_horizon=rl.Color(35, 43, 56, 255),
  ground_horizon=rl.Color(26, 31, 39, 255),
  ground_near=rl.Color(15, 18, 24, 255),
  road_surface=rl.Color(24, 28, 34, 255),
  road_shoulder=rl.Color(82, 91, 98, 175),
  lane_inner=rl.Color(235, 238, 239, 225),
  lane_outer=rl.Color(177, 185, 190, 140),
  lane_change=rl.Color(72, 166, 238, 235),
  road_edge=rl.Color(78, 89, 96, 170),
  path_blue=rl.Color(*TESLA_PATH_BLUE_MID, 200),
  path_cyan=rl.Color(*TESLA_PATH_BLUE_LIGHT, 125),
  path_disengaged=rl.Color(44, 62, 74, 105),
  blindspot=rl.Color(224, 76, 66, 255),
  set_speed=rl.Color(196, 201, 204, 255),
  max_active=rl.Color(72, 166, 238, 255),
  max_inactive=rl.Color(132, 140, 145, 255),
  wheel_active=rl.Color(42, 126, 210, 255),
)


def blend_color(light: rl.Color, dark: rl.Color, dark_fraction: float) -> rl.Color:
  amount = max(0.0, min(float(dark_fraction), 1.0))
  return rl.Color(*[
    round(a + (b - a) * amount)
    for a, b in zip((light.r, light.g, light.b, light.a),
                    (dark.r, dark.g, dark.b, dark.a), strict=True)
  ])


def palette_for_dark_fraction(dark_fraction: float) -> TeslaPalette:
  amount = max(0.0, min(float(dark_fraction), 1.0))
  if amount == 0.0:
    return LIGHT_PALETTE
  if amount == 1.0:
    return DARK_PALETTE
  return TeslaPalette(**{
    name: blend_color(getattr(LIGHT_PALETTE, name), getattr(DARK_PALETTE, name), amount)
    for name in TeslaPalette.__dataclass_fields__
  })


def tesla_wheel_color(enabled: bool, longitudinal_active: bool,
                      dark_fraction: float, alpha: int = 255) -> rl.Color:
  """Tint either built-in wheel style only while Tesla longitudinal is active."""
  alpha = max(0, min(int(alpha), 255))
  if not (enabled and longitudinal_active):
    return rl.Color(255, 255, 255, alpha)

  active = palette_for_dark_fraction(dark_fraction).wheel_active
  return rl.Color(active.r, active.g, active.b, alpha)


class TeslaAutoPaletteState:
  """Time-based Light/Dark controller shared by every Tesla renderer."""

  def __init__(self):
    self.dark_mode = False
    self.dark_fraction = 0.0
    self._candidate_since: float | None = None
    self._last_update: float | None = None

  def reset(self) -> None:
    self.dark_mode = False
    self.dark_fraction = 0.0
    self._candidate_since = None
    self._last_update = None

  def update(self, light_sensor: float, now: float | None = None) -> float:
    now = time.monotonic() if now is None else float(now)
    dt = 0.0 if self._last_update is None else max(0.0, min(now - self._last_update, 0.25))
    self._last_update = now

    valid = math.isfinite(light_sensor) and light_sensor >= 0.0
    wants_change = valid and (
      (not self.dark_mode and light_sensor <= TESLA_DARK_ENTER_LIGHT) or
      (self.dark_mode and light_sensor >= TESLA_LIGHT_ENTER_LIGHT)
    )
    if wants_change:
      if self._candidate_since is None:
        self._candidate_since = now
      elif now - self._candidate_since >= TESLA_PALETTE_DWELL_S:
        self.dark_mode = not self.dark_mode
        self._candidate_since = None
    else:
      self._candidate_since = None

    target = 1.0 if self.dark_mode else 0.0
    step = dt / TESLA_PALETTE_TRANSITION_S
    if self.dark_fraction < target:
      self.dark_fraction = min(target, self.dark_fraction + step)
    elif self.dark_fraction > target:
      self.dark_fraction = max(target, self.dark_fraction - step)
    return self.dark_fraction


def tesla_blue_cycle_color(phase: float, alpha: int) -> rl.Color:
  """Interpolate smoothly through Tesla's deep, mid, and light path blues."""
  wrapped = phase % 1.0
  scaled = wrapped * len(TESLA_PATH_BLUE_CYCLE)
  index = int(scaled) % len(TESLA_PATH_BLUE_CYCLE)
  fraction = scaled - int(scaled)
  start = TESLA_PATH_BLUE_CYCLE[index]
  end = TESLA_PATH_BLUE_CYCLE[(index + 1) % len(TESLA_PATH_BLUE_CYCLE)]
  channels = [round(a + (b - a) * fraction) for a, b in zip(start, end, strict=True)]
  return rl.Color(*channels, alpha)


def tesla_path_gradient_colors(palette: TeslaPalette, phase: float | None = None,
                                opacity: float = 1.0) -> list[rl.Color]:
  """Return static Tesla blue or an animated all-blue shade sequence."""
  opacity = max(0.0, min(float(opacity), 1.0))

  def faded(color: rl.Color) -> rl.Color:
    return rl.Color(color.r, color.g, color.b, round(color.a * opacity))

  if phase is None:
    far = palette.path_cyan
    return [
      faded(palette.path_blue),
      faded(far),
      rl.Color(far.r, far.g, far.b, 0),
    ]

  near = tesla_blue_cycle_color(phase, palette.path_blue.a)
  middle = tesla_blue_cycle_color(phase + 1.0 / 3.0, max(palette.path_cyan.a, 145))
  far = tesla_blue_cycle_color(phase + 2.0 / 3.0, palette.path_cyan.a)
  return [faded(near), faded(middle), faded(far), rl.Color(far.r, far.g, far.b, 0)]
