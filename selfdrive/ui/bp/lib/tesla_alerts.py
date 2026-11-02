"""Shared Tesla alert colors, geometry, and code-native severity glyphs."""

from __future__ import annotations

from dataclasses import dataclass

import pyray as rl
from cereal import log

from openpilot.selfdrive.ui.bp.lib.tesla_palette import blend_color

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus

TESLA_ALERT_LIGHT_BACKGROUND = rl.Color(240, 239, 237, 248)
TESLA_ALERT_DARK_BACKGROUND = rl.Color(20, 25, 33, 248)
TESLA_ALERT_LIGHT_TEXT = rl.Color(48, 51, 55, 255)
TESLA_ALERT_DARK_TEXT = rl.Color(236, 239, 241, 255)
TESLA_ALERT_LIGHT_SECONDARY = rl.Color(94, 99, 104, 255)
TESLA_ALERT_DARK_SECONDARY = rl.Color(174, 181, 187, 255)
TESLA_ALERT_LIGHT_BLUE = rl.Color(40, 145, 238, 255)
TESLA_ALERT_DARK_BLUE = rl.Color(72, 166, 238, 255)
TESLA_ALERT_LIGHT_AMBER = rl.Color(217, 138, 31, 255)
TESLA_ALERT_DARK_AMBER = rl.Color(242, 174, 55, 255)
TESLA_ALERT_LIGHT_RED = rl.Color(202, 39, 57, 255)
TESLA_ALERT_DARK_RED = rl.Color(242, 82, 79, 255)


@dataclass(frozen=True)
class TeslaAlertColors:
  background: rl.Color
  text: rl.Color
  secondary: rl.Color
  accent: rl.Color


def _scaled_alpha(color: rl.Color, alpha: float) -> rl.Color:
  return rl.Color(color.r, color.g, color.b, round(color.a * max(0.0, min(alpha, 1.0))))


def tesla_alert_colors(status: int, dark_fraction: float, alpha: float = 1.0) -> TeslaAlertColors:
  background = blend_color(TESLA_ALERT_LIGHT_BACKGROUND, TESLA_ALERT_DARK_BACKGROUND, dark_fraction)
  text = blend_color(TESLA_ALERT_LIGHT_TEXT, TESLA_ALERT_DARK_TEXT, dark_fraction)
  secondary = blend_color(TESLA_ALERT_LIGHT_SECONDARY, TESLA_ALERT_DARK_SECONDARY, dark_fraction)
  if status == AlertStatus.critical:
    accent = blend_color(TESLA_ALERT_LIGHT_RED, TESLA_ALERT_DARK_RED, dark_fraction)
  elif status == AlertStatus.userPrompt:
    accent = blend_color(TESLA_ALERT_LIGHT_AMBER, TESLA_ALERT_DARK_AMBER, dark_fraction)
  else:
    accent = blend_color(TESLA_ALERT_LIGHT_BLUE, TESLA_ALERT_DARK_BLUE, dark_fraction)
  return TeslaAlertColors(*(_scaled_alpha(color, alpha) for color in (background, text, secondary, accent)))


def tesla_alert_panel_rect(rect: rl.Rectangle, size: int, compact: bool = False) -> rl.Rectangle:
  """Return device-appropriate alert geometry without changing alert priority."""
  if size == AlertSize.full:
    return rl.Rectangle(rect.x, rect.y, rect.width, rect.height)

  if compact:
    height = rect.height * (0.48 if size == AlertSize.small else 0.74)
    return rl.Rectangle(rect.x, rect.y, rect.width, height)

  side_margin = 60.0 if size == AlertSize.small else 84.0
  bottom_margin = 40.0 if size == AlertSize.small else 54.0
  height = 150.0 if size == AlertSize.small else 330.0
  return rl.Rectangle(
    rect.x + side_margin,
    rect.y + rect.height - height - bottom_margin,
    rect.width - side_margin * 2.0,
    height,
  )


def draw_tesla_alert_glyph(center: rl.Vector2, radius: float, status: int,
                           dark_fraction: float, alpha: float = 1.0) -> None:
  """Draw a Tesla-like warning triangle or informational circle."""
  accent = tesla_alert_colors(status, dark_fraction, alpha).accent
  stroke = max(2.0, radius * 0.10)
  if status == AlertStatus.normal:
    rl.draw_ring(center, radius - stroke, radius, 0.0, 360.0, 48, accent)
    rl.draw_line_ex(
      rl.Vector2(center.x, center.y - radius * 0.22),
      rl.Vector2(center.x, center.y + radius * 0.34),
      stroke,
      accent,
    )
    rl.draw_circle_v(rl.Vector2(center.x, center.y - radius * 0.52), stroke * 0.62, accent)
    return

  top = rl.Vector2(center.x, center.y - radius)
  left = rl.Vector2(center.x - radius * 0.90, center.y + radius * 0.62)
  right = rl.Vector2(center.x + radius * 0.90, center.y + radius * 0.62)
  for start, end in ((top, left), (left, right), (right, top)):
    rl.draw_line_ex(start, end, stroke, accent)
  rl.draw_line_ex(
    rl.Vector2(center.x, center.y - radius * 0.35),
    rl.Vector2(center.x, center.y + radius * 0.15),
    stroke,
    accent,
  )
  rl.draw_circle_v(rl.Vector2(center.x, center.y + radius * 0.38), stroke * 0.62, accent)
