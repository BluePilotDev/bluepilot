import math

import pyray as rl
from cereal import log

from openpilot.selfdrive.ui.bp.lib.tesla_alerts import (
  TESLA_ALERT_DARK_BACKGROUND,
  TESLA_ALERT_DARK_RED,
  TESLA_ALERT_LIGHT_AMBER,
  TESLA_ALERT_LIGHT_BACKGROUND,
  TESLA_ALERT_LIGHT_BLUE,
  TESLA_ALERT_LIGHT_RED,
  tesla_alert_colors,
  tesla_alert_panel_rect,
)

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus


def _rgba(color: rl.Color) -> tuple[int, int, int, int]:
  return color.r, color.g, color.b, color.a


def test_tesla_alert_severity_colors_are_distinct() -> None:
  normal = tesla_alert_colors(AlertStatus.normal, 0.0)
  prompt = tesla_alert_colors(AlertStatus.userPrompt, 0.0)
  critical = tesla_alert_colors(AlertStatus.critical, 0.0)

  assert _rgba(normal.accent) == _rgba(TESLA_ALERT_LIGHT_BLUE)
  assert _rgba(prompt.accent) == _rgba(TESLA_ALERT_LIGHT_AMBER)
  assert _rgba(critical.accent) == _rgba(TESLA_ALERT_LIGHT_RED)


def test_tesla_alert_palette_has_exact_light_and_dark_endpoints() -> None:
  light = tesla_alert_colors(AlertStatus.critical, 0.0)
  dark = tesla_alert_colors(AlertStatus.critical, 1.0)

  assert _rgba(light.background) == _rgba(TESLA_ALERT_LIGHT_BACKGROUND)
  assert _rgba(dark.background) == _rgba(TESLA_ALERT_DARK_BACKGROUND)
  assert _rgba(dark.accent) == _rgba(TESLA_ALERT_DARK_RED)


def test_tesla_alert_alpha_scales_all_layers() -> None:
  colors = tesla_alert_colors(AlertStatus.userPrompt, 0.5, 0.25)

  assert colors.background.a == round(248 * 0.25)
  assert colors.text.a == colors.secondary.a == colors.accent.a == round(255 * 0.25)


def test_c3x_alert_panels_preserve_full_screen_safety_alerts() -> None:
  rect = rl.Rectangle(30, 20, 2100, 1020)
  full = tesla_alert_panel_rect(rect, AlertSize.full)
  mid = tesla_alert_panel_rect(rect, AlertSize.mid)
  small = tesla_alert_panel_rect(rect, AlertSize.small)

  assert (full.x, full.y, full.width, full.height) == (30, 20, 2100, 1020)
  assert mid.height == 330
  assert small.height == 150
  assert mid.x > rect.x and small.x > rect.x
  assert mid.y + mid.height < rect.y + rect.height


def test_c4_alert_panels_use_compact_top_area() -> None:
  rect = rl.Rectangle(0, 0, 536, 240)

  assert math.isclose(tesla_alert_panel_rect(rect, AlertSize.small, compact=True).height, 240 * 0.48, abs_tol=1e-5)
  assert math.isclose(tesla_alert_panel_rect(rect, AlertSize.mid, compact=True).height, 240 * 0.74, abs_tol=1e-5)
  full = tesla_alert_panel_rect(rect, AlertSize.full, compact=True)
  assert (full.x, full.y, full.width, full.height) == (0, 0, 536, 240)
