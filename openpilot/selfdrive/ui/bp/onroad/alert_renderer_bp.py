from typing import Optional
import pyray as rl
from openpilot.cereal import log

from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer, ALERT_PADDING
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus

# BluePilot: Pill positioned at bottom of display
PILL_BOTTOM_MARGIN = 40
PILL_SIDE_MARGIN = 60
PILL_PADDING_H = 40
PILL_PADDING_V = 22
PILL_LINE1_FONT_SIZE = 66
PILL_LINE2_FONT_SIZE = 56
PILL_LINE_SPACING = 10
PILL_HEIGHT_SINGLE = 110
PILL_HEIGHT_DOUBLE = 175

# Pill notification colors
PILL_BACKGROUND_COLOR = rl.Color(45, 45, 45, 255)


class AlertRendererBP(AlertRenderer):
  """BluePilot AlertRenderer with pill-shaped notifications below speed display."""

  def _render(self, rect: rl.Rectangle):
    from openpilot.selfdrive.ui.ui_state import ui_state
    alert = self.get_alert(ui_state.sm)
    bp_ui_log.state("AlertRenderer", "has_alert", alert is not None)
    if alert:
      bp_ui_log.state("AlertRenderer", "alert_size", alert.size)

    # SunnyPilot on-road screen-off timer (OnroadScreenOffBrightness / OnroadScreenOffTimer) is
    # driven from here every frame — must match stock AlertRenderer._render (MICI does this in
    # mici/onroad/alert_renderer.py). Calling super()._render for non-informational alerts would
    # duplicate this and double-decrement the timer.
    if gui_app.sunnypilot_ui():
      ui_state.onroad_brightness_handle_alerts(ui_state, alert)

    if not alert:
      return

    is_informational = (alert.status == AlertStatus.normal and alert.size != AlertSize.full)

    if is_informational:
      alert_rect = self._get_pill_rect(rect, alert)
      if alert_rect:
        self._draw_pill_background(alert_rect)
        text_rect = rl.Rectangle(
          alert_rect.x + PILL_PADDING_H, alert_rect.y + PILL_PADDING_V,
          alert_rect.width - 2 * PILL_PADDING_H, alert_rect.height - 2 * PILL_PADDING_V
        )
        self._draw_pill_text(text_rect, alert)
    else:
      # Same drawing as AlertRenderer._render after brightness hook (do not call super()._render)
      alert_rect = self._get_alert_rect(rect, alert.size)
      self._draw_background(alert_rect, alert)
      text_rect = rl.Rectangle(
        alert_rect.x + ALERT_PADDING,
        alert_rect.y + ALERT_PADDING,
        alert_rect.width - 2 * ALERT_PADDING,
        alert_rect.height - 2 * ALERT_PADDING
      )
      self._draw_text(text_rect, alert)

  def _get_pill_rect(self, rect: rl.Rectangle, alert) -> Optional[rl.Rectangle]:
    """Calculate pill-shaped notification rectangle at bottom of display, centered."""
    line1 = alert.text1 or ""
    line2 = alert.text2 or ""
    if not line1 and not line2:
      return None

    available_width = rect.width - 2 * PILL_SIDE_MARGIN
    if available_width < 100:
      return None

    has_two_lines = bool(line1 and line2)
    if has_two_lines:
      line1_size = measure_text_cached(self.font_bold, line1, PILL_LINE1_FONT_SIZE)
      line2_size = measure_text_cached(self.font_bold, line2, PILL_LINE2_FONT_SIZE)
      text_width = max(line1_size.x, line2_size.x)
      pill_height = PILL_HEIGHT_DOUBLE
    else:
      text = line1 or line2
      text_size = measure_text_cached(self.font_bold, text, PILL_LINE1_FONT_SIZE)
      text_width = text_size.x
      pill_height = PILL_HEIGHT_SINGLE

    pill_width = min(text_width + 2 * PILL_PADDING_H, available_width)
    pill_x = rect.x + (rect.width - pill_width) / 2
    pill_y = rect.y + rect.height - pill_height - PILL_BOTTOM_MARGIN

    return rl.Rectangle(pill_x, pill_y, pill_width, pill_height)

  def _draw_pill_background(self, rect: rl.Rectangle) -> None:
    rl.draw_rectangle_rounded(rect, 0.75, 10, PILL_BACKGROUND_COLOR)

  def _draw_pill_text(self, rect: rl.Rectangle, alert) -> None:
    """Draw text1 and text2 in pill (no wrapping). Line 1 primary font, line 2 slightly smaller."""
    line1 = alert.text1 or ""
    line2 = alert.text2 or ""
    if not line1 and not line2:
      return

    has_two_lines = bool(line1 and line2)
    if has_two_lines:
      line1_size = measure_text_cached(self.font_bold, line1, PILL_LINE1_FONT_SIZE)
      line2_size = measure_text_cached(self.font_bold, line2, PILL_LINE2_FONT_SIZE)
      total_height = line1_size.y + PILL_LINE_SPACING + line2_size.y
      start_y = rect.y + (rect.height - total_height) / 2

      line1_x = rect.x + (rect.width - line1_size.x) / 2
      rl.draw_text_ex(self.font_bold, line1, rl.Vector2(line1_x, start_y), PILL_LINE1_FONT_SIZE, 0, rl.WHITE)
      line2_x = rect.x + (rect.width - line2_size.x) / 2
      line2_y = start_y + line1_size.y + PILL_LINE_SPACING
      rl.draw_text_ex(self.font_bold, line2, rl.Vector2(line2_x, line2_y), PILL_LINE2_FONT_SIZE, 0, rl.WHITE)
    else:
      text = line1 or line2
      text_size = measure_text_cached(self.font_bold, text, PILL_LINE1_FONT_SIZE)
      x = rect.x + (rect.width - text_size.x) / 2
      y = rect.y + (rect.height - text_size.y) / 2
      rl.draw_text_ex(self.font_bold, text, rl.Vector2(x, y), PILL_LINE1_FONT_SIZE, 0, rl.WHITE)
