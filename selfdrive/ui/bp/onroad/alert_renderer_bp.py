from typing import Optional
import pyray as rl
from cereal import log

from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer, ALERT_PADDING
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wrap_text import wrap_text
from openpilot.selfdrive.ui.bp.lib.tesla_alerts import (
  draw_tesla_alert_glyph,
  tesla_alert_colors,
  tesla_alert_panel_rect,
)
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

  def __init__(self):
    super().__init__()
    self._tesla_style = False
    self._tesla_dark_fraction = 0.0
    self._tesla_font_medium = gui_app.font(FontWeight.MEDIUM)

  def set_tesla_style(self, enabled: bool, dark_fraction: float) -> None:
    self._tesla_style = bool(enabled)
    self._tesla_dark_fraction = float(dark_fraction)

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

    if self._tesla_style:
      self._draw_tesla_alert(rect, alert)
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

  def _draw_tesla_alert(self, rect: rl.Rectangle, alert) -> None:
    panel = tesla_alert_panel_rect(rect, alert.size)
    colors = tesla_alert_colors(alert.status, self._tesla_dark_fraction)
    if alert.size == AlertSize.full:
      rl.draw_rectangle_rec(panel, colors.background)
    else:
      shadow_rect = rl.Rectangle(panel.x + 6, panel.y + 7, panel.width, panel.height)
      rl.draw_rectangle_rounded(shadow_rect, 0.20, 10, rl.Color(0, 0, 0, 48))
      rl.draw_rectangle_rounded(panel, 0.20, 10, colors.background)
      border = rl.Color(colors.accent.r, colors.accent.g, colors.accent.b, 105)
      rl.draw_rectangle_rounded_lines_ex(panel, 0.20, 10, 2.0, border)

    if alert.size == AlertSize.small:
      text_rect = rl.Rectangle(panel.x + 42, panel.y + 18, panel.width - 84, panel.height - 36)
      self._draw_tesla_text_block(text_rect, alert, 56, 42, colors)
      return

    if alert.size == AlertSize.mid:
      glyph_center = rl.Vector2(panel.x + panel.width / 2.0, panel.y + 74)
      draw_tesla_alert_glyph(glyph_center, 38, alert.status, self._tesla_dark_fraction)
      text_rect = rl.Rectangle(panel.x + 60, panel.y + 124, panel.width - 120, panel.height - 142)
      self._draw_tesla_text_block(text_rect, alert, 64, 46, colors)
      return

    glyph_center = rl.Vector2(panel.x + panel.width / 2.0, panel.y + panel.height * 0.33)
    draw_tesla_alert_glyph(glyph_center, 58, alert.status, self._tesla_dark_fraction)
    text_rect = rl.Rectangle(
      panel.x + 150, panel.y + panel.height * 0.42,
      panel.width - 300, panel.height * 0.30,
    )
    self._draw_tesla_text_block(text_rect, alert, 78, 52, colors)

  def _draw_tesla_text_block(self, rect: rl.Rectangle, alert, title_size: int,
                              subtitle_size: int, colors) -> None:
    title_color = colors.accent if alert.status != AlertStatus.normal else colors.text
    title_lines = wrap_text(self._tesla_font_medium, alert.text1 or "", title_size, int(rect.width))
    subtitle_lines = wrap_text(self.font_regular, alert.text2 or "", subtitle_size, int(rect.width))
    title_height = measure_text_cached(self._tesla_font_medium, "Ag", title_size).y
    subtitle_height = measure_text_cached(self.font_regular, "Ag", subtitle_size).y
    gap = 18.0 if title_lines and subtitle_lines else 0.0
    total_height = len(title_lines) * title_height + gap + len(subtitle_lines) * subtitle_height
    current_y = rect.y + max(0.0, (rect.height - total_height) / 2.0)

    for line in title_lines:
      self._draw_tesla_centered_line(line, rect.x, current_y, rect.width,
                                     self._tesla_font_medium, title_size, title_color)
      current_y += title_height
    current_y += gap
    for line in subtitle_lines:
      self._draw_tesla_centered_line(line, rect.x, current_y, rect.width,
                                     self.font_regular, subtitle_size, colors.secondary)
      current_y += subtitle_height

  @staticmethod
  def _draw_tesla_centered_line(text: str, x: float, y: float, width: float,
                                 font: rl.Font, font_size: int, color: rl.Color) -> None:
    text_width = measure_text_cached(font, text, font_size).x
    rl.draw_text_ex(font, text, rl.Vector2(x + (width - text_width) / 2.0, y), font_size, 0, color)

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
