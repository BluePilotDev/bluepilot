"""BluePilot additions to comma 4's stock onroad alert renderer."""

import time

import pyray as rl

from openpilot.selfdrive.ui.bp.lib.tesla_alerts import (
  draw_tesla_alert_glyph,
  tesla_alert_colors,
  tesla_alert_panel_rect,
)
from openpilot.selfdrive.ui.bp.onroad.tesla_turn_signal import draw_tesla_turn_signal_texture
from openpilot.selfdrive.ui.mici.onroad.alert_renderer import (
  ALERT_MARGIN,
  TURN_SIGNAL_BLINK_PERIOD,
  Alert,
  AlertLayout,
  AlertRenderer,
  AlertSize,
  AlertStatus,
  IconSide,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wrap_text import wrap_text


class AlertRendererBP(AlertRenderer):
  """MICI alerts with Tesla-theme presentation and stock indicator behavior."""

  def __init__(self):
    super().__init__()
    self._tesla_style = False
    self._tesla_dark_fraction = 0.0
    self._tesla_font_medium = gui_app.font(FontWeight.MEDIUM)
    self._tesla_font_regular = gui_app.font(FontWeight.ROMAN)

  def set_tesla_style(self, enabled: bool, dark_fraction: float) -> None:
    self._tesla_style = bool(enabled)
    self._tesla_dark_fraction = float(dark_fraction)

  def _is_turn_signal(self, texture) -> bool:
    return texture in (self._txt_turn_signal_left, self._txt_turn_signal_right)

  def _icon_helper(self, alert: Alert) -> AlertLayout:
    layout = super()._icon_helper(alert)
    if not (self._tesla_style and layout.icon is not None and
            self._is_turn_signal(layout.icon.texture) and not ui_state.turn_signals):
      return layout

    # The user-facing Show Turn Signals toggle continues to own whether Tesla's
    # recolored built-in blinker is shown. Restore the full text width when off.
    text_rect = rl.Rectangle(
      self._rect.x + ALERT_MARGIN,
      self._alert_y_filter.x,
      self._rect.width - ALERT_MARGIN,
      self._rect.height,
    )
    return AlertLayout(text_rect, None)

  def _draw_icons(self, alert_layout: AlertLayout) -> None:
    if not self._tesla_style:
      super()._draw_icons(alert_layout)
      return
    if alert_layout.icon is None:
      return

    if time.monotonic() - self._turn_signal_timer > TURN_SIGNAL_BLINK_PERIOD:
      self._turn_signal_timer = time.monotonic()
      self._turn_signal_alpha_filter.x = 255 * 2
    else:
      self._turn_signal_alpha_filter.update(255 * 0.2)

    if alert_layout.icon.side == IconSide.left:
      pos_x = int(self._rect.x + alert_layout.icon.margin_x)
    else:
      pos_x = int(self._rect.x + self._rect.width - alert_layout.icon.margin_x - alert_layout.icon.texture.width)
    position = rl.Vector2(pos_x, self._rect.y + alert_layout.icon.margin_y)

    if self._is_turn_signal(alert_layout.icon.texture):
      icon_alpha = int(min(self._turn_signal_alpha_filter.x, 255) * self._alpha_filter.x)
      draw_tesla_turn_signal_texture(
        alert_layout.icon.texture, position, self._tesla_dark_fraction, icon_alpha,
      )
    else:
      icon_alpha = int(alert_layout.icon.alpha * self._alpha_filter.x)
      rl.draw_texture_ex(alert_layout.icon.texture, position, 0.0, 1.0,
                         rl.Color(255, 255, 255, icon_alpha))

  def _draw_background(self, alert: Alert) -> None:
    if not self._tesla_style:
      super()._draw_background(alert)
      return

    panel = tesla_alert_panel_rect(self._rect, alert.size, compact=True)
    colors = tesla_alert_colors(alert.status, self._tesla_dark_fraction, self._alpha_filter.x)
    rl.draw_rectangle_rec(panel, colors.background)
    accent_line = rl.Color(colors.accent.r, colors.accent.g, colors.accent.b,
                           round(150 * self._alpha_filter.x))
    rl.draw_rectangle(int(panel.x), int(panel.y), int(panel.width), 3, accent_line)

  def _draw_text(self, alert: Alert, alert_layout: AlertLayout) -> None:
    if not self._tesla_style:
      super()._draw_text(alert, alert_layout)
      return

    panel = tesla_alert_panel_rect(self._rect, alert.size, compact=True)
    colors = tesla_alert_colors(alert.status, self._tesla_dark_fraction, self._alpha_filter.x)
    has_builtin_icon = alert_layout.icon is not None
    if has_builtin_icon:
      text_x = alert_layout.text_rect.x
      text_width = alert_layout.text_rect.width
      text_y = panel.y + 10
      text_height = panel.height - 20
      title_size = 42
      subtitle_size = 24
    elif alert.size == AlertSize.small:
      glyph_center = rl.Vector2(panel.x + 38, panel.y + panel.height / 2.0)
      draw_tesla_alert_glyph(glyph_center, 18, alert.status, self._tesla_dark_fraction,
                             self._alpha_filter.x)
      text_x = panel.x + 70
      text_width = panel.width - 84
      text_y = panel.y + 10
      text_height = panel.height - 20
      title_size = 36
      subtitle_size = 22
    else:
      glyph_center = rl.Vector2(panel.x + panel.width / 2.0, panel.y + 42)
      draw_tesla_alert_glyph(glyph_center, 22, alert.status, self._tesla_dark_fraction,
                             self._alpha_filter.x)
      text_x = panel.x + 30
      text_width = panel.width - 60
      text_y = panel.y + 69
      text_height = panel.height - 77
      title_size = 40
      subtitle_size = 24

    title_color = colors.accent if alert.status != AlertStatus.normal else colors.text
    self._draw_tesla_text_block(
      alert.text1, alert.text2, rl.Rectangle(text_x, text_y, text_width, text_height),
      title_size, subtitle_size, title_color, colors.secondary,
    )

  def _draw_tesla_text_block(self, title: str, subtitle: str, rect: rl.Rectangle,
                              title_size: int, subtitle_size: int,
                              title_color: rl.Color, subtitle_color: rl.Color) -> None:
    title_lines = wrap_text(self._tesla_font_medium, title or "", title_size, int(rect.width))
    subtitle_lines = wrap_text(self._tesla_font_regular, subtitle or "", subtitle_size, int(rect.width))
    title_height = measure_text_cached(self._tesla_font_medium, "Ag", title_size).y
    subtitle_height = measure_text_cached(self._tesla_font_regular, "Ag", subtitle_size).y
    gap = 6.0 if title_lines and subtitle_lines else 0.0
    total_height = len(title_lines) * title_height + gap + len(subtitle_lines) * subtitle_height
    y = rect.y + max(0.0, (rect.height - total_height) / 2.0)

    for line in title_lines:
      line_width = measure_text_cached(self._tesla_font_medium, line, title_size).x
      rl.draw_text_ex(self._tesla_font_medium, line,
                      rl.Vector2(rect.x + (rect.width - line_width) / 2.0, y), title_size, 0, title_color)
      y += title_height
    y += gap
    for line in subtitle_lines:
      line_width = measure_text_cached(self._tesla_font_regular, line, subtitle_size).x
      rl.draw_text_ex(self._tesla_font_regular, line,
                      rl.Vector2(rect.x + (rect.width - line_width) / 2.0, y), subtitle_size, 0, subtitle_color)
      y += subtitle_height
