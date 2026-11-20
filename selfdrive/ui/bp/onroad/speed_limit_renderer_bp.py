"""Tesla-style speed-limit presentation for BluePilot HUDs."""

from __future__ import annotations

import pyray as rl

from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit import (
  AssistState,
  Colors,
  SpeedLimitMode,
  SpeedLimitRenderer,
)
from openpilot.selfdrive.ui.ui_state import ui_state


TESLA_SIGN_FILL = rl.Color(238, 238, 235, 255)
TESLA_SIGN_BORDER = rl.Color(103, 106, 108, 255)
TESLA_SIGN_TEXT = rl.Color(38, 40, 42, 255)
TESLA_SIGN_INVALID_TEXT = rl.Color(126, 130, 132, 255)
TESLA_SIGN_SHADOW = rl.Color(0, 0, 0, 65)


def tesla_speed_limit_sign_rect(rect: rl.Rectangle, is_metric: bool) -> rl.Rectangle:
  """Return the stock C3X speed-limit sign footprint."""
  width = UI_CONFIG.set_speed_width_metric if is_metric else UI_CONFIG.set_speed_width_imperial
  return rl.Rectangle(
    rect.x + 60 + width + 30 - 6,
    rect.y + 45 - 6,
    width,
    UI_CONFIG.set_speed_height + 6 * 2,
  )


def tesla_speed_limit_font_sizes(sign_height: float, digits: int) -> tuple[int, int]:
  """Scale Tesla sign typography without changing the sign's outer geometry."""
  title_size = max(12, round(sign_height * 0.16))
  value_scale = 0.38 if digits >= 3 else 0.43
  value_size = max(28, round(sign_height * value_scale))
  return title_size, value_size


class SpeedLimitRendererBP(SpeedLimitRenderer):
  """Stock SunnyPilot renderer with an opt-in Tesla surface."""

  def __init__(self):
    super().__init__()
    self._tesla_style = False

  def set_tesla_style(self, enabled: bool) -> None:
    self._tesla_style = bool(enabled)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._tesla_style:
      super()._render(rect)
      return

    if ui_state.speed_limit_mode == SpeedLimitMode.off:
      return

    sign_rect = tesla_speed_limit_sign_rect(rect, ui_state.is_metric)
    self._draw_tesla_sign(sign_rect, self._pre_active_fade.alpha)

    if self.speed_limit_assist_state == AssistState.preActive:
      self._draw_pre_active_arrow(sign_rect)
    else:
      self._draw_ahead_info(sign_rect)

  def _draw_tesla_sign(self, rect: rl.Rectangle, alpha: float = 1.0) -> None:
    speed_limit_warning_enabled = ui_state.speed_limit_mode >= SpeedLimitMode.warning
    has_limit = self.speed_limit_valid or self.speed_limit_last_valid
    is_overspeed = has_limit and round(self.speed_limit_final_last) < round(self.speed)
    value = str(round(self.speed_limit_last)) if has_limit else "---"

    alpha = max(0.0, min(float(alpha), 1.0))
    fill = rl.color_alpha(TESLA_SIGN_FILL, alpha)
    shadow = rl.color_alpha(TESLA_SIGN_SHADOW, alpha)
    text_color = TESLA_SIGN_TEXT if self.speed_limit_valid else TESLA_SIGN_INVALID_TEXT
    text_color = rl.color_alpha(text_color, alpha)

    # Tesla uses a blue outline to call attention to an exceeded limit.
    if speed_limit_warning_enabled and is_overspeed:
      border = rl.color_alpha(rl.Color(33, 116, 200, 255), alpha)
    else:
      border = rl.color_alpha(TESLA_SIGN_BORDER, alpha)

    radius = 0.18
    shadow_offset = max(2.0, rect.width * 0.025)
    shadow_rect = rl.Rectangle(rect.x + shadow_offset, rect.y + shadow_offset, rect.width, rect.height)
    rl.draw_rectangle_rounded(shadow_rect, radius, 10, shadow)
    rl.draw_rectangle_rounded(rect, radius, 10, fill)
    rl.draw_rectangle_rounded_lines_ex(rect, radius, 10, max(2.0, rect.width * 0.022), border)

    title_size, value_size = tesla_speed_limit_font_sizes(rect.height, len(value))
    center_x = rect.x + rect.width / 2
    self._draw_text_centered(
      self.font_demi, "SPEED", title_size,
      rl.Vector2(center_x, rect.y + rect.height * 0.19), text_color,
    )
    self._draw_text_centered(
      self.font_demi, "LIMIT", title_size,
      rl.Vector2(center_x, rect.y + rect.height * 0.35), text_color,
    )
    self._draw_text_centered(
      self.font_bold, value, value_size,
      rl.Vector2(center_x, rect.y + rect.height * 0.70), text_color,
    )

    if self.speed_limit_offset != 0 and has_limit:
      sign = "" if self.speed_limit_offset > 0 else "-"
      sub = f"{sign}{round(abs(self.speed_limit_offset))}"
      badge_size = rect.width * 0.29
      badge = rl.Rectangle(
        rect.x + rect.width - badge_size * 0.75,
        rect.y - badge_size * 0.24,
        badge_size,
        badge_size,
      )
      badge_fill = rl.color_alpha(Colors.DARK_GREY, alpha)
      rl.draw_rectangle_rounded(badge, 0.28, 8, badge_fill)
      self._draw_text_centered(
        self.font_bold, sub, max(18, round(badge_size * 0.48)),
        rl.Vector2(badge.x + badge.width / 2, badge.y + badge.height / 2),
        rl.color_alpha(Colors.WHITE, alpha),
      )
