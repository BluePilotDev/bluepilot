"""
BluePilot: Ford angle-mode Low/High Speed Factor stepper.

Shown alongside the lateral debug graph, only while Primary Control Variable is Angle.
A tap on Up/Down nudges FordLowSpeedFactor_ang / FordHighSpeedFactor_ang together, split
between the two in proportion to how much each contributes at the current speed -- the same
30/60 mph speed-blend LateralAngleExt.update_angle_strategy uses for low_gain_calc/high_gain_calc
(opendbc/sunnypilot/car/ford/lateral_angle_ext.py).
"""
import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.selfdrive.ui.ui_state import ui_state
from opendbc.bluepilot_lateral.core.lateral_curv_ext import PrimaryLateralControl
from bluepilot.ui.widgets.debug.debug_colors import DebugColors

LOW_KEY = "FordLowSpeedFactor_ang"
HIGH_KEY = "FordHighSpeedFactor_ang"
FACTOR_MIN = 0.5
FACTOR_MAX = 1.5
STEP = 0.01

# Value text is dimmed toward this floor opacity as a factor's speed-effectiveness drops to 0,
# so the fully-inactive factor still reads as ~60% rather than disappearing.
_DIM_FLOOR = 0.2

# Matches the speed breakpoints in LateralAngleExt.update_angle_strategy: fully low-factor
# at/below ~30 mph, fully high-factor at/above ~60 mph, blended in between.
_SPEED_LOW_MS = 13.5
_SPEED_HIGH_MS = 26.82

BUTTON_SIZE = 90
ROW_GAP = 12
VALUE_ROW_H = 70
LABEL_SIZE = 24
VALUE_SIZE = 34

# MICI's onroad screen is smaller than TICI's -- shrink everything to fit.
COMPACT_BUTTON_SIZE = 57
COMPACT_ROW_GAP = 6
COMPACT_VALUE_ROW_H = 44
COMPACT_LABEL_SIZE = 15
COMPACT_VALUE_SIZE = 22

# 60% opacity so the graph is still visible through the button
_BUTTON_BG = rl.Color(0, 0, 0, 153)
_BUTTON_BORDER = rl.Color(255, 255, 255, 200)


class AngleFactorAdjuster(Widget):
  """Vertical stack: [Up] [High value] [Low value] [Down]. Visible only in Ford angle mode."""

  def __init__(self, compact: bool = False):
    super().__init__()
    self.params = Params()
    self._button_size = COMPACT_BUTTON_SIZE if compact else BUTTON_SIZE
    self._row_gap = COMPACT_ROW_GAP if compact else ROW_GAP
    self._value_row_h = COMPACT_VALUE_ROW_H if compact else VALUE_ROW_H
    self._label_size = COMPACT_LABEL_SIZE if compact else LABEL_SIZE
    self._value_size = COMPACT_VALUE_SIZE if compact else VALUE_SIZE
    btn_font = 36 if compact else 58
    # TRANSPARENT_WHITE_TEXT has no opaque background of its own -- we draw our own
    # translucent background + border underneath so the graph shows through.
    self._up = self._child(Button("+", lambda: self._bump(1), font_size=btn_font,
                                   button_style=ButtonStyle.TRANSPARENT_WHITE_TEXT))
    self._down = self._child(Button("-", lambda: self._bump(-1), font_size=btn_font,
                                     button_style=ButtonStyle.TRANSPARENT_WHITE_TEXT))
    self._font_label = gui_app.font(FontWeight.NORMAL)
    self._font_value = gui_app.font(FontWeight.SEMI_BOLD)
    self.set_visible(self._is_angle_mode)

  def _is_angle_mode(self) -> bool:
    try:
      raw = self.params.get("FordPrefLateralControl", return_default=True) or 0
      return PrimaryLateralControl(int(raw)) == PrimaryLateralControl.angle
    except (TypeError, ValueError):
      return False

  def _weight_high(self) -> float:
    v_ego = 0.0
    sm = ui_state.sm
    if sm is not None and sm.valid.get('carState', False):
      v_ego = float(sm['carState'].vEgo)
    v_ego = max(_SPEED_LOW_MS, min(_SPEED_HIGH_MS, v_ego))
    return (v_ego - _SPEED_LOW_MS) / (_SPEED_HIGH_MS - _SPEED_LOW_MS)

  def _get(self, key: str) -> float:
    try:
      raw = self.params.get(key, return_default=True)
      return float(raw.decode("utf-8") if isinstance(raw, bytes) else raw)
    except (TypeError, ValueError):
      return 1.0

  def _set(self, key: str, value: float):
    value = round(max(FACTOR_MIN, min(FACTOR_MAX, value)), 3)
    self.params.put(key, value, block=False)

  def _bump(self, sign: int):
    w_high = self._weight_high()
    w_low = 1.0 - w_high
    self._set(LOW_KEY, self._get(LOW_KEY) + sign * STEP * w_low)
    self._set(HIGH_KEY, self._get(HIGH_KEY) + sign * STEP * w_high)

  def _draw_value(self, label: str, value: float, weight: float, rect: rl.Rectangle):
    """weight is this factor's effectiveness at the current speed (0-1): dims the text toward
    _DIM_FLOOR as effectiveness drops, so a fully-active factor reads 100% and a fully-inactive
    one still reads at ~_DIM_FLOOR rather than vanishing."""
    opacity = _DIM_FLOOR + (1.0 - _DIM_FLOOR) * weight
    label_color = rl.Color(DebugColors.LEGEND_LABEL.r, DebugColors.LEGEND_LABEL.g,
                           DebugColors.LEGEND_LABEL.b, int(DebugColors.LEGEND_LABEL.a * opacity))
    value_color = rl.Color(DebugColors.LEGEND_TEXT.r, DebugColors.LEGEND_TEXT.g,
                           DebugColors.LEGEND_TEXT.b, int(DebugColors.LEGEND_TEXT.a * opacity))

    # Third decimal is fixed at 20% opacity regardless of weight -- taps only ever move a
    # value by STEP=0.01, this digit just shows the sub-step position while blending.
    value_text = f"{value:.3f}"
    main_text, thousandths_digit = value_text[:-1], value_text[-1]
    thousandths_color = rl.Color(DebugColors.LEGEND_TEXT.r, DebugColors.LEGEND_TEXT.g,
                                 DebugColors.LEGEND_TEXT.b, int(DebugColors.LEGEND_TEXT.a * 0.2))

    lsz = measure_text_cached(self._font_label, label, self._label_size)
    vsz = measure_text_cached(self._font_value, value_text, self._value_size)
    main_sz = measure_text_cached(self._font_value, main_text, self._value_size)
    cx = rect.x + rect.width / 2
    rl.draw_text_ex(self._font_label, label, rl.Vector2(cx - lsz.x / 2, rect.y),
                    self._label_size, 0, label_color)
    value_y = rect.y + self._label_size + 2
    value_x = cx - vsz.x / 2
    rl.draw_text_ex(self._font_value, main_text, rl.Vector2(value_x, value_y),
                    self._value_size, 0, value_color)
    rl.draw_text_ex(self._font_value, thousandths_digit, rl.Vector2(value_x + main_sz.x, value_y),
                    self._value_size, 0, thousandths_color)

  def _draw_button_bg(self, rect: rl.Rectangle):
    roundness = 10 / (min(rect.width, rect.height) / 2)
    rl.draw_rectangle_rounded(rect, roundness, 10, _BUTTON_BG)
    rl.draw_rectangle_rounded_lines_ex(rect, roundness, 10, 2, _BUTTON_BORDER)

  def _render(self, rect: rl.Rectangle):
    btn_size = min(self._button_size, rect.width)
    up_rect = rl.Rectangle(rect.x, rect.y, rect.width, btn_size)
    self._draw_button_bg(up_rect)
    self._up.render(up_rect)

    w_high = self._weight_high()
    high_y = up_rect.y + up_rect.height + self._row_gap
    low_y = high_y + self._value_row_h + self._row_gap

    # Sliding highlight: sits over the High row at w_high=1 (>=60mph), the Low row at
    # w_high=0 (<=30mph), and glides between the two rows in between.
    track_top, track_bottom = high_y, low_y + self._value_row_h
    bar_h = self._value_row_h
    bar_y = track_top + (1.0 - w_high) * (track_bottom - track_top - bar_h)
    rl.draw_rectangle_rounded(rl.Rectangle(rect.x, bar_y, rect.width, bar_h), 0.25, 8,
                              DebugColors.SPEED_HIGHLIGHT)

    self._draw_value("High", self._get(HIGH_KEY), w_high, rl.Rectangle(rect.x, high_y, rect.width, self._value_row_h))
    self._draw_value("Low", self._get(LOW_KEY), 1.0 - w_high, rl.Rectangle(rect.x, low_y, rect.width, self._value_row_h))

    down_rect = rl.Rectangle(rect.x, low_y + self._value_row_h + self._row_gap, rect.width, btn_size)
    self._draw_button_bg(down_rect)
    self._down.render(down_rect)
