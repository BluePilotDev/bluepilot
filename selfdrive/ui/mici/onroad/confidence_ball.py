import math

import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.confidence_ball import ConfidenceBallSP
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

# BluePilot: import shader-backed confidence ball helper
from openpilot.bluepilot.ui.lib.bp_shaders import draw_shader_circle_gradient
# End BluePilot


def draw_circle_gradient(center_x: float, center_y: float, radius: int,
                         top: rl.Color, bottom: rl.Color) -> None:
  rl.draw_rectangle_gradient_v(int(center_x - radius), int(center_y - radius),
                               radius * 2, radius * 2,
                               top, bottom)

  outer_radius = math.ceil(radius * math.sqrt(2)) + 1
  rl.draw_ring(rl.Vector2(int(center_x), int(center_y)), radius, outer_radius,
               0.0, 360.0,
               20, rl.BLACK)


class ConfidenceBall(Widget, ConfidenceBallSP):
  def __init__(self, demo: bool = False, radius: float = 24, width: float = 60):
    Widget.__init__(self)
    ConfidenceBallSP.__init__(self)
    self._demo = demo
    # BluePilot: parameterize confidence rail geometry for MICI and TICI variants
    self._status_dot_radius = radius
    self._width = width
    # End BluePilot
    self._confidence_filter = FirstOrderFilter(-0.5, 0.5, 1 / gui_app.target_fps)

  def update_filter(self, value: float):
    self._confidence_filter.update(value)

  def _update_state(self):
    if self._demo:
      return

    if ui_state.status == UIStatus.DISENGAGED:
      self._confidence_filter.update(-0.5)
    elif ui_state.status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY):
      self._confidence_filter.update(1 - max(self.get_animate_status_probs() or [1]))
    else:
      self._confidence_filter.update((1 - max(ui_state.sm['modelV2'].meta.disengagePredictions.brakeDisengageProbs or [1])) *
                                     (1 - max(ui_state.sm['modelV2'].meta.disengagePredictions.steerOverrideProbs or [1])))

  def _render(self, _):
    content_rect = rl.Rectangle(
      self.rect.x,
      self.rect.y,
      self._width,
      self.rect.height,
    )

    # BluePilot: remap confidence across the full vertical rail
    filter_min = -0.5
    filter_max = 1.0
    normalized = (self._confidence_filter.x - filter_min) / (filter_max - filter_min)
    normalized = max(0.0, min(1.0, normalized))
    dot_height = content_rect.height - (normalized * content_rect.height) + self._status_dot_radius
    dot_height = content_rect.y + dot_height
    # End BluePilot

    if ui_state.status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY, UIStatus.ENGAGED) or self._demo:
      if self._confidence_filter.x > 0.5:
        top_dot_color = rl.Color(0, 255, 204, 255)
        bottom_dot_color = rl.Color(0, 255, 38, 255)
      elif self._confidence_filter.x > 0.2:
        top_dot_color = rl.Color(255, 200, 0, 255)
        bottom_dot_color = rl.Color(255, 115, 0, 255)
      else:
        top_dot_color = rl.Color(255, 0, 21, 255)
        bottom_dot_color = rl.Color(255, 0, 89, 255)
    elif ui_state.status == UIStatus.OVERRIDE:
      top_dot_color = rl.Color(255, 255, 255, 255)
      bottom_dot_color = rl.Color(82, 82, 82, 255)
    else:
      top_dot_color = rl.Color(50, 50, 50, 255)
      bottom_dot_color = rl.Color(13, 13, 13, 255)

    # BluePilot: render MADS beam when partially engaged
    if ui_state.status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY):
      color = self.get_lat_long_dot_color()
      self._draw_mads_beam(int(content_rect.x), int(content_rect.y), int(content_rect.width), int(content_rect.height),
                           rl.Color(color.r, color.g, color.b, 150))
    # End BluePilot

    self._draw_circle(content_rect.x + self._status_dot_radius, dot_height, self._status_dot_radius,
                      top_dot_color, bottom_dot_color)

  @staticmethod
  def _draw_mads_beam(x: int, y: int, width: int, height: int, color: rl.Color):
    transparent = rl.Color(color.r, color.g, color.b, 0)
    segments = 3
    seg_width = width // segments

    rl.draw_rectangle(x + seg_width, y, seg_width, height, color)
    rl.draw_rectangle_gradient_h(x, y, seg_width, height, transparent, color)
    rl.draw_rectangle_gradient_h(x + seg_width * (segments - 1), y, width - seg_width, height, color, transparent)

  @staticmethod
  def _draw_circle(cx: float, cy: float, radius: float, top: rl.Color, bottom: rl.Color):
    draw_shader_circle_gradient(cx, cy, radius, top, bottom)


# BluePilot: MICI confidence rail BP variant
class ConfidenceBallMiciBP(ConfidenceBall):
  BALL_WIDTH = 60

  def __init__(self, demo: bool = False):
    super().__init__(demo=demo, radius=24, width=self.BALL_WIDTH)
# End BluePilot


# BluePilot: TICI confidence rail BP variant
TICI_CONFIDENCE_BALL_R = 50
TICI_CONFIDENCE_BALL_MARGIN = 5
TICI_CONFIDENCE_BALL_W = TICI_CONFIDENCE_BALL_R * 2 + TICI_CONFIDENCE_BALL_MARGIN


class ConfidenceBallTiciBP(ConfidenceBall):
  BALL_WIDTH = TICI_CONFIDENCE_BALL_W

  def __init__(self, demo: bool = False):
    super().__init__(demo=demo, radius=TICI_CONFIDENCE_BALL_R, width=self.BALL_WIDTH)
# End BluePilot
