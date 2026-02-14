import math
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.system.ui.lib.shader_polygon import draw_circle_gradient

from openpilot.selfdrive.ui.sunnypilot.mici.onroad.confidence_ball import ConfidenceBallSP

class ConfidenceBall(Widget, ConfidenceBallSP):
  def __init__(self, demo: bool = False, radius: float=24):
    Widget.__init__(self)
    ConfidenceBallSP.__init__(self)
    self._demo = demo
    self._confidence_filter = FirstOrderFilter(-0.5, 0.5, 1 / gui_app.target_fps)
    self._status_dot_radius = radius

  def update_filter(self, value: float):
    self._confidence_filter.update(value)

  def _update_state(self):
    if self._demo:
      return

    # animate status dot in from bottom
    if ui_state.status == UIStatus.DISENGAGED:
      self._confidence_filter.update(-0.5)
    elif ui_state.status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY):
      self._confidence_filter.update(1 - max(self.get_animate_status_probs() or [1]))
    else:
      self._confidence_filter.update((1 - max(ui_state.sm['modelV2'].meta.disengagePredictions.brakeDisengageProbs or [1])) *
                                                        (1 - max(ui_state.sm['modelV2'].meta.disengagePredictions.steerOverrideProbs or [1])))

  def _render(self, _):
    # Use rect width directly (works for both MICI and TICI)
    # For MICI: rect.width matches SIDE_PANEL_WIDTH
    # For TICI: rect.width is CONFIDENCE_BALL_R (thinner bar)
    bar_width = self.rect.width
    content_rect = rl.Rectangle(
      self.rect.x,
      self.rect.y,
      bar_width,
      self.rect.height,
    )

    bottom_position = content_rect.height
    top_position = 0.0
    range_height = bottom_position - top_position

    # Map confidence filter to new range
    # Original: (1 - self._confidence_filter.x) maps -0.5->1.5 (top) and 1.0->0.0 (bottom)
    # We want to preserve this mapping but constrain to new range
    # Normalize filter.x from [-0.5, ~1.0] to [0, 1] where 0 = bottom, 1 = top
    filter_min = -0.5
    filter_max = 1.0
    normalized = (self._confidence_filter.x - filter_min) / (filter_max - filter_min)
    normalized = max(0.0, min(1.0, normalized))  # Clamp to [0, 1]

    # Map normalized [0, 1] to [bottom_position, top_position]
    # When normalized=0 (low confidence), ball at bottom_position
    # When normalized=1 (high confidence), ball at top_position
    dot_height = bottom_position - (normalized * range_height) + self._status_dot_radius
    dot_height = content_rect.y + dot_height  # Use content_rect.y, not self._rect.y

    # confidence zones
    if ui_state.status == UIStatus.ENGAGED or self._demo:
      if self._confidence_filter.x > 0.5:
        top_dot_color = rl.Color(0, 255, 204, 255)
        bottom_dot_color = rl.Color(0, 255, 38, 255)
      elif self._confidence_filter.x > 0.2:
        top_dot_color = rl.Color(255, 200, 0, 255)
        bottom_dot_color = rl.Color(255, 115, 0, 255)
      else:
        top_dot_color = rl.Color(255, 0, 21, 255)
        bottom_dot_color = rl.Color(255, 0, 89, 255)

    elif ui_state.status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY):
      top_dot_color = bottom_dot_color = self.get_lat_long_dot_color()

    elif ui_state.status == UIStatus.OVERRIDE:
      top_dot_color = rl.Color(255, 255, 255, 255)
      bottom_dot_color = rl.Color(82, 82, 82, 255)

    else:
      top_dot_color = rl.Color(50, 50, 50, 255)
      bottom_dot_color = rl.Color(13, 13, 13, 255)

    # Use bottom color for ring to match the ball (darker edge looks more natural)
    ring_color = bottom_dot_color
    # Position ball so it fits within the bar without going off the left edge
    # If bar is narrower than 2*radius, position ball so left edge aligns with bar left edge
    # Otherwise, position ball centered or aligned to right edge
    if content_rect.width < 2 * self._status_dot_radius:
      # Bar is narrower than ball diameter - position so left edge of ball is at bar left edge
      ball_center_x = content_rect.x + self._status_dot_radius
    else:
      # Bar is wide enough - position ball aligned to right edge of bar (original behavior)
      ball_center_x = content_rect.x + content_rect.width - self._status_dot_radius

    draw_circle_gradient(self.rect, ball_center_x, dot_height, self._status_dot_radius,
                         top_dot_color, bottom_dot_color)
