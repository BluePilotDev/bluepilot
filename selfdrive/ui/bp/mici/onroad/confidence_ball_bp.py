import math
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad.confidence_ball import ConfidenceBall, draw_circle_gradient
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
# BluePilot: GPU circle shader moved to BP module after upstream removal
from openpilot.bluepilot.ui.lib.bp_shaders import draw_shader_circle_gradient
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

class ConfidenceBallBP(ConfidenceBall):
  def __init__(self, demo: bool = False, radius: float=24, width: float = 60, align_right: bool = True):
    ConfidenceBall.__init__(self, demo=demo)
    self._align_right = align_right
    self._width = width
    self._status_dot_radius = radius

  def draw_mads_beam(self, x: int, y: int, width: int, height: int, color: rl.Color):
      transparent = rl.Color(color.r, color.g, color.b, 0)
      segments = 3
      seg_width = width // segments

      # Center segment: solid color
      rl.draw_rectangle(
          x + seg_width, y, seg_width, height,
          color
      )

      # Left segment: fade from transparent -> solid
      rl.draw_rectangle_gradient_h(
          x, y, seg_width, height,
          transparent,  # bottom-left
          color         # top-right
      )

      # Right segment: fade from solid -> transparent
      rl.draw_rectangle_gradient_h(
          x + seg_width * (segments-1), y, width - seg_width, height,
          color,        # bottom-left
          transparent   # top-right
      )

  def _render(self, _):
    bar_width = self._width
    x = self.rect.x if not self._align_right else self.rect.x + self.rect.width - bar_width
    content_rect = rl.Rectangle(
      x,
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
    dot_height = content_rect.y + dot_height

    # confidence zones
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

    if content_rect.width < 2 * self._status_dot_radius:
      # Bar is narrower than ball diameter - position so left edge of ball is at bar left edge
      ball_center_x = content_rect.x + self._status_dot_radius
    else:
      # Bar is wide enough - position ball aligned to right edge of bar (original behavior)
      ball_center_x = content_rect.x + content_rect.width - self._status_dot_radius

    # MADS beam (teal bar) only when LAT_ONLY or LONG_ONLY; no bar when ENGAGED
    if ui_state.status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY):
      color = self.get_lat_long_dot_color()
      color = rl.Color(color.r, color.g, color.b, 150)  # Set alpha for faded background
      self.draw_mads_beam(int(content_rect.x),
                          int(content_rect.y),
                          int(content_rect.width),
                          int(content_rect.height),
                          color)

    self._draw_circle(ball_center_x, dot_height, self._status_dot_radius,
                      top_dot_color, bottom_dot_color)

  def _draw_circle(self, cx: float, cy: float, radius: float, top: rl.Color, bottom: rl.Color):
    """Use GPU shader for smooth anti-aliased circle on TICI's larger display."""
    draw_shader_circle_gradient(cx, cy, radius, top, bottom)


class ConfidenceBallMiciBP(ConfidenceBallBP):
  BALL_WIDTH = 60
  def __init__(self, demo: bool = False):
    ConfidenceBallBP.__init__(self, demo=demo, radius=24, width=self.BALL_WIDTH, align_right=False)

TICI_CONFIDENCE_BALL_R = 50
TICI_CONFIDENCE_BALL_MARGIN = 5
TICI_CONFIDENCE_BALL_W = TICI_CONFIDENCE_BALL_R * 2 + TICI_CONFIDENCE_BALL_MARGIN

class ConfidenceBallTiciBP(ConfidenceBallBP):
  BALL_WIDTH = TICI_CONFIDENCE_BALL_W
  def __init__(self, demo: bool = False):
    ConfidenceBallBP.__init__(self, demo=demo, radius=TICI_CONFIDENCE_BALL_R, width=self.BALL_WIDTH, align_right=False)
