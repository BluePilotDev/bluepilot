import pyray as rl
from openpilot.selfdrive.ui.mici.onroad.confidence_ball import ConfidenceBall
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.bp.lib.tesla_palette import palette_for_dark_fraction
from openpilot.selfdrive.ui.bp.lib.tesla_status import draw_tesla_status_lamp, tesla_mads_lamp_colors
# BluePilot: GPU circle shader moved to BP module after upstream removal
from openpilot.bluepilot.ui.lib.bp_shaders import draw_shader_circle_gradient
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached


MICI_TESLA_STATUS_LABEL_SIZE = 18
MICI_TESLA_STATUS_LAMP_RADIUS = 15
MICI_TESLA_STATUS_LAMP_BEZEL = 3


def mici_tesla_status_layout(rect: rl.Rectangle, confidence_enabled: bool = True
                             ) -> tuple[float, float | None, float | None, float, float]:
  """Return the shared centerline and compact C4 CONF/MADS row positions."""
  center_x = rect.x + rect.width / 2
  if confidence_enabled:
    return center_x, rect.y + 31, rect.y + 75, rect.y + 112, rect.y + 156
  return center_x, None, None, rect.y + 31, rect.y + 75


def confidence_ball_colors(confidence: float, status: UIStatus, demo: bool = False) -> tuple[rl.Color, rl.Color]:
  """Return BluePilot's existing confidence gradient without positional behavior."""
  if status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY, UIStatus.ENGAGED) or demo:
    if confidence > 0.5:
      return rl.Color(0, 255, 204, 255), rl.Color(0, 255, 38, 255)
    if confidence > 0.2:
      return rl.Color(255, 200, 0, 255), rl.Color(255, 115, 0, 255)
    return rl.Color(255, 0, 21, 255), rl.Color(255, 0, 89, 255)

  if status == UIStatus.OVERRIDE:
    return rl.Color(255, 255, 255, 255), rl.Color(82, 82, 82, 255)

  return rl.Color(50, 50, 50, 255), rl.Color(13, 13, 13, 255)


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

    top_dot_color, bottom_dot_color = self.current_colors()

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

  def update_state_only(self) -> None:
    """Advance confidence filtering when a theme owns the visual rendering."""
    self._update_state()

  def current_colors(self) -> tuple[rl.Color, rl.Color]:
    return confidence_ball_colors(self._confidence_filter.x, ui_state.status, self._demo)

  def _draw_circle(self, cx: float, cy: float, radius: float, top: rl.Color, bottom: rl.Color):
    """Use GPU shader for smooth anti-aliased circle on TICI's larger display."""
    draw_shader_circle_gradient(cx, cy, radius, top, bottom)


class ConfidenceBallMiciBP(ConfidenceBallBP):
  BALL_WIDTH = 60
  def __init__(self, demo: bool = False):
    ConfidenceBallBP.__init__(self, demo=demo, radius=24, width=self.BALL_WIDTH, align_right=False)
    self._tesla_status_enabled = False
    self._tesla_confidence_enabled = False
    self._tesla_mads_active = False
    self._tesla_dark_fraction = 0.0
    self._tesla_status_font = gui_app.font(FontWeight.SEMI_BOLD)

  def set_tesla_status(self, enabled: bool, confidence_enabled: bool = True,
                       mads_active: bool = False, dark_fraction: float = 0.0) -> None:
    self._tesla_status_enabled = enabled
    self._tesla_confidence_enabled = confidence_enabled
    self._tesla_mads_active = mads_active
    self._tesla_dark_fraction = dark_fraction

  def _draw_centered_label(self, text: str, center_x: float, y: float, color: rl.Color) -> None:
    text_width = measure_text_cached(self._tesla_status_font, text, MICI_TESLA_STATUS_LABEL_SIZE).x
    pos = rl.Vector2(center_x - text_width / 2, y)
    rl.draw_text_ex(
      self._tesla_status_font, text, rl.Vector2(pos.x + 1, pos.y + 1),
      MICI_TESLA_STATUS_LABEL_SIZE, 0, rl.Color(0, 0, 0, 105),
    )
    rl.draw_text_ex(self._tesla_status_font, text, pos, MICI_TESLA_STATUS_LABEL_SIZE, 0, color)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._tesla_status_enabled:
      super()._render(rect)
      return

    center_x, conf_label_y, conf_lamp_y, mads_label_y, mads_lamp_y = mici_tesla_status_layout(
      self.rect, self._tesla_confidence_enabled,
    )
    label_color = palette_for_dark_fraction(self._tesla_dark_fraction).max_inactive
    if self._tesla_confidence_enabled and conf_label_y is not None and conf_lamp_y is not None:
      self._draw_centered_label(tr("CONF."), center_x, conf_label_y, label_color)
      conf_top, conf_bottom = self.current_colors()
      draw_tesla_status_lamp(
        center_x, conf_lamp_y, MICI_TESLA_STATUS_LAMP_RADIUS,
        MICI_TESLA_STATUS_LAMP_BEZEL, conf_top, conf_bottom,
      )

    self._draw_centered_label(tr("MADS"), center_x, mads_label_y, label_color)
    mads_top, mads_bottom = tesla_mads_lamp_colors(self._tesla_mads_active)
    draw_tesla_status_lamp(
      center_x, mads_lamp_y, MICI_TESLA_STATUS_LAMP_RADIUS,
      MICI_TESLA_STATUS_LAMP_BEZEL, mads_top, mads_bottom,
    )

TICI_CONFIDENCE_BALL_R = 50
TICI_CONFIDENCE_BALL_MARGIN = 5
TICI_CONFIDENCE_BALL_W = TICI_CONFIDENCE_BALL_R * 2 + TICI_CONFIDENCE_BALL_MARGIN

class ConfidenceBallTiciBP(ConfidenceBallBP):
  BALL_WIDTH = TICI_CONFIDENCE_BALL_W
  def __init__(self, demo: bool = False):
    ConfidenceBallBP.__init__(self, demo=demo, radius=TICI_CONFIDENCE_BALL_R, width=self.BALL_WIDTH, align_right=False)
