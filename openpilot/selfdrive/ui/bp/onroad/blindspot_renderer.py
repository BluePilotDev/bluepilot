import time
import math
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.params import UnknownKeyName
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log


class BlindspotRendererMixin:
  """Mixin for rendering blindspot screen edge indicators with pulsing animation.

  Parameterized by blind_spot_width:
    - TICI: 250px (wider screen)
    - MICI: 125px (smaller screen)
  """

  def _init_blindspot(self):
    self._blindspot_params = Params()
    self._blindspot_left_alpha_filter = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._blindspot_right_alpha_filter = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._blindspot_pulse_start_time = time.monotonic()
    # BluePilot: Cache param to avoid per-frame disk I/O (refresh every ~60 frames)
    self._blindspot_param_counter = 0
    try:
      self._show_blindspot_overlay = self._blindspot_params.get_bool("ShowBlindspotOverlay")
    except UnknownKeyName:
      self._show_blindspot_overlay = False

  def _draw_blindspot_screen_edges(self, rect: rl.Rectangle, blind_spot_width: int = 250):
    """Draw blindspot screen edge indicators - red gradient edge with pulsing animation."""
    # BluePilot: Refresh cached param periodically (~1s at 20fps)
    self._blindspot_param_counter += 1
    if self._blindspot_param_counter >= 60:
      self._blindspot_param_counter = 0
      try:
        self._show_blindspot_overlay = self._blindspot_params.get_bool("ShowBlindspotOverlay")
      except UnknownKeyName:
        self._show_blindspot_overlay = False

    if not self._show_blindspot_overlay:
      return

    bp_ui_log.state("Blindspot", "param_enabled", True)

    sm = ui_state.sm
    if not sm.valid['carState']:
      return

    car_state = sm['carState']
    bp_ui_log.state("Blindspot", "left", car_state.leftBlindspot)
    bp_ui_log.state("Blindspot", "right", car_state.rightBlindspot)
    left_blindspot = car_state.leftBlindspot
    right_blindspot = car_state.rightBlindspot

    # Update alpha filters for smooth fade in/out
    self._blindspot_left_alpha_filter.update(1.0 if left_blindspot else 0.0)
    self._blindspot_right_alpha_filter.update(1.0 if right_blindspot else 0.0)

    # Pulse animation: creates a brightness pulse effect
    PULSE_DURATION = 3.0  # seconds for one complete pulse cycle
    current_time = time.monotonic()
    pulse_phase = ((current_time - self._blindspot_pulse_start_time) % PULSE_DURATION) / PULSE_DURATION

    # Gradient opacity: starts at 75% and fades to 0% (fully transparent)
    EDGE_ALPHA_START = 0.75
    EDGE_ALPHA_END = 0.0

    x = int(rect.x)
    y = int(rect.y)
    h = int(rect.height)

    # Calculate brightness pulse: smooth sine wave from 0.3 (dim) to 1.0 (bright)
    brightness_pulse = 0.3 + 0.7 * (0.5 + 0.5 * math.sin(pulse_phase * 2 * math.pi))

    # Draw left edge red gradient indicator with brightness pulse
    if self._blindspot_left_alpha_filter.x > 0.01:
      filter_alpha = self._blindspot_left_alpha_filter.x
      edge_alpha = int(255 * EDGE_ALPHA_START * filter_alpha * brightness_pulse)
      inside_alpha = int(255 * EDGE_ALPHA_END * filter_alpha * brightness_pulse)
      edge_color = rl.Color(255, 0, 0, edge_alpha)
      inside_color = rl.Color(255, 0, 0, inside_alpha)
      rl.draw_rectangle_gradient_h(x, y, blind_spot_width, h, edge_color, inside_color)

    # Draw right edge red gradient indicator with brightness pulse
    if self._blindspot_right_alpha_filter.x > 0.01:
      filter_alpha = self._blindspot_right_alpha_filter.x
      edge_alpha = int(255 * EDGE_ALPHA_START * filter_alpha * brightness_pulse)
      inside_alpha = int(255 * EDGE_ALPHA_END * filter_alpha * brightness_pulse)
      edge_color = rl.Color(255, 0, 0, edge_alpha)
      inside_color = rl.Color(255, 0, 0, inside_alpha)
      rl.draw_rectangle_gradient_h(
        x + int(rect.width) - blind_spot_width, y,
        blind_spot_width, h,
        inside_color, edge_color
      )
