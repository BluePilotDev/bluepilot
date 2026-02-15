import time
import numpy as np
import pyray as rl
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.common.filter_simple import FirstOrderFilter

class Blindspot(Widget):
  def __init__(self, width: float = 150):
    super().__init__()
    self.params = Params()
    self._enabled = False
    self._width = width

    # Blindspot screen edge indicators (MICI style)
    self._blindspot_left_alpha_filter = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._blindspot_right_alpha_filter = FirstOrderFilter(0.0, 0.15, 1 / gui_app.target_fps)
    self._blindspot_pulse_start_time = time.monotonic()

  def _update_state(self):
    self._enabled = self.params.get_bool("BlindSpot")

  def _render(self, rect: rl.Rectangle) -> None:
    """Draw blindspot screen edge indicators (MICI style) - red gradient edge of screen when blindspot detected with pulsing animation"""
    if not self._enabled:
      return

    sm = ui_state.sm
    if not sm.valid['carState']:
      return

    car_state = sm['carState']
    left_blindspot = car_state.leftBlindspot
    right_blindspot = car_state.rightBlindspot

    # Update alpha filters for smooth fade in/out
    self._blindspot_left_alpha_filter.update(1.0 if left_blindspot else 0.0)
    self._blindspot_right_alpha_filter.update(1.0 if right_blindspot else 0.0)

    # Pulse animation: creates a brightness pulse effect
    PULSE_DURATION = 3.0  # seconds for one complete pulse cycle (twice as slow)
    current_time = time.monotonic()
    pulse_phase = ((current_time - self._blindspot_pulse_start_time) % PULSE_DURATION) / PULSE_DURATION

    # Gradient opacity: starts at 100% and fades to 30%
    EDGE_ALPHA_START = 1.0   # 100% opacity at the edge
    EDGE_ALPHA_END = 0.0     # 30% opacity at the inside edge (fully transparent)

    x = int(rect.x)
    y = int(rect.y)
    h = int(rect.height)

    # Calculate brightness pulse: smooth sine wave from 0.3 (dim) to 1.0 (bright)
    # pulse_phase goes from 0.0 to 1.0, so we use sine to create smooth pulsing
    brightness_pulse = 0.3 + 0.7 * (0.5 + 0.5 * np.sin(pulse_phase * 2 * np.pi))  # Range: 0.3 to 1.0

    # Draw left edge red gradient indicator with brightness pulse
    if self._blindspot_left_alpha_filter.x > 0.01:
      filter_alpha = self._blindspot_left_alpha_filter.x
      edge_alpha = min(255, int(255 * EDGE_ALPHA_START * filter_alpha * brightness_pulse)) # Apply brightness pulse
      inside_alpha = min(255,int(255 * EDGE_ALPHA_END * filter_alpha * brightness_pulse))  # Apply brightness pulse
      edge_color = rl.Color(255, 0, 0, edge_alpha)
      inside_color = rl.Color(255, 0, 0, inside_alpha)
      rl.draw_rectangle_gradient_h(
        x,
        y,
        self._width,
        h,
        edge_color,
        inside_color)

    # Draw right edge red gradient indicator with brightness pulse
    if self._blindspot_right_alpha_filter.x > 0.01:
      filter_alpha = self._blindspot_right_alpha_filter.x
      edge_alpha = int(255 * EDGE_ALPHA_START * filter_alpha * brightness_pulse)  # Apply brightness pulse
      inside_alpha = int(255 * EDGE_ALPHA_END * filter_alpha * brightness_pulse)  # Apply brightness pulse
      edge_color = rl.Color(255, 0, 0, edge_alpha)
      inside_color = rl.Color(255, 0, 0, inside_alpha)
      rl.draw_rectangle_gradient_h(
        x + int(rect.width) - self._width,
        y,
        self._width,
        h,
        inside_color,
        edge_color)
