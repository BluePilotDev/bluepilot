import numpy as np
import pyray as rl
import math
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from opendbc.sunnypilot.car.ford.carstate_ext import get_hev_power_flow_text, get_hev_engine_on_reason_text
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

SEGMENTS = 50
LINES = 20
SMOOTHING = 0.12
BAR_W = 20

DEMO = False

# Angles (radians)
BOTTOM = math.radians(90)
TOP = math.radians(-90)
POWERFLOW_REGEN_COLOR = rl.Color(100, 255, 100, 255)
POWERFLOW_DEMAND_COLOR = rl.Color(100, 150, 255, 255)

class MiciPowerflowGauge(Widget):
  """Widget to display powerflow gauge as an arch above the torque bar"""
  RADIUS = 20

  def __init__(self):
    super().__init__()
    self._value = 0
    self._inc = 0.01
    self._powerflow_filter = FirstOrderFilter(0.0, 0.0, 1.0 / gui_app.target_fps * 10)
    self._power_flow_mode_value = 0
    self._engine_on_reason_value = 0
    self._top_angle = -90
    if DEMO:
       self._demo_value = 0.0
       self._demo_inc = 0.01

  def _update_state(self):
    """Update power flow state and animate changes"""
    if not self._should_render():
      return

    sm = ui_state.sm
    try:
      car_state_bp = sm['carStateBP']
      throttle_demand = car_state_bp.hybridDrive.throttleDemandPercent
      # Clamp to expected range [-102.2, 102.4] and normalize to [-1, 1] for easier calculation
      # Positive = throttle demand (power out, should be blue)
      # Negative = regenerative braking (power in, should be green)
      normalized_value = np.clip(throttle_demand / 102.0, -1.0, 1.0)
      self._powerflow_filter.update(normalized_value)

      # Store current power flow mode and engine on reason for text display
      self._power_flow_mode_value = car_state_bp.hybridDrive.powerFlowModeValue
    except (KeyError, AttributeError, TypeError):
      self._power_flow_mode_value = 0

  def _should_render(self) -> bool:
    """Check if powerflow gauge should be rendered"""
    # Only render if hybrid power flow is enabled
    from openpilot.common.params import Params
    params = Params()
    power_flow_enabled = params.get_bool("FordPrefHybridPowerFlow")
    self._power_flow_use_alternate = params.get_bool("FordPrefHybridPowerFlowAlternate")
    if not power_flow_enabled:
      bp_ui_log.visibility("MiciPowerflow", False, reason="param_disabled")
      return False

    if DEMO:
       return True

    sm = ui_state.sm
    try:
      # Check if message exists and is recent enough
      if "carStateBP" not in sm.recv_frame:
        bp_ui_log.visibility("MiciPowerflow", False, reason="no_recv_frame")
        return False

      recv_frame = sm.recv_frame["carStateBP"]
      if recv_frame < ui_state.started_frame:
        bp_ui_log.visibility("MiciPowerflow", False, reason=f"stale_frame recv={recv_frame} started={ui_state.started_frame}")
        return False

      car_state_bp = sm['carStateBP']
      available = car_state_bp.hybridDrive.dataAvailable
      bp_ui_log.visibility("MiciPowerflow", available, reason=f"dataAvailable={available}")
      return available
    except (KeyError, AttributeError, TypeError) as e:
      bp_ui_log.visibility("MiciPowerflow", False, reason=f"exception: {e}")
      return False

  def set_wheel_rect(self, rect: rl.Rectangle):
     self._wheel_rect = rect

  def _render(self, rect: rl.Rectangle) -> None:
    """Render the powerflow gauge arch"""
    if not self._should_render():
      return

    if DEMO:
      self._demo_value += self._demo_inc
      if self._demo_value > 1.0 or self._demo_value < -1.0:
        self._demo_inc *= -1

    if self._power_flow_use_alternate:
      if DEMO:
        self.draw_circular_gauge(self._demo_value)
      else:
        self.draw_circular_gauge(self._powerflow_filter.x)

    else:
      if DEMO:
        self.draw_vertical_gauge(rect, self._demo_value)
      else:
        self.draw_vertical_gauge(rect, self._powerflow_filter.x)

  def draw_arc_segment(self, angle, color):
    x1 = self._center.x + math.cos(angle) * self._inner_radius
    y1 = self._center.y + math.sin(angle) * self._inner_radius
    x2 = self._center.x + math.cos(angle) * self._outer_radius
    y2 = self._center.y + math.sin(angle) * self._outer_radius

    rl.draw_line_ex(
        rl.Vector2(x1, y1),
        rl.Vector2(x2, y2),
        6,
        color
    )

  def draw_circular_gauge(self, value):
    self._center = rl.Vector2(self._wheel_rect.x + self._wheel_rect.width // 2, self._wheel_rect.y + self._wheel_rect.height // 2)
    self._outer_radius = self._wheel_rect.width // 2
    self._inner_radius = self._outer_radius - self.RADIUS * 1.1

    # --- Regen (left side) ---
    if value < 0:
        active = abs(value)
        for i in range(SEGMENTS):
            t = i / (SEGMENTS - 1)
            if t >= active:
                break

            angle = -TOP + t * (BOTTOM - TOP)
            self.draw_arc_segment(angle, POWERFLOW_REGEN_COLOR)

    # --- Throttle (right side) ---
    if value > 0:
        active = value
        for i in range(SEGMENTS):
            t = i / (SEGMENTS - 1)
            if t >= active:
                break

            angle = BOTTOM + t * (TOP - BOTTOM)
            self.draw_arc_segment(angle, POWERFLOW_DEMAND_COLOR)

  def lerp(a, b, t):
    return a + (b - a) * t

  def draw_vertical_gauge(self, rect: rl.Rectangle, value):
      bar_x = int(rect.x)
      bar_y = int(rect.y)
      bar_h = int(rect.height)

      segment_h = bar_h / LINES
      mid = bar_h * 0.6

      rl.draw_rectangle(
          bar_x,
          bar_y,
          BAR_W,
          bar_h,
          rl.Color(0,0,0,100)
      )

      demand_h = mid * abs(value)
      regen_h = (bar_h - mid) * abs(value)
      # --- Throttle side (above center) ---
      if value > 0:
          rl.draw_rectangle(
              bar_x,
              int(mid - demand_h),
              BAR_W,
              int(demand_h),
              POWERFLOW_DEMAND_COLOR
          )

      # --- Regen side (below center) ---
      elif value < 0:
          rl.draw_rectangle(
              bar_x,
              int(mid),
              BAR_W,
              int(regen_h),
              POWERFLOW_REGEN_COLOR
          )

      line_color = rl.Color(255,255,255,150)
      line_shadow = rl.Color(0,0,0,150)

      for i in range(LINES):
          y = int(segment_h * i)
          rl.draw_line(
              bar_x,
              y,
              bar_x + 5,
              y,
              line_color
          )
          rl.draw_line(
              bar_x,
              int(y-1),
              bar_x + 5,
              int(y-1),
              line_shadow
          )

          rl.draw_line(
              bar_x + BAR_W - 5,
              y,
              bar_x + BAR_W,
              y,
              line_color
          )
          rl.draw_line(
              bar_x + BAR_W - 5,
              int(y-1),
              bar_x + BAR_W,
              int(y-1),
              line_shadow
          )

      rl.draw_rectangle(
          bar_x,
          int(mid),
          BAR_W,
          2,
          rl.WHITE
      )

      rl.draw_line(
          bar_x,
          int(mid + 2),
          bar_x + BAR_W,
          int(mid + 2),
          line_shadow
      )