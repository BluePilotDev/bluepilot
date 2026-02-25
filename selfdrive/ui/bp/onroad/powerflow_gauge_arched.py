"""
Powerflow Gauge (Arched) for Hybrid Vehicles

Arch-shaped gauge above the torque bar showing power flow direction.
Restored from commit 7b9a2531b; select via FordPrefHybridGaugeStyle = "arched".
"""
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.mici.onroad.torque_bar import arc_bar_pts
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from opendbc.car.ford.helpers import get_hev_power_flow_text, get_hev_engine_on_reason_text

# Constants
POWERFLOW_ANGLE_SPAN = 15.0  # Slightly longer than torque bar (12.7 degrees)
POWERFLOW_RADIUS = 3400  # Slightly larger radius than torque bar (3300) for more curvature
POWERFLOW_LINE_HEIGHT = 60  # Height/thickness of the powerflow arch (for tick marks and animated bar)
POWERFLOW_BG_HEIGHT = 60 + 80 + 34 + 12  # Background height: arch + text offset + font size + padding
POWERFLOW_Y_OFFSET = 50  # Vertical offset above torque bar
POWERFLOW_BG_COLOR = rl.Color(20, 20, 20, 200)  # Translucent dark grey
POWERFLOW_TICK_COLOR = rl.Color(200, 200, 200, 255)  # Light grey for tick marks
POWERFLOW_BORDER_COLOR = rl.Color(200, 200, 200, 255)  # Light grey for border (same as tick marks)
POWERFLOW_TICK_LENGTH_RATIO = 0.10  # Tick marks extend 10% into the bar
POWERFLOW_BORDER_THICKNESS = 2.0  # Border line thickness
POWERFLOW_BAR_HEIGHT = 40  # Height/thickness of the animated power flow bar
POWERFLOW_CENTER_COLOR = rl.Color(255, 255, 255, 255)  # White at center (no power flow)
POWERFLOW_REGEN_COLOR = rl.Color(100, 255, 100, 255)  # Green for regenerative braking (left)
POWERFLOW_DEMAND_COLOR = rl.Color(100, 150, 255, 255)  # Brighter blue for throttle demand (right)
POWERFLOW_TEXT_FONT_SIZE = 51  # Font size for power flow mode and engine on reason text
POWERFLOW_TEXT_Y_OFFSET = 95  # Vertical offset below the powerflow meter arch
POWERFLOW_TEXT_COLOR = rl.Color(255, 255, 255, 255)  # White text
POWERFLOW_TEXT_BG_PADDING = 18  # Padding around text
# Move arch and battery together down (half of "arc bottom 10px from screen" shift)
ARCH_DOWN_OFFSET = 87
# Rotate whole grouping clockwise so left (battery) side is not lower than right (2°)
ARCH_ROTATION_DEG = 2.0

class PowerflowGaugeArched(Widget):
  """Arch-shaped powerflow gauge (above torque bar). Use when FordPrefHybridGaugeStyle is arched."""

  def __init__(self):
    super().__init__()
    self.set_visible(lambda: ui_state.sm.recv_frame.get("carStateBP", 0) > ui_state.started_frame)
    from openpilot.common.filter_simple import FirstOrderFilter
    self._powerflow_filter = FirstOrderFilter(0.0, 0.0, 1.0 / gui_app.target_fps * 10)
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._power_flow_mode_value = 0
    self._engine_on_reason_value = 0
    self._top_angle = -90
    self._torque_bar = None  # Set by AugmentedRoadViewBP so we use same logic as flat strip
    self._strip_drawn_by_view = False
    self._battery_visible = False  # When False, use 0° rotation (original); when True, use ARCH_ROTATION_DEG
    self._scale = 1.0  # 0.75 for small (gauge size 1), 1.0 for large (gauge size 2)

  def set_scale(self, scale: float) -> None:
    """Set size scale (0.75 = small, 1.0 = large). View sets before render."""
    self._scale = scale

  def set_torque_bar(self, torque_bar):
    """Use the same torque bar renderer as the flat meter (same update/filters)."""
    self._torque_bar = torque_bar

  def _update_state(self):
    if not self._should_render():
      return
    sm = ui_state.sm
    try:
      car_state_bp = sm['carStateBP']
      throttle_demand = car_state_bp.hybridDrive.throttleDemandPercent
      normalized_value = np.clip(throttle_demand / 102.0, -1.0, 1.0)
      self._powerflow_filter.update(normalized_value)
      self._power_flow_mode_value = car_state_bp.hybridDrive.powerFlowModeValue
      self._engine_on_reason_value = car_state_bp.hybridDrive.engineOnReasonValue
    except (KeyError, AttributeError, TypeError):
      self._power_flow_mode_value = 0
      self._engine_on_reason_value = 0

  def _should_render(self) -> bool:
    from openpilot.common.params import Params
    params = Params()
    if not params.get_bool("FordPrefHybridPowerFlow"):
      return False
    sm = ui_state.sm
    try:
      if "carStateBP" not in sm.recv_frame:
        return False
      if sm.recv_frame["carStateBP"] < ui_state.started_frame:
        return False
      return sm['carStateBP'].hybridDrive.dataAvailable
    except (KeyError, AttributeError, TypeError):
      return False

  def get_arch_geometry(self, rect: rl.Rectangle) -> dict:
    """Return (cx, cy, top_angle, powerflow_start_angle, powerflow_end_angle, outer_radius) for the powerflow arch.
    Uses self._scale (0.75 small, 1.0 large). Used by the view to draw the combo steering strip.
    """
    cx = rect.x + rect.width / 2 + 8
    torque_bar_radius = 3300 * self._scale
    torque_line_offset_estimate = 24 * self._scale
    base_cy = rect.y + rect.height + torque_bar_radius - torque_line_offset_estimate
    widget_height_estimate = 100 * self._scale
    cy = base_cy - widget_height_estimate + ARCH_DOWN_OFFSET * self._scale
    top_angle = -90 + ARCH_ROTATION_DEG
    powerflow_start_angle = top_angle - POWERFLOW_ANGLE_SPAN / 2
    powerflow_end_angle = top_angle + POWERFLOW_ANGLE_SPAN / 2
    mid_r = (POWERFLOW_RADIUS + POWERFLOW_LINE_HEIGHT / 2) * self._scale
    outer_radius = mid_r + (POWERFLOW_LINE_HEIGHT / 2) * self._scale
    return {
      "cx": cx, "cy": cy, "top_angle": top_angle,
      "powerflow_start_angle": powerflow_start_angle,
      "powerflow_end_angle": powerflow_end_angle,
      "outer_radius": outer_radius,
    }

  def render(self, rect: rl.Rectangle = None, strip_drawn_by_view: bool = False, battery_visible: bool = False, **kwargs) -> None:
    """Render the gauge. When strip_drawn_by_view is True, the view has already drawn the steering strip (combo span).
    When battery_visible is False, powerflow uses 0° rotation (original); when True, uses ARCH_ROTATION_DEG (2° CW)."""
    self._strip_drawn_by_view = strip_drawn_by_view
    self._battery_visible = battery_visible
    return super().render(rect)

  def _render(self, rect: rl.Rectangle) -> None:
    if not self._should_render():
      return
    try:
      s = self._scale
      cx = rect.x + rect.width / 2 + 8
      torque_bar_radius = 3300 * s
      torque_line_offset_estimate = 24 * s
      base_cy = rect.y + rect.height + torque_bar_radius - torque_line_offset_estimate
      widget_height_estimate = 100 * s
      cy = base_cy - widget_height_estimate + ARCH_DOWN_OFFSET * s

      # Original orientation (0°) when only powerflow; 2° CW when battery also visible
      rotation_deg = ARCH_ROTATION_DEG if self._battery_visible else 0.0
      top_angle = -90 + rotation_deg
      powerflow_start_angle = top_angle - POWERFLOW_ANGLE_SPAN / 2
      powerflow_end_angle = top_angle + POWERFLOW_ANGLE_SPAN / 2
      self._top_angle = top_angle

      line_h = POWERFLOW_LINE_HEIGHT * s
      mid_r = POWERFLOW_RADIUS * s + line_h / 2
      text_radius = mid_r + line_h / 2 - POWERFLOW_TEXT_Y_OFFSET * s
      bg_bottom_radius = text_radius - POWERFLOW_TEXT_FONT_SIZE * s / 2 - POWERFLOW_TEXT_BG_PADDING * s
      bg_top_radius = mid_r + line_h / 2
      bg_mid_radius = (bg_top_radius + bg_bottom_radius) / 2
      bg_height = bg_top_radius - bg_bottom_radius

      bg_pts = arc_bar_pts(
        cx, cy, bg_mid_radius, bg_height,
        powerflow_start_angle, powerflow_end_angle
      )
      draw_polygon(rect, bg_pts, color=POWERFLOW_BG_COLOR)

      outer_radius = mid_r + line_h / 2
      inner_radius = mid_r - line_h / 2
      num_segments = int(POWERFLOW_ANGLE_SPAN * 2)
      angle_step = POWERFLOW_ANGLE_SPAN / num_segments

      border_thickness = POWERFLOW_BORDER_THICKNESS * s
      for i in range(num_segments + 1):
        angle_deg = powerflow_start_angle + angle_step * i
        angle_rad = np.deg2rad(angle_deg)
        x = cx + np.cos(angle_rad) * outer_radius
        y = cy + np.sin(angle_rad) * outer_radius
        if i > 0:
          prev_angle_deg = powerflow_start_angle + angle_step * (i - 1)
          prev_angle_rad = np.deg2rad(prev_angle_deg)
          prev_x = cx + np.cos(prev_angle_rad) * outer_radius
          prev_y = cy + np.sin(prev_angle_rad) * outer_radius
          rl.draw_line_ex(
            rl.Vector2(prev_x, prev_y),
            rl.Vector2(x, y),
            border_thickness,
            POWERFLOW_BORDER_COLOR
          )

      for i in range(num_segments + 1):
        angle_deg = powerflow_start_angle + angle_step * i
        angle_rad = np.deg2rad(angle_deg)
        x = cx + np.cos(angle_rad) * inner_radius
        y = cy + np.sin(angle_rad) * inner_radius
        if i > 0:
          prev_angle_deg = powerflow_start_angle + angle_step * (i - 1)
          prev_angle_rad = np.deg2rad(prev_angle_deg)
          prev_x = cx + np.cos(prev_angle_rad) * inner_radius
          prev_y = cy + np.sin(prev_angle_rad) * inner_radius
          rl.draw_line_ex(
            rl.Vector2(prev_x, prev_y),
            rl.Vector2(x, y),
            border_thickness,
            POWERFLOW_BORDER_COLOR
          )

      tick_length = line_h * POWERFLOW_TICK_LENGTH_RATIO
      tick_thickness = 2.0 * s
      for percent in range(0, 101, 10):
        angle_deg = powerflow_start_angle + (powerflow_end_angle - powerflow_start_angle) * (percent / 100.0)
        angle_rad = np.deg2rad(angle_deg)
        outer_x_top = cx + np.cos(angle_rad) * outer_radius
        outer_y_top = cy + np.sin(angle_rad) * outer_radius
        inner_x_top = cx + np.cos(angle_rad) * (outer_radius - tick_length)
        inner_y_top = cy + np.sin(angle_rad) * (outer_radius - tick_length)
        outer_x_bottom = cx + np.cos(angle_rad) * inner_radius
        outer_y_bottom = cy + np.sin(angle_rad) * inner_radius
        inner_x_bottom = cx + np.cos(angle_rad) * (inner_radius + tick_length)
        inner_y_bottom = cy + np.sin(angle_rad) * (inner_radius + tick_length)
        rl.draw_line_ex(
          rl.Vector2(outer_x_top, outer_y_top),
          rl.Vector2(inner_x_top, inner_y_top),
          tick_thickness,
          POWERFLOW_TICK_COLOR
        )
        rl.draw_line_ex(
          rl.Vector2(outer_x_bottom, outer_y_bottom),
          rl.Vector2(inner_x_bottom, inner_y_bottom),
          tick_thickness,
          POWERFLOW_TICK_COLOR
        )

      # Arched steering/torque strip at top (skip when view draws combo strip across battery + powerflow)
      if not self._strip_drawn_by_view and self._torque_bar is not None:
        self._torque_bar.render_strip_arched(
          rect, cx, cy, self._top_angle,
          powerflow_start_angle, powerflow_end_angle, outer_radius,
          scale=self._scale,
        )

      bar_height = POWERFLOW_BAR_HEIGHT * s
      self._draw_powerflow_bar(rect, cx, cy, mid_r, bar_height, powerflow_start_angle, powerflow_end_angle)
      self._draw_arch_text_labels(rect, cx, cy, mid_r, powerflow_start_angle, powerflow_end_angle, self._top_angle)

    except Exception as e:
      from openpilot.common.swaglog import cloudlog
      import traceback
      cloudlog.error(f"PowerflowGaugeArched render error: {e}")
      cloudlog.error(traceback.format_exc())

  def _draw_arch_text_labels(self, rect, cx, cy, mid_r, start_angle, end_angle, center_angle):
    try:
      s = self._scale
      font_size = int(POWERFLOW_TEXT_FONT_SIZE * s)
      engine_reason_text = get_hev_engine_on_reason_text(getattr(self, '_engine_on_reason_value', 0))
      power_flow_text = get_hev_power_flow_text(getattr(self, '_power_flow_mode_value', 0))
      if not engine_reason_text and not power_flow_text:
        return
      text_radius = mid_r + (POWERFLOW_LINE_HEIGHT * s) / 2 - POWERFLOW_TEXT_Y_OFFSET * s
      text_start_angle = None
      text_end_angle = None

      if power_flow_text:
        center_text_angle = start_angle + (center_angle - start_angle) * 0.5
        powerflow_text_size = measure_text_cached(self._font_bold, power_flow_text, font_size)
        text_arc_length = powerflow_text_size.x
        text_angle_span_deg = np.rad2deg(text_arc_length / text_radius)
        powerflow_text_start_angle = center_text_angle - text_angle_span_deg / 2
        powerflow_text_end_angle = center_text_angle + text_angle_span_deg / 2
        if text_start_angle is None or powerflow_text_start_angle < text_start_angle:
          text_start_angle = powerflow_text_start_angle
        if text_end_angle is None or powerflow_text_end_angle > text_end_angle:
          text_end_angle = powerflow_text_end_angle
        cumulative_width = 0
        for i, char in enumerate(power_flow_text):
          char_text = char
          char_size = measure_text_cached(self._font_bold, char_text, font_size)
          char_width = char_size.x
          char_center_offset_angle = np.rad2deg((cumulative_width + char_width / 2) / text_radius)
          char_angle = powerflow_text_start_angle + char_center_offset_angle
          char_angle_rad = np.deg2rad(char_angle)
          char_x = cx + np.cos(char_angle_rad) * text_radius
          char_y = cy + np.sin(char_angle_rad) * text_radius
          char_rotation = char_angle + 90
          char_size_single = measure_text_cached(self._font_bold, char_text, font_size)
          char_origin = rl.Vector2(char_size_single.x / 2, char_size_single.y / 2)
          rl.draw_text_pro(
            self._font_bold,
            char_text,
            rl.Vector2(char_x, char_y),
            char_origin,
            char_rotation,
            font_size,
            0,
            POWERFLOW_TEXT_COLOR
          )
          cumulative_width += char_width

      if engine_reason_text:
        center_text_angle = center_angle + (end_angle - center_angle) * 0.5
        engine_text_size = measure_text_cached(self._font_bold, engine_reason_text, font_size)
        text_arc_length = engine_text_size.x
        text_angle_span_deg = np.rad2deg(text_arc_length / text_radius)
        engine_text_start_angle = center_text_angle - text_angle_span_deg / 2
        engine_text_end_angle = center_text_angle + text_angle_span_deg / 2
        if text_start_angle is None or engine_text_start_angle < text_start_angle:
          text_start_angle = engine_text_start_angle
        if text_end_angle is None or engine_text_end_angle > text_end_angle:
          text_end_angle = engine_text_end_angle
        cumulative_width = 0
        for i, char in enumerate(engine_reason_text):
          char_text = char
          char_size = measure_text_cached(self._font_bold, char_text, font_size)
          char_width = char_size.x
          char_center_offset_angle = np.rad2deg((cumulative_width + char_width / 2) / text_radius)
          char_angle = engine_text_start_angle + char_center_offset_angle
          char_angle_rad = np.deg2rad(char_angle)
          char_x = cx + np.cos(char_angle_rad) * text_radius
          char_y = cy + np.sin(char_angle_rad) * text_radius
          char_rotation = char_angle + 90
          char_size_single = measure_text_cached(self._font_bold, char_text, font_size)
          char_origin = rl.Vector2(char_size_single.x / 2, char_size_single.y / 2)
          rl.draw_text_pro(
            self._font_bold,
            char_text,
            rl.Vector2(char_x, char_y),
            char_origin,
            char_rotation,
            font_size,
            0,
            POWERFLOW_TEXT_COLOR
          )
          cumulative_width += char_width

    except Exception as e:
      from openpilot.common.swaglog import cloudlog
      import traceback
      cloudlog.error(f"PowerflowGaugeArched text label error: {e}")
      cloudlog.error(traceback.format_exc())

  def _draw_powerflow_bar(self, rect, cx, cy, mid_r, bar_height, start_angle, end_angle):
    powerflow_value = self._powerflow_filter.x
    center_angle = self._top_angle
    if abs(powerflow_value) < 0.01:
      return
    if powerflow_value < 0:
      bar_start_angle = center_angle
      bar_end_angle = center_angle + (start_angle - center_angle) * abs(powerflow_value)
      bar_end_angle = max(bar_end_angle, start_angle)
      bar_color = POWERFLOW_REGEN_COLOR
    else:
      bar_start_angle = center_angle
      bar_end_angle = center_angle + (end_angle - center_angle) * powerflow_value
      bar_end_angle = min(bar_end_angle, end_angle)
      bar_color = POWERFLOW_DEMAND_COLOR
    bar_pts = arc_bar_pts(
      cx, cy, mid_r, bar_height,
      bar_start_angle, bar_end_angle
    )
    draw_polygon(rect, bar_pts, color=bar_color)
