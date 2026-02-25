"""
Hybrid Battery Gauge (Arched style) for TICI UI

Arc segment on the left end of the powerflow arch. Shares the same center and radius
so the battery continues the top and bottom of the powerflow meter.
- Upper half of arc: SOC % with animated fill (same animation as previous battery icon).
- Bottom: volts and amps in powerflow string font.

Only used when FordPrefHybridGaugeStyle = "arched".
"""
import traceback
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.mici.onroad.torque_bar import arc_bar_pts
from openpilot.system.ui.lib.shader_polygon import draw_polygon

# Match powerflow arch so battery segment aligns (same center, radius, line height)
POWERFLOW_RADIUS = 3400
POWERFLOW_LINE_HEIGHT = 60
POWERFLOW_TEXT_FONT_SIZE = 51
POWERFLOW_TEXT_Y_OFFSET = 95
POWERFLOW_TEXT_BG_PADDING = 18
POWERFLOW_ANGLE_SPAN = 15.0
POWERFLOW_BAR_HEIGHT = 40  # Same as powerflow bar (fill uses this)
POWERFLOW_BG_COLOR = rl.Color(20, 20, 20, 200)
POWERFLOW_BORDER_COLOR = rl.Color(200, 200, 200, 255)
POWERFLOW_BORDER_THICKNESS = 2.0

# Battery arc sits to the left of powerflow; ~25% of powerflow angular width so it stays on-screen
# Same 2° clockwise rotation as powerflow so grouping is symmetrical
ARCH_ROTATION_DEG = 2.0
POWERFLOW_START_ANGLE = -90.0 - POWERFLOW_ANGLE_SPAN / 2 + ARCH_ROTATION_DEG  # -92.5
BATTERY_ANGLE_SPAN = POWERFLOW_ANGLE_SPAN * 0.25  # 3.75°, ~25% of powerflow
BATTERY_START_ANGLE = POWERFLOW_START_ANGLE - BATTERY_ANGLE_SPAN  # ~-96.25°
BATTERY_END_ANGLE = POWERFLOW_START_ANGLE  # -92.5, meets powerflow
# Same offset as powerflow so both move down together (half of "arc bottom 10px from screen" shift)
ARCH_DOWN_OFFSET = 87
# When only battery is visible (no powerflow): rotate 3° CW and move 25 px left (currently unused: powerflow off → flat battery)
BATTERY_SOLO_ROTATION_DEG = 3.0
BATTERY_SOLO_LEFT_OFFSET = 25

# Colors (same as before)
BATTERY_BG_COLOR = rl.Color(40, 40, 40, 220)
BATTERY_LOW_COLOR = rl.Color(200, 100, 100, 255)
BATTERY_MID_COLOR = rl.Color(200, 200, 100, 255)
BATTERY_HIGH_COLOR = rl.Color(100, 200, 100, 255)
TEXT_COLOR = rl.Color(255, 255, 255, 255)
CHARGING_COLOR = rl.Color(100, 255, 100, 255)
DISCHARGING_COLOR = rl.Color(255, 100, 100, 255)


class HybridBatteryGaugeArched(Widget):
  """Arched battery: arc segment left of powerflow. Upper half = SOC fill, bottom = volts/amps."""

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._left_offset = 0
    self._powerflow_visible = True  # When False, use solo rotation (3° CW) and 25 px left
    self._scale = 1.0  # 0.75 small, 1.0 large (FordPrefHybridDriveGaugeSize)
    from openpilot.common.filter_simple import FirstOrderFilter
    self._soc_filter = FirstOrderFilter(50.0, 50.0, 1.0 / gui_app.target_fps * 10)

  def set_scale(self, scale: float) -> None:
    """Set gauge scale (0.75 small, 1.0 large). View calls before render."""
    self._scale = max(0.5, min(1.0, scale))

  def _update_state(self):
    battery_data = self._get_battery_data()
    if battery_data is not None:
      self._soc_filter.update(battery_data['soc'])

  def _should_render(self) -> bool:
    if not self._params.get_bool("FordPrefHybridBatteryStatus"):
      return False
    sm = ui_state.sm
    try:
      if "carStateBP" not in sm.recv_frame:
        return False
      if sm.recv_frame["carStateBP"] < ui_state.started_frame:
        return False
      return sm['carStateBP'].hybridBattery.dataAvailable
    except (KeyError, AttributeError, TypeError):
      return False

  def _get_battery_data(self):
    sm = ui_state.sm
    try:
      car_state_bp = sm['carStateBP']
      battery = car_state_bp.hybridBattery
      return {
        'soc': battery.socActual,
        'voltage': battery.voltActual,
        'amps': battery.ampsActual,
        'soc_min': battery.socMinPerc,
        'soc_max': battery.socMaxPerc,
      }
    except (KeyError, AttributeError, TypeError):
      return {
        'soc': 50.0,
        'voltage': 0.0,
        'amps': 0.0,
        'soc_min': 0.0,
        'soc_max': 100.0,
      }

  def _get_battery_fill_color(self, soc: float) -> rl.Color:
    if soc < 20:
      return BATTERY_LOW_COLOR
    elif soc < 50:
      return BATTERY_MID_COLOR
    return BATTERY_HIGH_COLOR

  def _get_cx_cy(self, rect: rl.Rectangle):
    """Same (cx, cy) as powerflow when powerflow visible; when solo, cx moved left by BATTERY_SOLO_LEFT_OFFSET. Uses _scale."""
    s = self._scale
    cx = rect.x + rect.width / 2 + 8
    if not self._powerflow_visible:
      cx -= BATTERY_SOLO_LEFT_OFFSET * s
    torque_bar_radius = 3300 * s
    torque_line_offset_estimate = 24 * s
    base_cy = rect.y + rect.height + torque_bar_radius - torque_line_offset_estimate
    widget_height_estimate = 100 * s
    cy = base_cy - widget_height_estimate + ARCH_DOWN_OFFSET * s
    return cx, cy

  def set_powerflow_visible(self, visible: bool) -> None:
    """Set whether the powerflow gauge is visible this frame (view calls before render)."""
    self._powerflow_visible = visible

  def render(self, rect: rl.Rectangle = None, left_offset: int = 0, powerflow_visible: bool = True) -> None:
    if rect is not None:
      self.set_rect(rect)
    self._left_offset = left_offset
    self._powerflow_visible = powerflow_visible
    return super().render(rect)

  def _render(self, rect: rl.Rectangle) -> None:
    try:
      if not self._should_render():
        return
      s = self._scale
      battery_data = self._get_battery_data()
      if battery_data is None:
        return
      soc = battery_data['soc']
      voltage = battery_data['voltage']
      amps = battery_data['amps']
      animated_soc = self._soc_filter.x
      fill_color = self._get_battery_fill_color(soc)

      cx, cy = self._get_cx_cy(rect)
      # When powerflow visible: 2° CW (match combo). When solo: 3° CW, no steering bar
      rotation_deg = ARCH_ROTATION_DEG if self._powerflow_visible else BATTERY_SOLO_ROTATION_DEG
      powerflow_start_angle = -90.0 - POWERFLOW_ANGLE_SPAN / 2 + rotation_deg
      battery_end_angle = powerflow_start_angle
      battery_start_angle = powerflow_start_angle - BATTERY_ANGLE_SPAN

      line_h = POWERFLOW_LINE_HEIGHT * s
      mid_r = POWERFLOW_RADIUS * s + line_h / 2
      outer_radius = mid_r + line_h / 2
      inner_radius = mid_r - line_h / 2
      # Background arch (same extent as powerflow: down to text area)
      text_radius = mid_r + line_h / 2 - POWERFLOW_TEXT_Y_OFFSET * s
      font_size = POWERFLOW_TEXT_FONT_SIZE * s
      bg_bottom_radius = text_radius - font_size / 2 - POWERFLOW_TEXT_BG_PADDING * s
      bg_top_radius = mid_r + line_h / 2
      bg_mid_radius = (bg_top_radius + bg_bottom_radius) / 2
      bg_height = bg_top_radius - bg_bottom_radius

      # 1) Battery arc background (continues powerflow arch on the left)
      bg_pts = arc_bar_pts(
        cx, cy, bg_mid_radius, bg_height,
        battery_start_angle, battery_end_angle
      )
      draw_polygon(rect, bg_pts, color=POWERFLOW_BG_COLOR)

      # 2) Battery arc border (outer and inner edges)
      border_thickness = POWERFLOW_BORDER_THICKNESS * s
      num_segments = max(2, int(BATTERY_ANGLE_SPAN * 2))
      angle_step = BATTERY_ANGLE_SPAN / num_segments
      for i in range(num_segments + 1):
        angle_deg = battery_start_angle + angle_step * i
        angle_rad = np.deg2rad(angle_deg)
        x_outer = cx + np.cos(angle_rad) * outer_radius
        y_outer = cy + np.sin(angle_rad) * outer_radius
        x_inner = cx + np.cos(angle_rad) * inner_radius
        y_inner = cy + np.sin(angle_rad) * inner_radius
        if i > 0:
          prev_deg = battery_start_angle + angle_step * (i - 1)
          prev_rad = np.deg2rad(prev_deg)
          prev_x_o = cx + np.cos(prev_rad) * outer_radius
          prev_y_o = cy + np.sin(prev_rad) * outer_radius
          prev_x_i = cx + np.cos(prev_rad) * inner_radius
          prev_y_i = cy + np.sin(prev_rad) * inner_radius
          rl.draw_line_ex(rl.Vector2(prev_x_o, prev_y_o), rl.Vector2(x_outer, y_outer),
                          border_thickness, POWERFLOW_BORDER_COLOR)
          rl.draw_line_ex(rl.Vector2(prev_x_i, prev_y_i), rl.Vector2(x_inner, y_inner),
                          border_thickness, POWERFLOW_BORDER_COLOR)

      # 3) Upper half of arc: SOC fill (same geometry as powerflow bar, different color) + SOC % text
      # Inset fill from both edges by ~7 px so it doesn't run off the arc
      fill_inset_deg = np.rad2deg(7.0 / mid_r)
      fill_start_angle = battery_start_angle + fill_inset_deg
      fill_end_angle = fill_start_angle + (BATTERY_ANGLE_SPAN - fill_inset_deg) * (animated_soc / 100.0)
      bar_height = POWERFLOW_BAR_HEIGHT * s
      if animated_soc > 0.5:
        fill_pts = arc_bar_pts(
          cx, cy, mid_r, bar_height,
          fill_start_angle, fill_end_angle
        )
        draw_polygon(rect, fill_pts, color=fill_color)

      # SOC % text in center of upper half
      half_thickness = line_h / 2
      soc_text = f"{int(soc)}%"
      soc_angle_deg = (battery_start_angle + battery_end_angle) / 2
      soc_angle_rad = np.deg2rad(soc_angle_deg)
      soc_radius = mid_r + half_thickness / 2
      soc_y_offset = 22 * s
      soc_x = cx + np.cos(soc_angle_rad) * soc_radius
      soc_y = cy + np.sin(soc_angle_rad) * soc_radius + soc_y_offset  # down so it doesn't sit over top of meter
      soc_font_size = int(font_size)
      soc_size = measure_text_cached(self._font_bold, soc_text, soc_font_size)
      # Center text at (soc_x, soc_y); draw_text_pro uses origin for rotation center
      soc_origin = rl.Vector2(soc_size.x / 2, soc_size.y / 2)
      soc_rotation = soc_angle_deg + 90  # tangent to arc
      rl.draw_text_pro(
        self._font_bold,
        soc_text,
        rl.Vector2(soc_x, soc_y),
        soc_origin,
        soc_rotation,
        soc_font_size,
        0,
        TEXT_COLOR
      )

      # 4) Bottom: volts and amps side by side (10% smaller, lowercase, no +/- on amps)
      va_font_size = int(font_size * 0.9)
      text_radius_bottom = mid_r + line_h / 2 - POWERFLOW_TEXT_Y_OFFSET * s
      voltage_text = f"{int(voltage)}V"
      amps_text = f"{int(abs(amps))}A"  # no sign; color indicates charge/discharge
      va_angle_deg = (battery_start_angle + battery_end_angle) / 2
      va_angle_rad = np.deg2rad(va_angle_deg)
      va_radius = text_radius_bottom - font_size * 0.6
      va_center_y_offset = 15 * s
      va_center_x = cx + np.cos(va_angle_rad) * va_radius
      va_center_y = cy + np.sin(va_angle_rad) * va_radius - va_center_y_offset  # up
      v_size = measure_text_cached(self._font_bold, voltage_text, va_font_size)
      a_size = measure_text_cached(self._font_bold, amps_text, va_font_size)
      gap = int(5 * s)
      tangent_x = -np.sin(va_angle_rad)
      tangent_y = np.cos(va_angle_rad)
      # Side by side: voltage left, amps right (along arc tangent), centered as a group
      half_total = (v_size.x + gap + a_size.x) / 2
      v_x = va_center_x - (half_total - v_size.x / 2) * tangent_x
      v_y = va_center_y - (half_total - v_size.x / 2) * tangent_y
      a_x = va_center_x + (half_total - a_size.x / 2) * tangent_x
      a_y = va_center_y + (half_total - a_size.x / 2) * tangent_y
      v_origin = rl.Vector2(v_size.x / 2, v_size.y / 2)
      a_origin = rl.Vector2(a_size.x / 2, a_size.y / 2)
      rl.draw_text_pro(
        self._font_bold,
        voltage_text,
        rl.Vector2(v_x, v_y),
        v_origin,
        va_angle_deg + 90,
        va_font_size,
        0,
        TEXT_COLOR
      )
      amps_color = CHARGING_COLOR if amps > 0 else DISCHARGING_COLOR if amps < 0 else TEXT_COLOR
      rl.draw_text_pro(
        self._font_bold,
        amps_text,
        rl.Vector2(a_x, a_y),
        a_origin,
        va_angle_deg + 90,
        va_font_size,
        0,
        amps_color
      )
    except Exception as e:
      cloudlog.error(f"HybridBatteryGaugeArched render error: {e}")
      cloudlog.error(traceback.format_exc())
