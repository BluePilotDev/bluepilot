"""
Hybrid Battery Gauge Widget for TICI UI

Displays a horizontal battery gauge with:
- Battery shape (double A battery style)
- State of charge percentage (to the right)
- Voltage and Amps (below battery)
- Color coding: Green for charging (positive amps), Red for discharging (negative amps)
"""
import traceback
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiRenderer
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log


# Base constants (at scale 1.0, these produce the "natural" size)
BATTERY_WIDTH = 90
BATTERY_HEIGHT = 50
BATTERY_TERMINAL_WIDTH = 20
BATTERY_TERMINAL_HEIGHT = 20
BATTERY_ROUNDNESS = 0.3
BATTERY_BORDER_THICKNESS = 3
BATTERY_X_SPACING = 40
BATTERY_Y_MARGIN = 30

SOC_FONT_SIZE = 48
SOC_X_SPACING = 15

VOLTAGE_AMPS_Y_OFFSET = 8
VOLTAGE_AMPS_FONT_SIZE = 53
VOLTAGE_AMPS_LINE_SPACING = 12
BACKGROUND_PADDING = 15
BACKGROUND_ROUNDNESS = 0.3
BACKGROUND_GLOW_EXPANSION = 4

# Dynamic scaling reference
FULL_CONTENT_WIDTH = 2100.0

# Target heights for each gauge size tier (matching power flow gauge)
# Small (1) = previous large; Large (2) = new bigger tier
GAUGE_SIZE_HEIGHTS = {1: 120, 2: 155}

# Colors
BACKGROUND_BOX_COLOR = rl.Color(20, 20, 20, 100)
BATTERY_BG_COLOR = rl.Color(40, 40, 40, 220)
BATTERY_BORDER_COLOR = rl.Color(200, 200, 200, 255)
BATTERY_LOW_COLOR = rl.Color(200, 100, 100, 255)
BATTERY_MID_COLOR = rl.Color(200, 200, 100, 255)
BATTERY_HIGH_COLOR = rl.Color(100, 200, 100, 255)
TEXT_COLOR = rl.Color(255, 255, 255, 255)
CHARGING_COLOR = rl.Color(100, 255, 100, 255)
DISCHARGING_COLOR = rl.Color(255, 100, 100, 255)

PARAM_REFRESH_FRAMES = 60


class HybridBatteryGauge(Widget):
  """Widget to display hybrid battery status gauge"""

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._left_offset = 0
    self._draw_background_flag = True
    self._x_offset = 0.0  # External X shift for horizontal centering in shared container
    self._y_offset = 0.0  # External Y shift for vertical centering with driver monitor

    # Smooth animation for SOC changes
    self._soc_filter = FirstOrderFilter(50.0, 50.0, 1.0 / gui_app.target_fps * 10)

    # Gauge size param
    self._gauge_size = 1
    self._param_frame_counter = PARAM_REFRESH_FRAMES

  def _update_state(self):
    """Update battery state and animate SOC changes"""
    self._param_frame_counter += 1
    if self._param_frame_counter >= PARAM_REFRESH_FRAMES:
      self._param_frame_counter = 0
      try:
        self._gauge_size = int(self._params.get("FordPrefHybridDriveGaugeSize", return_default=True))
      except (TypeError, ValueError):
        self._gauge_size = 1

    battery_data = self._get_battery_data()
    if battery_data is not None:
      self._soc_filter.update(battery_data['soc'])

  def _should_render(self) -> bool:
    """Check if battery gauge should be rendered"""
    battery_status_enabled = self._params.get_bool("FordPrefHybridBatteryStatus")
    if not battery_status_enabled:
      bp_ui_log.visibility("HybridBattery", False, reason="param_disabled")
      return False

    sm = ui_state.sm
    try:
      if "carStateBP" not in sm.recv_frame:
        bp_ui_log.visibility("HybridBattery", False, reason="no_recv_frame")
        return False
      recv_frame = sm.recv_frame["carStateBP"]
      if recv_frame < ui_state.started_frame:
        bp_ui_log.visibility("HybridBattery", False, reason=f"stale_frame recv={recv_frame} started={ui_state.started_frame}")
        return False
      car_state_bp = sm['carStateBP']
      available = car_state_bp.hybridBattery.dataAvailable
      bp_ui_log.visibility("HybridBattery", available, reason=f"dataAvailable={available}")
      return available
    except (KeyError, AttributeError, TypeError) as e:
      bp_ui_log.visibility("HybridBattery", False, reason=f"exception: {e}")
      return False

  def _get_battery_data(self):
    """Get battery data from carStateBP message"""
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
    """Get battery fill color based on SOC"""
    if soc < 20:
      return BATTERY_LOW_COLOR
    elif soc < 50:
      return BATTERY_MID_COLOR
    else:
      return BATTERY_HIGH_COLOR

  def _compute_bottom_margin(self, s: float) -> float:
    """Compute bottom margin accounting for developer UI bottom bar."""
    margin = BATTERY_Y_MARGIN * s
    if ui_state.developer_ui in (DeveloperUiRenderer.DEV_UI_BOTTOM, DeveloperUiRenderer.DEV_UI_BOTH):
      margin += DeveloperUiRenderer.BOTTOM_BAR_HEIGHT
    return margin

  def _compute_scale(self, rect_width: float) -> float:
    """Compute combined scale factor from content width and gauge size tier."""
    # Base scale from content width (sidebar awareness)
    width_scale = min(1.0, rect_width / FULL_CONTENT_WIDTH)

    # Gauge size tier scale: compute from target height vs natural total height
    # Natural total height at scale=1.0: bat_h + va_y_off + va_font*2 + va_line_sp
    natural_height = BATTERY_HEIGHT + VOLTAGE_AMPS_Y_OFFSET + (VOLTAGE_AMPS_FONT_SIZE * 2) + VOLTAGE_AMPS_LINE_SPACING
    target_height = GAUGE_SIZE_HEIGHTS.get(self._gauge_size, GAUGE_SIZE_HEIGHTS[1])
    # We want the content to fit in target_height minus padding
    content_target = target_height - BACKGROUND_PADDING * 2
    size_scale = content_target / natural_height

    return width_scale * size_scale

  def render(self, rect: rl.Rectangle = None, left_offset: int = 0, y_offset: float = 0.0) -> None:
    """Override render to accept left_offset and y_offset parameters."""
    if rect is not None:
      self.set_rect(rect)
    self._left_offset = left_offset
    self._x_offset = 0.0
    self._y_offset = y_offset
    self._draw_background_flag = True
    return super().render(rect)

  def render_at(self, rect: rl.Rectangle, left_offset: int = 0, draw_background: bool = True,
                x_offset: float = 0.0, y_offset: float = 0.0) -> None:
    """Render at specified rect with optional background control for shared container."""
    if rect is not None:
      self.set_rect(rect)
    self._left_offset = left_offset
    self._x_offset = x_offset
    self._y_offset = y_offset
    self._draw_background_flag = draw_background
    self._update_state()
    if self._should_render():
      self._render(rect)

  def get_bounding_rect(self, rect: rl.Rectangle, left_offset: int = 0,
                       x_offset: float = 0.0, y_offset: float = 0.0):
    """Return the background box rect if gauge would be visible, else None.

    Used by the shared container logic to compute the union rect.
    """
    if not self._should_render():
      return None

    battery_data = self._get_battery_data()
    if battery_data is None:
      return None

    s = self._compute_scale(rect.width)
    soc = battery_data['soc']

    bat_w = BATTERY_WIDTH * s
    bat_h = BATTERY_HEIGHT * s
    term_w = BATTERY_TERMINAL_WIDTH * s
    soc_x_sp = SOC_X_SPACING * s
    soc_font = max(14, int(SOC_FONT_SIZE * s))
    va_font = max(14, int(VOLTAGE_AMPS_FONT_SIZE * s))
    va_y_off = VOLTAGE_AMPS_Y_OFFSET * s
    va_line_sp = VOLTAGE_AMPS_LINE_SPACING * s
    bg_pad = BACKGROUND_PADDING * s
    bat_x_sp = BATTERY_X_SPACING * s
    bat_y_margin = self._compute_bottom_margin(s)

    driver_monitor_right_edge = 250 * min(1.0, rect.width / FULL_CONTENT_WIDTH)
    battery_x_base = left_offset + driver_monitor_right_edge + bat_x_sp
    x = battery_x_base - (bat_w * 0.25) - 5 * min(1.0, rect.width / FULL_CONTENT_WIDTH) + x_offset
    total_height = bat_h + va_y_off + (va_font * 2) + va_line_sp
    y = rect.y + rect.height - total_height - bat_y_margin

    soc_text = f"{int(soc)}%"
    soc_text_size = measure_text_cached(self._font_bold, soc_text, soc_font)
    soc_text_end_x = x + bat_w + term_w + soc_x_sp + soc_text_size.x

    background_width = soc_text_end_x - x + bg_pad * 2
    background_height = total_height + bg_pad * 2
    background_x = x - bg_pad
    background_y = y - bg_pad + y_offset

    return rl.Rectangle(background_x, background_y, background_width, background_height)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render the battery gauge"""
    try:
      left_offset = self._left_offset
      draw_background = self._draw_background_flag

      if not self._should_render():
        return

      battery_data = self._get_battery_data()
      if battery_data is None:
        return

      soc = battery_data['soc']
      voltage = battery_data['voltage']
      amps = battery_data['amps']

      animated_soc = self._soc_filter.x

      s = self._compute_scale(rect.width)

      # Scaled dimensions
      bat_w = BATTERY_WIDTH * s
      bat_h = BATTERY_HEIGHT * s
      term_w = BATTERY_TERMINAL_WIDTH * s
      term_h = BATTERY_TERMINAL_HEIGHT * s
      border_thick = max(1, BATTERY_BORDER_THICKNESS * s)
      soc_font = max(14, int(SOC_FONT_SIZE * s))
      va_font = max(14, int(VOLTAGE_AMPS_FONT_SIZE * s))
      soc_x_sp = SOC_X_SPACING * s
      va_y_off = VOLTAGE_AMPS_Y_OFFSET * s
      va_line_sp = VOLTAGE_AMPS_LINE_SPACING * s
      bg_pad = BACKGROUND_PADDING * s
      glow_exp = BACKGROUND_GLOW_EXPANSION * s
      bat_x_sp = BATTERY_X_SPACING * s
      bat_y_margin = self._compute_bottom_margin(s)

      # Position: use width_scale for driver monitor offset (independent of gauge size)
      width_scale = min(1.0, rect.width / FULL_CONTENT_WIDTH)
      driver_monitor_right_edge = 250 * width_scale
      battery_x_base = left_offset + driver_monitor_right_edge + bat_x_sp
      x = battery_x_base - (bat_w * 0.25) - 5 * width_scale + self._x_offset
      total_height = bat_h + va_y_off + (va_font * 2) + va_line_sp
      y = rect.y + rect.height - total_height - bat_y_margin + self._y_offset

      # Background box dimensions
      soc_text = f"{int(soc)}%"
      soc_text_size = measure_text_cached(self._font_bold, soc_text, soc_font)
      soc_text_end_x = x + bat_w + term_w + soc_x_sp + soc_text_size.x
      background_width = soc_text_end_x - x + bg_pad * 2
      background_height = total_height + bg_pad * 2
      background_x = x - bg_pad
      background_y = y - bg_pad

      # Draw background box (unless shared container draws it)
      if draw_background:
        background_rect = rl.Rectangle(background_x, background_y, background_width, background_height)
        glow_rect = rl.Rectangle(
          background_x - glow_exp,
          background_y - glow_exp,
          background_width + glow_exp * 2,
          background_height + glow_exp * 2,
        )
        rl.draw_rectangle_rounded(
          glow_rect, BACKGROUND_ROUNDNESS, 10, rl.Color(20, 20, 20, int(BACKGROUND_BOX_COLOR.a * 0.3))
        )
        rl.draw_rectangle_rounded(background_rect, BACKGROUND_ROUNDNESS, 10, BACKGROUND_BOX_COLOR)

      # Battery body
      battery_body = rl.Rectangle(x, y, bat_w, bat_h)
      rl.draw_rectangle_rounded(battery_body, BATTERY_ROUNDNESS, 10, BATTERY_BG_COLOR)

      # Battery fill based on animated SOC
      fill_width = int(bat_w * (animated_soc / 100.0))
      if fill_width > 0:
        fill_rect = rl.Rectangle(x, y, fill_width, bat_h)
        fill_color = self._get_battery_fill_color(soc)
        rl.draw_rectangle_rounded(fill_rect, BATTERY_ROUNDNESS, 10, fill_color)

      # Battery border
      rl.draw_rectangle_rounded_lines_ex(
        battery_body, BATTERY_ROUNDNESS, 10, border_thick, BATTERY_BORDER_COLOR
      )

      # Battery terminal (positive end, on the right)
      terminal_x = x + bat_w
      terminal_y = y + (bat_h - term_h) / 2
      terminal_rect = rl.Rectangle(terminal_x, terminal_y, term_w, term_h)
      rl.draw_rectangle_rounded(terminal_rect, 0.5, 10, BATTERY_BG_COLOR)
      rl.draw_rectangle_rounded_lines_ex(
        terminal_rect, 0.5, 10, border_thick, BATTERY_BORDER_COLOR
      )

      # SOC percentage to the right of battery
      soc_x = x + bat_w + term_w + soc_x_sp
      soc_y = y + (bat_h - soc_text_size.y) / 2
      rl.draw_text_ex(
        self._font_bold, soc_text, rl.Vector2(soc_x, soc_y),
        soc_font, 0, TEXT_COLOR
      )

      # Voltage and amps below battery
      voltage_text = f"{int(voltage)}V"
      amps_text = f"{int(amps):+d}A"

      voltage_x = x
      amps_x = x
      voltage_y = y + bat_h + va_y_off
      amps_y = voltage_y + va_font + va_line_sp

      rl.draw_text_ex(
        self._font_medium, voltage_text, rl.Vector2(voltage_x, voltage_y),
        va_font, 0, TEXT_COLOR
      )

      amps_color = CHARGING_COLOR if amps > 0 else DISCHARGING_COLOR if amps < 0 else TEXT_COLOR
      rl.draw_text_ex(
        self._font_medium, amps_text, rl.Vector2(amps_x, amps_y),
        va_font, 0, amps_color
      )
    except Exception as e:
      cloudlog.error(f"HybridBatteryGauge render error: {e}")
      cloudlog.error(traceback.format_exc())
