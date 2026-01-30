import pyray as rl
import time
from collections.abc import Callable
from cereal import log
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos, FONT_SCALE
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

SIDEBAR_WIDTH = 375  # Increased to accommodate 75px button
# Column widths: left is 4x wider than right, accounting for margins
# Total available: 375 - 12 (left margin) - 12 (between) - 12 (right margin) = 339px
# Right = 339 / 5 = 68px, Left = 4 * 68 = 272px
LEFT_COLUMN_WIDTH = 272  # 4x wider than right column
RIGHT_COLUMN_WIDTH = 68
PANEL_MARGIN = 10
PANEL_PADDING = 15
# PANEL_HEIGHT is calculated dynamically to fill available space up to gear button
LABEL_FONT_SIZE = 28
VALUE_FONT_SIZE = 24
SMALL_FONT_SIZE = 20

# Button positions (right column)
SETTINGS_BTN_WIDTH = 75  # Width of settings button
SETTINGS_BTN_HEIGHT = 75  # Height of settings button (square)
SETTINGS_BTN_Y = 0  # Will be positioned at bottom
FAN_ICON_SIZE = 80
FAN_SPEED_Y_OFFSET = 60

ThermalStatus = log.DeviceState.ThermalStatus
NetworkType = log.DeviceState.NetworkType


# Color scheme
class Colors:
  # Background colors
  SIDEBAR_BG = rl.Color(45, 45, 45, 255)  # Dark grey
  PANEL_BG = rl.Color(60, 60, 60, 255)  # Medium grey

  # Text colors
  WHITE = rl.WHITE
  WHITE_DIM = rl.Color(255, 255, 255, 170)
  GRAY = rl.Color(150, 150, 150, 255)

  # Status colors (for left edge indicator)
  GOOD = rl.Color(0, 255, 0, 255)  # Green
  WARNING = rl.Color(255, 255, 51, 255)  # Yellow (RGB: 255, 255, 51)
  DANGER = rl.Color(201, 34, 49, 255)  # Red
  OFFLINE = rl.Color(128, 128, 128, 255)  # Grey
  OFFLINE_YELLOW = rl.Color(255, 255, 51, 255)  # Yellow for offline status (same as WARNING)

  # UI elements
  METRIC_BORDER = rl.Color(255, 255, 255, 85)
  BUTTON_NORMAL = rl.WHITE
  BUTTON_PRESSED = rl.Color(255, 255, 255, 166)


NETWORK_TYPES = {
  NetworkType.none: tr_noop("--"),
  NetworkType.wifi: tr_noop("Wi-Fi"),
  NetworkType.ethernet: tr_noop("ETH"),
  NetworkType.cell2G: tr_noop("2G"),
  NetworkType.cell3G: tr_noop("3G"),
  NetworkType.cell4G: tr_noop("LTE"),
  NetworkType.cell5G: tr_noop("5G"),
}

class Sidebar(Widget):
  def __init__(self):
    super().__init__()
    # Network data
    self._net_ssid = "--"
    self._net_strength = 0

    # System metrics
    self._cpu_util = 0
    self._cpu_temp = 0.0
    self._gpu_util = 0
    self._gpu_temp = 0.0
    self._memory_util = 0
    self._fan_speed = 0
    self._fan_rotation = 0.0  # For animation
    self._fan: rl.Texture = gui_app.texture('images/button_fan.png', FAN_ICON_SIZE, FAN_ICON_SIZE)

    # Status colors
    self._vehicle_color = Colors.OFFLINE_YELLOW
    self._connect_color = Colors.OFFLINE_YELLOW
    self._sunnylink_color = Colors.OFFLINE_YELLOW

    # Fonts
    self._font_regular = gui_app.font(FontWeight.NORMAL)
    self._font_bold = gui_app.font(FontWeight.SEMI_BOLD)

    # Icons
    # Load settings icon at original size, we'll scale it when drawing
    self._settings_img = gui_app.texture("images/button_settings2.png", 120, 120)  # Original size
    # Fan icon will be drawn procedurally (no texture needed)

    # Callbacks
    self._on_settings_click: Callable | None = None
    self._on_flag_click: Callable | None = None
    self._open_settings_callback: Callable | None = None

  def set_callbacks(self, on_settings: Callable | None = None, on_flag: Callable | None = None,
                    open_settings: Callable | None = None):
    self._on_settings_click = on_settings
    self._on_flag_click = on_flag
    self._open_settings_callback = open_settings

  def _render(self, rect: rl.Rectangle):
    # Background - dark grey
    rl.draw_rectangle_rec(rect, Colors.SIDEBAR_BG)

    # Calculate column positions
    left_col_x = rect.x + PANEL_MARGIN
    right_col_x = rect.x + LEFT_COLUMN_WIDTH + PANEL_MARGIN * 2

    # Draw left column (status panels)
    self._draw_left_column(left_col_x, rect.y, LEFT_COLUMN_WIDTH, rect.height)

    # Draw right column (fan and gear)
    self._draw_right_column(right_col_x, rect.y, RIGHT_COLUMN_WIDTH, rect.height)

  def _update_state(self):
    sm = ui_state.sm
    if not sm.updated['deviceState']:
      return

    device_state = sm['deviceState']

    # Update network status
    self._update_network_status(device_state)

    # Update system metrics
    self._update_system_metrics(device_state)

    # Update status colors
    self._update_status_colors(device_state)

    # Update fan animation
    if self._fan_speed > 0:
      self._fan_rotation += self._fan_speed * 0.1
      if self._fan_rotation >= 360:
        self._fan_rotation -= 360

  def _update_network_status(self, device_state):
    # Get network strength (0-5 bars)
    try:
      strength = device_state.networkStrength
      self._net_strength = max(0, min(5, strength.raw + 1)) if strength.raw > 0 else 0
    except (AttributeError, ValueError):
      self._net_strength = 0

    # Get SSID from networkInfo.state (for WiFi) or use network type
    try:
      if device_state.networkType == NetworkType.wifi:
        # Try to get SSID from networkInfo.state
        net_info = device_state.networkInfo
        if net_info and hasattr(net_info, 'state') and net_info.state and net_info.state != "":
          self._net_ssid = str(net_info.state)
        elif net_info and hasattr(net_info, 'technology') and net_info.technology and net_info.technology != "":
          self._net_ssid = str(net_info.technology)
        else:
          self._net_ssid = tr(tr_noop("Wi-Fi"))
      else:
        # For non-WiFi, show network type
        net_type = NETWORK_TYPES.get(device_state.networkType.raw, tr_noop("--"))
        self._net_ssid = tr(net_type)
    except (AttributeError, ValueError, TypeError):
      # Fallback if network info is unavailable
      self._net_ssid = tr(tr_noop("--"))

  def _update_system_metrics(self, device_state):
    try:
      # CPU utilization (average of all cores)
      cpu_usage = device_state.cpuUsagePercent
      if cpu_usage and len(cpu_usage) > 0:
        self._cpu_util = int(sum(cpu_usage) / len(cpu_usage))
      else:
        self._cpu_util = 0
    except (AttributeError, ValueError, TypeError):
      self._cpu_util = 0

    try:
      # CPU temperature (max of all cores)
      cpu_temps = device_state.cpuTempC
      if cpu_temps and len(cpu_temps) > 0:
        self._cpu_temp = max(cpu_temps)
      else:
        self._cpu_temp = 0.0
    except (AttributeError, ValueError, TypeError):
      self._cpu_temp = 0.0

    try:
      # GPU utilization
      self._gpu_util = int(device_state.gpuUsagePercent) if device_state.gpuUsagePercent else 0
    except (AttributeError, ValueError, TypeError):
      self._gpu_util = 0

    try:
      # GPU temperature (max of all sensors)
      gpu_temps = device_state.gpuTempC
      if gpu_temps and len(gpu_temps) > 0:
        self._gpu_temp = max(gpu_temps)
      else:
        self._gpu_temp = 0.0
    except (AttributeError, ValueError, TypeError):
      self._gpu_temp = 0.0

    try:
      # Memory utilization
      self._memory_util = int(device_state.memoryUsagePercent) if device_state.memoryUsagePercent else 0
    except (AttributeError, ValueError, TypeError):
      self._memory_util = 0

    try:
      # Fan speed
      self._fan_speed = int(device_state.fanSpeedPercentDesired) if device_state.fanSpeedPercentDesired else 0
    except (AttributeError, ValueError, TypeError):
      self._fan_speed = 0

  def _update_status_colors(self, device_state):
    try:
      # Vehicle status (panda)
      if ui_state.panda_type == log.PandaState.PandaType.unknown:
        self._vehicle_color = Colors.OFFLINE_YELLOW  # Dark yellow when not connected
      else:
        self._vehicle_color = Colors.GOOD
    except (AttributeError, ValueError):
      self._vehicle_color = Colors.OFFLINE_YELLOW

    try:
      # Connect status
      last_ping = device_state.lastAthenaPingTime
      if last_ping == 0:
        self._connect_color = Colors.OFFLINE_YELLOW  # Dark yellow when offline
      elif time.monotonic_ns() - last_ping < 80_000_000_000:  # 80 seconds
        self._connect_color = Colors.GOOD
      else:
        self._connect_color = Colors.DANGER
    except (AttributeError, ValueError, TypeError):
      self._connect_color = Colors.OFFLINE_YELLOW

    # Sunnylink status
    try:
      if hasattr(ui_state, 'sunnylink_state') and ui_state.sunnylink_state:
        if ui_state.sunnylink_state.is_paired():
          self._sunnylink_color = Colors.GOOD
        else:
          self._sunnylink_color = Colors.OFFLINE_YELLOW  # Dark yellow when not paired
      else:
        self._sunnylink_color = Colors.OFFLINE_YELLOW  # Dark yellow when unavailable
    except Exception:
      self._sunnylink_color = Colors.OFFLINE_YELLOW  # Dark yellow on error

  def _draw_left_column(self, x: float, y: float, width: float, height: float):
    """Draw all status panels in the left column"""
    # Calculate available height: from top margin to gear button top
    # Gear button top is at: height - SETTINGS_BTN_HEIGHT - PANEL_MARGIN
    gear_button_top = height - SETTINGS_BTN_HEIGHT - PANEL_MARGIN
    available_height = gear_button_top - PANEL_MARGIN  # Subtract top margin

    # We have 7 panels with 6 gaps between them
    # Make panels 10% taller by reducing gap space proportionally
    num_panels = 7
    num_gaps = num_panels - 1
    # Reduce gaps by ~10% to accommodate taller panels
    adjusted_gap = PANEL_MARGIN * 0.9
    total_gap_height = num_gaps * adjusted_gap
    panel_height = (available_height - total_gap_height) / num_panels * 1.1  # 10% taller

    current_y = y + PANEL_MARGIN

    # Network panel
    current_y = self._draw_status_panel(x, current_y, width, "Network",
                                        bottom_left=self._net_ssid,
                                        bottom_right_callback=self._draw_signal_bars,
                                        status_color=self._get_network_status_color(),
                                        panel_height=panel_height)

    # CPU panel
    current_y = self._draw_status_panel(x, current_y, width, "CPU",
                                        bottom_left=f"{self._cpu_util}%",
                                        bottom_right=f"{int(self._cpu_temp)}°C",
                                        status_color=self._get_temp_status_color(self._cpu_temp),
                                        panel_height=panel_height)

    # GPU panel
    current_y = self._draw_status_panel(x, current_y, width, "GPU",
                                        bottom_left=f"{self._gpu_util}%",
                                        bottom_right=f"{int(self._gpu_temp)}°C",
                                        status_color=self._get_temp_status_color(self._gpu_temp),
                                        panel_height=panel_height)

    # Memory panel
    current_y = self._draw_status_panel(x, current_y, width, "Memory",
                                        bottom_left=f"{self._memory_util}%",
                                        status_color=self._get_util_status_color(self._memory_util),
                                        panel_height=panel_height)

    # Vehicle panel (label only)
    current_y = self._draw_status_panel(x, current_y, width, "Vehicle",
                                        status_color=self._vehicle_color,
                                        panel_height=panel_height)

    # Connect panel (label only)
    current_y = self._draw_status_panel(x, current_y, width, "Connect",
                                        status_color=self._connect_color,
                                        panel_height=panel_height)

    # Sunnylink panel (label only)
    current_y = self._draw_status_panel(x, current_y, width, "Sunnylink",
                                        status_color=self._sunnylink_color,
                                        panel_height=panel_height)

  def _draw_status_panel(self, x: float, y: float, width: float, label: str,
                        bottom_left: str = "", bottom_right: str = "",
                        bottom_right_callback: Callable | None = None,
                        status_color: rl.Color = Colors.GOOD,
                        panel_height: float = 125) -> float:
    """Draw a status panel with colored left edge. Returns next Y position."""
    panel_rect = rl.Rectangle(x, y, width, panel_height)

    # Draw panel background (medium grey rounded rectangle)
    rl.draw_rectangle_rounded(panel_rect, 0.2, 10, Colors.PANEL_BG)

    # Draw colored left edge (~5% of width)
    edge_width = width * 0.05
    edge_rect = rl.Rectangle(panel_rect.x, panel_rect.y, edge_width, panel_rect.height)
    rl.draw_rectangle_rounded(edge_rect, 0.2, 10, status_color)

    # Draw label (upper left)
    label_y = y + PANEL_PADDING
    label_pos = rl.Vector2(x + edge_width + PANEL_PADDING, label_y)
    rl.draw_text_ex(self._font_bold, label, label_pos, LABEL_FONT_SIZE, 0, Colors.WHITE)

    # Draw bottom content
    if bottom_left or bottom_right or bottom_right_callback:
      content_y = y + panel_height - PANEL_PADDING - VALUE_FONT_SIZE * FONT_SCALE

      # Bottom left
      if bottom_left:
        left_pos = rl.Vector2(x + edge_width + PANEL_PADDING, content_y)
        rl.draw_text_ex(self._font_regular, str(bottom_left), left_pos, VALUE_FONT_SIZE, 0, Colors.WHITE_DIM)

      # Bottom right (or callback)
      if bottom_right_callback:
        bottom_right_callback(x + width - PANEL_PADDING, content_y)
      elif bottom_right:
        right_text_size = measure_text_cached(self._font_regular, bottom_right, VALUE_FONT_SIZE)
        right_pos = rl.Vector2(x + width - PANEL_PADDING - right_text_size.x, content_y)
        rl.draw_text_ex(self._font_regular, bottom_right, right_pos, VALUE_FONT_SIZE, 0, Colors.WHITE_DIM)

    # Use adjusted gap for spacing between panels
    adjusted_gap = PANEL_MARGIN * 0.9
    return y + panel_height + adjusted_gap

  def _draw_signal_bars(self, x: float, y: float):
    """Draw 5 signal strength bars (callback for Network panel)"""
    bar_width = 4
    bar_spacing = 2
    bar_heights = [8, 12, 16, 20, 24]  # Increasing heights

    for i in range(5):
      bar_height = bar_heights[i]
      bar_x = x - (5 - i) * (bar_width + bar_spacing) - bar_width
      bar_y = y - bar_height

      # Color based on signal strength
      if i < self._net_strength:
        color = Colors.WHITE
      else:
        color = Colors.GRAY

      rl.draw_rectangle(int(bar_x), int(bar_y), bar_width, bar_height, color)

  def _draw_right_column(self, x: float, y: float, width: float, height: float):
    """Draw fan icon, fan speed, and gear icon in right column"""
    # Fan icon (top)

    fan_x = x + width // 2 + FAN_ICON_SIZE // 2
    fan_y = y + PANEL_MARGIN
    self._draw_fan_icon(fan_x, fan_y)

    # Fan speed percentage (below fan)
    fan_speed_y = fan_y + FAN_ICON_SIZE + 10
    fan_speed_text = f"{self._fan_speed}%"
    text_size = measure_text_cached(self._font_regular, fan_speed_text, LABEL_FONT_SIZE)
    text_x = x + (width - text_size.x) / 2
    text_pos = rl.Vector2(text_x, fan_speed_y)
    rl.draw_text_ex(self._font_regular, fan_speed_text, text_pos, LABEL_FONT_SIZE, 0, Colors.WHITE_DIM)

    # Gear icon (bottom) - position at bottom of sidebar, extend upward
    # Button bottom should be at: y + height - PANEL_MARGIN
    # Button top should be at: y + height - PANEL_MARGIN - SETTINGS_BTN_HEIGHT
    # So button Y position is: height - SETTINGS_BTN_HEIGHT - PANEL_MARGIN
    button_bottom_y = height - PANEL_MARGIN
    button_top_y = button_bottom_y - SETTINGS_BTN_HEIGHT
    self._draw_settings_button(x + (width - SETTINGS_BTN_WIDTH) / 2, y + button_top_y, SETTINGS_BTN_WIDTH, SETTINGS_BTN_HEIGHT)

  def _draw_fan_icon(self, x: float, y: float):
    """Draw static fan icon (QT-style fan shape with 4 curved blades)"""

    src_rect = rl.Rectangle(0, 0, self._fan.width, self._fan.height)
    origin_offset_w = self._fan.width // 2
    origin_offset_h = self._fan.height // 2
    origin = (origin_offset_w, origin_offset_h)
    dest_rect = rl.Rectangle(x - origin_offset_w, y + origin_offset_h, self._fan.width, self._fan.height)
    rl.draw_texture_pro(self._fan, src_rect, dest_rect, origin, self._fan_rotation, rl.WHITE)

  def _draw_settings_button(self, x: float, y: float, width: float, height: float):
    """Draw gear icon button"""
    mouse_pos = rl.get_mouse_position()
    mouse_down = self.is_pressed and rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT)

    button_rect = rl.Rectangle(x, y, width, height)
    settings_down = mouse_down and rl.check_collision_point_rec(mouse_pos, button_rect)

    tint = Colors.BUTTON_PRESSED if settings_down else Colors.BUTTON_NORMAL
    # Scale texture to desired size
    src_rect = rl.Rectangle(0, 0, self._settings_img.width, self._settings_img.height)
    dest_rect = rl.Rectangle(x, y, width, height)
    rl.draw_texture_pro(self._settings_img, src_rect, dest_rect, rl.Vector2(0, 0), 0, tint)

    # Store button rect for click handling
    self._settings_btn_rect = button_rect

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if hasattr(self, '_settings_btn_rect') and rl.check_collision_point_rec(mouse_pos, self._settings_btn_rect):
      if self._on_settings_click:
        self._on_settings_click()

  def _get_network_status_color(self) -> rl.Color:
    """Get color based on network strength"""
    if self._net_strength == 0:
      return Colors.OFFLINE
    elif self._net_strength <= 2:
      return Colors.WARNING
    else:
      return Colors.GOOD

  def _get_temp_status_color(self, temp: float) -> rl.Color:
    """Get color based on temperature"""
    if temp >= 80:
      return Colors.DANGER
    elif temp >= 70:
      return Colors.WARNING
    else:
      return Colors.GOOD

  def _get_util_status_color(self, util: int) -> rl.Color:
    """Get color based on utilization percentage"""
    if util >= 90:
      return Colors.DANGER
    elif util >= 75:
      return Colors.WARNING
    else:
      return Colors.GOOD
