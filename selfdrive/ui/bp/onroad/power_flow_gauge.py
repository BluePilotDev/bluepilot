"""
Power Flow Gauge for BluePilot (raylib port of Qt HybridGaugesOverlay).

Horizontal center-zero bidirectional power bar with text labels.
Flat styling matching the battery gauge container.
"""
import numpy as np
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter, BounceFilter
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiRenderer
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from opendbc.car.ford.helpers import get_hev_engine_on_reason_text, get_hev_power_flow_text
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

# --- Size presets (width_ratio, height) for each gauge_scale ---
# Small (1) = previous large; Large (2) = new bigger tier; bar_ratio reduced for a thinner bar
SIZE_PRESETS = {
    1: {"width_ratio": 0.32, "height": 125, "font_size": 28, "bar_ratio": 0.40},
    2: {"width_ratio": 0.38, "height": 160, "font_size": 32, "bar_ratio": 0.40},
}
SIZE_PRESETS_SIDEBAR = {
    1: {"width_ratio": 0.28, "height": 125, "font_size": 28, "bar_ratio": 0.40},
    2: {"width_ratio": 0.34, "height": 160, "font_size": 32, "bar_ratio": 0.40},
}

# Background (matching battery gauge style)
BACKGROUND_BOX_COLOR = rl.Color(20, 20, 20, 100)
BACKGROUND_ROUNDNESS = 0.3
BACKGROUND_GLOW_EXPANSION = 4
BACKGROUND_PADDING = 15

# Bar styling
BAR_BG_COLOR = rl.Color(52, 73, 94, 200)
BAR_INSET_COLOR = rl.Color(0, 0, 0, 40)
BAR_BORDER_COLOR = rl.Color(189, 195, 199, 120)
BAR_BORDER_THICKNESS = 1.5
BAR_HIGHLIGHT_ALPHA = 30  # Top-half highlight on active fill
BAR_ROUND_RADIUS = 28.0  # Rounded track ends
BAR_MARGIN = 8

# Center line
CENTER_LINE_COLOR = rl.Color(236, 240, 241, 180)

# Scale markers
MARKER_COLOR = rl.Color(200, 200, 200, 150)
MARKER_COLOR_MAJOR = rl.Color(236, 240, 241, 200)

# Power bar fill colors (flat)
COLOR_REGEN = rl.Color(0, 220, 80, 255)
COLOR_EV = rl.Color(30, 144, 255, 255)
COLOR_ICE_LOW = rl.Color(200, 200, 200, 255)
COLOR_ICE_HIGH = rl.Color(255, 140, 0, 255)

# Border colors by mode
BORDER_COLOR_REGEN = rl.Color(0, 255, 127, 180)
BORDER_COLOR_EV = rl.Color(30, 144, 255, 180)
BORDER_COLOR_HYBRID = rl.Color(100, 149, 237, 180)
BORDER_COLOR_DEFAULT = rl.Color(70, 130, 180, 160)

# EV bracket colors
BRACKET_COLOR_NORMAL = rl.Color(243, 156, 18, 200)

# Text
TEXT_COLOR = rl.Color(236, 240, 241, 220)

# Sizing reference
FULL_CONTENT_WIDTH = 2100.0
PARAM_REFRESH_FRAMES = 60
BORDER_THICKNESS = 2.0


class PowerFlowGauge(Widget):
  """Horizontal center-zero power flow gauge, ported from Qt HybridGaugesOverlay."""

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._font_bold = gui_app.font(FontWeight.BOLD)

    # Smooth filters — low RC for snappy response (Qt used raw values with no filter)
    self._powerflow_filter = FirstOrderFilter(0.0, 0.05, 1.0 / gui_app.target_fps)
    self._bracket_scale_filter = BounceFilter(1.0, 0.08, 1.0 / gui_app.target_fps, bounce=3)

    # State
    self._power_flow_mode_value = 0
    self._engine_on_reason_value = 0
    self._threshold = 0.0

    # Params
    self._powerflow_enabled = False
    self._gauge_size = 1
    self._param_frame_counter = PARAM_REFRESH_FRAMES

  # --- State updates ---

  def _update_state(self):
    self._refresh_params_if_needed()
    if not self._should_render():
      return

    try:
      car_state_bp = ui_state.sm["carStateBP"]
      hd = car_state_bp.hybridDrive
      normalized = np.clip(hd.throttleDemandPercent / 102.0, -1.0, 1.0)
      self._powerflow_filter.update(normalized)
      self._threshold = hd.throttleThresholdPercent
      self._power_flow_mode_value = hd.powerFlowModeValue
      self._engine_on_reason_value = hd.engineOnReasonValue
      self._update_bracket_animation(normalized, self._threshold)
    except (KeyError, AttributeError, TypeError):
      self._power_flow_mode_value = 0
      self._engine_on_reason_value = 0

  def _refresh_params_if_needed(self):
    self._param_frame_counter += 1
    if self._param_frame_counter >= PARAM_REFRESH_FRAMES:
      self._param_frame_counter = 0
      self._powerflow_enabled = self._params.get_bool("FordPrefHybridPowerFlow")
      try:
        self._gauge_size = int(self._params.get("FordPrefHybridDriveGaugeSize", return_default=True))
      except (TypeError, ValueError):
        self._gauge_size = 1

  def should_render(self) -> bool:
    """Public method — refreshes params before checking visibility."""
    self._refresh_params_if_needed()
    return self._should_render()

  def _should_render(self) -> bool:
    if not self._powerflow_enabled:
      bp_ui_log.visibility("PowerFlowGauge", False, reason="param_disabled")
      return False
    try:
      if "carStateBP" not in ui_state.sm.recv_frame:
        bp_ui_log.visibility("PowerFlowGauge", False, reason="no_recv_frame")
        return False
      if ui_state.sm.recv_frame["carStateBP"] < ui_state.started_frame:
        bp_ui_log.visibility("PowerFlowGauge", False, reason=f"stale_frame recv={ui_state.sm.recv_frame['carStateBP']} started={ui_state.started_frame}")
        return False
      available = ui_state.sm["carStateBP"].hybridDrive.dataAvailable
      bp_ui_log.visibility("PowerFlowGauge", available, reason=f"dataAvailable={available}")
      return available
    except (KeyError, AttributeError, TypeError) as e:
      bp_ui_log.visibility("PowerFlowGauge", False, reason=f"exception: {e}")
      return False

  def _update_bracket_animation(self, current_value, threshold):
    """Smooth bracket scale animation when demand approaches EV threshold."""
    if threshold <= 0:
      self._bracket_scale_filter.update(1.0)
      return
    is_near = abs(abs(current_value * 100) - threshold) < 5.0
    target_scale = 1.3 if is_near else 1.0
    self._bracket_scale_filter.update(target_scale)

  # --- Sizing ---

  def get_gauge_rect(self, content_rect: rl.Rectangle, sidebar_visible: bool,
                     confidence_ball_visible: bool) -> rl.Rectangle:
    """Compute the gauge's position and size based on scale settings."""
    presets = SIZE_PRESETS_SIDEBAR if sidebar_visible else SIZE_PRESETS
    preset = presets.get(self._gauge_size, presets[1])

    gauge_width = content_rect.width * preset["width_ratio"]
    gauge_height = preset["height"]

    # Developer UI adjustments
    dev_ui = ui_state.developer_ui
    if dev_ui in (DeveloperUiRenderer.DEV_UI_RIGHT, DeveloperUiRenderer.DEV_UI_BOTH):
      gauge_width *= 0.85

    bottom_margin = 30
    if dev_ui in (DeveloperUiRenderer.DEV_UI_BOTTOM, DeveloperUiRenderer.DEV_UI_BOTH):
      bottom_margin += 70

    # Confidence ball adjustment
    if confidence_ball_visible:
      gauge_width *= 0.92

    # Bottom-center position
    gauge_x = content_rect.x + (content_rect.width - gauge_width) / 2
    gauge_y = content_rect.y + content_rect.height - gauge_height - bottom_margin

    return rl.Rectangle(gauge_x, gauge_y, gauge_width, gauge_height)

  def get_gauge_size(self) -> int:
    """Return current gauge size tier for battery gauge to match."""
    return self._gauge_size

  # --- Rendering ---

  def render_at(self, gauge_rect: rl.Rectangle, draw_background: bool = True, draw_border: bool = True):
    """Render the power flow gauge at the specified rect.

    Args:
        gauge_rect: The rectangle to render in.
        draw_background: Whether to draw the background box (False when shared container draws it).
        draw_border: Whether to draw the mode-colored border (False when shared container draws it
            around the entire gauge area instead).
    """
    self._update_state()
    if not self._should_render():
      return

    try:
      if draw_background:
        self._draw_background(gauge_rect)
      if draw_border:
        self._draw_border(gauge_rect)
      self._draw_power_bar(gauge_rect)
      self._draw_text_area(gauge_rect)
    except Exception as e:
      cloudlog.error(f"PowerFlowGauge render error: {e}")

  def _render(self, rect: rl.Rectangle) -> None:
    """Standard Widget render (standalone mode)."""
    if not self._should_render():
      return
    gauge_rect = self.get_gauge_rect(rect, False, False)
    self.render_at(gauge_rect, draw_background=True)

  def _draw_background(self, rect: rl.Rectangle):
    """Flat dark background matching battery gauge style."""
    glow_rect = rl.Rectangle(
      rect.x - BACKGROUND_GLOW_EXPANSION,
      rect.y - BACKGROUND_GLOW_EXPANSION,
      rect.width + BACKGROUND_GLOW_EXPANSION * 2,
      rect.height + BACKGROUND_GLOW_EXPANSION * 2,
    )
    rl.draw_rectangle_rounded(
      glow_rect, BACKGROUND_ROUNDNESS, 10,
      rl.Color(20, 20, 20, int(BACKGROUND_BOX_COLOR.a * 0.3)),
    )
    rl.draw_rectangle_rounded(rect, BACKGROUND_ROUNDNESS, 10, BACKGROUND_BOX_COLOR)

  def _draw_border(self, rect: rl.Rectangle):
    """Subtle colored border that changes by power mode."""
    border_color = self.get_border_color()
    rl.draw_rectangle_rounded_lines_ex(
      rect, BACKGROUND_ROUNDNESS, 10, BORDER_THICKNESS, border_color,
    )

  def get_border_color(self) -> rl.Color:
    """Get border color based on current power flow state.

    Public so the shared container can draw the same mode-colored border
    around the entire gauge area (battery + power flow together).
    """
    value = self._powerflow_filter.x
    if value < -0.01:
      return BORDER_COLOR_REGEN
    if self._is_ev_mode():
      return BORDER_COLOR_EV
    if self._is_hybrid_mode():
      return BORDER_COLOR_HYBRID
    return BORDER_COLOR_DEFAULT

  def _draw_power_bar(self, rect: rl.Rectangle):
    """Draw the center-zero bidirectional power bar."""
    presets = SIZE_PRESETS_SIDEBAR if rect.width < FULL_CONTENT_WIDTH * 0.3 else SIZE_PRESETS
    preset = presets.get(self._gauge_size, presets[1])
    bar_height_ratio = preset["bar_ratio"]

    bar_h = int(rect.height * bar_height_ratio)
    bar_rect = rl.Rectangle(
      rect.x + BAR_MARGIN,
      rect.y + BAR_MARGIN,
      rect.width - 2 * BAR_MARGIN,
      bar_h - 2 * BAR_MARGIN,
    )
    if bar_rect.width <= 0 or bar_rect.height <= 0:
      return

    center_x = bar_rect.x + bar_rect.width / 2
    value = self._powerflow_filter.x
    threshold = self._threshold
    is_ev = self._is_ev_mode()

    roundness = min(1.0, BAR_ROUND_RADIUS / max(1, bar_rect.height))

    # Track background
    if is_ev and threshold > 0:
      ev_width = bar_rect.width * (threshold / 100.0)
      track_rect = rl.Rectangle(
        center_x - ev_width / 2, bar_rect.y,
        ev_width, bar_rect.height,
      )
      rl.draw_rectangle_rounded(track_rect, roundness, 10, BAR_BG_COLOR)
    else:
      track_rect = bar_rect
      rl.draw_rectangle_rounded(bar_rect, roundness, 10, BAR_BG_COLOR)

    # Inset shadow inside track (matches Qt)
    inset_rect = rl.Rectangle(track_rect.x + 2, track_rect.y + 2, track_rect.width - 4, track_rect.height - 4)
    inset_roundness = min(1.0, max(0, BAR_ROUND_RADIUS - 2) / max(1, inset_rect.height))
    rl.draw_rectangle_rounded(inset_rect, inset_roundness, 10, BAR_INSET_COLOR)

    # Light border around bar track
    rl.draw_rectangle_rounded_lines_ex(track_rect, roundness, 10, BAR_BORDER_THICKNESS, BAR_BORDER_COLOR)

    # Active fill bar — mirrored from center, rounded outer edges, flat at center
    if abs(value) > 0.005:
      fill_half_width = bar_rect.width * abs(value) / 2
      if value < 0:
        fill_color = COLOR_REGEN
      else:
        fill_color = self._get_demand_color(value, is_ev)
      overshoot = bar_rect.height

      # Left half: rounded left end, clipped flat at center
      rounded_rect_l = rl.Rectangle(
        center_x - fill_half_width, bar_rect.y,
        fill_half_width + overshoot, bar_rect.height,
      )
      bp_ui_log.scissor("PowerFlowGauge", "begin",
                         x=int(center_x - fill_half_width), y=int(bar_rect.y),
                         w=int(fill_half_width), h=int(bar_rect.height))
      rl.begin_scissor_mode(
        int(center_x - fill_half_width), int(bar_rect.y),
        int(fill_half_width), int(bar_rect.height),
      )
      if rounded_rect_l.width > 0:
        rl.draw_rectangle_rounded(rounded_rect_l, roundness, 10, fill_color)
      rl.end_scissor_mode()
      bp_ui_log.scissor("PowerFlowGauge", "end")

      # Right half: clipped flat at center, rounded right end
      rounded_rect_r = rl.Rectangle(
        center_x - overshoot, bar_rect.y,
        fill_half_width + overshoot, bar_rect.height,
      )
      bp_ui_log.scissor("PowerFlowGauge", "begin",
                         x=int(center_x), y=int(bar_rect.y),
                         w=int(fill_half_width), h=int(bar_rect.height))
      rl.begin_scissor_mode(
        int(center_x), int(bar_rect.y),
        int(fill_half_width), int(bar_rect.height),
      )
      if rounded_rect_r.width > 0:
        rl.draw_rectangle_rounded(rounded_rect_r, roundness, 10, fill_color)
      rl.end_scissor_mode()
      bp_ui_log.scissor("PowerFlowGauge", "end")

      # Top-half highlight on active fill for 3D effect (matches Qt)
      fill_rect_full = rl.Rectangle(
        center_x - fill_half_width, bar_rect.y + 2,
        fill_half_width * 2, bar_rect.height // 2 - 2,
      )
      rl.draw_rectangle_rounded(
        fill_rect_full, roundness, 10,
        rl.Color(255, 255, 255, BAR_HIGHLIGHT_ALPHA),
      )

    # Center line
    rl.draw_line_ex(
      rl.Vector2(center_x, bar_rect.y + 1),
      rl.Vector2(center_x, bar_rect.y + bar_rect.height - 1),
      2.0, CENTER_LINE_COLOR,
    )

    # Scale markers (only in non-EV mode)
    if not is_ev:
      for pct in (-75, -50, -25, 25, 50, 75):
        mx = center_x + bar_rect.width * pct / 200.0
        is_major = (pct % 50 == 0)
        marker_h = 8 if is_major else 4
        color = MARKER_COLOR_MAJOR if is_major else MARKER_COLOR
        rl.draw_line_ex(
          rl.Vector2(mx, bar_rect.y),
          rl.Vector2(mx, bar_rect.y + marker_h),
          1.0, color,
        )
        rl.draw_line_ex(
          rl.Vector2(mx, bar_rect.y + bar_rect.height - marker_h),
          rl.Vector2(mx, bar_rect.y + bar_rect.height),
          1.0, color,
        )

    # EV threshold brackets
    if is_ev and threshold > 0:
      self._draw_threshold_brackets(bar_rect, center_x, threshold)

  def _draw_threshold_brackets(self, bar_rect, center_x, threshold):
    """Draw animated threshold brackets at EV range boundary."""
    half_threshold = threshold / 2.0
    value = self._powerflow_filter.x
    proximity = (abs(value) * 100.0 / threshold) * 100.0 if threshold > 0 else 0
    scale = self._bracket_scale_filter.x

    # Color based on proximity
    if proximity < 80.0:
      bracket_color = BRACKET_COLOR_NORMAL
    else:
      t = min((proximity - 80.0) / 20.0, 1.0)
      bracket_color = rl.Color(
        int(243 - t * 43), int(156 - t * 156), 18,
        int(200 + t * 55),
      )

    bracket_width = int(12 * scale)
    bracket_depth = int(8 * scale)
    line_thick = 3.0

    for side in (-1, 1):
      bx = center_x + side * (bar_rect.width * half_threshold / 100.0)

      # Top bracket: horizontal + vertical
      rl.draw_line_ex(
        rl.Vector2(bx - side * bracket_width, bar_rect.y + 1),
        rl.Vector2(bx, bar_rect.y + 1),
        line_thick, bracket_color,
      )
      rl.draw_line_ex(
        rl.Vector2(bx, bar_rect.y + 1),
        rl.Vector2(bx, bar_rect.y + bracket_depth),
        line_thick, bracket_color,
      )

      # Bottom bracket: horizontal + vertical
      rl.draw_line_ex(
        rl.Vector2(bx - side * bracket_width, bar_rect.y + bar_rect.height - 1),
        rl.Vector2(bx, bar_rect.y + bar_rect.height - 1),
        line_thick, bracket_color,
      )
      rl.draw_line_ex(
        rl.Vector2(bx, bar_rect.y + bar_rect.height - 1),
        rl.Vector2(bx, bar_rect.y + bar_rect.height - bracket_depth),
        line_thick, bracket_color,
      )

  def _draw_text_area(self, rect: rl.Rectangle):
    """Draw mode/reason text below the power bar."""
    presets = SIZE_PRESETS_SIDEBAR if rect.width < FULL_CONTENT_WIDTH * 0.3 else SIZE_PRESETS
    preset = presets.get(self._gauge_size, presets[1])
    bar_h = int(rect.height * preset["bar_ratio"])
    font_size = preset["font_size"]

    text_y_start = rect.y + bar_h
    text_height = rect.height - bar_h

    power_flow_text = get_hev_power_flow_text(self._power_flow_mode_value)
    engine_reason_text = get_hev_engine_on_reason_text(self._engine_on_reason_value)

    if not power_flow_text and not engine_reason_text:
      return

    # Combine text
    if power_flow_text and engine_reason_text:
      combined = f"{power_flow_text}  |  {engine_reason_text}"
    elif power_flow_text:
      combined = power_flow_text
    else:
      combined = engine_reason_text

    # Shrink font if text exceeds available width
    max_width = rect.width - 20
    actual_size = font_size
    text_size = measure_text_cached(self._font_bold, combined, actual_size)
    iterations = 0
    while text_size.x > max_width and actual_size > 14 and iterations < 20:
      actual_size -= 1
      text_size = measure_text_cached(self._font_bold, combined, actual_size)
      iterations += 1

    # Center text in text area
    tx = rect.x + (rect.width - text_size.x) / 2
    ty = text_y_start + (text_height - text_size.y) / 2
    rl.draw_text_ex(self._font_bold, combined, rl.Vector2(tx, ty), actual_size, 0, TEXT_COLOR)

  # --- Helpers ---

  def _is_ev_mode(self) -> bool:
    text = get_hev_power_flow_text(self._power_flow_mode_value)
    return "Electric" in text if text else False

  def _is_hybrid_mode(self) -> bool:
    text = get_hev_power_flow_text(self._power_flow_mode_value)
    return "Hybrid" in text if text else False

  def _get_demand_color(self, value: float, is_ev: bool) -> rl.Color:
    """Get color for positive demand bar."""
    if is_ev:
      return COLOR_EV
    # Progressive blend from grey to orange above 33% demand
    norm = min(abs(value), 1.0)
    if norm <= 0.33:
      return COLOR_ICE_LOW
    blend = (norm - 0.33) / 0.67
    return rl.Color(
      int(COLOR_ICE_LOW.r + (COLOR_ICE_HIGH.r - COLOR_ICE_LOW.r) * blend),
      int(COLOR_ICE_LOW.g + (COLOR_ICE_HIGH.g - COLOR_ICE_LOW.g) * blend),
      int(COLOR_ICE_LOW.b + (COLOR_ICE_HIGH.b - COLOR_ICE_LOW.b) * blend),
      255,
    )
