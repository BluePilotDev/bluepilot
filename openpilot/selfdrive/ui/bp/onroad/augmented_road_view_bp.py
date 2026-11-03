import time
from enum import IntEnum
import pyray as rl
from openpilot.cereal import log, messaging
from openpilot.common.params import Params
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.bp.onroad.cameraview_bp import CameraViewBP
from openpilot.selfdrive.ui.bp.onroad.blindspot_renderer import BlindspotRendererMixin
from openpilot.selfdrive.ui.bp.onroad.hud_renderer_bp import HudRendererBP
from openpilot.selfdrive.ui.bp.onroad.alert_renderer_bp import AlertRendererBP
from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import ModelRendererBP
from openpilot.selfdrive.ui.bp.onroad.driver_state_bp import DriverStateRendererBP
from openpilot.selfdrive.ui.bp.lib.dm_icon_style import DMIconStyle
from openpilot.selfdrive.ui.bp.onroad.hybrid_battery_gauge import HybridBatteryGauge
from openpilot.selfdrive.ui.bp.onroad.hybrid_battery_gauge_arched import HybridBatteryGaugeArched, BATTERY_START_ANGLE
from openpilot.selfdrive.ui.bp.onroad.power_flow_gauge import PowerFlowGauge
from openpilot.selfdrive.ui.bp.onroad.powerflow_gauge_arched import PowerflowGaugeArched, POWERFLOW_ANGLE_SPAN
from openpilot.selfdrive.ui.bp.onroad.torque_bar_renderer_bp import TorqueBarRendererBP
from openpilot.selfdrive.ui.bp.onroad.rad_racer_theme import RadRacerTheme
from openpilot.selfdrive.ui.bp.mici.onroad.confidence_ball_bp import ConfidenceBallTiciBP
from openpilot.selfdrive.ui.onroad.driver_state import BTN_SIZE
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiState, get_bottom_dev_ui_offset
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log
# BluePilot: unified theme selector (BPThemePack param)
from openpilot.selfdrive.ui.bp.lib import theme_pack, theme_scene


class GaugeStyle(IntEnum):
  flat = 0
  arched = 1


# BluePilot: Margin to keep confidence ball inside the colored border
BALL_BORDER_MARGIN = UI_BORDER_SIZE // 2  # 15px

# Shared container styling (matches battery/power flow gauge backgrounds)
SHARED_BG_COLOR = rl.Color(20, 20, 20, 100)
SHARED_BG_ROUNDNESS = 0.3
SHARED_BG_GLOW_EXPANSION = 4
SHARED_INNER_GAP = 10  # Pixels between battery and power flow gauges inside shared container
SHARED_PADDING = 8     # Padding around inner content in shared container
SHARED_BORDER_THICKNESS = 2.0  # Power flow mode-colored border thickness

# Torque crown strip (thin torque bar integrated into the top of the shared container)
TORQUE_STRIP_HEIGHT = 11   # Height of the torque strip inside shared container
TORQUE_STRIP_GAP = 3       # Gap between strip bottom and gauge content top

# Full screen reference for sidebar detection
FULL_CONTENT_WIDTH = 2100.0


class AugmentedRoadViewBP(CameraViewBP, AugmentedRoadView, BlindspotRendererMixin):
  """BluePilot AugmentedRoadView with blindspot indicators, gauges, and BP renderers."""

  BLIND_SPOT_WIDTH = 250  # Wider for TICI's larger screen

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._init_blindspot()
    self._bp_params = Params()

    # BluePilot: Replace renderers with BP versions
    self.model_renderer = ModelRendererBP()
    self._hud_renderer = HudRendererBP()
    self.alert_renderer = AlertRendererBP()
    self.driver_state_renderer = DriverStateRendererBP(DMIconStyle.COMMA_3X)
    self._battery_gauge_bp = HybridBatteryGauge()
    self._power_flow_gauge = PowerFlowGauge()
    self._battery_gauge_arched = HybridBatteryGaugeArched()
    self._power_flow_gauge_arched = PowerflowGaugeArched()
    # BluePilot: Torque bar renderer (renders as crown strip inside gauge container)
    self._torque_bar = TorqueBarRendererBP(scale=3.0)
    self._power_flow_gauge_arched.set_torque_bar(self._torque_bar)

    # BluePilot: Add confidence ball on left side (MADS beam + enhanced coloring)
    self._confidence_ball = ConfidenceBallTiciBP()
    self._show_confidence_ball = self._bp_params.get_bool("BPShowConfidenceBall")
    self._param_counter = 0
    self._hybrid_gauge_style = GaugeStyle(self._bp_params.get("FordPrefGaugeStyle", return_default=True) or 0)
    # BluePilot: Cache param to avoid per-frame disk I/O (refreshed in existing 60-frame block)
    self._hide_onroad_border = self._bp_params.get_bool("BPHideOnroadBorder")
    try:
      self._cached_gauge_size = int(self._bp_params.get("FordPrefHybridDriveGaugeSize", return_default=True))
    except (TypeError, ValueError):
      self._cached_gauge_size = 2

    # BluePilot: Rad Racer 8-bit theme (drawing host; selection lives in theme_scene)
    self._rad_racer_theme = RadRacerTheme()

  def update_fade_out_bottom_overlay(self, _content_rect):
    """BluePilot: Skip MICI fade overlay on TICI — causes unwanted black gradient at bottom."""
    pass

  def _render(self, rect):
    """Override render to add blindspot, gauges, confidence ball on left."""
    bp_ui_log.tick()
    if not ui_state.started:
      return

    # Refresh param periodically (~1s at 60fps)
    self._param_counter += 1
    if self._param_counter >= 60:
      self._param_counter = 0
      self._show_confidence_ball = self._bp_params.get_bool("BPShowConfidenceBall")
      self._hide_onroad_border = self._bp_params.get_bool("BPHideOnroadBorder")
      try:
        self._cached_gauge_size = int(self._bp_params.get("FordPrefHybridDriveGaugeSize", return_default=True))
      except (TypeError, ValueError):
        self._cached_gauge_size = 2
      self._hybrid_gauge_style = GaugeStyle(self._bp_params.get("FordPrefGaugeStyle", return_default=True) or 0)

    self._switch_stream_if_needed(ui_state.sm)
    self._update_calibration()

    # Create inner content area with border padding
    self._content_rect = rl.Rectangle(
      rect.x + UI_BORDER_SIZE,
      rect.y + UI_BORDER_SIZE,
      rect.width - 2 * UI_BORDER_SIZE,
      rect.height - 2 * UI_BORDER_SIZE,
    )

    # BluePilot: Offset rect pushes HUD/driver state/alerts right of the confidence ball
    ball_offset = (ConfidenceBallTiciBP.BALL_WIDTH + BALL_BORDER_MARGIN) if self._show_confidence_ball else 0
    ui_rect = rl.Rectangle(
      self._content_rect.x + ball_offset,
      self._content_rect.y,
      self._content_rect.width - ball_offset,
      self._content_rect.height,
    )

    bp_ui_log.scissor("AugRoadView", "begin",
                       x=int(self._content_rect.x), y=int(self._content_rect.y),
                       w=int(self._content_rect.width), h=int(self._content_rect.height))
    rl.begin_scissor_mode(
      int(self._content_rect.x),
      int(self._content_rect.y),
      int(self._content_rect.width),
      int(self._content_rect.height)
    )

    # Render the base camera view. Minimal Driving View suppression lives in CameraViewBP.
    CameraViewBP._render(self, rect)

    # BluePilot: unified theme scene dispatch. A scene that replaces the HUD (Rad Racer)
    # takes over the whole render; pack scenes draw a background pass here and a
    # foreground pass just before alerts.
    _scene = theme_scene.active_scene()
    if _scene is not None and _scene.replaces_hud():
      self._render_rad_racer_scene(rect)
      return
    if _scene is not None:
      _scene.draw_background(self._content_rect, getattr(self, "_bp_hide_camera_view", False))

    # Render model (uses full content rect for camera-space overlays)
    self.model_renderer.render(self._content_rect)

    # SP fade overlay
    self.update_fade_out_bottom_overlay(self._content_rect)

    # BluePilot: Render confidence ball on left side (narrow rect = ball strip only, not full width)
    if self._show_confidence_ball:
      ball_strip_width = ConfidenceBallTiciBP.BALL_WIDTH + BALL_BORDER_MARGIN
      # Semi-transparent dark backdrop so ball is visible against bright camera feed
      rl.draw_rectangle_gradient_h(
        int(self._content_rect.x),
        int(self._content_rect.y),
        int(ball_strip_width),
        int(self._content_rect.height),
        rl.Color(20, 20, 20, 120),  # darker at left edge
        rl.Color(20, 20, 20, 0),    # fade to transparent at right edge
      )
      ball_rect = rl.Rectangle(
        self._content_rect.x + BALL_BORDER_MARGIN,
        self._content_rect.y,
        ball_strip_width,
        self._content_rect.height,
      )
      self._confidence_ball.render(ball_rect)

    # BluePilot: Draw blindspot screen edge indicators ON TOP of confidence ball backdrop
    # so the red safety warning is never obscured
    self._draw_blindspot_screen_edges(self._content_rect, self.BLIND_SPOT_WIDTH)

    # BluePilot: Render HUD, driver state before gauges and alerts
    self._hud_renderer.set_gradient_rect(self._content_rect)
    self._hud_renderer.render(ui_rect)

    bp_ui_log.scissor("AugRoadView", "reset (defensive)")
    # Defensive: re-establish scissor before drawing driver state and battery. Some HUD widgets
    # (e.g. speed limit, brake status, unified gauge) can leave raylib state in a bad way on device,
    # causing the bottom-left widgets to be clipped or not drawn. Calling begin_scissor_mode again
    # (without end first) flushes pending draws and resets the scissor rect in one GPU call instead of two.
    rl.begin_scissor_mode(
      int(self._content_rect.x),
      int(self._content_rect.y),
      int(self._content_rect.width),
      int(self._content_rect.height)
    )
    self.driver_state_renderer.render(ui_rect)

    # BluePilot: Update torque bar filter state (once per frame, before _render_gauges uses it)
    self._torque_bar.update()

    # BluePilot: Render gauges + torque crown strip inside shared container
    gauge_height_offset, hybrid_active = self._render_gauges(self._content_rect, ball_offset)
    bp_ui_log.state("AugRoadView", "hybrid_active", hybrid_active)
    bp_ui_log.state("AugRoadView", "gauge_height_offset", round(gauge_height_offset))

    # BluePilot: When no hybrid gauge is active, fall back to the stock arc torque bar
    if not hybrid_active:
      torque_rect = ui_rect
      if ui_state.developer_ui in (DeveloperUiState.BOTTOM, DeveloperUiState.BOTH):
        torque_rect = rl.Rectangle(ui_rect.x, ui_rect.y, ui_rect.width, ui_rect.height - get_bottom_dev_ui_offset())
      self._torque_bar.render(torque_rect, gauge_height_offset=gauge_height_offset)

    # BluePilot: theme scene foreground — decor + near particles over the HUD, under alerts.
    if _scene is not None and not _scene.replaces_hud():
      _scene.draw_foreground(self._content_rect, getattr(self, "_bp_hide_camera_view", False))

    # Alerts last so they are never covered by gauges or other overlays.
    # BluePilot: Full-screen alerts (e.g. reverse gear) must use content_rect so they cover
    # the confidence ball strip; otherwise the camera shows through where the ball was.
    alert = self.alert_renderer.get_alert(ui_state.sm)
    alert_rect = (
      self._content_rect
      if (alert and alert.size == log.SelfdriveState.AlertSize.full and self._show_confidence_ball)
      else ui_rect
    )
    self.alert_renderer.render(alert_rect)

    bp_ui_log.scissor("AugRoadView", "end")
    rl.end_scissor_mode()

    # BluePilot: Conditionally draw border
    if not self._hide_onroad_border:
      self._draw_border(rect)

  def _render_rad_racer_scene(self, rect: rl.Rectangle):
    """Render the full Rad Racer 8-bit scene: skyline, road, sprites, gauge cluster.

    Called from _render with scissor mode already active on content_rect; ends
    scissor mode and draws the border before returning (mirrors the stock path).
    """
    content_rect = self._content_rect

    # Sky/signs project car-space points before the model road is rendered.
    self.model_renderer.prepare_projection(content_rect)

    # Sky, stars, skyline, roadside signs (behind the road)
    self._rad_racer_theme.render_background(content_rect, self.model_renderer)

    # Green game road (ModelRendererBP handles the 8-bit styling internally)
    self.model_renderer.render(content_rect)

    # Blindspot red edges stay on — safety overlay
    self._draw_blindspot_screen_edges(content_rect, self.BLIND_SPOT_WIDTH)

    # Gauge cluster panel first, then sprites so the ego car overlaps the panel edge
    cluster_top = self._rad_racer_theme.cluster_top(content_rect)
    self._torque_bar.update()
    self._rad_racer_theme.render_cluster(content_rect, self._torque_bar)
    self._rad_racer_theme.render_foreground(content_rect, self.model_renderer, cluster_top)

    # Driver monitor floats bottom-left of the road view, just above the gauge cluster
    self.driver_state_renderer.render(self._rad_racer_theme.driver_monitor_rect(content_rect))

    # Alerts always on top
    self.alert_renderer.render(content_rect)

    bp_ui_log.scissor("AugRoadView", "end (rad racer)")
    rl.end_scissor_mode()

    if not self._hide_onroad_border:
      self._draw_border(rect)

  def _get_dm_center_y(self, content_rect: rl.Rectangle) -> float:
    """Get the driver monitor face icon's vertical center Y coordinate.

    This matches the positioning in DriverStateRendererSP._pre_calculate_drawing_elements():
      position_y = rect.y + height - (UI_BORDER_SIZE + BTN_SIZE // 2) - dev_ui_offset
    """
    dev_ui_offset = get_bottom_dev_ui_offset()
    return content_rect.y + content_rect.height - (UI_BORDER_SIZE + BTN_SIZE // 2) - dev_ui_offset

  def _render_gauges(self, content_rect: rl.Rectangle, ball_offset: float) -> tuple[float, bool]:
    """Render power flow, battery gauges, and torque strip, vertically centered with the driver monitor.

    The torque strip ("crown strip") is a thin horizontal torque bar rendered at the top
    of the shared container when a hybrid gauge (battery or power flow) is active.
    When no hybrid gauge is active, the strip is not rendered — the stock arc torque bar
    is used instead (handled by the caller).

    When gauge style is arched and powerflow is ON: use arched powerflow + arched battery.
    When powerflow is OFF: always use flat battery (no arched battery solo).
    When style is flat: use flat battery and/or flat powerflow.

    Returns:
        (gauge_height_offset, hybrid_active): gauge_height_offset is pixels from bottom of
            content_rect to top of gauge area (0 if nothing visible). hybrid_active is True
            when at least one hybrid gauge was rendered (strip was used for torque).
    """
    # Arched style + powerflow on: use arched layout. Powerflow off → flat battery only.
    if getattr(self, "_hybrid_gauge_style", GaugeStyle.flat) == GaugeStyle.arched:
      self._power_flow_gauge_arched._update_state()
      if self._power_flow_gauge_arched._should_render():
        gauge_size = min(max(self._cached_gauge_size, 1), 2)  # 1 = small, 2 = large
        arched_scale = 0.75 if gauge_size == 1 else 1.0
        return self._render_gauges_arched(content_rect, ball_offset, scale=arched_scale)
      # Powerflow off: fall through to flat path (flat battery)

    left_offset = content_rect.x + ball_offset
    sidebar_visible = content_rect.width < (FULL_CONTENT_WIDTH * 0.9)
    content_bottom = content_rect.y + content_rect.height

    # Driver monitor center Y — container will be vertically centered to this
    dm_center_y = self._get_dm_center_y(content_rect)

    # Check visibility of each element
    battery_rect = self._battery_gauge_bp.get_bounding_rect(content_rect, left_offset)
    pf_visible = self._power_flow_gauge.should_render()
    hybrid_active = battery_rect is not None or pf_visible
    torque_strip_visible = ui_state.torque_bar and hybrid_active

    # Strip adds height to the container when visible
    strip_allocation = (TORQUE_STRIP_HEIGHT + TORQUE_STRIP_GAP) if torque_strip_visible else 0

    # Track the top of the gauge area
    gauge_top = content_bottom  # default: nothing visible, no offset

    if battery_rect is not None and pf_visible:
      # Both gauges visible: horizontally center the combined container
      pf_rect = self._power_flow_gauge.get_gauge_rect(
        content_rect, sidebar_visible, self._show_confidence_ball,
      )

      total_inner_width = battery_rect.width + SHARED_INNER_GAP + pf_rect.width
      combined_left = content_rect.x + (content_rect.width - total_inner_width) / 2

      # Gauge content height (without strip)
      gauge_content_height = max(battery_rect.height, pf_rect.height)
      container_inner_height = gauge_content_height + strip_allocation

      # Vertically center the entire container with driver monitor
      container_top = dm_center_y - container_inner_height / 2

      # Strip at top, gauges below
      strip_top = container_top
      gauge_content_top = container_top + strip_allocation

      pf_y = gauge_content_top
      pf_x = combined_left + battery_rect.width + SHARED_INNER_GAP
      battery_y = gauge_content_top + (gauge_content_height - battery_rect.height) / 2

      # Shared container rect with padding
      shared_rect = rl.Rectangle(
        combined_left - SHARED_PADDING,
        container_top - SHARED_PADDING,
        total_inner_width + SHARED_PADDING * 2,
        container_inner_height + SHARED_PADDING * 2,
      )

      # Draw shared background + power flow mode-colored border
      self._draw_shared_background(shared_rect)
      border_color = self._power_flow_gauge.get_border_color()
      rl.draw_rectangle_rounded_lines_ex(
        shared_rect, SHARED_BG_ROUNDNESS, 10, SHARED_BORDER_THICKNESS, border_color,
      )

      # Torque strip at the top of the inner content
      if torque_strip_visible:
        strip_rect = rl.Rectangle(combined_left, strip_top, total_inner_width, TORQUE_STRIP_HEIGHT)
        self._torque_bar.render_strip(strip_rect)

      # Battery at centered position
      battery_x_offset = combined_left - battery_rect.x
      battery_y_offset = battery_y - battery_rect.y
      self._battery_gauge_bp.render_at(content_rect, left_offset, draw_background=False,
                                       x_offset=battery_x_offset, y_offset=battery_y_offset)

      # Power flow gauge
      adjusted_pf_rect = rl.Rectangle(pf_x, pf_y, pf_rect.width, pf_rect.height)
      self._power_flow_gauge.render_at(adjusted_pf_rect, draw_background=False, draw_border=False)

      gauge_top = shared_rect.y

    elif pf_visible:
      # Only power flow visible: shared container with optional strip
      pf_rect = self._power_flow_gauge.get_gauge_rect(
        content_rect, sidebar_visible, self._show_confidence_ball,
      )

      total_height = pf_rect.height + strip_allocation
      container_top = dm_center_y - total_height / 2

      strip_top = container_top
      pf_y = container_top + strip_allocation

      # Shared container around power flow + optional strip
      shared_rect = rl.Rectangle(
        pf_rect.x - SHARED_PADDING,
        container_top - SHARED_PADDING,
        pf_rect.width + SHARED_PADDING * 2,
        total_height + SHARED_PADDING * 2,
      )

      self._draw_shared_background(shared_rect)
      border_color = self._power_flow_gauge.get_border_color()
      rl.draw_rectangle_rounded_lines_ex(
        shared_rect, SHARED_BG_ROUNDNESS, 10, SHARED_BORDER_THICKNESS, border_color,
      )

      if torque_strip_visible:
        strip_rect = rl.Rectangle(pf_rect.x, strip_top, pf_rect.width, TORQUE_STRIP_HEIGHT)
        self._torque_bar.render_strip(strip_rect)

      centered_pf_rect = rl.Rectangle(pf_rect.x, pf_y, pf_rect.width, pf_rect.height)
      self._power_flow_gauge.render_at(centered_pf_rect, draw_background=False, draw_border=False)
      gauge_top = shared_rect.y

    elif battery_rect is not None:
      # Only battery visible
      if torque_strip_visible:
        # Wrap battery + strip in a shared container
        total_height = battery_rect.height + strip_allocation
        container_top = dm_center_y - total_height / 2

        strip_top = container_top
        battery_content_top = container_top + strip_allocation

        shared_rect = rl.Rectangle(
          battery_rect.x - SHARED_PADDING,
          container_top - SHARED_PADDING,
          battery_rect.width + SHARED_PADDING * 2,
          total_height + SHARED_PADDING * 2,
        )
        self._draw_shared_background(shared_rect)
        rl.draw_rectangle_rounded_lines_ex(
          shared_rect, SHARED_BG_ROUNDNESS, 10, SHARED_BORDER_THICKNESS,
          rl.Color(120, 120, 120, 100),
        )

        strip_rect = rl.Rectangle(battery_rect.x, strip_top, battery_rect.width, TORQUE_STRIP_HEIGHT)
        self._torque_bar.render_strip(strip_rect)

        y_shift = battery_content_top - battery_rect.y
        self._battery_gauge_bp.render_at(content_rect, left_offset, draw_background=False,
                                         x_offset=0.0, y_offset=y_shift)
        gauge_top = shared_rect.y
      else:
        # Battery only, no strip — original behavior
        battery_center_y = battery_rect.y + battery_rect.height / 2
        y_shift = dm_center_y - battery_center_y
        self._battery_gauge_bp.render(content_rect, left_offset, y_offset=y_shift)

    # Return offset and whether hybrid gauges were active
    return max(0.0, content_bottom - gauge_top), hybrid_active

  # Arched gauge layout height (approx. arch + text + battery area) for torque bar offset
  _ARCHED_GAUGE_HEIGHT_OFFSET = 220.0

  def _render_gauges_arched(self, content_rect: rl.Rectangle, ball_offset: float, scale: float = 1.0) -> tuple[float, bool]:
    """Render arched-style powerflow and battery gauges (gauge style = arched).
    scale: 0.75 for small (gauge size 1), 1.0 for large (gauge size 2).
    When both are visible, the steering strip spans battery + powerflow with center at screen middle.
    """
    left_offset = int(content_rect.x + ball_offset)
    self._battery_gauge_arched.set_scale(scale)
    self._power_flow_gauge_arched.set_scale(scale)
    self._battery_gauge_arched._update_state()
    self._power_flow_gauge_arched._update_state()
    battery_visible = self._battery_gauge_arched._should_render()
    pf_visible = self._power_flow_gauge_arched._should_render()
    hybrid_active = battery_visible or pf_visible

    strip_drawn_by_view = False
    if battery_visible and pf_visible and ui_state.torque_bar and self._power_flow_gauge_arched._torque_bar is not None:
      geo = self._power_flow_gauge_arched.get_arch_geometry(content_rect)
      # Center fill one powerflow tick left so zero aligns with visual center of battery+powerflow
      fill_center = geo["top_angle"] - POWERFLOW_ANGLE_SPAN * 0.10
      self._torque_bar.render_strip_arched(
        content_rect,
        geo["cx"], geo["cy"], geo["top_angle"],
        BATTERY_START_ANGLE,
        geo["powerflow_end_angle"],
        geo["outer_radius"],
        fill_center_angle=fill_center,
        scale=scale,
      )
      strip_drawn_by_view = True

    if battery_visible:
      self._battery_gauge_arched.set_powerflow_visible(pf_visible)
      self._battery_gauge_arched.render(content_rect, left_offset=left_offset)
    if pf_visible:
      self._power_flow_gauge_arched.render(
        content_rect,
        strip_drawn_by_view=strip_drawn_by_view,
        battery_visible=battery_visible,
      )

    offset = self._ARCHED_GAUGE_HEIGHT_OFFSET * scale if hybrid_active else 0.0
    return offset, hybrid_active

  def _draw_shared_background(self, rect: rl.Rectangle):
    """Draw shared background container (glow + fill). Border drawn separately."""
    glow_exp = SHARED_BG_GLOW_EXPANSION
    glow_rect = rl.Rectangle(
      rect.x - glow_exp, rect.y - glow_exp,
      rect.width + glow_exp * 2, rect.height + glow_exp * 2,
    )
    rl.draw_rectangle_rounded(
      glow_rect, SHARED_BG_ROUNDNESS, 10,
      rl.Color(20, 20, 20, int(SHARED_BG_COLOR.a * 0.3)),
    )
    rl.draw_rectangle_rounded(rect, SHARED_BG_ROUNDNESS, 10, SHARED_BG_COLOR)
