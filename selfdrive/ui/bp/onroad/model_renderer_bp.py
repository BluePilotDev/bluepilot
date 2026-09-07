import numpy as np
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer, LeadVehicle, CLIP_MARGIN, MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE
from openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp import ChevronMetricsBP
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
# BluePilot: Rainbow shader moved to BP module after upstream removal
from openpilot.bluepilot.ui.lib.bp_shaders import draw_rainbow_polygon
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log
from openpilot.selfdrive.ui.bp.lib.longitudinal_visuals import (
  approach_tesla_geometry_alpha,
  advance_tesla_blue_phase,
  legacy_rainbow_cycle_rate,
  longitudinal_control_active,
  rainbow_cycle_rate,
  tesla_geometry_reliable,
  tesla_path_mode,
)
from openpilot.selfdrive.ui.bp.lib.blindspot_visuals import tesla_blindspot_lane_active
from openpilot.selfdrive.ui.bp.lib.lane_change_visuals import tesla_lane_change_lane_active
from openpilot.selfdrive.ui.bp.lib.tesla_palette import palette_for_dark_fraction, tesla_path_gradient_colors
# BluePilot: seasonal theme packs (colors.json overrides for road colors)
from openpilot.selfdrive.ui.bp.lib import theme_pack

# BluePilot: Lane line colors by status (upstream removed LANE_LINE_COLORS dict)
LANE_LINE_COLORS_BP = {
  UIStatus.DISENGAGED: rl.Color(0, 0, 0, 255),
  UIStatus.ENGAGED: rl.Color(0, 255, 80, 255),
  UIStatus.OVERRIDE: rl.Color(145, 155, 149, 255),
}

# BluePilot: Radar/vision lead indicator colors
LEAD_RADAR_GLOW = rl.Color(0, 134, 233, 255)
LEAD_RADAR_CHEVRON_BASE = rl.Color(0, 100, 200, 255)
LEAD_VISION_GLOW = rl.Color(218, 202, 37, 255)
LEAD_VISION_CHEVRON_BASE = rl.Color(201, 34, 49, 255)

# BluePilot: Overlay size scale factors (Small=0, Medium=1, Large=2)
OVERLAY_SCALE_FACTORS = {0: 0.6, 1: 1.0, 2: 1.5}
LANE_LINE_WIDTH = 0.05
MINIMAL_VIEW_LANE_LINE_WIDTH = 0.075
MINIMAL_VIEW_OUTER_LANE_LINE_WIDTH = 0.09
ROAD_EDGE_WIDTH = 0.05
MINIMAL_VIEW_ROAD_EDGE_WIDTH = 0.075
MINIMAL_VIEW_NEUTRAL_LANE_COLOR = rl.Color(185, 205, 225, 255)

# BluePilot: Rad Racer 8-bit theme road styling
# BluePilot: Rad Racer road constants + drawing methods moved to rad_racer_road.py so the
# comma four (MICI) renderer can share them; values unchanged.
from openpilot.selfdrive.ui.bp.onroad.rad_racer_road import (
  RadRacerRoadMixin, RAD_RACER_ROAD_EDGE_WIDTH, RAD_RACER_DASH_LEN_M, RAD_RACER_GAP_LEN_M,
)


def lead_values_indicate_handoff(previous_values: tuple[float, float, float] | None,
                                 current_values: tuple[float, float, float],
                                 previous_track_id: int, current_track_id: int) -> bool:
  """Detect a fused leadOne identity change without reacting to ordinary radar jitter."""
  if previous_values is None:
    return False

  previous_radar = previous_track_id >= 0
  current_radar = current_track_id >= 0
  if previous_radar and current_radar and previous_track_id != current_track_id:
    return True

  d_jump = abs(current_values[0] - previous_values[0])
  y_jump = abs(current_values[1] - previous_values[1])
  v_jump = abs(current_values[2] - previous_values[2])

  # A fused lead cannot physically move this far between 20 Hz radarState
  # updates. Treat any one-axis discontinuity as an unreliable/new target so a
  # visible actor fades away instead of following a noisy vision depth jump.
  return d_jump > 6.0 or y_jump > 0.65 or v_jump > 3.0


class ModelRendererBP(RadRacerRoadMixin, ModelRenderer):
  """BluePilot ModelRenderer with enhanced lane lines, path smoothing, and radar overlay."""

  def __init__(self):
    super().__init__()
    self._bp_params = Params()
    self._rainbow_v = 0.0
    self._tesla_blue_phase = 0.0
    self._tesla_path_visibility = 0.0

    # BluePilot: Replace SP chevron metrics with BP version (horizontal boxed layout)
    self.chevron_metrics = ChevronMetricsBP()

    # Path smoothing: store previous smoothed path for temporal damping
    self._previous_path_projected_points = np.empty((0, 2), dtype=np.float32)
    self._path_smoothing_damping = 0.3

    # BluePilot: Track whether each lead is radar-sourced
    self._lead_is_radar = [False, False]

    # BluePilot: Overlay size scale factor (read from param periodically)
    self._overlay_scale = 1.0

    # BluePilot: Cache params to avoid per-frame disk I/O (refreshed in existing 60-frame block)
    self.ford_overlay_enabled = self._bp_params.get_bool("FordPrefShowRadarLeadOverlay")
    self._disable_lane_line_status_color = self._bp_params.get_bool("BPDisableLaneLineStatusColor")
    self._hide_camera_view = self._bp_params.get_bool("BPHideCameraView")
    self._rainbow_lane_lines = self._bp_params.get_bool("BPRainbowLines")
    self._tesla_style = theme_pack.tesla_active(self._bp_params)
    self._tesla_dark_fraction = ui_state.tesla_dark_fraction
    # BluePilot: Rad Racer 8-bit theme (green game road, dash scroll animation state)
    self._rad_racer = theme_pack.rad_racer_active(self._bp_params)
    self._dash_phase = 0.0
    self._transform_dirty = True
    # BluePilot: seasonal theme pack (None when disabled)
    self._theme_pack = theme_pack.get_active_pack(force=True)

    # BluePilot: Lead position smoothing filters to reduce radar jitter
    dt = 1 / gui_app.target_fps
    self._lead_d_filters = [FirstOrderFilter(0, 0.4, dt, initialized=False),
                            FirstOrderFilter(0, 0.4, dt, initialized=False)]
    self._lead_y_filters = [FirstOrderFilter(0, 0.5, dt, initialized=False),
                            FirstOrderFilter(0, 0.5, dt, initialized=False)]
    self._lead_v_filters = [FirstOrderFilter(0, 0.3, dt, initialized=False),
                            FirstOrderFilter(0, 0.3, dt, initialized=False)]
    self._lead_was_active = [False, False]
    self._lead_track_ids = [-1, -1]
    self._lead_raw_values: list[tuple[float, float, float] | None] = [None, None]
    self._lead_generations = [0, 0]

  def prepare_projection(self, rect: rl.Rectangle) -> None:
    """Set clip region so _map_to_screen works before render().

    Rad Racer draws skyline/signs behind the road and calls _map_to_screen during
    that background pass, before model_renderer.render() initializes the clip rect.
    """
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

  def set_tesla_style(self, enabled: bool, dark_fraction: float = 0.0) -> None:
    if enabled != self._tesla_style:
      self._transform_dirty = True
      self._tesla_path_visibility = 0.0
    if enabled and not self._tesla_style:
      self._lead_was_active = [False, False]
      self._lead_track_ids = [-1, -1]
      self._lead_raw_values = [None, None]
    self._tesla_style = enabled
    self._tesla_dark_fraction = dark_fraction

  def _refresh_bp_params(self) -> None:
    """Refresh cached BluePilot params and invalidate model geometry when visual widths change."""
    hide_camera_view = self._bp_params.get_bool("BPHideCameraView")
    if hide_camera_view != self._hide_camera_view:
      self._transform_dirty = True
    self._hide_camera_view = hide_camera_view

    self.set_tesla_style(theme_pack.tesla_active(self._bp_params), ui_state.tesla_dark_fraction)

    rad_racer = theme_pack.rad_racer_active(self._bp_params)
    if rad_racer != self._rad_racer:
      self._transform_dirty = True
    self._rad_racer = rad_racer

    self.ford_overlay_enabled = self._bp_params.get_bool("FordPrefShowRadarLeadOverlay")
    self._disable_lane_line_status_color = self._bp_params.get_bool("BPDisableLaneLineStatusColor")
    self._rainbow_lane_lines = self._bp_params.get_bool("BPRainbowLines")
    new_pack = theme_pack.get_active_pack()
    if (new_pack is None) != (self._theme_pack is None):
      self._transform_dirty = True  # themed lane widths differ; re-project on toggle
    self._theme_pack = new_pack

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm
    self._tesla_dark_fraction = ui_state.tesla_dark_fraction

    if self._tesla_style:
      self._rainbow_v = rainbow_cycle_rate(sm)
      self._tesla_blue_phase = advance_tesla_blue_phase(
        self._tesla_blue_phase, self._rainbow_v, gui_app.target_fps,
      )
    elif ui_state.rainbow_path or self._rainbow_lane_lines:
      self._rainbow_v = legacy_rainbow_cycle_rate(sm)

    if (sm.recv_frame["liveCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      calib_frame = sm.recv_frame["liveCalibration"]
      model_frame = sm.recv_frame["modelV2"]
      reason = f"stale calib={calib_frame} model={model_frame} started={ui_state.started_frame}"
      bp_ui_log.visibility("ModelRenderer", False, reason=reason)
      return

    bp_ui_log.state("ModelRenderer", "render_active", True)

    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    self._experimental_mode = sm['selfdriveState'].experimentalMode

    live_calib = sm['liveCalibration']
    from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
    self._path_offset_z = live_calib.height[0] if live_calib.height else HEIGHT_INIT[0]

    if self._counter % 60 == 0:
      self._camera_offset = ui_state.params.get("CameraOffset", return_default=True) if ui_state.active_bundle else 0.0
      try:
        size_val = int(self._bp_params.get("FordPrefRadarOverlaySize", return_default=True))
      except (TypeError, ValueError):
        size_val = 1
      self._overlay_scale = OVERLAY_SCALE_FACTORS.get(size_val, 1.0)
      # BluePilot: Refresh cached params (avoids per-frame disk I/O)
      self._refresh_bp_params()
    self._counter += 1

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.alive['radarState'] and sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None
    if radar_state is None:
      self._lead_was_active = [False, False]
      self._lead_track_ids = [-1, -1]
      self._lead_raw_values = [None, None]
      self._lead_vehicles = [LeadVehicle(), LeadVehicle()]

    # BluePilot: Ford radar overlay and Tesla's single fused actor both reuse
    # the existing filtered lead geometry even without openpilot longitudinal.
    prepare_lead_visuals = (
      self._longitudinal_control or self.ford_overlay_enabled or self._tesla_style
    ) and radar_state is not None
    bp_ui_log.state("ModelRenderer", "render_lead", prepare_lead_visuals)

    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      if prepare_lead_visuals:
        self._update_leads(radar_state, path_x_array)
        # BluePilot: Track radar vs vision status for lead coloring
        self._update_lead_radar_status(radar_state)

      self._transform_dirty = False

    # BluePilot: Advance dash scroll animation for the Rad Racer road
    if self._rad_racer:
      v_ego = sm['carState'].vEgo if sm.valid['carState'] else 0.0
      period = RAD_RACER_DASH_LEN_M + RAD_RACER_GAP_LEN_M
      self._dash_phase = (self._dash_phase + max(0.0, v_ego) / gui_app.target_fps) % period

    self._draw_lane_lines()
    self._draw_path(sm)

    # BluePilot: In Rad Racer theme, leads are drawn as sprites by RadRacerTheme
    if prepare_lead_visuals and radar_state and not self._rad_racer and not self._tesla_style:
      self._draw_lead_indicator()
      # BluePilot: Pass radar/overlay state to BP chevron metrics for boxed layout
      self.chevron_metrics.ford_overlay_enabled = self.ford_overlay_enabled
      self.chevron_metrics.lead_is_radar = self._lead_is_radar
      self.chevron_metrics.overlay_scale = self._overlay_scale
      self.chevron_metrics.draw_lead_status(sm, radar_state, self._rect, self._lead_vehicles)

  def smoothed_primary_lead(self) -> tuple[float, float, float] | None:
    """Return BluePilot's filtered leadOne state while that fused lead is active."""
    if not self._lead_was_active[0]:
      return None
    return (
      float(self._lead_d_filters[0].x),
      float(self._lead_y_filters[0].x),
      float(self._lead_v_filters[0].x),
    )

  def primary_lead_generation(self) -> int:
    """Return a token that changes whenever leadOne becomes a different vehicle."""
    return self._lead_generations[0]

  def _update_lead_radar_status(self, radar_state):
    """Track whether each lead is radar-sourced for coloring."""
    leads = [radar_state.leadOne, radar_state.leadTwo]
    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.status:
        self._lead_is_radar[i] = getattr(lead_data, 'radar', False)
      else:
        self._lead_is_radar[i] = False

  def _update_leads(self, radar_state, path_x_array):
    """Update lead positions with temporal smoothing to reduce radar jitter."""
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    leads = [radar_state.leadOne, radar_state.leadTwo]

    dt = 1 / gui_app.target_fps
    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.status:
        values = np.asarray((lead_data.dRel, lead_data.yRel, lead_data.vRel), dtype=np.float64)
        if not np.all(np.isfinite(values)) or values[0] <= 0.0:
          self._lead_was_active[i] = False
          self._lead_track_ids[i] = -1
          self._lead_raw_values[i] = None
          continue

        track_id = int(getattr(lead_data, 'radarTrackId', -1))
        current_values = (float(values[0]), float(values[1]), float(values[2]))
        lead_changed = lead_values_indicate_handoff(
          self._lead_raw_values[i], current_values, self._lead_track_ids[i], track_id,
        )
        # Reset filters when a lead first appears so we don't lerp from stale data
        if not self._lead_was_active[i] or lead_changed:
          self._lead_d_filters[i] = FirstOrderFilter(lead_data.dRel, 0.4, dt)
          self._lead_y_filters[i] = FirstOrderFilter(lead_data.yRel, 0.5, dt)
          self._lead_v_filters[i] = FirstOrderFilter(lead_data.vRel, 0.3, dt)
          self._lead_generations[i] += 1

        # Smooth radarState's fused lead values.
        d_rel = self._lead_d_filters[i].update(lead_data.dRel)
        y_rel = self._lead_y_filters[i].update(lead_data.yRel)
        v_rel = self._lead_v_filters[i].update(lead_data.vRel)

        idx = self._get_path_length_idx(path_x_array, d_rel)
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel + self._camera_offset, z + self._path_offset_z)
        if point:
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, point, self._rect)

        self._lead_was_active[i] = True
        self._lead_track_ids[i] = track_id
        self._lead_raw_values[i] = current_values
      else:
        self._lead_was_active[i] = False
        self._lead_track_ids[i] = -1
        self._lead_raw_values[i] = None

  def _update_lead_vehicle(self, d_rel, v_rel, point, rect):
    """Override to apply overlay size scale factor to chevron geometry."""
    speed_buff, lead_buff = 10.0, 40.0

    fill_alpha = 0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    # Apply overlay scale factor to chevron size
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35 * self._overlay_scale
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha))

  def _update_model(self, lead, path_x_array):
    """Update model with doubled lane line width and path smoothing."""
    super()._update_model(lead, path_x_array)

    # BluePilot: Redo lane lines and road edges with enhanced widths.
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    for i, lane_line in enumerate(self._lane_lines):
      is_current_lane = (i == 1 or i == 2)
      lane_line_width = LANE_LINE_WIDTH
      if self._hide_camera_view or self._rad_racer or self._theme_pack is not None or self._tesla_style:
        lane_line_width = MINIMAL_VIEW_LANE_LINE_WIDTH if is_current_lane else MINIMAL_VIEW_OUTER_LANE_LINE_WIDTH
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, lane_line_width * self._lane_line_probs[i], 0.0, max_idx, max_distance
      )

    # BluePilot: Rad Racer solid outer lines are thicker than minimal view
    if self._rad_racer:
      road_edge_width = RAD_RACER_ROAD_EDGE_WIDTH
    elif self._hide_camera_view or self._theme_pack is not None or self._tesla_style:
      road_edge_width = MINIMAL_VIEW_ROAD_EDGE_WIDTH
    else:
      road_edge_width = ROAD_EDGE_WIDTH
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, road_edge_width, 0.0, max_idx, max_distance)

    self._apply_smooth_path()

  # Pre-computed Gaussian kernel weights for spatial smoothing (exp(-0.5 * j^2) for j=-2..2)
  _GAUSS_KERNEL = np.array([np.exp(-0.5 * j * j) for j in range(-2, 3)], dtype=np.float32)

  def _apply_smooth_path(self):
    """Apply Gaussian-weighted spatial smoothing with temporal damping to reduce path sway."""
    if self._path.projected_points.size == 0:
      return

    n = len(self._path.projected_points)
    if n < 4:
      self._previous_path_projected_points = self._path.projected_points.copy()
      return

    smoothed = self._path.projected_points.copy()

    # Vectorized Gaussian smoothing of y-coordinates for interior points
    y = self._path.projected_points[:, 1]
    kernel = self._GAUSS_KERNEL
    # np.convolve with 'same' mode applies the kernel across all points; we only use interior results
    conv = np.convolve(y, kernel, mode='same')
    # Weight sums vary at edges; compute them the same way
    ones = np.ones(n, dtype=np.float32)
    weight_sums = np.convolve(ones, kernel, mode='same')
    # Apply only to interior points (indices 2..n-2)
    smoothed[2:n-1, 1] = conv[2:n-1] / weight_sums[2:n-1]

    # Vectorized temporal damping
    if (self._previous_path_projected_points.size > 0 and
        len(self._previous_path_projected_points) == n):
      damping = self._path_smoothing_damping
      smoothed[:, 1] = self._previous_path_projected_points[:, 1] + \
        (smoothed[:, 1] - self._previous_path_projected_points[:, 1]) * (1.0 - damping)

    self._previous_path_projected_points = smoothed.copy()
    self._path.projected_points = smoothed

  def _draw_lane_lines(self):
    """Draw lane lines with enhanced rendering and glow effects."""
    if self._tesla_style:
      self._draw_tesla_lane_lines()
      return
    # BluePilot: Rad Racer theme replaces all road rendering with green game lines
    if self._rad_racer:
      self._draw_rad_racer_road()
      return
    self._draw_enhanced_lane_lines()

  def _draw_tesla_lane_lines(self):
    """Draw quiet, confidence-weighted geometry for the gray environment view."""
    palette = palette_for_dark_fraction(self._tesla_dark_fraction)
    for i, lane_line in enumerate(self._lane_lines):
      blindspot_active = tesla_blindspot_lane_active(ui_state.sm, i)
      lane_change_active = tesla_lane_change_lane_active(ui_state.sm, i)
      if (lane_line.projected_points.size == 0 or
          (not blindspot_active and not lane_change_active and self._lane_line_probs[i] < 0.25)):
        continue
      if blindspot_active:
        glow = self._expand_polygon(lane_line.projected_points, 4.0)
        if glow.size:
          draw_polygon(self._rect, glow, rl.Color(palette.blindspot.r, palette.blindspot.g, palette.blindspot.b, 70))
        draw_polygon(self._rect, lane_line.projected_points, palette.blindspot)
      elif lane_change_active:
        glow = self._expand_polygon(lane_line.projected_points, 7.0)
        if glow.size:
          draw_polygon(
            self._rect, glow,
            rl.Color(palette.lane_change.r, palette.lane_change.g, palette.lane_change.b, 58),
          )
        draw_polygon(self._rect, lane_line.projected_points, palette.lane_change)
      else:
        is_current_lane = i in (1, 2)
        base = palette.lane_inner if is_current_lane else palette.lane_outer
        confidence = float(np.clip(self._lane_line_probs[i], 0.20, 1.0))
        draw_polygon(self._rect, lane_line.projected_points,
                     rl.Color(base.r, base.g, base.b, int(base.a * confidence)))

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue
      confidence = float(np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0))
      if confidence < 0.18:
        continue
      draw_polygon(self._rect, road_edge.projected_points,
                   rl.Color(palette.road_edge.r, palette.road_edge.g,
                            palette.road_edge.b, int(palette.road_edge.a * confidence)))

  # BluePilot: _rad_racer_road_color / _draw_rad_racer_road / _draw_curbed_line /
  # _draw_road_ribbon_quad / _draw_dashed_line moved verbatim to RadRacerRoadMixin
  # (rad_racer_road.py) so the comma four renderer can share them.

  def _get_ll_color(self, prob: float, is_current_lane: bool) -> rl.Color:
    """Get lane line color based on UI status with confidence-based brightness.

    Current lanes use status color (green when engaged, gray on override, black when disengaged).
    Outer lanes use white. All lanes go black when disengaged.
    When BPDisableLaneLineStatusColor is enabled, current lanes use white instead of status color.
    """
    if self._hide_camera_view and ui_state.status != UIStatus.ENGAGED:
      return MINIMAL_VIEW_NEUTRAL_LANE_COLOR

    if ui_state.status == UIStatus.DISENGAGED:
      return rl.Color(0, 0, 0, 255)

    # BluePilot: theme pack lane color replaces status/white coloring (disengaged stays black)
    if self._theme_pack is not None:
      pack_color = self._theme_pack.rl_colors().get("LaneLines")
      if pack_color is not None:
        return pack_color

    if not is_current_lane or self._disable_lane_line_status_color:
      return rl.Color(255, 255, 255, 255)

    base = LANE_LINE_COLORS_BP.get(ui_state.status, LANE_LINE_COLORS_BP[UIStatus.DISENGAGED])
    brightness = np.interp(prob, [0.0, 0.5, 1.0], [0.4, 0.7, 1.0])
    return rl.Color(int(base.r * brightness), int(base.g * brightness), int(base.b * brightness), 255)

  def _draw_enhanced_lane_lines(self):
    """Draw enhanced lane lines with glow effects and confidence-based brightness."""
    rainbow_lane_lines_active = self._rainbow_lane_lines_active(ui_state.sm)
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0 or self._lane_line_probs[i] < 0.4:
        continue

      base_alpha = np.clip(self._lane_line_probs[i] * 0.8, 0.3, 0.8)
      is_current_lane = (i == 1 or i == 2)
      if not is_current_lane:
        base_alpha *= 0.4

      if rainbow_lane_lines_active and is_current_lane:
        draw_rainbow_polygon(self._rect, lane_line.projected_points, rainbow_v=self._rainbow_v, alpha=float(base_alpha * 0.75))
      else:
        base_color = self._get_ll_color(float(self._lane_line_probs[i]), is_current_lane)
        scaled_alpha = int(base_alpha * 255)
        color = rl.Color(base_color.r, base_color.g, base_color.b, scaled_alpha)
        draw_polygon(self._rect, lane_line.projected_points, color)

    self._draw_lane_glow_effects()

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue
      edge_alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0) * 0.6
      edge_base = self._road_edge_base_color()
      color = rl.Color(edge_base.r, edge_base.g, edge_base.b, int(edge_alpha * 255))
      draw_polygon(self._rect, road_edge.projected_points, color)

    self._draw_road_edge_glow_effects()

  def _draw_lane_glow_effects(self):
    """Draw single glow layer around lane lines with confidence-based brightness.

    Reduced from 3 layers to 1 for performance (saves ~8 draw_polygon + _expand_polygon calls/frame).
    """
    rainbow_lane_lines_active = self._rainbow_lane_lines_active(ui_state.sm)
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0 or self._lane_line_probs[i] < 0.4:
        continue
      base_alpha = np.clip(self._lane_line_probs[i] * 0.8, 0.3, 0.8)
      is_current_lane = (i == 1 or i == 2)
      if rainbow_lane_lines_active and is_current_lane:
        continue
      if not is_current_lane:
        base_alpha *= 0.4
      base_color = self._get_ll_color(float(self._lane_line_probs[i]), is_current_lane)
      expanded_points = self._expand_polygon(lane_line.projected_points, 12.0)
      if expanded_points.size > 0:
        # Themed lanes get a stronger halo so the pack color reads as glow, not just a line
        glow_mult = 0.22 if self._theme_pack is not None else 0.12
        alpha = int(base_alpha * glow_mult * 255)
        color = rl.Color(base_color.r, base_color.g, base_color.b, alpha)
        draw_polygon(self._rect, expanded_points, color)

  def _road_edge_base_color(self) -> rl.Color:
    """Road edge color: theme pack RoadEdges when set, stock red otherwise."""
    if self._theme_pack is not None:
      pack_color = self._theme_pack.rl_colors().get("RoadEdges")
      if pack_color is not None:
        return pack_color
    return rl.Color(255, 0, 0, 255)

  def _rainbow_lane_lines_active(self, sm) -> bool:
    """Return true when inner lane lines should use the rainbow shader."""
    if not self._rainbow_lane_lines or self._disable_lane_line_status_color:
      return False

    return longitudinal_control_active(sm, ui_state.status)

  def _draw_road_edge_glow_effects(self):
    """Draw single glow layer around road edges.

    Reduced from 3 layers to 1 for performance (saves ~4 draw_polygon + _expand_polygon calls/frame).
    """
    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue
      edge_alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0)
      if edge_alpha < 0.3:
        continue
      expanded_points = self._expand_polygon(road_edge.projected_points, 18.0)
      if expanded_points.size > 0:
        glow_mult = 0.16 if self._theme_pack is not None else 0.08
        alpha = int(edge_alpha * glow_mult * 255)
        edge_base = self._road_edge_base_color()
        color = rl.Color(edge_base.r, edge_base.g, edge_base.b, alpha)
        draw_polygon(self._rect, expanded_points, color)

  def _expand_polygon(self, points: np.ndarray, width: float) -> np.ndarray:
    """Expand ribbon polygon outward for glow effect, tapering at ends.

    The polygon is a ribbon in [L0..Lk-1, Rk-1..R0] order. The expansion
    width is scaled proportionally to the local ribbon width so the glow
    tapers naturally where the line narrows (near the horizon and at the
    bottom of screen) instead of creating blobs at the ends.
    """
    if points.size == 0 or len(points) < 4:
      return np.empty((0, 2), dtype=np.float32)

    n = len(points)
    half = n // 2

    # Vectorized local ribbon widths
    left = points[:half]
    right = points[n - 1:n - 1 - half:-1]  # reversed right side
    local_widths = np.linalg.norm(right - left, axis=1)

    max_width = np.max(local_widths)
    if max_width < 1e-6:
      return np.empty((0, 2), dtype=np.float32)

    # Per-vertex scale: proportional to local ribbon width
    scales = np.empty(n, dtype=np.float32)
    s = local_widths / max_width
    scales[:half] = s
    scales[n - 1:n - 1 - half:-1] = s

    # Vectorized edge and normal computation
    prev_idx = np.arange(n) - 1
    prev_idx[0] = n - 1
    next_idx = np.arange(n) + 1
    next_idx[-1] = 0

    edge1 = points - points[prev_idx]  # (n, 2)
    edge2 = points[next_idx] - points  # (n, 2)

    len1 = np.linalg.norm(edge1, axis=1, keepdims=True)
    len2 = np.linalg.norm(edge2, axis=1, keepdims=True)
    len1 = np.where(len1 < 1e-6, 1.0, len1)
    len2 = np.where(len2 < 1e-6, 1.0, len2)
    edge1 = edge1 / len1
    edge2 = edge2 / len2

    # Normals: rotate edges 90 degrees ([-y, x])
    normal1 = np.stack([-edge1[:, 1], edge1[:, 0]], axis=1)
    normal2 = np.stack([-edge2[:, 1], edge2[:, 0]], axis=1)
    normal = (normal1 + normal2) * 0.5
    normal_len = np.linalg.norm(normal, axis=1, keepdims=True)
    normal_len = np.where(normal_len < 1e-6, 1.0, normal_len)
    normal = normal / normal_len

    return (points + normal * (width * scales)[:, None]).astype(np.float32)

  def _draw_path(self, sm):
    """Draw path with status-colored edges."""
    if self._tesla_style:
      palette = palette_for_dark_fraction(self._tesla_dark_fraction)
      if not self._path.projected_points.size:
        return
      self._tesla_path_visibility = approach_tesla_geometry_alpha(
        self._tesla_path_visibility,
        tesla_geometry_reliable(self._path.raw_points, sm),
        gui_app.target_fps,
      )
      if self._tesla_path_visibility <= 0.01:
        return
      mode = tesla_path_mode(
        ui_state.rainbow_path,
        longitudinal_control_active(sm, ui_state.status),
      )
      if mode == "rainbow":
        draw_rainbow_polygon(
          self._rect,
          self._path.projected_points,
          rainbow_v=self._rainbow_v,
          alpha=0.6 * self._tesla_path_visibility,
        )
      else:
        phase = self._tesla_blue_phase if mode == "blue_cycle" else None
        colors = tesla_path_gradient_colors(palette, phase, self._tesla_path_visibility)
        draw_polygon(
          self._rect,
          self._path.projected_points,
          gradient=Gradient(
            start=(0.0, 1.0),
            end=(0.0, 0.0),
            colors=colors,
            stops=[0.0, 0.32, 0.68, 1.0] if phase is not None else [0.0, 0.55, 1.0],
          ),
        )
      return
    # BluePilot: Rad Racer theme draws the path as a dashed center line in _draw_rad_racer_road
    if self._rad_racer:
      return

    if ui_state.rainbow_path:
      draw_rainbow_polygon(self._rect, self._path.projected_points, rainbow_v=self._rainbow_v)
    elif (themed_gradient := self._themed_path_gradient()) is not None:
      # BluePilot: theme pack path colors replace the throttle/no-throttle gradient
      draw_polygon(self._rect, self._path.projected_points, gradient=themed_gradient)
    else:
      super()._draw_path(sm)

    self._draw_path_edges()

  def _themed_path_gradient(self) -> Gradient | None:
    """Bottom-to-top path gradient from the active theme pack, or None.

    PathEdge tints the near end, Path carries the ribbon and fades out at the horizon —
    same alpha profile as the stock throttle gradient so depth perception is unchanged.
    """
    if self._theme_pack is None:
      return None
    colors = self._theme_pack.rl_colors()
    path = colors.get("Path")
    if path is None:
      return None
    edge = colors.get("PathEdge", path)
    return Gradient(
      start=(0.0, 1.0),
      end=(0.0, 0.0),
      colors=[rl.Color(edge.r, edge.g, edge.b, 190), rl.Color(path.r, path.g, path.b, 150), rl.Color(path.r, path.g, path.b, 30)],
      stops=[0.0, 0.5, 1.0],
    )

  def _draw_path_edges(self):
    """Draw path edges (left, right, and front) with status-based colors.

    Uses rl.draw_line_strip for batched line rendering instead of individual draw_line_ex calls.
    """
    if not self._path.projected_points.size:
      return

    points = self._path.projected_points
    num_points = len(points)
    mid_point = num_points // 2
    if mid_point < 2:
      return

    left_edge = points[:mid_point]
    right_edge = points[mid_point:][::-1]

    pack_edge = self._theme_pack.rl_colors().get("PathEdge") if self._theme_pack is not None else None
    if pack_edge is not None and ui_state.status != UIStatus.DISENGAGED:
      # BluePilot: theme pack path edge color (disengaged keeps the status blackout)
      edge_color = pack_edge
    elif self._disable_lane_line_status_color and ui_state.status != UIStatus.DISENGAGED:
      edge_color = rl.Color(255, 255, 255, 255)
    else:
      edge_color = LANE_LINE_COLORS_BP.get(ui_state.status, LANE_LINE_COLORS_BP[UIStatus.DISENGAGED])

    # Convert each polyline edge into a thin ribbon polygon (2 draw_polygon calls vs ~65 draw_line_ex).
    # This preserves the ~4px visual thickness while batching into 2 GPU calls.
    half_w = 2.0  # Half of the 4px line thickness
    for edge_pts in (left_edge, right_edge):
      if len(edge_pts) < 2:
        continue
      # Offset each point perpendicular to its segment to form a ribbon
      ribbon = self._line_to_ribbon(edge_pts, half_w)
      if ribbon.size > 0:
        draw_polygon(self._rect, ribbon, edge_color)

    # Front connecting line (single call)
    if len(left_edge) > 0 and len(right_edge) > 0:
      rl.draw_line_ex(
        rl.Vector2(float(left_edge[-1][0]), float(left_edge[-1][1])),
        rl.Vector2(float(right_edge[-1][0]), float(right_edge[-1][1])),
        4.0, edge_color
      )

  @staticmethod
  def _line_to_ribbon(pts: np.ndarray, half_w: float) -> np.ndarray:
    """Convert a polyline into a ribbon polygon of given half-width.

    Returns points in [L0..Ln, Rn..R0] order suitable for draw_polygon.
    """
    n = len(pts)
    if n < 2:
      return np.empty((0, 2), dtype=np.float32)
    # Compute segment directions
    diffs = np.diff(pts, axis=0)  # (n-1, 2)
    lengths = np.linalg.norm(diffs, axis=1, keepdims=True)
    lengths = np.where(lengths < 1e-6, 1.0, lengths)
    dirs = diffs / lengths
    # Per-vertex normals: average of adjacent segment normals
    normals = np.empty((n, 2), dtype=np.float32)
    seg_normals = np.stack([-dirs[:, 1], dirs[:, 0]], axis=1)  # rotate 90°
    normals[0] = seg_normals[0]
    normals[-1] = seg_normals[-1]
    normals[1:-1] = (seg_normals[:-1] + seg_normals[1:]) * 0.5
    nlen = np.linalg.norm(normals, axis=1, keepdims=True)
    nlen = np.where(nlen < 1e-6, 1.0, nlen)
    normals = normals / nlen
    # Build ribbon: left side then right side reversed
    left = pts + normals * half_w
    right = pts - normals * half_w
    return np.vstack([left, right[::-1]]).astype(np.float32)

  def _draw_lead_indicator(self):
    """Draw lead vehicles with dynamic colors based on detection source (radar vs vision)."""
    if self.ford_overlay_enabled:
      #Chevron is handled by the Metric renderer.
      return

    for i, lead in enumerate(self._lead_vehicles):
      if not lead.glow or not lead.chevron:
        continue

      is_radar = self._lead_is_radar[i] if i < len(self._lead_is_radar) else False

      # BluePilot: Blue for radar leads, yellow/red for vision leads
      if is_radar:
        glow_color = LEAD_RADAR_GLOW
        chevron_color = rl.Color(LEAD_RADAR_CHEVRON_BASE.r, LEAD_RADAR_CHEVRON_BASE.g,
                                 LEAD_RADAR_CHEVRON_BASE.b, lead.fill_alpha)
      else:
        glow_color = LEAD_VISION_GLOW
        # BluePilot: theme pack lead marker color for vision leads (radar leads stay blue)
        vision_base = LEAD_VISION_CHEVRON_BASE
        if self._theme_pack is not None:
          vision_base = self._theme_pack.rl_colors().get("LeadMarker", vision_base)
        chevron_color = rl.Color(vision_base.r, vision_base.g, vision_base.b, lead.fill_alpha)

      rl.draw_triangle_fan(lead.glow, len(lead.glow), glow_color)
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), chevron_color)
