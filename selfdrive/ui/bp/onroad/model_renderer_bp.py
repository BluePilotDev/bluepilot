import numpy as np
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer, LeadVehicle, CLIP_MARGIN, MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE
from openpilot.selfdrive.ui.bp.onroad.chevron_metrics_bp import ChevronMetricsBP
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

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


class ModelRendererBP(ModelRenderer):
  """BluePilot ModelRenderer with enhanced lane lines, path smoothing, and radar overlay."""

  def __init__(self):
    super().__init__()
    self._bp_params = Params()

    # BluePilot: Replace SP chevron metrics with BP version (horizontal boxed layout)
    self.chevron_metrics = ChevronMetricsBP()

    # Path smoothing: store previous smoothed path for temporal damping
    self._previous_path_projected_points = np.empty((0, 2), dtype=np.float32)
    self._path_smoothing_damping = 0.3

    # BluePilot: Track whether each lead is radar-sourced
    self._lead_is_radar = [False, False]

    # BluePilot: Overlay size scale factor (read from param periodically)
    self._overlay_scale = 1.0

    # BluePilot: Lead position smoothing filters to reduce radar jitter
    dt = 1 / gui_app.target_fps
    self._lead_d_filters = [FirstOrderFilter(0, 0.4, dt, initialized=False),
                            FirstOrderFilter(0, 0.4, dt, initialized=False)]
    self._lead_y_filters = [FirstOrderFilter(0, 0.5, dt, initialized=False),
                            FirstOrderFilter(0, 0.5, dt, initialized=False)]
    self._lead_v_filters = [FirstOrderFilter(0, 0.3, dt, initialized=False),
                            FirstOrderFilter(0, 0.3, dt, initialized=False)]
    self._lead_was_active = [False, False]

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

    if ui_state.rainbow_path:
      self._rainbow_v = np.clip(sm['carState'].vEgo, 2.5, 35) / 30

    if (sm.recv_frame["liveCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      bp_ui_log.visibility("ModelRenderer", False, reason=f"stale calib={sm.recv_frame['liveCalibration']} model={sm.recv_frame['modelV2']} started={ui_state.started_frame}")
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
    self._counter += 1

    if sm.updated['carParams']:
      self._longitudinal_control = sm['carParams'].openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None

    # BluePilot: Ford radar overlay feature - show leads even without longitudinal control
    self.ford_overlay_enabled = self._bp_params.get_bool("FordPrefShowRadarLeadOverlay")
    render_lead_indicator = (self._longitudinal_control or self.ford_overlay_enabled) and radar_state is not None
    bp_ui_log.state("ModelRenderer", "render_lead", render_lead_indicator)

    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      if render_lead_indicator:
        self._update_leads(radar_state, path_x_array)
        # BluePilot: Track radar vs vision status for lead coloring
        self._update_lead_radar_status(radar_state)

      self._transform_dirty = False

    self._draw_lane_lines()
    self._draw_path(sm)

    if render_lead_indicator and radar_state:
      self._draw_lead_indicator()
      # BluePilot: Pass radar/overlay state to BP chevron metrics for boxed layout
      self.chevron_metrics.ford_overlay_enabled = self.ford_overlay_enabled
      self.chevron_metrics.lead_is_radar = self._lead_is_radar
      self.chevron_metrics.overlay_scale = self._overlay_scale
      self.chevron_metrics.draw_lead_status(sm, radar_state, self._rect, self._lead_vehicles)

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
        # Reset filters when a lead first appears so we don't lerp from stale data
        if not self._lead_was_active[i]:
          self._lead_d_filters[i] = FirstOrderFilter(lead_data.dRel, 0.4, dt)
          self._lead_y_filters[i] = FirstOrderFilter(lead_data.yRel, 0.5, dt)
          self._lead_v_filters[i] = FirstOrderFilter(lead_data.vRel, 0.3, dt)

        # Smooth the raw radar values
        d_rel = self._lead_d_filters[i].update(lead_data.dRel)
        y_rel = self._lead_y_filters[i].update(lead_data.yRel)
        v_rel = self._lead_v_filters[i].update(lead_data.vRel)

        idx = self._get_path_length_idx(path_x_array, d_rel)
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel + self._camera_offset, z + self._path_offset_z)
        if point:
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, point, self._rect)

        self._lead_was_active[i] = True
      else:
        self._lead_was_active[i] = False

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

    # BluePilot: Redo lane lines and road edges with doubled width (0.05 vs upstream 0.025)
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    for i, lane_line in enumerate(self._lane_lines):
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, 0.05 * self._lane_line_probs[i], 0.0, max_idx, max_distance
      )

    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, 0.05, 0.0, max_idx, max_distance)

    self._apply_smooth_path()

  def _apply_smooth_path(self):
    """Apply Gaussian-weighted spatial smoothing with temporal damping to reduce path sway."""
    if self._path.projected_points.size == 0:
      return

    if len(self._path.projected_points) < 4:
      self._previous_path_projected_points = self._path.projected_points.copy()
      return

    n = len(self._path.projected_points)
    smoothed = np.zeros_like(self._path.projected_points)

    for i in range(n):
      pt = self._path.projected_points[i].copy()
      if 1 < i < n - 1:
        y_smooth = 0.0
        weight_sum = 0.0
        for j in range(-2, 3):
          idx = i + j
          if 0 <= idx < n:
            weight = np.exp(-0.5 * j * j)
            y_smooth += self._path.projected_points[idx][1] * weight
            weight_sum += weight
        if weight_sum > 0:
          pt[1] = y_smooth / weight_sum
      smoothed[i] = pt

    if (self._previous_path_projected_points.size > 0 and
        len(self._previous_path_projected_points) == len(smoothed)):
      damping = self._path_smoothing_damping
      for i in range(len(smoothed)):
        y_diff = smoothed[i][1] - self._previous_path_projected_points[i][1]
        smoothed[i][1] = self._previous_path_projected_points[i][1] + y_diff * (1.0 - damping)
    else:
      self._previous_path_projected_points = smoothed.copy()

    self._previous_path_projected_points = smoothed.copy()
    self._path.projected_points = smoothed

  def _draw_lane_lines(self):
    """Draw lane lines with enhanced rendering and glow effects."""
    self._draw_enhanced_lane_lines()

  def _get_ll_color(self, prob: float, is_current_lane: bool) -> rl.Color:
    """Get lane line color based on UI status with confidence-based brightness.

    Current lanes use status color (green when engaged, gray on override, black when disengaged).
    Outer lanes use white. All lanes go black when disengaged.
    """
    if ui_state.status == UIStatus.DISENGAGED:
      return rl.Color(0, 0, 0, 255)

    if not is_current_lane:
      return rl.Color(255, 255, 255, 255)

    base = LANE_LINE_COLORS_BP.get(ui_state.status, LANE_LINE_COLORS_BP[UIStatus.DISENGAGED])
    brightness = np.interp(prob, [0.0, 0.5, 1.0], [0.4, 0.7, 1.0])
    return rl.Color(int(base.r * brightness), int(base.g * brightness), int(base.b * brightness), 255)

  def _draw_enhanced_lane_lines(self):
    """Draw enhanced lane lines with glow effects and confidence-based brightness."""
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0 or self._lane_line_probs[i] < 0.4:
        continue

      base_alpha = np.clip(self._lane_line_probs[i] * 0.8, 0.3, 0.8)
      is_current_lane = (i == 1 or i == 2)
      if not is_current_lane:
        base_alpha *= 0.4

      base_color = self._get_ll_color(float(self._lane_line_probs[i]), is_current_lane)
      scaled_alpha = int(base_alpha * 255)
      color = rl.Color(base_color.r, base_color.g, base_color.b, scaled_alpha)
      draw_polygon(self._rect, lane_line.projected_points, color)

    self._draw_lane_glow_effects()

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue
      edge_alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0) * 0.6
      color = rl.Color(255, 0, 0, int(edge_alpha * 255))
      draw_polygon(self._rect, road_edge.projected_points, color)

    self._draw_road_edge_glow_effects()

  def _draw_lane_glow_effects(self):
    """Draw glow effects around lane lines with confidence-based brightness."""
    glow_widths = [20.0, 12.0, 6.0]
    glow_alphas = [0.05, 0.10, 0.20]

    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0 or self._lane_line_probs[i] < 0.4:
        continue
      base_alpha = np.clip(self._lane_line_probs[i] * 0.8, 0.3, 0.8)
      is_current_lane = (i == 1 or i == 2)
      if not is_current_lane:
        base_alpha *= 0.4
      base_color = self._get_ll_color(float(self._lane_line_probs[i]), is_current_lane)
      for glow_width, glow_alpha in zip(glow_widths, glow_alphas):
        expanded_points = self._expand_polygon(lane_line.projected_points, glow_width)
        if expanded_points.size > 0:
          alpha = int(base_alpha * glow_alpha * 255)
          color = rl.Color(base_color.r, base_color.g, base_color.b, alpha)
          draw_polygon(self._rect, expanded_points, color)

  def _draw_road_edge_glow_effects(self):
    """Draw glow effects around road edges."""
    glow_widths = [28.0, 18.0, 10.0]
    glow_alphas = [0.03, 0.07, 0.15]

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue
      edge_alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0)
      if edge_alpha < 0.3:
        continue
      for glow_width, glow_alpha in zip(glow_widths, glow_alphas):
        expanded_points = self._expand_polygon(road_edge.projected_points, glow_width)
        if expanded_points.size > 0:
          alpha = int(edge_alpha * glow_alpha * 255)
          color = rl.Color(255, 0, 0, alpha)
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

    # Compute local ribbon width at each paired left/right vertex
    local_widths = np.empty(half, dtype=np.float32)
    for i in range(half):
      local_widths[i] = np.linalg.norm(points[n - 1 - i] - points[i])

    max_width = np.max(local_widths)
    if max_width < 1e-6:
      return np.empty((0, 2), dtype=np.float32)

    # Per-vertex scale: proportional to local ribbon width
    scales = np.empty(n, dtype=np.float32)
    for i in range(half):
      s = local_widths[i] / max_width
      scales[i] = s          # left-side vertex
      scales[n - 1 - i] = s  # corresponding right-side vertex

    expanded = []
    for i in range(n):
      prev_idx = (i - 1) % n
      next_idx = (i + 1) % n
      p_prev = points[prev_idx]
      p_curr = points[i]
      p_next = points[next_idx]
      edge1 = p_curr - p_prev
      edge2 = p_next - p_curr
      len1 = np.linalg.norm(edge1)
      len2 = np.linalg.norm(edge2)
      if len1 > 1e-6:
        edge1 = edge1 / len1
      if len2 > 1e-6:
        edge2 = edge2 / len2
      normal1 = np.array([-edge1[1], edge1[0]])
      normal2 = np.array([-edge2[1], edge2[0]])
      normal = (normal1 + normal2) / 2.0
      normal_len = np.linalg.norm(normal)
      if normal_len > 1e-6:
        normal = normal / normal_len
      expanded.append(p_curr + normal * width * scales[i])

    return np.array(expanded, dtype=np.float32)

  def _draw_path(self, sm):
    """Draw path with status-colored edges."""

    if ui_state.rainbow_path:
      draw_polygon(self._rect, self._path.projected_points, rainbow=True, rainbow_v=self._rainbow_v)
    else:
      super()._draw_path(sm)

    self._draw_path_edges()

  def _draw_path_edges(self):
    """Draw path edges (left, right, and front) with status-based colors."""
    if not self._path.projected_points.size:
      return

    points = self._path.projected_points
    num_points = len(points)
    mid_point = num_points // 2
    if mid_point < 2:
      return

    left_edge = points[:mid_point]
    right_edge = points[mid_point:][::-1]

    edge_color = LANE_LINE_COLORS_BP.get(ui_state.status, LANE_LINE_COLORS_BP[UIStatus.DISENGAGED])

    for i in range(len(left_edge) - 1):
      rl.draw_line_ex(
        rl.Vector2(left_edge[i][0], left_edge[i][1]),
        rl.Vector2(left_edge[i + 1][0], left_edge[i + 1][1]),
        4.0, edge_color
      )

    for i in range(len(right_edge) - 1):
      rl.draw_line_ex(
        rl.Vector2(right_edge[i][0], right_edge[i][1]),
        rl.Vector2(right_edge[i + 1][0], right_edge[i + 1][1]),
        4.0, edge_color
      )

    if len(left_edge) > 0 and len(right_edge) > 0:
      rl.draw_line_ex(
        rl.Vector2(left_edge[-1][0], left_edge[-1][1]),
        rl.Vector2(right_edge[-1][0], right_edge[-1][1]),
        4.0, edge_color
      )

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
        chevron_color = rl.Color(LEAD_VISION_CHEVRON_BASE.r, LEAD_VISION_CHEVRON_BASE.g,
                                 LEAD_VISION_CHEVRON_BASE.b, lead.fill_alpha)

      rl.draw_triangle_fan(lead.glow, len(lead.glow), glow_color)
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), chevron_color)
