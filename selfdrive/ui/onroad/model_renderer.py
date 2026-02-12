import colorsys
import math
import time
import numpy as np
import pyray as rl
from cereal import messaging, car
from dataclasses import dataclass, field
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.selfdrive.locationd.calibrationd import HEIGHT_INIT
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.system.ui.widgets import Widget

from openpilot.selfdrive.ui.sunnypilot.onroad.model_renderer import ChevronMetrics, ModelRendererSP
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.model_renderer import LANE_LINE_COLORS_SP

CLIP_MARGIN = 500
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0

THROTTLE_COLORS = [
  rl.Color(13, 248, 122, 102),   # HSLF(148/360, 0.94, 0.51, 0.4)
  rl.Color(114, 255, 92, 89),    # HSLF(112/360, 1.0, 0.68, 0.35)
  rl.Color(114, 255, 92, 0),     # HSLF(112/360, 1.0, 0.68, 0.0)
]

NO_THROTTLE_COLORS = [
  rl.Color(242, 242, 242, 102), # HSLF(148/360, 0.0, 0.95, 0.4)
  rl.Color(242, 242, 242, 89),  # HSLF(112/360, 0.0, 0.95, 0.35)
  rl.Color(242, 242, 242, 0),   # HSLF(112/360, 0.0, 0.95, 0.0)
]

LANE_LINE_COLORS = {
  UIStatus.DISENGAGED: rl.Color(200, 200, 200, 255),
  UIStatus.OVERRIDE: rl.Color(255, 255, 255, 255),
  UIStatus.ENGAGED: rl.Color(0, 255, 64, 255),  # Bright green when engaged
  **LANE_LINE_COLORS_SP,
}


@dataclass
class ModelPoints:
  raw_points: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
  projected_points: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))


@dataclass
class LeadVehicle:
  glow: list[float] = field(default_factory=list)
  chevron: list[float] = field(default_factory=list)
  fill_alpha: int = 0
  is_radar: bool = False
  flip_chevron: bool = False  # Flip chevron upside down when close


class ModelRenderer(Widget, ChevronMetrics, ModelRendererSP):
  def __init__(self):
    Widget.__init__(self)
    ChevronMetrics.__init__(self)
    ModelRendererSP.__init__(self)
    self._longitudinal_control = False
    self._experimental_mode = False
    self._blend_filter = FirstOrderFilter(1.0, 0.25, 1 / gui_app.target_fps)
    self._prev_allow_throttle = True
    self._lane_line_probs = np.zeros(4, dtype=np.float32)
    self._road_edge_stds = np.zeros(2, dtype=np.float32)
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    self._path_offset_z = HEIGHT_INIT[0]
    self._params = Params()

    # Initialize ModelPoints objects
    self._path = ModelPoints()
    self._lane_lines = [ModelPoints() for _ in range(4)]
    self._road_edges = [ModelPoints() for _ in range(2)]
    self._acceleration_x = np.empty((0,), dtype=np.float32)

    # Path smoothing: store previous smoothed path for temporal damping
    self._previous_path_projected_points = np.empty((0, 2), dtype=np.float32)
    self._path_smoothing_damping = 0.3  # Higher = more damping (0.0-1.0)

    # Torque filter for lane line coloring (orange when high torque)
    self._torque_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    # Transform matrix (3x3 for car space to screen space)
    self._car_space_transform = np.zeros((3, 3), dtype=np.float32)
    self._transform_dirty = True
    self._clip_region = None

    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=[],
      stops=[],
    )

    # Get longitudinal control setting from car parameters
    if car_params := self._params.get("CarParams"):
      cp = messaging.log_from_bytes(car_params, car.CarParams)
      self._longitudinal_control = cp.openpilotLongitudinalControl

  def set_transform(self, transform: np.ndarray):
    self._car_space_transform = transform.astype(np.float32)
    self._transform_dirty = True

  def _render(self, rect: rl.Rectangle):
    sm = ui_state.sm

    if ui_state.rainbow_path:
      #basis about 70MPH, range ~5.6-78MPH, normalized for shader
      self._rainbow_v = np.clip(sm['carState'].vEgo, 2.5, 35) / 30

    # Check if data is up-to-date
    if (sm.recv_frame["liveCalibration"] < ui_state.started_frame or
        sm.recv_frame["modelV2"] < ui_state.started_frame):
      return

    # Set up clipping region
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

    # Update state
    self._experimental_mode = sm['selfdriveState'].experimentalMode

    live_calib = sm['liveCalibration']
    self._path_offset_z = live_calib.height[0] if live_calib.height else HEIGHT_INIT[0]

    # Get carParams - try message stream first (works in device), fallback to params (works in replay)
    # Similar pattern to how controllerStateBP/lateralUncertainty is accessed
    car_params = None

    # Try to get from message stream (published by card.py every 50 seconds)
    if sm.valid['carParams']:
      try:
        car_params = sm['carParams']
        self._longitudinal_control = car_params.openpilotLongitudinalControl
      except (KeyError, AttributeError):
        pass

    # Fallback to params-based CarParams (replay writes to params, ui_state updates every 5 seconds)
    if car_params is None:
      # Always check ui_state.CP as fallback (replay writes CarParams to params)
      if ui_state.CP:
        car_params = ui_state.CP
        # Use longitudinal control from params if available
        if ui_state.CP.alphaLongitudinalAvailable:
          self._longitudinal_control = ui_state.has_longitudinal_control
        else:
          self._longitudinal_control = ui_state.CP.openpilotLongitudinalControl

    model = sm['modelV2']
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = radar_state.leadOne if radar_state else None

    # Check for radar overlay feature
    # Show overlay when enabled - lead data comes from model/radarState, available for all vehicles
    ford_overlay_enabled = self._params.get_bool("FordPrefShowRadarLeadOverlay")

    render_lead_indicator = (self._longitudinal_control or ford_overlay_enabled) and radar_state is not None

    # Update model data when needed
    model_updated = sm.updated['modelV2']
    if model_updated or sm.updated['radarState'] or self._transform_dirty:
      if model_updated:
        self._update_raw_points(model)

      path_x_array = self._path.raw_points[:, 0]
      if path_x_array.size == 0:
        return

      self._update_model(lead_one, path_x_array)
      if render_lead_indicator:
        self._update_leads(radar_state, path_x_array, ford_overlay_enabled)
      
      # Update torque filter for lane line coloring
      if sm.valid['carOutput']:
        try:
          self._torque_filter.update(-sm['carOutput'].actuatorsOutput.torque)
        except (KeyError, AttributeError):
          pass
      
      self._transform_dirty = False

    # Draw elements
    self._draw_lane_lines()
    self._draw_path(sm)

    if render_lead_indicator and radar_state:
      # Always draw chevron first, then text overlay
      self._draw_lead_indicator()
      self.chevron_metrics.draw_lead_status(sm, radar_state, self._rect, self._lead_vehicles, ford_overlay_enabled=ford_overlay_enabled)

  def _update_raw_points(self, model):
    """Update raw 3D points from model data"""
    self._path.raw_points = np.array([model.position.x, model.position.y, model.position.z], dtype=np.float32).T

    for i, lane_line in enumerate(model.laneLines):
      self._lane_lines[i].raw_points = np.array([lane_line.x, lane_line.y, lane_line.z], dtype=np.float32).T

    for i, road_edge in enumerate(model.roadEdges):
      self._road_edges[i].raw_points = np.array([road_edge.x, road_edge.y, road_edge.z], dtype=np.float32).T

    self._lane_line_probs = np.array(model.laneLineProbs, dtype=np.float32)
    self._road_edge_stds = np.array(model.roadEdgeStds, dtype=np.float32)
    self._acceleration_x = np.array(model.acceleration.x, dtype=np.float32)

  def _update_leads(self, radar_state, path_x_array, ford_overlay_enabled: bool = False):
    """Update positions of lead vehicles"""
    self._lead_vehicles = [LeadVehicle(), LeadVehicle()]
    leads = [radar_state.leadOne, radar_state.leadTwo]

    for i, lead_data in enumerate(leads):
      if lead_data and lead_data.status:
        d_rel, y_rel, v_rel = lead_data.dRel, lead_data.yRel, lead_data.vRel
        is_radar = lead_data.radar if hasattr(lead_data, 'radar') else False

        # Deadband logic: move text above chevron at 50ft (15.24m), move back below at 60ft (18.29m)
        # dRel is in meters, so convert feet to meters: 1 ft = 0.3048 m
        CLOSE_MODE_THRESHOLD_M = 50.0 * 0.3048  # 50 feet = 15.24 meters
        NORMAL_MODE_THRESHOLD_M = 60.0 * 0.3048  # 60 feet = 18.29 meters

        # Update ChevronMetrics state (ModelRenderer inherits from ChevronMetrics)
        if d_rel < CLOSE_MODE_THRESHOLD_M:
          self._close_mode = True
        elif d_rel > NORMAL_MODE_THRESHOLD_M:
          self._close_mode = False
        # Otherwise keep current state (deadband between thresholds)
        flip_chevron = False  # Don't flip chevron, just move text

        idx = self._get_path_length_idx(path_x_array, d_rel)

        # Get z-coordinate from path at the lead vehicle position
        z = self._path.raw_points[idx, 2] if idx < len(self._path.raw_points) else 0.0
        point = self._map_to_screen(d_rel, -y_rel, z + self._path_offset_z)
        if point:
          self._lead_vehicles[i] = self._update_lead_vehicle(d_rel, v_rel, point, self._rect, ford_overlay_enabled, is_radar, flip_chevron)

  def _update_model(self, lead, path_x_array):
    """Update model visualization data based on model message"""
    max_distance = np.clip(path_x_array[-1], MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
    max_idx = self._get_path_length_idx(self._lane_lines[0].raw_points[:, 0], max_distance)

    # Update lane lines using raw points (doubled width: 0.025 -> 0.05)
    for i, lane_line in enumerate(self._lane_lines):
      lane_line.projected_points = self._map_line_to_polygon(
        lane_line.raw_points, 0.05 * self._lane_line_probs[i], 0.0, max_idx, max_distance
      )

    # Update road edges using raw points (doubled width: 0.025 -> 0.05)
    for road_edge in self._road_edges:
      road_edge.projected_points = self._map_line_to_polygon(road_edge.raw_points, 0.05, 0.0, max_idx, max_distance)

    # Update path using raw points
    if lead and lead.status:
      lead_d = lead.dRel * 2.0
      max_distance = np.clip(lead_d - min(lead_d * 0.35, 10.0), 0.0, max_distance)

    max_idx = self._get_path_length_idx(path_x_array, max_distance)
    self._path.projected_points = self._map_line_to_polygon(
      self._path.raw_points, 0.9, self._path_offset_z, max_idx, max_distance, allow_invert=False
    )

    # Apply path smoothing to reduce swaying
    self._apply_smooth_path()

    self._update_experimental_gradient()

  def _update_experimental_gradient(self):
    """Pre-calculate experimental mode gradient colors"""
    if not self._experimental_mode:
      return

    max_len = min(len(self._path.projected_points) // 2, len(self._acceleration_x))

    segment_colors = []
    gradient_stops = []

    i = 0
    while i < max_len:
      # Some points (screen space) are out of frame (rect space)
      track_y = self._path.projected_points[i][1]
      if track_y < self._rect.y or track_y > (self._rect.y + self._rect.height):
        i += 1
        continue

      # Calculate color based on acceleration (0 is bottom, 1 is top)
      lin_grad_point = 1 - (track_y - self._rect.y) / self._rect.height

      # speed up: 120, slow down: 0
      path_hue = np.clip(60 + self._acceleration_x[i] * 35, 0, 120)

      saturation = min(abs(self._acceleration_x[i] * 1.5), 1)
      lightness = np.interp(saturation, [0.0, 1.0], [0.95, 0.62])
      alpha = np.interp(lin_grad_point, [0.75 / 2.0, 0.75], [0.4, 0.0])

      # Use HSL to RGB conversion
      color = self._hsla_to_color(path_hue / 360.0, saturation, lightness, alpha)

      gradient_stops.append(lin_grad_point)
      segment_colors.append(color)

      # Skip a point, unless next is last
      i += 1 + (1 if (i + 2) < max_len else 0)

    # Store the gradient in the path object
    self._exp_gradient = Gradient(
      start=(0.0, 1.0),  # Bottom of path
      end=(0.0, 0.0),  # Top of path
      colors=segment_colors,
      stops=gradient_stops,
    )

  def _apply_smooth_path(self):
    """Apply path smoothing to reduce swaying - ported from bp-dev QT UI

    Uses Gaussian-weighted spatial smoothing on Y-axis (lateral movement) combined
    with temporal damping to reduce path oscillation and swaying.
    """
    if self._path.projected_points.size == 0:
      return

    # Need at least 4 points for smoothing
    if len(self._path.projected_points) < 4:
      self._previous_path_projected_points = self._path.projected_points.copy()
      return

    n = len(self._path.projected_points)
    smoothed = np.zeros_like(self._path.projected_points)

    # Apply Gaussian-weighted spatial smoothing to Y-axis (lateral movement)
    for i in range(n):
      pt = self._path.projected_points[i].copy()

      # Apply Gaussian smoothing to Y coordinate for points not at edges
      if i > 1 and i < n - 1:
        y_smooth = 0.0
        weight_sum = 0.0

        # Gaussian weights for nearby points (±2 points)
        for j in range(-2, 3):
          idx = i + j
          if 0 <= idx < n:
            weight = np.exp(-0.5 * j * j)  # Gaussian weight
            y_smooth += self._path.projected_points[idx][1] * weight
            weight_sum += weight

        if weight_sum > 0:
          pt[1] = y_smooth / weight_sum

      smoothed[i] = pt

    # Apply temporal damping to reduce oscillation
    # Only apply if previous path has same number of points (path structure unchanged)
    if (self._previous_path_projected_points.size > 0 and
        len(self._previous_path_projected_points) == len(smoothed)):
      damping = self._path_smoothing_damping
      for i in range(len(smoothed)):
        y_diff = smoothed[i][1] - self._previous_path_projected_points[i][1]
        smoothed[i][1] = self._previous_path_projected_points[i][1] + y_diff * (1.0 - damping)
    else:
      # Path structure changed, reset temporal smoothing
      self._previous_path_projected_points = smoothed.copy()

    # Store smoothed path for next frame
    self._previous_path_projected_points = smoothed.copy()
    self._path.projected_points = smoothed

  def _update_lead_vehicle(self, d_rel, v_rel, point, rect, ford_overlay_enabled: bool = False, is_radar: bool = False, flip_chevron: bool = False):
    # flip_chevron parameter kept for compatibility but not used - chevron always stays normal
    speed_buff, lead_buff = 10.0, 40.0

    # Calculate fill alpha
    fill_alpha = 0
    if d_rel < lead_buff:
      fill_alpha = 255 * (1.0 - (d_rel / lead_buff))
      if v_rel < 0:
        fill_alpha += 255 * (-1 * (v_rel / speed_buff))
      fill_alpha = min(fill_alpha, 255)

    # Calculate size and position
    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35
    x = np.clip(point[0], 0.0, rect.width - sz / 2)
    base_y = min(point[1], rect.height - sz * 0.6)

    g_xo = sz / 5
    g_yo = sz / 10

    # Create glow and chevron points
    # Normal: chevron points down [bottom_right, top, bottom_left]
    # Always use normal orientation - don't flip chevron
    y = base_y
    glow = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
    chevron = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

    return LeadVehicle(glow=glow, chevron=chevron, fill_alpha=int(fill_alpha), is_radar=is_radar, flip_chevron=flip_chevron)

  def _draw_lane_lines(self):
    """Draw lane lines and road edges - always use enhanced rendering"""
    self._draw_enhanced_lane_lines()

  def _get_ll_color(self, prob: float, adjacent: bool, left: bool):
    """Get lane line color based on probability, adjacency, and torque - ported from MICI"""
    alpha = np.clip(prob, 0.0, 0.7)
    if adjacent:
      _base_color = LANE_LINE_COLORS.get(ui_state.status, LANE_LINE_COLORS[UIStatus.DISENGAGED])
      color = rl.Color(_base_color.r, _base_color.g, _base_color.b, int(alpha * 255))

      # Turn adjacent lane lines orange if torque is high
      torque = self._torque_filter.x
      high_torque = abs(torque) > 0.6
      if high_torque and (left == (torque > 0)):
        # Blend with orange color based on torque magnitude
        orange_color = rl.Color(255, 115, 0, int(alpha * 255))
        blend_factor = np.interp(abs(torque), [0.6, 0.8], [0.0, 1.0])
        # Simple linear blend between two colors
        inv_factor = 1.0 - blend_factor
        color = rl.Color(
          int(color.r * inv_factor + orange_color.r * blend_factor),
          int(color.g * inv_factor + orange_color.g * blend_factor),
          int(color.b * inv_factor + orange_color.b * blend_factor),
          color.a
        )
    else:
      color = rl.Color(255, 255, 255, int(alpha * 255))

    if ui_state.status == UIStatus.DISENGAGED:
      color = rl.Color(0, 0, 0, int(alpha * 255))

    return color

  def _draw_enhanced_lane_lines(self):
    """Draw enhanced lane lines with glow effects - ported from bp-dev QT UI"""
    # Draw wide lane line polygons with enhanced visibility
    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0 or self._lane_line_probs[i] < 0.4:
        continue

      base_alpha = np.clip(self._lane_line_probs[i] * 0.8, 0.3, 0.8)
      is_current_lane = (i == 1 or i == 2)
      if not is_current_lane:
        base_alpha *= 0.4  # Dim outer lanes

      # Use MICI color logic: green for adjacent lanes when engaged, white for outer lanes
      # i in (1, 2) means adjacent (current lane), i in (0, 1) means left side
      # Note: _get_ll_color uses prob directly for alpha, but we want to use base_alpha for enhanced rendering
      # So we get the color with full alpha and then apply base_alpha scaling
      base_color = self._get_ll_color(float(self._lane_line_probs[i]), is_current_lane, i in (0, 1))
      # Scale alpha based on base_alpha (which is already scaled from prob)
      scaled_alpha = int(base_color.a * (base_alpha / 0.7) if base_alpha < 0.7 else base_color.a)
      color = rl.Color(base_color.r, base_color.g, base_color.b, scaled_alpha)
      draw_polygon(self._rect, lane_line.projected_points, color)

    # Add horizontal glow effects for enhanced visibility
    self._draw_lane_glow_effects()

    # Draw road edges with enhanced red warning
    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      edge_alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0) * 0.6
      color = rl.Color(255, 0, 0, int(edge_alpha * 255))
      draw_polygon(self._rect, road_edge.projected_points, color)

    # Add road edge glow effects
    self._draw_road_edge_glow_effects()

  def _draw_lane_glow_effects(self):
    """Draw glow effects around lane lines - ported from bp-dev QT UI"""
    # Three-layer glow for smooth falloff
    glow_widths = [24.0, 16.0, 8.0]
    glow_alphas = [0.08, 0.15, 0.3]

    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0 or self._lane_line_probs[i] < 0.4:
        continue

      base_alpha = np.clip(self._lane_line_probs[i] * 0.8, 0.3, 0.8)
      is_current_lane = (i == 1 or i == 2)
      if not is_current_lane:
        base_alpha *= 0.4

      # Use MICI color logic for glow effects too - get base color (without alpha scaling)
      base_color = self._get_ll_color(float(self._lane_line_probs[i]), is_current_lane, i in (0, 1))
      
      # Draw glow layers (outer to inner)
      for layer_idx, (glow_width, glow_alpha) in enumerate(zip(glow_widths, glow_alphas)):
        # Expand polygon outward for glow effect
        expanded_points = self._expand_polygon(lane_line.projected_points, glow_width)
        if expanded_points.size > 0:
          # Apply base_alpha and glow_alpha to the base color's RGB, keeping the color hue
          alpha = int(base_alpha * glow_alpha * 255)
          color = rl.Color(base_color.r, base_color.g, base_color.b, alpha)
          draw_polygon(self._rect, expanded_points, color)

  def _draw_road_edge_glow_effects(self):
    """Draw glow effects around road edges - ported from bp-dev QT UI"""
    # Red warning glow with three layers
    glow_widths = [36.0, 24.0, 12.0]
    glow_alphas = [0.05, 0.1, 0.2]

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      edge_alpha = np.clip(1.0 - self._road_edge_stds[i], 0.0, 1.0)
      if edge_alpha < 0.3:
        continue

      # Draw glow layers (outer to inner)
      for layer_idx, (glow_width, glow_alpha) in enumerate(zip(glow_widths, glow_alphas)):
        # Expand polygon outward for glow effect
        expanded_points = self._expand_polygon(road_edge.projected_points, glow_width)
        if expanded_points.size > 0:
          alpha = int(edge_alpha * glow_alpha * 255)
          color = rl.Color(255, 0, 0, alpha)
          draw_polygon(self._rect, expanded_points, color)


  def _expand_polygon(self, points: np.ndarray, width: float) -> np.ndarray:
    """Expand polygon outward by width pixels for glow effect"""
    if points.size == 0 or len(points) < 3:
      return np.empty((0, 2), dtype=np.float32)

    # For each point, calculate outward normal and expand
    expanded = []
    n = len(points)
    
    for i in range(n):
      # Get previous and next points for normal calculation
      prev_idx = (i - 1) % n
      next_idx = (i + 1) % n
      
      p_prev = points[prev_idx]
      p_curr = points[i]
      p_next = points[next_idx]
      
      # Calculate edge vectors
      edge1 = p_curr - p_prev
      edge2 = p_next - p_curr
      
      # Normalize edge vectors
      len1 = np.linalg.norm(edge1)
      len2 = np.linalg.norm(edge2)
      
      if len1 > 1e-6:
        edge1 = edge1 / len1
      if len2 > 1e-6:
        edge2 = edge2 / len2
      
      # Calculate outward normal (perpendicular to edge, pointing outward)
      # For a polygon, we need to determine which side is "outward"
      # Simple approach: use average of edge normals
      normal1 = np.array([-edge1[1], edge1[0]])  # 90 degree rotation
      normal2 = np.array([-edge2[1], edge2[0]])
      
      # Average normal
      normal = (normal1 + normal2) / 2.0
      normal_len = np.linalg.norm(normal)
      if normal_len > 1e-6:
        normal = normal / normal_len
      
      # Expand point outward
      expanded_point = p_curr + normal * width
      expanded.append(expanded_point)
    
    return np.array(expanded, dtype=np.float32)

  def _draw_path(self, sm):
    """Draw path with dynamic coloring based on mode and throttle state."""
    if not self._path.projected_points.size:
      return

    allow_throttle = sm['longitudinalPlan'].allowThrottle or not self._longitudinal_control
    self._blend_filter.update(int(allow_throttle))

    if ui_state.rainbow_path:
      #self.rainbow_path.draw_rainbow_path(self._rect, self._path)
      draw_polygon(self._rect, self._path.projected_points, rainbow=True, rainbow_v=self._rainbow_v)
      # Draw status-colored edges even for rainbow path
      self._draw_path_edges()
      return

    if self._experimental_mode:
      # Draw with acceleration coloring
      if len(self._exp_gradient.colors) > 1:
        draw_polygon(self._rect, self._path.projected_points, gradient=self._exp_gradient)
      else:
        draw_polygon(self._rect, self._path.projected_points, rl.Color(255, 255, 255, 30))
    else:
      # Blend throttle/no throttle colors based on transition
      blend_factor = round(self._blend_filter.x * 100) / 100
      blended_colors = self._blend_colors(NO_THROTTLE_COLORS, THROTTLE_COLORS, blend_factor)
      gradient = Gradient(
        start=(0.0, 1.0),  # Bottom of path
        end=(0.0, 0.0),  # Top of path
        colors=blended_colors,
        stops=[0.0, 0.5, 1.0],
      )
      draw_polygon(self._rect, self._path.projected_points, gradient=gradient)
    
    # Draw status-colored edges on top of the filled path
    self._draw_path_edges()

  def _draw_path_edges(self):
    """Draw path edges (left, right, and front) with status-based colors."""
    if not self._path.projected_points.size:
      return
    
    points = self._path.projected_points
    num_points = len(points)
    
    # Path polygon structure: [left_0..left_N-1, right_N-1..right_0]
    # Split point: where left edge ends and right edge begins
    mid_point = num_points // 2
    
    if mid_point < 2:
      return  # Need at least 2 points per edge
    
    # Extract left and right edges
    # Left edge: first half, already in correct order (near to far)
    left_edge = points[:mid_point]
    # Right edge: second half, stored in reverse (far to near), reverse to get near to far
    right_edge = points[mid_point:][::-1]
    
    # Get status color for edges (same as lane lines)
    edge_color = LANE_LINE_COLORS.get(ui_state.status, LANE_LINE_COLORS[UIStatus.DISENGAGED])
    
    # Draw left edge (from near to far)
    for i in range(len(left_edge) - 1):
      rl.draw_line_ex(
        rl.Vector2(left_edge[i][0], left_edge[i][1]),
        rl.Vector2(left_edge[i + 1][0], left_edge[i + 1][1]),
        4.0,  # Line thickness (doubled from 2.0)
        edge_color
      )
    
    # Draw right edge (from near to far)
    for i in range(len(right_edge) - 1):
      rl.draw_line_ex(
        rl.Vector2(right_edge[i][0], right_edge[i][1]),
        rl.Vector2(right_edge[i + 1][0], right_edge[i + 1][1]),
        4.0,  # Line thickness (doubled from 2.0)
        edge_color
      )
    
    # Draw front edge (connects far left point to far right point)
    if len(left_edge) > 0 and len(right_edge) > 0:
      rl.draw_line_ex(
        rl.Vector2(left_edge[-1][0], left_edge[-1][1]),  # Last left point (far)
        rl.Vector2(right_edge[-1][0], right_edge[-1][1]),  # Last right point (far)
        4.0,  # Line thickness (doubled from 2.0)
        edge_color
      )

  def _draw_lead_indicator(self):
    # Draw lead vehicles if available
    for lead in self._lead_vehicles:
      if not lead.glow or not lead.chevron:
        continue

      # Dynamic colors based on detection source: blue for radar, yellow for vision
      if lead.is_radar:
        # Blue for radar detection
        glow_color = rl.Color(0, 134, 233, 255)  # Blue glow
        chevron_color = rl.Color(0, 100, 200, lead.fill_alpha)  # Blue chevron
      else:
        # Yellow for vision detection
        glow_color = rl.Color(218, 202, 37, 255)  # Yellow glow
        chevron_color = rl.Color(201, 34, 49, lead.fill_alpha)  # Red chevron (original)

      rl.draw_triangle_fan(lead.glow, len(lead.glow), glow_color)
      rl.draw_triangle_fan(lead.chevron, len(lead.chevron), chevron_color)

  @staticmethod
  def _get_path_length_idx(pos_x_array: np.ndarray, path_distance: float) -> int:
    """Get the index corresponding to the given path distance"""
    if len(pos_x_array) == 0:
      return 0
    indices = np.where(pos_x_array <= path_distance)[0]
    return indices[-1] if indices.size > 0 else 0

  def _map_to_screen(self, in_x, in_y, in_z):
    """Project a point in car space to screen space"""
    input_pt = np.array([in_x, in_y, in_z])
    pt = self._car_space_transform @ input_pt

    if abs(pt[2]) < 1e-6:
      return None

    x, y = pt[0] / pt[2], pt[1] / pt[2]

    clip = self._clip_region
    if not (clip.x <= x <= clip.x + clip.width and clip.y <= y <= clip.y + clip.height):
      return None

    return (x, y)

  def _map_line_to_polygon(self, line: np.ndarray, y_off: float, z_off: float, max_idx: int, max_distance: float, allow_invert: bool = True) -> np.ndarray:
    """Convert 3D line to 2D polygon for rendering."""
    if line.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    # Slice points and filter non-negative x-coordinates
    points = line[:max_idx + 1]

    # Interpolate around max_idx so path end is smooth (max_distance is always >= p0.x)
    if 0 < max_idx < line.shape[0] - 1:
      p0 = line[max_idx]
      p1 = line[max_idx + 1]
      x0, x1 = p0[0], p1[0]
      interp_y = np.interp(max_distance, [x0, x1], [p0[1], p1[1]])
      interp_z = np.interp(max_distance, [x0, x1], [p0[2], p1[2]])
      interp_point = np.array([max_distance, interp_y, interp_z], dtype=points.dtype)
      points = np.concatenate((points, interp_point[None, :]), axis=0)

    points = points[points[:, 0] >= 0]
    if points.shape[0] == 0:
      return np.empty((0, 2), dtype=np.float32)

    N = points.shape[0]
    # Generate left and right 3D points in one array using broadcasting
    offsets = np.array([[0, -y_off, z_off], [0, y_off, z_off]], dtype=np.float32)
    points_3d = points[None, :, :] + offsets[:, None, :]  # Shape: 2xNx3
    points_3d = points_3d.reshape(2 * N, 3)  # Shape: (2*N)x3

    # Transform all points to projected space in one operation
    proj = self._car_space_transform @ points_3d.T  # Shape: 3x(2*N)
    proj = proj.reshape(3, 2, N)
    left_proj = proj[:, 0, :]
    right_proj = proj[:, 1, :]

    # Filter points where z is sufficiently large
    valid_proj = (np.abs(left_proj[2]) >= 1e-6) & (np.abs(right_proj[2]) >= 1e-6)
    if not np.any(valid_proj):
      return np.empty((0, 2), dtype=np.float32)

    # Compute screen coordinates
    left_screen = left_proj[:2, valid_proj] / left_proj[2, valid_proj][None, :]
    right_screen = right_proj[:2, valid_proj] / right_proj[2, valid_proj][None, :]

    # Define clip region bounds
    clip = self._clip_region
    x_min, x_max = clip.x, clip.x + clip.width
    y_min, y_max = clip.y, clip.y + clip.height

    # Filter points within clip region
    left_in_clip = (
      (left_screen[0] >= x_min) & (left_screen[0] <= x_max) &
      (left_screen[1] >= y_min) & (left_screen[1] <= y_max)
    )
    right_in_clip = (
      (right_screen[0] >= x_min) & (right_screen[0] <= x_max) &
      (right_screen[1] >= y_min) & (right_screen[1] <= y_max)
    )
    both_in_clip = left_in_clip & right_in_clip

    if not np.any(both_in_clip):
      return np.empty((0, 2), dtype=np.float32)

    # Select valid and clipped points
    left_screen = left_screen[:, both_in_clip]
    right_screen = right_screen[:, both_in_clip]

    # Handle Y-coordinate inversion on hills
    if not allow_invert and left_screen.shape[1] > 1:
      y = left_screen[1, :]  # y-coordinates
      keep = y == np.minimum.accumulate(y)
      if not np.any(keep):
        return np.empty((0, 2), dtype=np.float32)
      left_screen = left_screen[:, keep]
      right_screen = right_screen[:, keep]

    return np.vstack((left_screen.T, right_screen[:, ::-1].T)).astype(np.float32)

  @staticmethod
  def _hsla_to_color(h, s, l, a):
    rgb = colorsys.hls_to_rgb(h, l, s)
    return rl.Color(
      int(rgb[0] * 255),
      int(rgb[1] * 255),
      int(rgb[2] * 255),
      int(a * 255)
    )

  @staticmethod
  def _blend_colors(begin_colors, end_colors, t):
    if t >= 1.0:
      return end_colors
    if t <= 0.0:
      return begin_colors

    inv_t = 1.0 - t
    return [rl.Color(
      int(inv_t * start.r + t * end.r),
      int(inv_t * start.g + t * end.g),
      int(inv_t * start.b + t * end.b),
      int(inv_t * start.a + t * end.a)
    ) for start, end in zip(begin_colors, end_colors, strict=True)]
