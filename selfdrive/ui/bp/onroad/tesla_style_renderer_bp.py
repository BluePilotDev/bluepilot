"""Tesla-inspired environment layer for BluePilot's onroad view.

This renderer replaces only the camera/model scene.  AugmentedRoadViewBP keeps
the existing HUD, alerts, driver monitoring, gauges, confidence indicators,
and safety blind-spot edges above it.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.bp.lib.tesla_palette import blend_color, palette_for_dark_fraction
from openpilot.selfdrive.ui.bp.lib.longitudinal_visuals import tesla_geometry_reliable
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient

MAX_LEAD_DISTANCE_M = 140.0
NOMINAL_LANE_WIDTH_M = 3.7
LEAD_LANE_WIDTH_FRACTION = 0.55
LEAD_HEIGHT_TO_WIDTH = 456.0 / 512.0
LEAD_MAX_WIDTH_PX = 150.0
LEAD_SPRITE_ASSET = "images/tesla_lead_sedan_far.png"
LEAD_FULL_SCALE_DISTANCE_M = 8.0
LEAD_FAR_SCALE_DISTANCE_M = 55.0
LEAD_FAR_DISTANCE_SCALE = 0.70
LEAD_SHADOW_RADIUS_X_FRACTION = 0.36
LEAD_SHADOW_RADIUS_Y_FRACTION = 0.018
LEAD_SHADOW_CENTER_Y_FRACTION = -0.020
LEAD_FADE_SECONDS = 0.25
ROAD_CHAIN_POINT_COUNT = 9
ROAD_DEFAULT_HORIZON_Y_FRACTION = 0.44
ROAD_DROPOUT_HOLD_SECONDS = 0.15
ROAD_DROPOUT_FADE_SECONDS = 0.25
ROAD_TRACKING_SECONDS = 0.24
ROAD_STANDSTILL_FREEZE_MPS = 0.5

LeadValues = tuple[float, float, float]


@dataclass
class LeadFadeState:
  """Fade between lead identities without interpolating the actor across cars."""
  displayed_values: LeadValues | None = None
  displayed_generation: int | None = None
  pending_values: LeadValues | None = None
  pending_generation: int | None = None
  opacity: float = 0.0
  phase: str = "hidden"

  def reset(self) -> None:
    self.displayed_values = None
    self.displayed_generation = None
    self.pending_values = None
    self.pending_generation = None
    self.opacity = 0.0
    self.phase = "hidden"

  def update(self, candidate: LeadValues | None, generation: int | None,
             opacity_step: float) -> tuple[LeadValues | None, float]:
    """Advance fade-out-before-fade-in state and return actor data to draw."""
    step = float(np.clip(opacity_step, 0.0, 1.0))

    if self.phase == "hidden":
      if candidate is None:
        return None, 0.0
      self.displayed_values = candidate
      self.displayed_generation = generation
      self.phase = "fading_in"

    elif self.phase in ("visible", "fading_in"):
      if candidate is None or generation != self.displayed_generation:
        self.pending_values = candidate
        self.pending_generation = generation if candidate is not None else None
        self.phase = "fading_out"
      else:
        self.displayed_values = candidate

    elif self.phase == "fading_out":
      if candidate is not None and generation == self.displayed_generation:
        # A one-frame dropout recovered as the same lead; reverse the fade.
        self.displayed_values = candidate
        self.pending_values = None
        self.pending_generation = None
        self.phase = "fading_in"
      else:
        # Follow the newest replacement while the old actor continues fading.
        self.pending_values = candidate
        self.pending_generation = generation if candidate is not None else None

    if self.phase == "fading_out":
      values_to_draw = self.displayed_values
      self.opacity = max(0.0, self.opacity - step)
      if self.opacity <= 1e-6:
        self.opacity = 0.0
        if self.pending_values is None:
          self.reset()
        else:
          self.displayed_values = self.pending_values
          self.displayed_generation = self.pending_generation
          self.pending_values = None
          self.pending_generation = None
          self.phase = "fading_in"
      return values_to_draw, self.opacity

    self.opacity = min(1.0, self.opacity + step)
    if self.opacity >= 1.0:
      self.phase = "visible"
    return self.displayed_values, self.opacity


@dataclass
class RoadGeometryState:
  """Track real model road geometry and age it out without inventing a road."""
  points: np.ndarray | None = None
  opacity: float = 0.0
  _last_update: float | None = None
  _dropout_since: float | None = None
  _dropout_opacity: float = 0.0

  def reset(self) -> None:
    self.points = None
    self.opacity = 0.0
    self._last_update = None
    self._dropout_since = None
    self._dropout_opacity = 0.0

  def update(self, target: np.ndarray | None, reliable: bool, freeze: bool = False,
             now: float | None = None) -> tuple[np.ndarray | None, float]:
    now = time.monotonic() if now is None else float(now)
    dt = 0.0 if self._last_update is None else max(0.0, min(now - self._last_update, 0.25))
    self._last_update = now
    candidate = None if target is None else np.asarray(target, dtype=np.float32)
    candidate_valid = (
      candidate is not None and candidate.ndim == 2 and candidate.shape[1] == 2 and
      len(candidate) >= 4 and np.all(np.isfinite(candidate))
    )

    if freeze and self.points is not None:
      # At standstill the physical road is not moving. Keep the last real
      # geometry instead of chasing the model's rapidly changing hypotheses.
      self._dropout_since = None
      return self.points.copy(), self.opacity

    if reliable and candidate_valid:
      self._dropout_since = None
      if self.points is None or self.points.shape != candidate.shape:
        self.points = candidate.copy()
      elif dt > 0.0:
        alpha = 1.0 - float(np.exp(-dt / ROAD_TRACKING_SECONDS))
        self.points += (candidate - self.points) * alpha
      self.opacity = 1.0
      return self.points.copy(), self.opacity

    if self.points is None:
      self.opacity = 0.0
      return None, 0.0
    if self._dropout_since is None:
      self._dropout_since = now
      self._dropout_opacity = self.opacity
    dropout_age = now - self._dropout_since
    if dropout_age > ROAD_DROPOUT_HOLD_SECONDS:
      fade_age = dropout_age - ROAD_DROPOUT_HOLD_SECONDS
      self.opacity = self._dropout_opacity * float(np.clip(
        1.0 - fade_age / ROAD_DROPOUT_FADE_SECONDS, 0.0, 1.0,
      ))
    if self.opacity <= 0.0:
      self.points = None
      return None, 0.0
    return self.points.copy(), self.opacity


def color_with_opacity(color: rl.Color, opacity: float) -> rl.Color:
  return rl.Color(color.r, color.g, color.b, round(color.a * float(np.clip(opacity, 0.0, 1.0))))


def _bottom_road_anchor(edge: list[tuple[float, float]], rect: rl.Rectangle,
                        fallback_x_fraction: float, outward: int) -> tuple[float, float]:
  """Extend one projected road edge to the bottom of the viewport.

  The model-backed road begins at 6 m, which is often halfway up the display.
  Extrapolating its nearest segment avoids closing the road ribbon across that
  point and exposing a horizontal strip of ground beneath it.
  """
  bottom_y = float(rect.y + rect.height + 1.0)
  fallback_x = float(rect.x + rect.width * fallback_x_fraction)
  if len(edge) < 2:
    return fallback_x, bottom_y

  near = np.asarray(edge[0], dtype=np.float64)
  next_near = np.asarray(edge[1], dtype=np.float64)
  if not (np.all(np.isfinite(near)) and np.all(np.isfinite(next_near))):
    return fallback_x, bottom_y

  dy = float(near[1] - next_near[1])
  if abs(dy) < 1.0:
    return fallback_x, bottom_y

  x = float(near[0] + (bottom_y - near[1]) * (near[0] - next_near[0]) / dy)
  if not np.isfinite(x):
    return fallback_x, bottom_y

  # Road edges should widen toward the viewer, never fold back across their
  # nearest model sample on a sharp curve or unusual hill profile.
  if near[1] < bottom_y:
    x = min(x, float(near[0])) if outward < 0 else max(x, float(near[0]))
  x = float(np.clip(x, rect.x, rect.x + rect.width))
  return x, bottom_y


def extend_road_edges_to_bottom(left: list[tuple[float, float]], right: list[tuple[float, float]],
                                rect: rl.Rectangle) -> tuple[list[tuple[float, float]],
                                                             list[tuple[float, float]]]:
  """Return paired road-edge chains whose near closure is below the viewport."""
  if not left or not right:
    return left, right

  bottom_y = float(rect.y + rect.height)
  if left[0][1] >= bottom_y and right[0][1] >= bottom_y:
    return left, right

  left_anchor = _bottom_road_anchor(left, rect, 0.05, -1)
  right_anchor = _bottom_road_anchor(right, rect, 0.95, 1)
  if left_anchor[0] >= right_anchor[0]:
    left_anchor = (float(rect.x + rect.width * 0.05), left_anchor[1])
    right_anchor = (float(rect.x + rect.width * 0.95), right_anchor[1])
  return [left_anchor, *left], [right_anchor, *right]


def _resample_road_edge(edge: list[tuple[float, float]], count: int) -> np.ndarray | None:
  points = np.asarray(edge, dtype=np.float32)
  if points.ndim != 2 or points.shape[0] < 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
    return None

  lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
  cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
  total = float(cumulative[-1])
  if total < 1e-3:
    return None

  samples = np.linspace(0.0, total, count)
  return np.column_stack((
    np.interp(samples, cumulative, points[:, 0]),
    np.interp(samples, cumulative, points[:, 1]),
  )).astype(np.float32)


def normalized_road_polygon(left: list[tuple[float, float]], right: list[tuple[float, float]],
                            rect: rl.Rectangle) -> np.ndarray | None:
  """Create a fixed-size two-chain polygon suitable for temporal interpolation."""
  left, right = extend_road_edges_to_bottom(left, right, rect)
  left_points = _resample_road_edge(left, ROAD_CHAIN_POINT_COUNT)
  right_points = _resample_road_edge(right, ROAD_CHAIN_POINT_COUNT)
  if left_points is None or right_points is None:
    return None
  if np.any(left_points[:, 0] >= right_points[:, 0]):
    return None
  return np.concatenate((left_points, right_points[::-1])).astype(np.float32)


def valid_primary_lead_values(lead) -> tuple[float, float, float] | None:
  """Validate the fused primary lead without inventing a raw-radar fallback."""
  if lead is None or not bool(getattr(lead, "status", False)):
    return None
  values = (float(lead.dRel), float(lead.yRel), float(lead.vRel))
  if not all(np.isfinite(value) for value in values):
    return None
  if not 0.0 < values[0] <= MAX_LEAD_DISTANCE_M:
    return None
  return values


def road_geometry_reliable(path_points, sm) -> bool:
  """Validate live XYZ model geometry without changing model-path presentation."""
  if not (sm.valid.get("modelV2", False) and tesla_geometry_reliable(path_points, sm)):
    return False
  if hasattr(sm, "alive") and not sm.alive.get("modelV2", False):
    return False
  points = np.asarray(path_points, dtype=np.float64)
  return bool(points.ndim == 2 and points.shape[1] >= 3 and np.all(np.isfinite(points[:, :3])))


def lead_actor_width(lane_width_px: float, rect_height: float, d_rel: float) -> float:
  """Size the lead from lane perspective with extra attenuation in the distance."""
  display_scale = max(0.45, float(rect_height) / 1080.0)
  distance_scale = float(np.interp(
    d_rel,
    (LEAD_FULL_SCALE_DISTANCE_M, LEAD_FAR_SCALE_DISTANCE_M),
    (1.0, LEAD_FAR_DISTANCE_SCALE),
  ))
  min_width = 30.0 * display_scale
  perspective_width = float(np.clip(lane_width_px * LEAD_LANE_WIDTH_FRACTION,
                                    min_width, LEAD_MAX_WIDTH_PX * display_scale))
  return max(min_width, perspective_width * distance_scale)


def lead_actor_height(width: float) -> float:
  return float(width) * LEAD_HEIGHT_TO_WIDTH


def lead_actor_base_y(projected_y: float, width: float, rect: rl.Rectangle) -> float:
  """Keep a close lead fully visible when its ground point projects offscreen."""
  height = lead_actor_height(width)
  display_scale = max(0.45, float(rect.height) / 1080.0)
  bottom_margin = max(24.0 * display_scale, height * 0.14)
  return min(float(projected_y), float(rect.y + rect.height - bottom_margin))


def project_car_space_unclipped(transform: np.ndarray, in_x: float,
                                in_y: float, in_z: float) -> tuple[float, float] | None:
  """Project traffic geometry without the model renderer's offscreen clip guard."""
  point = np.asarray(transform, dtype=np.float64) @ np.asarray((in_x, in_y, in_z), dtype=np.float64)
  if point.shape != (3,) or not np.all(np.isfinite(point)) or abs(float(point[2])) < 1e-6:
    return None
  projected = (float(point[0] / point[2]), float(point[1] / point[2]))
  return projected if all(np.isfinite(value) for value in projected) else None


class TeslaStyleRendererBP:
  def __init__(self, relative_projection: bool = False, show_lead_vehicle: bool = True,
               dark_fraction: float = 0.0):
    self.relative_projection = relative_projection
    self.show_lead_vehicle = show_lead_vehicle
    self._dark_fraction = dark_fraction
    self._enabled = False
    self._lead_fade = LeadFadeState()
    self._road_geometry = RoadGeometryState()
    self._lead_sedan_texture: rl.Texture | None = None

  def set_enabled(self, enabled: bool) -> None:
    if enabled != self._enabled:
      self._lead_fade.reset()
      self._road_geometry.reset()
    self._enabled = enabled

  def set_dark_fraction(self, dark_fraction: float) -> None:
    self._dark_fraction = max(0.0, min(float(dark_fraction), 1.0))

  def _to_screen(self, point, rect: rl.Rectangle):
    if point is None:
      return None
    x, y = float(point[0]), float(point[1])
    if self.relative_projection:
      x += rect.x
      y += rect.y
    return x, y

  def _project(self, model_renderer, rect: rl.Rectangle, d_rel: float, y_rel: float):
    path = model_renderer._path.raw_points
    z = 0.0
    if path.size:
      idx = model_renderer._get_path_length_idx(path[:, 0], d_rel)
      if idx < len(path):
        z = float(path[idx, 2])
    # ModelRenderer clips ordinary model geometry outside a generous screen
    # margin. A very close lead's road-contact point can legitimately project
    # beyond that margin, so retain it here and clamp the actor after sizing.
    point = project_car_space_unclipped(
      model_renderer._car_space_transform,
      d_rel,
      -y_rel + float(getattr(model_renderer, "_camera_offset", 0.0)),
      z + float(getattr(model_renderer, "_path_offset_z", 0.0)),
    )
    return self._to_screen(point, rect)

  def _projected_road_polygon(self, rect: rl.Rectangle, model_renderer) -> np.ndarray | None:
    path = model_renderer._path.raw_points
    if path.size and path.ndim == 2 and path.shape[1] >= 3 and np.all(np.isfinite(path)):
      max_forward_distance = float(np.max(path[:, 0]))
      left: list[tuple[float, float]] = []
      right: list[tuple[float, float]] = []
      for distance in (6.0, 10.0, 16.0, 25.0, 38.0, 55.0, 78.0, 100.0):
        if distance > max_forward_distance:
          continue
        idx = model_renderer._get_path_length_idx(path[:, 0], distance)
        if idx >= len(path):
          continue
        center_y = float(path[idx, 1])
        z = float(path[idx, 2]) + float(getattr(model_renderer, "_path_offset_z", 0.0))
        left_pt = self._to_screen(model_renderer._map_to_screen(distance, center_y - 5.2, z), rect)
        right_pt = self._to_screen(model_renderer._map_to_screen(distance, center_y + 5.2, z), rect)
        if left_pt is not None and right_pt is not None:
          left.append(left_pt)
          right.append(right_pt)
      if len(left) >= 3 and len(right) >= 3:
        return normalized_road_polygon(left, right, rect)
    return None

  def _road_polygon(self, rect: rl.Rectangle, model_renderer) -> tuple[np.ndarray | None, float]:
    projected = self._projected_road_polygon(rect, model_renderer)
    reliable = road_geometry_reliable(model_renderer._path.raw_points, ui_state.sm) and projected is not None
    sm = ui_state.sm
    car_available = (
      sm.valid.get("carState", False) and
      (not hasattr(sm, "alive") or sm.alive.get("carState", False))
    )
    freeze = car_available and float(sm["carState"].vEgo) < ROAD_STANDSTILL_FREEZE_MPS
    return self._road_geometry.update(projected, reliable, freeze=freeze)

  def render_background(self, rect: rl.Rectangle, model_renderer) -> None:
    palette = palette_for_dark_fraction(self._dark_fraction)
    model_renderer.prepare_projection(rect)
    road, road_opacity = self._road_polygon(rect, model_renderer)
    far_points = (
      road[np.argsort(road[:, 1])[:2]]
      if road is not None and len(road) >= 2
      else np.empty((0, 2))
    )
    default_horizon_y = rect.y + rect.height * ROAD_DEFAULT_HORIZON_Y_FRACTION
    if len(far_points):
      model_horizon_y = float(np.mean(far_points[:, 1]))
      horizon_y = default_horizon_y + (model_horizon_y - default_horizon_y) * road_opacity
    else:
      horizon_y = default_horizon_y
    horizon_y = float(np.clip(horizon_y, rect.y + rect.height * 0.20, rect.y + rect.height * 0.55))

    sky_h = max(1, int(horizon_y - rect.y))
    ground_h = max(1, int(rect.y + rect.height - horizon_y))
    rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y), int(rect.width), sky_h,
                                 palette.sky_top, palette.sky_horizon)
    rl.draw_rectangle_gradient_v(int(rect.x), int(horizon_y), int(rect.width), ground_h,
                                 palette.ground_horizon, palette.ground_near)
    if road is None or road_opacity <= 0.0:
      return

    draw_polygon(
      rect,
      road,
      gradient=Gradient(
        start=(0.0, 1.0),
        end=(0.0, 0.0),
        colors=[
          color_with_opacity(palette.road_surface, road_opacity * 0.78),
          color_with_opacity(palette.road_surface, road_opacity),
        ],
        stops=[0.0, 1.0],
      ),
    )

    half = len(road) // 2
    if half >= 2:
      left = road[:half]
      right = road[half:][::-1]
      for edge in (left, right):
        for start, end in zip(edge[:-1], edge[1:], strict=True):
          shoulder = color_with_opacity(palette.road_shoulder, road_opacity)
          rl.draw_line_ex(rl.Vector2(float(start[0]), float(start[1])),
                          rl.Vector2(float(end[0]), float(end[1])), 3.0, shoulder)

  def _draw_lead_vehicle(self, cx: float, base_y: float, width: float,
                         opacity: float = 1.0) -> None:
    """Draw the road-level sedan while preserving distance-based scaling."""
    height = lead_actor_height(width)

    def fade(color: rl.Color) -> rl.Color:
      return color_with_opacity(color, opacity)

    shadow = fade(blend_color(rl.Color(0, 0, 0, 38), rl.Color(0, 0, 0, 48), self._dark_fraction))

    # The PNG's tire contact pixels end about 2% above its destination bottom.
    # Keep this subtle ellipse tucked behind that contact patch so the actor
    # reads as planted on the road instead of floating above a separate oval.
    rl.draw_ellipse(int(cx), int(base_y + height * LEAD_SHADOW_CENTER_Y_FRACTION),
                    max(1, int(width * LEAD_SHADOW_RADIUS_X_FRACTION)),
                    max(1, int(height * LEAD_SHADOW_RADIUS_Y_FRACTION)),
                    shadow)

    if self._lead_sedan_texture is None:
      self._lead_sedan_texture = gui_app.texture(LEAD_SPRITE_ASSET)
    texture = self._lead_sedan_texture
    source = rl.Rectangle(0, 0, texture.width, texture.height)
    destination = rl.Rectangle(cx, base_y, width, height)
    origin = rl.Vector2(width / 2.0, height)
    rl.draw_texture_pro(
      texture, source, destination, origin, 0.0,
      color_with_opacity(rl.Color(255, 255, 255, 255), opacity),
    )

  def _projected_lane_width(self, rect: rl.Rectangle, model_renderer,
                            d_rel: float, y_rel: float) -> float | None:
    half_lane = NOMINAL_LANE_WIDTH_M / 2.0
    left = self._project(model_renderer, rect, d_rel, y_rel - half_lane)
    right = self._project(model_renderer, rect, d_rel, y_rel + half_lane)
    if left is None or right is None:
      return None
    width = abs(float(right[0]) - float(left[0]))
    return width if np.isfinite(width) and width > 0.0 else None

  def render_traffic(self, rect: rl.Rectangle, model_renderer) -> None:
    # The comma 4 keeps BluePilot's compact stock lead/radar presentation.
    if not self.show_lead_vehicle:
      return

    sm = ui_state.sm
    candidate = None
    generation = None
    if sm.alive.get("radarState", False) and sm.valid.get("radarState", False):
      radar_state = sm["radarState"]
      raw_values = valid_primary_lead_values(radar_state.leadOne)
      if raw_values is not None:
        candidate = model_renderer.smoothed_primary_lead() or raw_values
        generation = model_renderer.primary_lead_generation()

    fade_step = 1.0 / max(1.0, float(gui_app.target_fps) * LEAD_FADE_SECONDS)
    display_values, opacity = self._lead_fade.update(candidate, generation, fade_step)
    if display_values is None or opacity <= 0.0:
      return

    d_rel, y_rel, _ = display_values
    if not all(np.isfinite(value) for value in (d_rel, y_rel)):
      return

    point = self._project(model_renderer, rect, d_rel, y_rel)
    if point is None:
      return
    cx, base_y = point
    lane_width = self._projected_lane_width(rect, model_renderer, d_rel, y_rel)
    if lane_width is None:
      return
    width = lead_actor_width(lane_width, rect.height, d_rel)
    height = lead_actor_height(width)
    if not (rect.x - width <= cx <= rect.x + rect.width + width and
            rect.y - height <= base_y):
      return
    base_y = lead_actor_base_y(base_y, width, rect)
    self._draw_lead_vehicle(cx, base_y, width, opacity)
