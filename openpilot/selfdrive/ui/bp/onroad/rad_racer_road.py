"""
BluePilot: Rad Racer 8-bit road drawing, shared between the TICI and MICI model renderers.

Extracted verbatim from selfdrive/ui/bp/onroad/model_renderer_bp.py so the comma four
(MICI) renderer can reuse the exact same green game road without duplicating it. The
mixin only touches the attribute contract both ModelRenderer families share:
_road_edges, _road_edge_stds, _lane_lines, _lane_line_probs, _path, _rect,
_map_to_screen, and the host-managed _dash_phase (advanced with vEgo by the host class).
"""
import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.onroad.model_renderer import MAX_DRAW_DISTANCE
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.shader_polygon import draw_polygon

RAD_RACER_ROAD_EDGE_WIDTH = 0.24       # solid outer lines (meters, half-width)
RAD_RACER_DASH_HALF_WIDTH = 0.09       # dashed lane line quad half-width (meters)
RAD_RACER_DASH_LEN_M = 4.0
RAD_RACER_GAP_LEN_M = 4.0
RAD_RACER_CURB_LEN_M = 3.0             # length of each red/white curb block (meters)
RAD_RACER_GREEN = rl.Color(72, 216, 0, 255)
RAD_RACER_DIM = rl.Color(110, 110, 110, 255)  # road color when disengaged
RAD_RACER_PATH_SHADE = rl.Color(150, 255, 120, 110)      # planned path fill when engaged
RAD_RACER_PATH_SHADE_DIM = rl.Color(120, 120, 120, 80)   # planned path fill when disengaged
RAD_RACER_CURB_RED = rl.Color(255, 48, 48, 255)
RAD_RACER_CURB_WHITE = rl.Color(255, 255, 255, 255)


class RadRacerRoadMixin:
  """8-bit road rendering for ModelRenderer subclasses (see module docstring)."""

  def _rad_racer_road_color(self) -> rl.Color:
    """Green when lateral control is engaged, dim gray otherwise (safety cue)."""
    if ui_state.status in (UIStatus.ENGAGED, UIStatus.OVERRIDE):
      return RAD_RACER_GREEN
    return RAD_RACER_DIM

  def _draw_rad_racer_road(self):
    """8-bit road: red/white curbs, light green planned path ribbon, dashed lane lines."""
    color = self._rad_racer_road_color()

    # Race-track curbs on the outer road edges
    for i, road_edge in enumerate(self._road_edges):
      if road_edge.raw_points.shape[0] < 2:
        continue
      edge_alpha = np.clip(1.0 - self._road_edge_stds[i], 0.3, 1.0)
      self._draw_curbed_line(road_edge.raw_points, RAD_RACER_ROAD_EDGE_WIDTH, edge_alpha)

    # Planned path ribbon — shaded fill so it reads clearly vs dashed lane lines
    if self._path.projected_points.size > 0:
      engaged = ui_state.status in (UIStatus.ENGAGED, UIStatus.OVERRIDE)
      shade = RAD_RACER_PATH_SHADE if engaged else RAD_RACER_PATH_SHADE_DIM
      draw_polygon(self._rect, self._path.projected_points, shade)

    # Dashed lane lines
    for i, lane_line in enumerate(self._lane_lines):
      if self._lane_line_probs[i] < 0.4:
        continue
      self._draw_dashed_line(lane_line.raw_points, RAD_RACER_DASH_HALF_WIDTH, color, z_off=0.0)

  def _draw_curbed_line(self, raw_points: np.ndarray, half_width: float, alpha: float = 1.0, z_off: float = 0.0):
    """Draw a polyline as solid alternating red/white curb blocks (race-track edge)."""
    if raw_points.shape[0] < 2:
      return

    xs = raw_points[:, 0]
    x_min = max(float(xs[0]), 1.0)
    x_max = min(float(xs[-1]), MAX_DRAW_DISTANCE)
    if x_max <= x_min:
      return

    red = rl.Color(RAD_RACER_CURB_RED.r, RAD_RACER_CURB_RED.g, RAD_RACER_CURB_RED.b, int(255 * alpha))
    white = rl.Color(RAD_RACER_CURB_WHITE.r, RAD_RACER_CURB_WHITE.g, RAD_RACER_CURB_WHITE.b, int(255 * alpha))
    period = 2 * RAD_RACER_CURB_LEN_M
    start = x_min - ((x_min + self._dash_phase) % period)
    s = start
    while s < x_max:
      for offset, color in ((0.0, red), (RAD_RACER_CURB_LEN_M, white)):
        x0 = max(s + offset, x_min)
        x1 = min(s + offset + RAD_RACER_CURB_LEN_M, x_max)
        if x1 <= x0:
          continue
        self._draw_road_ribbon_quad(raw_points, x0, x1, half_width, color, z_off)
      s += period

  def _draw_road_ribbon_quad(
    self, raw_points: np.ndarray, x0: float, x1: float, half_width: float, color: rl.Color, z_off: float = 0.0,
  ):
    """Project one ribbon segment between x0 and x1 into screen space and fill it."""
    xs = raw_points[:, 0]
    y0 = float(np.interp(x0, xs, raw_points[:, 1]))
    y1 = float(np.interp(x1, xs, raw_points[:, 1]))
    z0 = float(np.interp(x0, xs, raw_points[:, 2])) + z_off
    z1 = float(np.interp(x1, xs, raw_points[:, 2])) + z_off

    p_near_l = self._map_to_screen(x0, y0 - half_width, z0)
    p_near_r = self._map_to_screen(x0, y0 + half_width, z0)
    p_far_r = self._map_to_screen(x1, y1 + half_width, z1)
    p_far_l = self._map_to_screen(x1, y1 - half_width, z1)
    if None in (p_near_l, p_near_r, p_far_r, p_far_l):
      return

    pts = [p_near_l, p_near_r, p_far_r, p_far_l]
    area2 = sum(pts[i][0] * pts[(i + 1) % 4][1] - pts[(i + 1) % 4][0] * pts[i][1] for i in range(4))
    if area2 > 0:
      pts.reverse()
    quad = [rl.Vector2(p[0], p[1]) for p in pts]
    rl.draw_triangle_fan(quad, len(quad), color)

  def _draw_dashed_line(self, raw_points: np.ndarray, half_width: float, color: rl.Color, z_off: float = 0.0):
    """Draw a polyline as chunky scrolling dashes. Each dash is one projected quad."""
    if raw_points.shape[0] < 2:
      return

    xs = raw_points[:, 0]
    x_min = max(float(xs[0]), 1.0)
    x_max = min(float(xs[-1]), MAX_DRAW_DISTANCE)
    if x_max <= x_min:
      return

    period = RAD_RACER_DASH_LEN_M + RAD_RACER_GAP_LEN_M
    # Dashes scroll toward the viewer as the car moves (phase advances with distance traveled)
    start = x_min - ((x_min + self._dash_phase) % period)
    s = start
    while s < x_max:
      x0 = max(s, x_min)
      x1 = min(s + RAD_RACER_DASH_LEN_M, x_max)
      s += period
      if x1 <= x0:
        continue
      self._draw_road_ribbon_quad(raw_points, x0, x1, half_width, color, z_off)
