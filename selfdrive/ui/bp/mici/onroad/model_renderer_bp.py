import numpy as np
import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.mici.onroad.model_renderer import ModelRenderer, THROTTLE_COLORS, NO_THROTTLE_COLORS, CLIP_MARGIN
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
# BluePilot: Rainbow shader moved to BP module after upstream removal
from openpilot.bluepilot.ui.lib.bp_shaders import draw_rainbow_polygon
# BluePilot: Rad Racer 8-bit road, shared with the TICI renderer
from openpilot.selfdrive.ui.bp.onroad.rad_racer_road import RadRacerRoadMixin, RAD_RACER_DASH_LEN_M, RAD_RACER_GAP_LEN_M
# BluePilot: seasonal theme packs (colors.json overrides for road colors)
from openpilot.selfdrive.ui.bp.lib import theme_pack

class ModelRendererBP(RadRacerRoadMixin, ModelRenderer):
  def __init__(self):
    super().__init__()
    self._bp_params = Params()
    self._rainbow_v = 20
    self._disable_lane_line_status_color = self._bp_params.get_bool("BPDisableLaneLineStatusColor")
    self._rainbow_lane_lines = self._bp_params.get_bool("BPRainbowLines")
    # BluePilot: Rad Racer 8-bit theme (green game road; dash scroll animation state)
    self._rad_racer = theme_pack.rad_racer_active(self._bp_params)
    self._dash_phase = 0.0
    # BluePilot: seasonal theme pack (None when disabled)
    self._theme_pack = theme_pack.get_active_pack(force=True)

  def prepare_projection(self, rect: rl.Rectangle) -> None:
    """Set clip region so _map_to_screen works before render().

    Rad Racer draws skyline/signs behind the road and calls _map_to_screen during
    that background pass, before the base render() initializes the clip rect.
    """
    self._clip_region = rl.Rectangle(
      rect.x - CLIP_MARGIN, rect.y - CLIP_MARGIN, rect.width + 2 * CLIP_MARGIN, rect.height + 2 * CLIP_MARGIN
    )

  def _update_state(self):
    super()._update_state()
    sm = ui_state.sm

    if self._counter % 60 == 0:
      self._disable_lane_line_status_color = self._bp_params.get_bool("BPDisableLaneLineStatusColor")
      self._rainbow_lane_lines = self._bp_params.get_bool("BPRainbowLines")
      self._rad_racer = theme_pack.rad_racer_active(self._bp_params)
      self._theme_pack = theme_pack.get_active_pack()

    if ui_state.rainbow_path or self._rainbow_lane_lines:
      v = sm['carState'].vEgo
      self._rainbow_v = np.clip(v, 2.5, 35) / 30

    # BluePilot: Advance dash scroll animation for the Rad Racer road
    if self._rad_racer and sm.valid.get('carState', False):
      period = RAD_RACER_DASH_LEN_M + RAD_RACER_GAP_LEN_M
      self._dash_phase = (self._dash_phase + max(0.0, sm['carState'].vEgo) / gui_app.target_fps) % period

  def _draw_path(self, sm):
    # BluePilot: Rad Racer theme draws the path ribbon in _draw_rad_racer_road
    if self._rad_racer:
      return
    if ui_state.rainbow_path:
      draw_rainbow_polygon(self._rect, self._path.projected_points, rainbow_v=self._rainbow_v)
    elif (themed_gradient := self._themed_path_gradient()) is not None:
      # BluePilot: theme pack path colors replace the throttle/no-throttle gradient
      if not self._path.projected_points.size:
        return
      path_pts = self._path.projected_points + np.array([self._rect.x, self._rect.y], dtype=np.float32)
      if ui_state.status == UIStatus.DISENGAGED:
        draw_polygon(self._rect, path_pts, rl.Color(0, 0, 0, 90))
      else:
        draw_polygon(self._rect, path_pts, gradient=themed_gradient)
    else:
      super()._draw_path(sm)

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

  def _get_ll_color(self, prob: float, adjacent: bool, left: bool):
    """BluePilot: theme pack lane color with upstream's confidence-based alpha (disengaged stays black)."""
    if self._theme_pack is not None and ui_state.status != UIStatus.DISENGAGED:
      pack_color = self._theme_pack.rl_colors().get("LaneLines")
      if pack_color is not None:
        alpha = float(np.clip(prob, 0.0, 0.7)) * (pack_color.a / 255.0)
        return rl.Color(pack_color.r, pack_color.g, pack_color.b, int(alpha * 255))
    return super()._get_ll_color(prob, adjacent, left)

  def _draw_lane_lines(self):
    """Draw lane lines and road edges, with optional rainbow inner lane lines."""
    # BluePilot: Rad Racer theme replaces all road rendering with the 8-bit game road
    if self._rad_racer:
      self._draw_rad_racer_road()
      return
    offset = np.array([self._rect.x, self._rect.y], dtype=np.float32)
    rainbow_lane_lines_active = self._rainbow_lane_lines_active(ui_state.sm)

    for i, lane_line in enumerate(self._lane_lines):
      if lane_line.projected_points.size == 0:
        continue

      points = lane_line.projected_points + offset
      if rainbow_lane_lines_active and i in (1, 2):
        alpha = float(np.clip(self._lane_line_probs[i], 0.0, 0.7))
        draw_rainbow_polygon(self._rect, points, rainbow_v=self._rainbow_v, alpha=alpha)
      else:
        color = self._get_ll_color(float(self._lane_line_probs[i]), i in (1, 2), i in (0, 1))
        draw_polygon(self._rect, points, color)

    for i, road_edge in enumerate(self._road_edges):
      if road_edge.projected_points.size == 0:
        continue

      color = self._get_ll_color(float(1.0 - self._road_edge_stds[i]), float(self._lane_line_probs[i + 1]) < 0.25, i == 0)
      draw_polygon(self._rect, road_edge.projected_points + offset, color)

  def _rainbow_lane_lines_active(self, sm) -> bool:
    if not self._rainbow_lane_lines or self._disable_lane_line_status_color:
      return False

    if sm.valid.get('carControl', False) and sm['carControl'].longActive:
      return True

    return ui_state.status in (UIStatus.ENGAGED, UIStatus.LONG_ONLY)
