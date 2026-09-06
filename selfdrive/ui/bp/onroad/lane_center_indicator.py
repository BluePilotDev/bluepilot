"""
BluePilot: draws where the lane-centering logic thinks the lane center is, at the bottom of the
planned path.

The marker is the *target* the lane positioning code aims for -- not the raw laneline midpoint --
so it carries every term that feeds the controller: the model-position / laneline-center
confidence blend, the user's in-lane offset, and the temporary wheel-nudge offset riding on top of
it (``lane_offset_nudge.py``, read live off ``controllerStateBP``). It draws solid only when the
trim can actually act on that target -- lateral engaged, gain and speed ramp non-zero -- and faded
otherwise. Both control modes are covered, each mirroring its own controller:

- angle mode: ``lane_center_trim.py`` (curvature-domain trim). Its confidence blend is reused
  directly from that module rather than re-derived here, so the marker can't drift away from what
  the controller actually does.
- curvature mode: ``lateral_curv_ext.py``'s ``path_offset`` blend. That math lives inline in the
  controller's update loop with no reusable entry point, so it is restated here -- keep the two in
  sync if the controller's blend changes.

Nothing here feeds control; it is display only. It renders whenever lane positioning is enabled
for the active mode, so there is no separate toggle to forget.
"""
import numpy as np
import pyray as rl
from numpy import interp

from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient

from opendbc.sunnypilot.car.ford.lateral_curv_ext import PrimaryLateralControl
from opendbc.sunnypilot.car.ford.lane_center_trim import LaneCenterTrim, _SPEED_RAMP_BP, _SPEED_RAMP_V
from opendbc.sunnypilot.car.ford.lane_offset_nudge import _TOTAL_OFFSET_MAX_M
from openpilot.selfdrive.modeld.constants import ModelConstants

# Curvature-mode blend constants, mirrored from lateral_curv_ext.py (see module docstring).
_CURV_LOOKUP_TIME_S = 0.2
_CURV_WIDTH_TOLERANCE_BP = (3.75, 4.25)
_CURV_WIDTH_TOLERANCE_V = (0.81, 0.59)
_CURV_CONFIDENCE_BP = (0.6, 0.8)
_CURV_SPEED_BP = (0.0, 9.0, 15.0)
_CURV_SPEED_V = (0.0, 0.0, 1.0)

# Distances (m ahead) tried for the marker, nearest first -- the nearest one that projects inside
# the clip region wins. Starts at 10 m rather than right at the bumper: nearer than that the marker
# lands under the BP bottom overlays (torque bar, hybrid gauge) and off the bottom of the frame.
# The MICI road view is a fraction of the height, so the same overlays reach much further up it --
# hence the farther set, which puts the marker above them.
_MARKER_DISTANCES_M = (10.0, 14.0, 20.0, 28.0)
_MARKER_DISTANCES_COMPACT_M = (20.0, 26.0, 34.0, 44.0)

# The nudge shadow: a translucent strip laid down the road at the nudged center, fading out with
# distance so it reads as a shadow on the lane rather than a second path.
_SHADOW_HALF_W_M = 0.22
_SHADOW_NEAR_M = 5.0  # starts under the bottom overlays so the strip has no visible cut edge
_SHADOW_FAR_M = 40.0
_SHADOW_SAMPLES = 9
_SHADOW_MIN_M = 0.02  # below this there is no nudge worth drawing

_TARGET_COLOR = rl.Color(0, 220, 255, 235)
_TARGET_GLOW = rl.Color(0, 220, 255, 70)
_PATH_TICK_COLOR = rl.Color(255, 255, 255, 170)
_STEM_H = 44.0  # px from the anchor point up to the diamond
_REF_VIEW_H = 790.0  # road view height the pixel sizes above were tuned against (TICI)
_COMPACT_VIEW_H = 400.0  # below this the view is a MICI-sized screen

_PARAM_REFRESH_FRAMES = 60

# The trim only runs with lateral engaged, so the marker fades when it isn't -- it still shows the
# computed target (useful while tuning before engaging), just not as a live command.
_INACTIVE_ALPHA = 0.45


class LaneCenterIndicatorMixin:
  """Lane-center target marker for the BP ModelRenderer families.

  Host contract (both TICI and MICI renderers already satisfy it): ``_bp_params``, ``_path``,
  ``_camera_offset``, ``_path_offset_z``, ``_rect``, ``_map_to_screen``.
  """

  def _init_lane_center_indicator(self) -> None:
    self._lc_trim = LaneCenterTrim()  # reused for its confidence blend only, never for control
    self._lc_frame = 0
    self._lc_enabled = False
    self._lc_is_angle = True
    self._lc_offset = 0.0
    self._lc_nudge_m = 0.0  # live wheel-nudge offset from controllerStateBP
    self._lc_gain = 0.0
    self._lc_lane_full = False
    self._refresh_lane_center_params()

  def _refresh_lane_center_params(self) -> None:
    p = self._bp_params
    try:
      mode = PrimaryLateralControl(p.get("FordPrefLateralControl", return_default=True) or 0)
    except (TypeError, ValueError):
      mode = PrimaryLateralControl.curvature
    self._lc_is_angle = mode == PrimaryLateralControl.angle

    try:
      if self._lc_is_angle:
        self._lc_enabled = p.get_bool("enable_lane_positioning_ang")
        self._lc_offset = float(p.get("custom_path_offset_ang", return_default=True))
        self._lc_gain = float(p.get("lane_centering_strength_ang", return_default=True))
      else:
        self._lc_enabled = p.get_bool("enable_lane_positioning_curv")
        self._lc_offset = float(p.get("custom_path_offset_curv", return_default=True))
        self._lc_gain = float(p.get("LC_PID_gain_UI_curv", return_default=True)) / 100.0
        self._lc_lane_full = p.get_bool("enable_lane_full_mode_curv")
    except (TypeError, ValueError):
      self._lc_enabled = False

  def _draw_lane_center_indicator(self, sm) -> None:
    self._lc_frame += 1
    if self._lc_frame % _PARAM_REFRESH_FRAMES == 0:
      self._refresh_lane_center_params()

    if not self._lc_enabled or self._path.raw_points.shape[0] < 2:
      return

    model = sm['modelV2']
    v_ego = sm['carState'].vEgo if sm.valid['carState'] else 0.0
    # The wheel-nudge offset is runtime state in the car process, not a param -- without it the
    # marker would ignore an offset the driver just set (lane_offset_nudge.py).
    self._lc_nudge_m = float(sm['controllerStateBP'].nudgeLaneOffsetMeters) if sm.valid['controllerStateBP'] else 0.0

    # Distance first: the target is evaluated at whatever distance the marker ends up drawn at,
    # so a curving lane can't put the marker beside a different piece of road than it describes.
    anchor = self._marker_anchor()
    if anchor is None:
      return
    marker_x, path_pt, z = anchor

    solution = self._lane_center_solution(model, float(v_ego), marker_x)
    if solution is None:
      return
    target_y, model_y, authority = solution

    # The marker is only a live command when the trim can actually act: lateral engaged, and the
    # gain and speed ramp both non-zero. Otherwise it fades to a preview of where the target is.
    active = authority > 0.0 and bool(sm.valid['carControl'] and sm['carControl'].latActive)

    target_pt = self._map_to_screen(marker_x, target_y + self._camera_offset, z)
    if target_pt is None:
      return

    off_x, off_y = self._lc_screen_offset()
    target_pt = (target_pt[0] + off_x, target_pt[1] + off_y)
    path_pt = (path_pt[0] + off_x, path_pt[1] + off_y)

    # Shadow first so the marker sits on top of it.
    if abs(self._lc_nudge_m) >= _SHADOW_MIN_M:
      self._draw_nudge_shadow(target_y - model_y, active)
    self._draw_marker(target_pt, path_pt, active)

  def _lane_center_solution(self, model, v_ego: float, marker_x: float):
    """Returns (target_y, model_y, authority) or None.

    ``target_y`` and ``model_y`` are model frame (positive right, camera offset not yet applied) at
    the marker's own distance -- what the trim steers toward, and where the model's own path runs.
    Their difference is the lateral shift the shadow is drawn at. ``authority`` is the gain x
    speed-ramp product: zero means the trim is computing a target it cannot act on yet.
    """
    try:
      pos_x = np.asarray(model.position.x, dtype=float)
      pos_y = np.asarray(model.position.y, dtype=float)
      if pos_x.size < 2 or pos_x.size != pos_y.size or not np.all(np.diff(pos_x) > 0):
        return None
      if not (np.isfinite(pos_x).all() and np.isfinite(pos_y).all()):
        return None
    except (AttributeError, TypeError, ValueError):
      return None

    if not (np.isfinite(self._lc_offset) and np.isfinite(self._lc_gain) and np.isfinite(self._lc_nudge_m)):
      return None

    # Menu offset + wheel nudge, clipped the same way the controller clips the pair.
    offset = float(np.clip(self._lc_offset + self._lc_nudge_m, -_TOTAL_OFFSET_MAX_M, _TOTAL_OFFSET_MAX_M))

    if self._lc_is_angle:
      # Angle mode: the confidence blend comes straight from the controller's own module, but is
      # evaluated at the marker distance so the drawn position tracks the lane the driver can see
      # there rather than the controller's lookahead sample point.
      speed_factor = float(interp(v_ego, _SPEED_RAMP_BP, _SPEED_RAMP_V))
      scale, center_near = self._lc_trim._laneline_blend(model, marker_x)
      model_y_near = float(np.interp(marker_x, pos_x, pos_y))
      target_y = model_y_near * (1.0 - scale) + center_near * scale + offset
      return target_y, model_y_near, float(np.clip(self._lc_gain, 0.0, 1.0)) * speed_factor

    # Curvature mode: path_offset is already a near-field lateral position, so it *is* the target.
    blend = self._curv_blend(model, offset)
    if blend is None:
      return None
    _, path_offset = blend
    speed_factor = float(interp(v_ego, _CURV_SPEED_BP, _CURV_SPEED_V))
    model_y = float(np.interp(marker_x, pos_x, pos_y))
    return path_offset, model_y, float(np.clip(self._lc_gain, 0.0, 10.0)) * speed_factor

  def _curv_blend(self, model, offset: float):
    """Curvature mode's path_offset: (confidence_scale, path_offset_m). Mirrors lateral_curv_ext."""
    try:
      position_y = interp(_CURV_LOOKUP_TIME_S, ModelConstants.T_IDXS, model.position.y)
      left_y = float(model.laneLines[1].y[0])
      right_y = float(model.laneLines[2].y[0])
      lanelines_y = (left_y + right_y) / 2

      width_tolerance = float(interp(right_y - left_y, _CURV_WIDTH_TOLERANCE_BP, _CURV_WIDTH_TOLERANCE_V))
      confidence = min(float(model.laneLineProbs[1]), float(model.laneLineProbs[2]), width_tolerance)
      if not self._lc_lane_full:
        confidence = 0.0
      scale = float(np.clip(interp(confidence, _CURV_CONFIDENCE_BP, (0.0, 1.0)), 0.0, 1.0))
      path_offset = float(position_y) * (1 - scale) + lanelines_y * scale + offset
      return scale, path_offset
    except (AttributeError, IndexError, TypeError, ValueError):
      return None

  def _lc_screen_offset(self) -> tuple[float, float]:
    """Screen-space origin for drawing. The MICI renderer projects rect-relative, TICI absolute."""
    return 0.0, 0.0

  def _marker_anchor(self):
    """Nearest in-frame marker distance: (x, path center screen point, world z at x)."""
    path_x = self._path.raw_points[:, 0]
    path_y = self._path.raw_points[:, 1]
    path_z = self._path.raw_points[:, 2]

    distances = _MARKER_DISTANCES_COMPACT_M if self._rect.height < _COMPACT_VIEW_H else _MARKER_DISTANCES_M
    for x in distances:
      z = float(np.interp(x, path_x, path_z)) + self._path_offset_z
      path_pt = self._map_to_screen(x, float(np.interp(x, path_x, path_y)), z)
      if path_pt is not None:
        return x, path_pt, z
    return None

  def _draw_nudge_shadow(self, delta_y: float, active: bool) -> None:
    """Translucent strip down the road at the nudged center, so the offset the driver just set is
    visible as a position on the lane and not only as a marker."""
    path_x = self._path.raw_points[:, 0]
    path_y = self._path.raw_points[:, 1]
    path_z = self._path.raw_points[:, 2]
    off_x, off_y = self._lc_screen_offset()

    left, right = [], []
    for x in np.linspace(_SHADOW_NEAR_M, _SHADOW_FAR_M, _SHADOW_SAMPLES):
      x = float(x)
      y = float(np.interp(x, path_x, path_y)) + delta_y + self._camera_offset
      z = float(np.interp(x, path_x, path_z)) + self._path_offset_z
      pl = self._map_to_screen(x, y - _SHADOW_HALF_W_M, z)
      pr = self._map_to_screen(x, y + _SHADOW_HALF_W_M, z)
      if pl is None or pr is None:
        continue
      left.append((pl[0] + off_x, pl[1] + off_y))
      right.append((pr[0] + off_x, pr[1] + off_y))

    if len(left) < 2:
      return

    # draw_polygon wants the ribbon as [L0..Lk, Rk..R0]
    poly = np.array(left + right[::-1], dtype=np.float32)
    alpha = 1.0 if active else _INACTIVE_ALPHA
    gradient = Gradient(
      start=(0.0, 1.0),  # near end of the strip
      end=(0.0, 0.0),    # far end, faded out
      colors=[rl.Color(0, 220, 255, int(105 * alpha)),
              rl.Color(0, 220, 255, int(60 * alpha)),
              rl.Color(0, 220, 255, 0)],
      stops=[0.0, 0.5, 1.0],
    )
    draw_polygon(self._rect, poly, gradient=gradient)

  @staticmethod
  def _fade(color: rl.Color, active: bool) -> rl.Color:
    if active:
      return color
    return rl.Color(color.r, color.g, color.b, int(color.a * _INACTIVE_ALPHA))

  def _scale(self) -> float:
    """Marker/readout sizes are in TICI pixels; the MICI road view is much smaller."""
    return float(np.clip(self._rect.height / _REF_VIEW_H, 0.4, 1.0))

  def _draw_marker(self, target_pt, path_pt, active: bool) -> None:
    target_color = self._fade(_TARGET_COLOR, active)
    sc = self._scale()
    stem = _STEM_H * sc
    tx, ty = float(target_pt[0]), float(target_pt[1])
    px, py = float(path_pt[0]), float(path_pt[1])

    # Error bar: how far the target sits from where the path currently runs
    rl.draw_line_ex(rl.Vector2(px, py), rl.Vector2(tx, ty), 4.0 * sc, self._fade(_TARGET_GLOW, active))
    rl.draw_line_ex(rl.Vector2(px, py - 12.0 * sc), rl.Vector2(px, py + 12.0 * sc), 3.0 * sc,
                    self._fade(_PATH_TICK_COLOR, active))

    # Target: vertical stem topped with a diamond, so it reads as a position, not a lane line
    rl.draw_line_ex(rl.Vector2(tx, ty + 16.0 * sc), rl.Vector2(tx, ty - stem), 6.0 * sc, target_color)
    # Winding matches the lead chevron's (right -> top -> left), which is the order raylib's
    # backface culling wants here -- reversed, the diamond silently doesn't draw.
    half = 16.0 * sc
    rl.draw_triangle_fan([(tx + half, ty - stem), (tx, ty - stem - half),
                          (tx - half, ty - stem), (tx, ty - stem + half)], 4, target_color)
