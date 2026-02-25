"""
BluePilot Torque Bar Renderer

A standalone torque bar for the BP onroad UI, derived from sunnypilot's TorqueBar
but fully separate for easier maintenance and upstream syncing.

Key differences from upstream:
- Smoother filters (higher time constants) to reduce visual jitter/glitchiness
- Supports gauge_height_offset to position the arc above battery/power flow gauges
- Uses lateralUncertainty from controllerStateBP for angleState vehicles (Tesla etc.)
- Softer color transitions and more refined visual feel
"""
import math
from functools import wraps
from collections import OrderedDict

import numpy as np
import pyray as rl
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad import blend_colors
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.shader_polygon import draw_polygon, Gradient
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

# Arc geometry (legacy arc — kept for MICI / fallback)
TORQUE_ANGLE_SPAN = 12.7

# Crown strip geometry (horizontal torque bar inside shared container)
STRIP_HEIGHT = 11            # Pixels tall for the torque strip
STRIP_TRACK_COLOR = rl.Color(52, 73, 94, 140)    # Subtle dark track (matches power flow BAR_BG_COLOR)
STRIP_DIVIDER_COLOR = rl.Color(100, 100, 100, 80) # 1px separator below strip
STRIP_CENTER_TICK_COLOR = rl.Color(200, 200, 200, 140)
STRIP_CORNER_SEGMENTS = 10

# Arched strip (same data as flat strip, drawn on top of arched powerflow meter)
STRIP_ARCH_THICKNESS = 12
STRIP_ARCH_GAP = 2  # Gap above powerflow outer edge
STRIP_ARCH_TRACK_COLOR = rl.Color(52, 73, 94, 180)


def _quantized_lru_cache(maxsize=128):
  def decorator(func):
    cache = OrderedDict()
    @wraps(func)
    def wrapper(cx, cy, r_mid, thickness, a0_deg, a1_deg, **kwargs):
      key = (round(cx), round(cy), round(r_mid),
             round(thickness),
             round(a0_deg * 10) / 10,
             round(a1_deg * 10) / 10,
             tuple(sorted(kwargs.items())))
      if key in cache:
        cache.move_to_end(key)
      else:
        if len(cache) >= maxsize:
          cache.popitem(last=False)
        result = func(cx, cy, r_mid, thickness, a0_deg, a1_deg, **kwargs)
        cache[key] = result
      return cache[key]
    return wrapper
  return decorator


@_quantized_lru_cache(maxsize=256)
def _arc_bar_pts(cx: float, cy: float,
                 r_mid: float, thickness: float,
                 a0_deg: float, a1_deg: float,
                 *, max_points: int = 100, cap_segs: int = 10,
                 cap_radius: float = 7, px_per_seg: float = 2.0) -> np.ndarray:
  """Return Nx2 np.float32 points for a closed polygon (rounded thick arc).

  Duplicated from upstream torque_bar.arc_bar_pts to keep BP fully independent.
  """
  def get_cap(left: bool, a_deg: float):
    nx, ny = math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))
    tx, ty = -ny, nx
    mx, my = cx + nx * r_mid, cy + ny * r_mid

    ex = mx + nx * (half - cap_radius)
    ey = my + ny * (half - cap_radius)

    if not left:
      alpha = np.deg2rad(np.linspace(90, 0, cap_segs + 2))[1:-1]
    else:
      alpha = np.deg2rad(np.linspace(180, 90, cap_segs + 2))[1:-1]
    cap_end = np.c_[ex + np.cos(alpha) * cap_radius * tx + np.sin(alpha) * cap_radius * nx,
                    ey + np.cos(alpha) * cap_radius * ty + np.sin(alpha) * cap_radius * ny]

    ex2 = mx + nx * (-half + cap_radius)
    ey2 = my + ny * (-half + cap_radius)

    if not left:
      alpha2 = np.deg2rad(np.linspace(0, -90, cap_segs + 1))[:-1]
    else:
      alpha2 = np.deg2rad(np.linspace(90 - 90 - 90, 0 - 90 - 90, cap_segs + 1))[:-1]
    cap_end_bot = np.c_[ex2 + np.cos(alpha2) * cap_radius * tx + np.sin(alpha2) * cap_radius * nx,
                        ey2 + np.cos(alpha2) * cap_radius * ty + np.sin(alpha2) * cap_radius * ny]

    if not left:
      cap_end = np.vstack((cap_end, cap_end_bot))
    else:
      cap_end = np.vstack((cap_end_bot, cap_end))
    return cap_end

  if a1_deg < a0_deg:
    a0_deg, a1_deg = a1_deg, a0_deg
  half = thickness * 0.5
  cap_radius = min(cap_radius, half)
  span = max(1e-3, a1_deg - a0_deg)

  arc_len = r_mid * math.radians(span)
  arc_segs = max(6, int(arc_len / px_per_seg))
  max_arc = (max_points - (4 * cap_segs + 3)) // 2
  arc_segs = max(6, min(arc_segs, max_arc))

  ang_o = np.deg2rad(np.linspace(a0_deg, a1_deg, arc_segs + 1))
  outer = np.c_[cx + np.cos(ang_o) * (r_mid + half),
                cy + np.sin(ang_o) * (r_mid + half)]

  cap_end = get_cap(False, a1_deg)

  ang_i = np.deg2rad(np.linspace(a1_deg, a0_deg, arc_segs + 1))
  inner = np.c_[cx + np.cos(ang_i) * (r_mid - half),
                cy + np.sin(ang_i) * (r_mid - half)]

  cap_start = get_cap(True, a0_deg)

  pts = np.vstack((outer, cap_end, inner, cap_start, outer[:1])).astype(np.float32)
  pts = np.roll(pts, cap_segs, axis=0)
  return pts


class TorqueBarRendererBP:
  """BluePilot torque bar renderer — smoother, repositionable, independent of upstream.

  This is NOT a Widget subclass. It's rendered explicitly by the augmented road view
  so we have full control over when and where it draws relative to gauges and alerts.
  """

  def __init__(self, scale: float = 3.0):
    self._scale = scale
    # Smoother filters: higher rc = slower response = less jittery
    # Upstream uses rc=0.1 for both; we use 0.2/0.15 for a calmer feel
    self._torque_filter = FirstOrderFilter(0.0, 0.2, 1.0 / gui_app.target_fps)
    self._alpha_filter = FirstOrderFilter(0.0, 0.15, 1.0 / gui_app.target_fps)

  def update(self):
    """Update torque state from car messages. Call once per frame."""
    # BluePilot: Use lateral uncertainty from controllerStateBP on angleState vehicles
    try:
      if ui_state.sm['controlsState'].lateralControlState.which() == 'angleState':
        if ui_state.sm.valid.get("controllerStateBP", False):
          try:
            lateral_uncertainty = ui_state.sm['controllerStateBP'].lateralUncertainty
            self._torque_filter.update(min(max(lateral_uncertainty, -1.0), 1.0))
            self._update_alpha()
            return
          except (KeyError, AttributeError):
            pass

        # angleState fallback: acceleration-based
        controls_state = ui_state.sm['controlsState']
        car_state = ui_state.sm['carState']
        live_parameters = ui_state.sm['liveParameters']
        lateral_acceleration = controls_state.curvature * car_state.vEgo ** 2 - live_parameters.roll * ACCELERATION_DUE_TO_GRAVITY
        max_lateral_acceleration = 3
        actual_lateral_accel = controls_state.curvature * car_state.vEgo ** 2
        desired_lateral_accel = controls_state.desiredCurvature * car_state.vEgo ** 2
        accel_diff = desired_lateral_accel - actual_lateral_accel
        self._torque_filter.update(min(max(lateral_acceleration / max_lateral_acceleration + accel_diff, -1.0), 1.0))
      else:
        # Non-angleState: use actuator torque output
        self._torque_filter.update(-ui_state.sm['carOutput'].actuatorsOutput.torque)
    except (KeyError, AttributeError):
      pass

    self._update_alpha()

  def _update_alpha(self):
    """Update visibility alpha based on engagement status."""
    self._alpha_filter.update(ui_state.status not in (UIStatus.DISENGAGED, UIStatus.LONG_ONLY))
    bp_ui_log.state("TorqueBar", "alpha", round(self._alpha_filter.x, 2))
    bp_ui_log.state("TorqueBar", "ui_status", ui_state.status.name)

  def render(self, rect: rl.Rectangle, gauge_height_offset: float = 0.0):
    """Render the torque bar arc.

    Args:
        rect: The UI rect to position the arc within.
        gauge_height_offset: Pixels to subtract from rect height to push the arc above gauges.
    """
    if not ui_state.torque_bar:
      return

    # Shrink effective rect to push arc above the gauge area
    effective_rect = rect
    if gauge_height_offset > 0:
      effective_rect = rl.Rectangle(rect.x, rect.y, rect.width, rect.height - gauge_height_offset)

    torque = self._torque_filter.x
    alpha = self._alpha_filter.x

    if alpha < 0.01:
      return

    abs_torque = abs(torque)

    # Arc geometry — offset/height scale with torque magnitude
    # BluePilot: Reduced max height vs upstream (56→28) for a subtler, less exaggerated look
    # at high torque. The bar grows slightly thicker but doesn't balloon.
    torque_line_offset = np.interp(abs_torque, [0.5, 1.0], [22 * self._scale, 26 * self._scale])
    torque_line_height = np.interp(abs_torque, [0.5, 1.0], [14 * self._scale, 28 * self._scale])

    # Background alpha varies with torque magnitude
    bg_alpha = np.interp(abs_torque, [0.5, 1.0], [0.25, 0.5])

    # Colors depend on engagement status
    is_active = ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY)

    if is_active:
      bg_color = rl.Color(255, 255, 255, int(255 * bg_alpha * alpha))
    else:
      bg_color = rl.Color(255, 255, 255, int(255 * 0.15 * alpha))

    # Arc center and radius
    torque_line_radius = 1200 * self._scale
    top_angle = -90
    bg_angle_span = alpha * TORQUE_ANGLE_SPAN
    start_angle = top_angle - bg_angle_span / 2
    end_angle = top_angle + bg_angle_span / 2
    mid_r = torque_line_radius + torque_line_height / 2

    cx = effective_rect.x + effective_rect.width / 2 + 8
    cy = effective_rect.y + effective_rect.height + torque_line_radius - torque_line_offset

    # Background arc
    bg_pts = _arc_bar_pts(cx, cy, mid_r, torque_line_height, start_angle, end_angle,
                          cap_radius=7 * self._scale)
    draw_polygon(effective_rect, bg_pts, color=bg_color)

    # Active torque fill arc
    a0 = top_angle
    a1 = a0 + bg_angle_span / 2 * torque
    fill_pts = _arc_bar_pts(cx, cy, mid_r, torque_line_height, a0, a1,
                            cap_radius=7 * self._scale)

    # Gradient from center to ~65% of the arc width
    start_grad_pt = cx / effective_rect.width
    if torque < 0:
      end_grad_pt = (cx * (1 - 0.65) + (min(bg_pts[:, 0]) * 0.65)) / effective_rect.width
    else:
      end_grad_pt = (cx * (1 - 0.65) + (max(bg_pts[:, 0]) * 0.65)) / effective_rect.width

    if is_active:
      # Smooth color transition: white → yellow/orange at high torque
      high_blend = max(0.0, abs_torque - 0.75) * 4
      start_color = blend_colors(
        rl.Color(255, 255, 255, int(255 * 0.9 * alpha)),
        rl.Color(255, 200, 0, int(255 * alpha)),
        high_blend,
      )
      end_color = blend_colors(
        rl.Color(255, 255, 255, int(255 * 0.9 * alpha)),
        rl.Color(255, 115, 0, int(255 * alpha)),
        high_blend,
      )
    else:
      start_color = end_color = rl.Color(255, 255, 255, int(255 * 0.35 * alpha))

    gradient = Gradient(
      start=(start_grad_pt, 0),
      end=(end_grad_pt, 0),
      colors=[start_color, end_color],
      stops=[0.0, 1.0],
    )
    draw_polygon(effective_rect, fill_pts, gradient=gradient)

    # Center dot (only at low torque)
    if abs_torque < 0.5:
      dot_y = effective_rect.y + effective_rect.height - torque_line_offset - torque_line_height / 2
      rl.draw_circle(int(cx), int(dot_y), 10 // 2 * self._scale,
                     rl.Color(182, 182, 182, int(255 * 0.9 * alpha)))

  def render_strip(self, strip_rect: rl.Rectangle):
    """Render the torque bar as a horizontal crown strip inside a container.

    A thin bidirectional bar that fills from center outward — left for left torque,
    right for right torque. Uses the same white→yellow→orange gradient as the arc.

    Args:
        strip_rect: Rectangle allocated for the strip (full inner width, STRIP_HEIGHT tall).
    """
    if not ui_state.torque_bar:
      return

    torque = self._torque_filter.x
    alpha = self._alpha_filter.x

    if alpha < 0.01:
      return

    abs_torque = abs(torque)
    is_active = ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY)

    # Pill-shaped roundness for the thin strip
    roundness = min(1.0, 14.0 / max(1, strip_rect.height))

    # --- Track background ---
    track_color = rl.Color(STRIP_TRACK_COLOR.r, STRIP_TRACK_COLOR.g,
                           STRIP_TRACK_COLOR.b, int(STRIP_TRACK_COLOR.a * max(alpha, 0.4)))
    rl.draw_rectangle_rounded(strip_rect, roundness, STRIP_CORNER_SEGMENTS, track_color)

    # --- Active fill (bidirectional from center) ---
    if abs_torque > 0.005:
      center_x = strip_rect.x + strip_rect.width / 2
      fill_half_width = strip_rect.width * abs_torque / 2

      # Color: same gradient logic as the arc
      if is_active:
        high_blend = max(0.0, abs_torque - 0.75) * 4
        fill_color = blend_colors(
          rl.Color(255, 255, 255, int(255 * 0.9 * alpha)),
          rl.Color(255, 200, 0, int(255 * alpha)),
          high_blend,
        )
      else:
        fill_color = rl.Color(255, 255, 255, int(255 * 0.35 * alpha))

      # Scissor + overshoot for flat inner edge, rounded outer edge (same as PowerFlowGauge)
      overshoot = strip_rect.height
      if torque < 0:
        # Left fill
        rounded_rect = rl.Rectangle(
          center_x - fill_half_width, strip_rect.y,
          fill_half_width + overshoot, strip_rect.height,
        )
        rl.begin_scissor_mode(
          int(center_x - fill_half_width), int(strip_rect.y),
          int(fill_half_width), int(strip_rect.height),
        )
      else:
        # Right fill
        rounded_rect = rl.Rectangle(
          center_x - overshoot, strip_rect.y,
          fill_half_width + overshoot, strip_rect.height,
        )
        rl.begin_scissor_mode(
          int(center_x), int(strip_rect.y),
          int(fill_half_width), int(strip_rect.height),
        )

      if rounded_rect.width > 0:
        rl.draw_rectangle_rounded(rounded_rect, roundness, STRIP_CORNER_SEGMENTS, fill_color)
      rl.end_scissor_mode()

    # --- Center tick ---
    center_x = strip_rect.x + strip_rect.width / 2
    rl.draw_line_ex(
      rl.Vector2(center_x, strip_rect.y + 1),
      rl.Vector2(center_x, strip_rect.y + strip_rect.height - 1),
      1.5, rl.Color(STRIP_CENTER_TICK_COLOR.r, STRIP_CENTER_TICK_COLOR.g,
                     STRIP_CENTER_TICK_COLOR.b, int(STRIP_CENTER_TICK_COLOR.a * alpha)),
    )

    # --- Bottom divider line ---
    divider_y = strip_rect.y + strip_rect.height
    rl.draw_line_ex(
      rl.Vector2(strip_rect.x + 4, divider_y),
      rl.Vector2(strip_rect.x + strip_rect.width - 4, divider_y),
      1.0, rl.Color(STRIP_DIVIDER_COLOR.r, STRIP_DIVIDER_COLOR.g,
                     STRIP_DIVIDER_COLOR.b, int(STRIP_DIVIDER_COLOR.a * alpha)),
    )

  def render_strip_arched(self, rect: rl.Rectangle, cx: float, cy: float,
                         top_angle: float, start_angle: float, end_angle: float,
                         powerflow_outer_radius: float,
                         fill_center_angle: float = None,
                         scale: float = 1.0) -> None:
    """Render the same steering/torque strip as an arc above the arched powerflow meter.

    Uses the same torque/alpha state as the flat strip (updated by update() each frame).
    fill_center_angle: center of positive/negative fill (default top_angle). Use when combo
    strip spans battery+powerflow to shift center left by one tick (e.g. top_angle - 1.5°).
    scale: 0.75 for small arched gauge, 1.0 for large.
    """
    if not ui_state.torque_bar:
      return
    torque = self._torque_filter.x
    alpha = self._alpha_filter.x
    if alpha < 0.01:
      return
    center = fill_center_angle if fill_center_angle is not None else top_angle
    strip_thickness = STRIP_ARCH_THICKNESS * scale
    strip_mid_r = powerflow_outer_radius + STRIP_ARCH_GAP * scale + strip_thickness / 2
    track_color = rl.Color(
      STRIP_ARCH_TRACK_COLOR.r, STRIP_ARCH_TRACK_COLOR.g,
      STRIP_ARCH_TRACK_COLOR.b, int(STRIP_ARCH_TRACK_COLOR.a * max(alpha, 0.4)),
    )
    track_pts = _arc_bar_pts(
      cx, cy, strip_mid_r, strip_thickness,
      start_angle, end_angle,
    )
    draw_polygon(rect, track_pts, color=track_color)
    abs_torque = abs(torque)
    if abs_torque > 0.005:
      is_active = ui_state.status in (UIStatus.ENGAGED, UIStatus.LAT_ONLY)
      if is_active:
        high_blend = max(0.0, abs_torque - 0.75) * 4
        fill_color = blend_colors(
          rl.Color(255, 255, 255, int(255 * 0.9 * alpha)),
          rl.Color(255, 200, 0, int(255 * alpha)),
          high_blend,
        )
      else:
        fill_color = rl.Color(255, 255, 255, int(255 * 0.35 * alpha))
      if torque < 0:
        bar_start_angle = center + (start_angle - center) * abs_torque
        bar_end_angle = center
      else:
        bar_start_angle = center
        bar_end_angle = center + (end_angle - center) * torque
      fill_pts = _arc_bar_pts(
        cx, cy, strip_mid_r, strip_thickness,
        bar_start_angle, bar_end_angle,
      )
      draw_polygon(rect, fill_pts, color=fill_color)
