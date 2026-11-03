"""
Rad Racer 8-bit theme renderer for the TICI onroad view.

Draws the retro scene around the (green) model road rendered by ModelRendererBP:
  - background: black sky, pixel star field, procedural city skyline at the horizon
  - roadside signs that scroll toward the viewer with vehicle speed
  - sprites: yellow/green lead cars at the projected lead positions, red ego car
  - bottom gauge cluster: lead time, ground speed + speed limit, torque bar, lead speed

Enabled via the BPRadRacerTheme param. Purely visual — no control-path impact.
"""
import numpy as np
import pyray as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.bp.onroad import rad_racer_assets as assets
from openpilot.selfdrive.ui.bp.onroad.rad_racer_assets import (
  PixelFont, PURPLE, PURPLE_LIGHT, PURPLE_DARK, GREEN, TEAL, TAN, RED, BLACK, WHITE, YELLOW,
)

# --- Layout ---
CLUSTER_HEIGHT = 240
CLUSTER_MARGIN = 10          # gap between panel edge and inner boxes
DM_RESERVED_WIDTH = 250      # left inset for cluster label layout (legacy wedge art)
DM_CLUSTER_GAP = 16          # clearance between DM widget and gauge cluster top

# --- Scene ---
SKYLINE_PARALLAX = 0.55        # how much skyline shifts vs path center at the horizon
SKYLINE_LOOKAHEAD_M = 150.0    # forward distance used to estimate road bend
SKYLINE_OFFSET_SMOOTH = 0.14   # scroll smoothing (higher = snappier)
STAR_PARALLAX = 0.22           # stars drift slightly with the skyline
SIGN_SPACING_M = 55.0        # distance between consecutive roadside signs
SIGN_MIN_M = 7.0             # respawn signs once they pass this distance
SIGN_MAX_M = SIGN_MIN_M + 3 * SIGN_SPACING_M
SIGN_LATERAL_M = 7.5         # lateral offset from path center, alternating sides
SIGN_TOP_Z = 4.2             # sign face top height in meters
LEAD_MAX_DRAW_M = 120.0

# --- Cluster bar geometry ---
TORQUE_SEGMENTS = 17         # odd count so there is a true center block
SPEED_LABEL_PX = 5.0
SPEED_VALUE_PX = 9.0
SPEED_ROW_GAP = 12
SPEED_LABEL_VALUE_GAP = 20
SPEED_VALUE_SHIFT = 28       # nudge readouts right so label+value block stays centered

UNLIT = rl.Color(28, 20, 44, 255)
BOX_BORDER = rl.Color(20, 10, 40, 255)


class RadRacerTheme:
  """Renders the 8-bit racing scene. Textures are built lazily on first render.

  Screen-scale knobs live as class attributes (TICI 2160x1080 values here) so device
  variants can subclass and rescale — see RadRacerThemeMici for the comma four.
  """

  SKYLINE_PX_SCALE = 4.2       # skyline texture pixel size on screen
  SKYLINE_MAX_OFFSET = 420.0   # clamp skyline travel in pixels
  STAR_COUNT = 28
  EGO_CAR_PX = 9.0             # ego sprite pixel scale
  EGO_CLUSTER_PAD = 6          # gap between ego sprite bottom and gauge cluster top
  LEAD_EGO_GAP = 14            # min gap between lead sprite bottom and ego sprite top
  LEAD_SPRITE_PX = (10.0, 5.0, 2.0)  # lead sprite pixel scale at dRel 5/40/100 m

  def __init__(self):
    self._loaded = False
    self._font: PixelFont | None = None
    self._ego_tex = None
    self._lead_tex = [None, None]
    self._sign_tex = None
    self._skyline_tex = None
    self._stars: list[tuple[float, float, int]] = []

    # Roadside sign scroll state: [(distance_m, side), ...] with side in {-1, +1}
    self._signs = [(SIGN_MIN_M + (i + 1) * SIGN_SPACING_M, -1 if i % 2 else 1) for i in range(3)]
    self._skyline_scroll = 0.0

  def _ensure_loaded(self):
    if self._loaded:
      return
    self._font = PixelFont()
    self._ego_tex = assets.build_ego_car_texture()
    self._lead_tex = [
      assets.build_lead_car_texture(YELLOW, rl.Color(188, 128, 20, 255)),
      assets.build_lead_car_texture(TEAL, rl.Color(36, 148, 112, 255)),
    ]
    self._sign_tex = assets.build_sign_texture()
    self._skyline_tex = assets.build_skyline_texture()
    import random
    rng = random.Random(7)
    self._stars = [(rng.random(), rng.random(), rng.randint(2, 4)) for _ in range(self.STAR_COUNT)]
    self._loaded = True

  # ------------------------------------------------------------------
  # Background: sky, stars, skyline, signs (drawn before the model road)
  # ------------------------------------------------------------------

  def render_background(self, rect: rl.Rectangle, model_renderer):
    self._ensure_loaded()

    horizon_y = self._horizon_y(rect, model_renderer)
    skyline_scroll = self._update_skyline_scroll(rect, model_renderer)

    # Star field in the sky band (subtle parallax with the skyline)
    sky_h = max(1.0, horizon_y - rect.y - self.SKYLINE_PX_SCALE * self._skyline_tex.height)
    star_shift = skyline_scroll * STAR_PARALLAX
    for fx, fy, size in self._stars:
      sx = rect.x + fx * rect.width + star_shift
      sy = rect.y + fy * sky_h
      color = WHITE if size >= 3 else TAN
      rl.draw_rectangle(int(sx), int(sy), size, size, color)

    # City skyline, bottom-aligned to the horizon, tiled with curvature parallax
    sky_w = self._skyline_tex.width * self.SKYLINE_PX_SCALE
    sky_hh = self._skyline_tex.height * self.SKYLINE_PX_SCALE
    x = rect.x - (skyline_scroll % sky_w)
    while x < rect.x + rect.width:
      w = min(sky_w, rect.x + rect.width - x)
      src = rl.Rectangle(0, 0, w / self.SKYLINE_PX_SCALE, self._skyline_tex.height)
      dst = rl.Rectangle(x, horizon_y - sky_hh, w, sky_hh)
      rl.draw_texture_pro(self._skyline_tex, src, dst, rl.Vector2(0, 0), 0.0, rl.WHITE)
      x += sky_w

    self._update_and_draw_signs(rect, model_renderer)

  def _horizon_y(self, rect: rl.Rectangle, model_renderer) -> float:
    pt = model_renderer._map_to_screen(400.0, 0.0, 0.0)
    if pt is not None:
      return float(np.clip(pt[1], rect.y + rect.height * 0.18, rect.y + rect.height * 0.60))
    return rect.y + rect.height * 0.40

  def _update_skyline_scroll(self, rect: rl.Rectangle, model_renderer) -> float:
    """Pan the tiled skyline opposite path bend, Rad Racer-style parallax at the horizon."""
    sample = self._path_point_at(model_renderer, SKYLINE_LOOKAHEAD_M)
    if sample is None:
      return self._skyline_scroll

    look, path_y, z_off = sample
    pt = model_renderer._map_to_screen(look, path_y, z_off)
    if pt is None:
      return self._skyline_scroll

    center_x = rect.x + rect.width / 2
    # Path center left of screen -> scroll skyline right (classic horizon parallax)
    target = float(np.clip((center_x - pt[0]) * SKYLINE_PARALLAX, -self.SKYLINE_MAX_OFFSET, self.SKYLINE_MAX_OFFSET))
    self._skyline_scroll += SKYLINE_OFFSET_SMOOTH * (target - self._skyline_scroll)
    return self._skyline_scroll

  @staticmethod
  def _path_point_at(model_renderer, distance_m: float) -> tuple[float, float, float] | None:
    """Sample the planned path at distance_m using live model data when available."""
    z_off = getattr(model_renderer, "_path_offset_z", 0.0)
    cam_off = getattr(model_renderer, "_camera_offset", 0.0)
    sm = ui_state.sm

    if sm.valid['modelV2'] and len(sm['modelV2'].position.x) >= 2:
      xs = np.asarray(sm['modelV2'].position.x, dtype=np.float64)
      ys = np.asarray(sm['modelV2'].position.y, dtype=np.float64)
      zs = np.asarray(sm['modelV2'].position.z, dtype=np.float64)
      look = min(distance_m, float(xs[-1]))
      if look < 10.0:
        return None
      return (
        look,
        float(np.interp(look, xs, ys)) + cam_off,
        float(np.interp(look, xs, zs)) + z_off,
      )

    path = model_renderer._path.raw_points
    if path.shape[0] < 2:
      return None
    look = min(distance_m, float(path[-1, 0]))
    if look < 10.0:
      return None
    return look, float(np.interp(look, path[:, 0], path[:, 1])), z_off

  def _update_and_draw_signs(self, rect: rl.Rectangle, model_renderer):
    v_ego = ui_state.sm['carState'].vEgo if ui_state.sm.valid['carState'] else 0.0
    dt = 1.0 / gui_app.target_fps

    new_signs = []
    for d, side in self._signs:
      d -= max(0.0, v_ego) * dt
      if d < SIGN_MIN_M:
        d += SIGN_MAX_M - SIGN_MIN_M
        side = -side
      new_signs.append((d, side))
    self._signs = sorted(new_signs, reverse=True)  # draw far signs first

    for d, side in self._signs:
      base = model_renderer._map_to_screen(d, side * SIGN_LATERAL_M, 0.0)
      top = model_renderer._map_to_screen(d, side * SIGN_LATERAL_M, SIGN_TOP_Z)
      if base is None or top is None:
        continue
      bx, by = base
      _, ty = top
      pole_h = by - ty
      if pole_h < 6 or by < rect.y or bx < rect.x - 200 or bx > rect.x + rect.width + 200:
        continue
      # Pole
      pole_w = max(2.0, pole_h * 0.10)
      rl.draw_rectangle(int(bx - pole_w / 2), int(ty), int(pole_w), int(pole_h), TEAL)
      # Sign face sits on top of the pole
      face_px = max(1.0, pole_h * 0.10)
      fw = self._sign_tex.width * face_px
      fh = self._sign_tex.height * face_px
      src = rl.Rectangle(0, 0, self._sign_tex.width, self._sign_tex.height)
      dst = rl.Rectangle(bx - fw / 2, ty - fh, fw, fh)
      rl.draw_texture_pro(self._sign_tex, src, dst, rl.Vector2(0, 0), 0.0, rl.WHITE)

  # ------------------------------------------------------------------
  # Foreground: lead car sprites + ego car (drawn after the model road)
  # ------------------------------------------------------------------

  def _ego_path_center_x(self, model_renderer) -> float | None:
    """Horizontal center of the planned path at the near (bottom) edge of the ribbon."""
    pts = model_renderer._path.projected_points
    if pts.size == 0 or len(pts) < 4:
      return None
    return float((pts[0][0] + pts[-1][0]) / 2.0)

  def render_foreground(self, rect: rl.Rectangle, model_renderer, cluster_top: float):
    self._ensure_loaded()

    ego_w = self._ego_tex.width * self.EGO_CAR_PX
    ego_h = self._ego_tex.height * self.EGO_CAR_PX
    ego_cx = self._ego_path_center_x(model_renderer)
    if ego_cx is None:
      ego_cx = rect.x + rect.width / 2
    ego_top = cluster_top - ego_h - self.EGO_CLUSTER_PAD
    max_lead_bottom = ego_top - self.LEAD_EGO_GAP

    # Ego car first so lead sprites always paint on top when close
    ego_src = rl.Rectangle(0, 0, self._ego_tex.width, self._ego_tex.height)
    ego_dst = rl.Rectangle(ego_cx - ego_w / 2, ego_top, ego_w, ego_h)
    rl.draw_texture_pro(self._ego_tex, ego_src, ego_dst, rl.Vector2(0, 0), 0.0, rl.WHITE)

    sm = ui_state.sm
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    if radar_state is not None:
      leads = [radar_state.leadOne, radar_state.leadTwo]
      # Draw the farther lead first so the closer one overlaps it
      order = sorted(range(2), key=lambda i: -(leads[i].dRel if leads[i] and leads[i].status else 0.0))
      for i in order:
        lead = leads[i]
        lv = model_renderer._lead_vehicles[i] if i < len(model_renderer._lead_vehicles) else None
        if not (lead and lead.status) or lv is None or not lv.chevron or lead.dRel > LEAD_MAX_DRAW_M:
          continue
        cx = lv.chevron[1][0]
        # Anchor lead bottom to chevron base, but never below the ego car icon
        base_y = min(float(lv.chevron[0][1]), max_lead_bottom)
        px = float(np.interp(lead.dRel, [5.0, 40.0, 100.0], self.LEAD_SPRITE_PX))
        tex = self._lead_tex[i]
        w, h = tex.width * px, tex.height * px
        src = rl.Rectangle(0, 0, tex.width, tex.height)
        dst = rl.Rectangle(cx - w / 2, base_y - h, w, h)
        rl.draw_texture_pro(tex, src, dst, rl.Vector2(0, 0), 0.0, rl.WHITE)

  # ------------------------------------------------------------------
  # Gauge cluster
  # ------------------------------------------------------------------

  def cluster_top(self, rect: rl.Rectangle) -> float:
    return rect.y + rect.height - CLUSTER_HEIGHT

  def driver_monitor_rect(self, rect: rl.Rectangle) -> rl.Rectangle:
    """Layout rect so the DM icon anchors bottom-left of the road view, above the cluster."""
    top = self.cluster_top(rect)
    return rl.Rectangle(rect.x, rect.y, rect.width, max(0.0, top - rect.y - DM_CLUSTER_GAP))

  def render_cluster(self, rect: rl.Rectangle, torque_bar):
    self._ensure_loaded()
    font = self._font

    top = self.cluster_top(rect)
    panel = rl.Rectangle(rect.x, top, rect.width, CLUSTER_HEIGHT)

    # Panel body with top edge highlight and darker side wedges
    rl.draw_rectangle_rec(panel, PURPLE)
    rl.draw_rectangle(int(panel.x), int(panel.y), int(panel.width), 6, PURPLE_LIGHT)
    wedge_w = DM_RESERVED_WIDTH - 30
    for side_x, direction in ((panel.x, 1), (panel.x + panel.width, -1)):
      pts = [
        rl.Vector2(side_x, panel.y),
        rl.Vector2(side_x, panel.y + panel.height),
        rl.Vector2(side_x + direction * wedge_w, panel.y + panel.height),
        rl.Vector2(side_x + direction * (wedge_w - 46), panel.y),
      ]
      if direction < 0:
        pts.reverse()
      rl.draw_triangle_fan(pts, len(pts), PURPLE_DARK)
    # Diagonal accent stripes on the wedges
    for i in range(4):
      off = 16 + i * 24
      rl.draw_line_ex(rl.Vector2(panel.x + off, panel.y + panel.height),
                      rl.Vector2(panel.x + off + 46, panel.y), 6, PURPLE_LIGHT if i % 2 else PURPLE)
      rl.draw_line_ex(rl.Vector2(panel.x + panel.width - off, panel.y + panel.height),
                      rl.Vector2(panel.x + panel.width - off - 46, panel.y), 6, PURPLE_LIGHT if i % 2 else PURPLE)

    # --- Gather data ---
    sm = ui_state.sm
    v_ego = sm['carState'].vEgo if sm.valid['carState'] else 0.0
    speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    unit_text = "KM/H" if ui_state.is_metric else "MPH"
    speed_text = f"{max(0, round(v_ego * speed_conv)):03d}"

    lead_time_text = "---"
    lead_speed_text = "---"
    radar_state = sm['radarState'] if sm.valid['radarState'] else None
    if radar_state is not None and radar_state.leadOne and radar_state.leadOne.status:
      lead = radar_state.leadOne
      if lead.dRel > 0 and v_ego > 0.5:
        t = lead.dRel / v_ego
        if 0 < t < 100:
          lead_time_text = f"{t:.1f}"
      lead_speed_text = f"{max(0, round((lead.vRel + v_ego) * speed_conv)):03d}"

    limit_text = self._speed_limit_text(sm, speed_conv)

    # Cluster always shows the steering bar — it's part of the game HUD
    torque = float(np.clip(torque_bar._torque_filter.x, -1.0, 1.0))

    # --- Left: lead vehicle time gap ---
    box_y = top + 46
    left_cx = panel.x + DM_RESERVED_WIDTH + 150
    self._label_value_box(font, left_cx, box_y, "Lead Time", lead_time_text)

    # --- Right: lead vehicle speed ---
    right_cx = panel.x + panel.width - DM_RESERVED_WIDTH - 150
    self._label_value_box(font, right_cx, box_y, "Lead Speed", lead_speed_text)

    # --- Center column: torque bar + stacked ground speed / speed limit ---
    center_x = panel.x + panel.width / 2
    self._draw_torque_segments(center_x, top + 14, torque)
    self._draw_center_speed_stack(font, center_x, top + 52, speed_text, limit_text, unit_text)

  @staticmethod
  def _speed_limit_text(sm, speed_conv: float) -> str:
    if not sm.valid['longitudinalPlanSP']:
      return "---"
    resolver = sm['longitudinalPlanSP'].speedLimit.resolver
    if not (resolver.speedLimitValid or resolver.speedLimitLastValid):
      return "---"
    return f"{max(0, round(resolver.speedLimitLast * speed_conv)):03d}"

  def _draw_center_speed_stack(
    self, font: PixelFont, center_x: float, y: float, ground_speed: str, limit_speed: str, unit_text: str,
  ):
    """Ground speed (top) and detected speed limit (bottom), labels left / values shifted right."""
    pad_x, pad_y = 24, 14
    labels = ("Ground Speed", "Speed Limit")
    values = (f"{ground_speed} {unit_text}", limit_speed if limit_speed == "---" else f"{limit_speed} {unit_text}")

    max_label_w = max(font.measure(label, SPEED_LABEL_PX) for label in labels)
    max_value_w = max(font.measure(value, SPEED_VALUE_PX) for value in values)
    value_row_h = assets.GLYPH_H * SPEED_VALUE_PX
    inner_w = max_label_w + SPEED_LABEL_VALUE_GAP + SPEED_VALUE_SHIFT + max_value_w
    inner_h = value_row_h * 2 + SPEED_ROW_GAP
    box_w = inner_w + pad_x * 2
    box_h = inner_h + pad_y * 2
    box = rl.Rectangle(center_x - box_w / 2, y, box_w, box_h)
    rl.draw_rectangle_rec(box, BLACK)
    rl.draw_rectangle_lines_ex(box, 3, BOX_BORDER)

    label_x = box.x + pad_x
    value_x = box.x + pad_x + max_label_w + SPEED_LABEL_VALUE_GAP + SPEED_VALUE_SHIFT
    for i, (label, value) in enumerate(zip(labels, values, strict=True)):
      row_y = box.y + pad_y + i * (value_row_h + SPEED_ROW_GAP)
      label_y = row_y + (value_row_h - assets.GLYPH_H * SPEED_LABEL_PX) / 2
      font.draw(label, label_x, label_y, SPEED_LABEL_PX, GREEN)
      font.draw(value, value_x, row_y, SPEED_VALUE_PX, GREEN)

  def _label_value_box(self, font: PixelFont, cx: float, y: float, label: str, value: str):
    label_px, value_px = 5.0, 7.0
    # Label box
    lw = font.measure(label, label_px) + 32
    lh = assets.GLYPH_H * label_px + 18
    lbox = rl.Rectangle(cx - lw / 2, y, lw, lh)
    rl.draw_rectangle_rec(lbox, BLACK)
    rl.draw_rectangle_lines_ex(lbox, 3, BOX_BORDER)
    font.draw_centered(label, cx, y + 9, label_px, GREEN)
    # Value box below
    vw = max(font.measure(value, value_px) + 36, lw)
    vh = assets.GLYPH_H * value_px + 20
    vbox = rl.Rectangle(cx - vw / 2, y + lh + 14, vw, vh)
    rl.draw_rectangle_rec(vbox, BLACK)
    rl.draw_rectangle_lines_ex(vbox, 3, BOX_BORDER)
    font.draw_centered(value, cx, vbox.y + 10, value_px, GREEN)

  def _draw_torque_segments(self, center_x: float, y: float, torque: float):
    """Center-zero segmented steering torque bar: green center, white mids, red ends."""
    seg_w, seg_h, gap = 24, 20, 5
    half = TORQUE_SEGMENTS // 2
    total_w = TORQUE_SEGMENTS * seg_w + (TORQUE_SEGMENTS - 1) * gap
    x0 = center_x - total_w / 2
    lit = round(abs(torque) * half)
    for i in range(TORQUE_SEGMENTS):
      pos = i - half  # -half..half, 0 = center block
      seg_x = x0 + i * (seg_w + gap)
      dist = abs(pos)
      if dist <= half * 0.4:
        on_color = GREEN
      elif dist <= half * 0.75:
        on_color = WHITE
      else:
        on_color = RED
      is_lit = pos == 0 or (torque < 0 and -lit <= pos < 0) or (torque > 0 and 0 < pos <= lit)
      rl.draw_rectangle(int(seg_x), int(y), seg_w, seg_h, on_color if is_lit else UNLIT)
