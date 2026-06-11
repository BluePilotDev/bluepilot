"""Procedural feather-style line icons for BP MICI category tiles.

These mirror the inline SVG icons used in the mockup (mockups/mici_home.html):
sliders / wifi / cpu / upload-cloud / terminal / vehicle / car. Drawn with
raylib lines/circles/rings so we don't ship new asset files. Each is
center-anchored at (cx, cy) and scaled by `size`.
"""
import math
import pyray as rl

from openpilot.selfdrive.ui.bp.mici.widgets import bp_palette as P


def _halo(cx: float, cy: float, size: float, color: rl.Color) -> None:
  """Soft blue halo behind an icon. Three concentric circles fading outward."""
  for r_mul, alpha in ((1.6, 0.06), (1.1, 0.10), (0.7, 0.12)):
    c = rl.Color(color.r, color.g, color.b, int(255 * alpha))
    rl.draw_circle_gradient(rl.Vector2(int(cx), int(cy)), size * r_mul, c, rl.Color(0, 0, 0, 0))


def _draw_with_halo(draw_fn, cx: float, cy: float, size: float,
                    icon_color: rl.Color = P.TEXT, halo_color: rl.Color = None) -> None:
  if halo_color is None:
    halo_color = P.ACCENT
  _halo(cx, cy, size, halo_color)
  draw_fn(cx, cy, size, icon_color)


# ----------------------------------------------------------------
# Individual icon glyphs — center at (cx, cy), bbox roughly 2*size square.
# ----------------------------------------------------------------

def _sliders(cx: float, cy: float, size: float, color: rl.Color) -> None:
  th = max(3, size * 0.10)
  half = size * 0.85
  # Two horizontal bars with a circular knob at one position each.
  for sign, knob_at in ((-1, 0.65), (+1, 0.35)):
    y = cy + sign * size * 0.40
    rl.draw_line_ex(rl.Vector2(cx - half, y), rl.Vector2(cx + half, y), th, color)
    knob_x = cx - half + knob_at * (half * 2)
    rl.draw_circle(int(knob_x), int(y), th * 1.7, color)
    # Knob inner ring (for depth)
    rl.draw_circle(int(knob_x), int(y), th * 0.7, P.BG_DEEP)


def _wifi(cx: float, cy: float, size: float, color: rl.Color) -> None:
  th = max(3, size * 0.10)
  # Three concentric arcs above a dot — feather wifi shape.
  for r_mul in (1.0, 0.65, 0.32):
    rl.draw_ring(rl.Vector2(cx, cy + size * 0.35),
                 size * r_mul - th * 0.6, size * r_mul + th * 0.6,
                 200, 340, 32, color)
  rl.draw_circle(int(cx), int(cy + size * 0.45), th * 0.9, color)


def _cpu(cx: float, cy: float, size: float, color: rl.Color) -> None:
  th = max(3, size * 0.10)
  s = size * 0.80
  # Outer rounded square
  outer = rl.Rectangle(cx - s, cy - s, s * 2, s * 2)
  rl.draw_rectangle_rounded_lines_ex(outer, 0.10, 8, th, color)
  # Inner filled square
  inner = rl.Rectangle(cx - s * 0.45, cy - s * 0.45, s * 0.90, s * 0.90)
  rl.draw_rectangle_rounded_lines_ex(inner, 0.05, 6, th, color)
  # Pins on each side (3 per side)
  pin_len = s * 0.30
  for i in range(-1, 2):
    o = i * s * 0.5
    # top
    rl.draw_line_ex(rl.Vector2(cx + o, cy - s), rl.Vector2(cx + o, cy - s - pin_len), th * 0.8, color)
    # bottom
    rl.draw_line_ex(rl.Vector2(cx + o, cy + s), rl.Vector2(cx + o, cy + s + pin_len), th * 0.8, color)
    # left
    rl.draw_line_ex(rl.Vector2(cx - s, cy + o), rl.Vector2(cx - s - pin_len, cy + o), th * 0.8, color)
    # right
    rl.draw_line_ex(rl.Vector2(cx + s, cy + o), rl.Vector2(cx + s + pin_len, cy + o), th * 0.8, color)


def _upload_cloud(cx: float, cy: float, size: float, color: rl.Color) -> None:
  th = max(3, size * 0.10)
  # Cloud body — three circles + a base line.
  rl.draw_circle(int(cx - size * 0.45), int(cy - size * 0.05), size * 0.35, color)
  rl.draw_circle(int(cx + size * 0.10), int(cy - size * 0.30), size * 0.45, color)
  rl.draw_circle(int(cx + size * 0.55), int(cy + size * 0.05), size * 0.35, color)
  rl.draw_rectangle(int(cx - size * 0.55), int(cy - size * 0.05),
                    int(size * 1.1), int(size * 0.40), color)
  # Carve inner shape so the cloud reads as outline (subtract a slightly
  # smaller copy filled with BG_DEEP).
  inner_color = P.BG_DEEP
  rl.draw_circle(int(cx - size * 0.45), int(cy - size * 0.05), size * 0.35 - th, inner_color)
  rl.draw_circle(int(cx + size * 0.10), int(cy - size * 0.30), size * 0.45 - th, inner_color)
  rl.draw_circle(int(cx + size * 0.55), int(cy + size * 0.05), size * 0.35 - th, inner_color)
  rl.draw_rectangle(int(cx - size * 0.55) + int(th * 0.7), int(cy - size * 0.05),
                    int(size * 1.1) - int(th * 1.4), int(size * 0.40) - int(th * 0.7), inner_color)
  # Up arrow inside the cloud
  arrow_w = size * 0.32
  arrow_h = size * 0.55
  rl.draw_line_ex(rl.Vector2(cx, cy - arrow_h * 0.30),
                  rl.Vector2(cx, cy + arrow_h * 0.45), th, color)
  rl.draw_line_ex(rl.Vector2(cx, cy - arrow_h * 0.30),
                  rl.Vector2(cx - arrow_w * 0.5, cy - arrow_h * 0.05), th, color)
  rl.draw_line_ex(rl.Vector2(cx, cy - arrow_h * 0.30),
                  rl.Vector2(cx + arrow_w * 0.5, cy - arrow_h * 0.05), th, color)


def _terminal(cx: float, cy: float, size: float, color: rl.Color) -> None:
  th = max(3, size * 0.12)
  # ">" chevron on the left
  rl.draw_line_ex(rl.Vector2(cx - size * 0.7, cy - size * 0.35),
                  rl.Vector2(cx - size * 0.05, cy + size * 0.05), th, color)
  rl.draw_line_ex(rl.Vector2(cx - size * 0.05, cy + size * 0.05),
                  rl.Vector2(cx - size * 0.7, cy + size * 0.45), th, color)
  # "_" underscore on the right
  rl.draw_line_ex(rl.Vector2(cx + size * 0.05, cy + size * 0.45),
                  rl.Vector2(cx + size * 0.75, cy + size * 0.45), th, color)


def _vehicle(cx: float, cy: float, size: float, color: rl.Color) -> None:
  th = max(3, size * 0.10)
  # Simple car silhouette: body rect + roof bump + 2 wheels.
  body = rl.Rectangle(cx - size * 0.9, cy - size * 0.05, size * 1.8, size * 0.55)
  rl.draw_rectangle_rounded_lines_ex(body, 0.30, 6, th, color)
  # Roof
  roof_x1 = cx - size * 0.55
  roof_x2 = cx + size * 0.55
  roof_y = cy - size * 0.45
  rl.draw_line_ex(rl.Vector2(cx - size * 0.65, cy - size * 0.05),
                  rl.Vector2(roof_x1, roof_y), th, color)
  rl.draw_line_ex(rl.Vector2(roof_x1, roof_y), rl.Vector2(roof_x2, roof_y), th, color)
  rl.draw_line_ex(rl.Vector2(roof_x2, roof_y),
                  rl.Vector2(cx + size * 0.65, cy - size * 0.05), th, color)
  # Wheels
  rl.draw_circle(int(cx - size * 0.55), int(cy + size * 0.55), th * 1.6, color)
  rl.draw_circle(int(cx + size * 0.55), int(cy + size * 0.55), th * 1.6, color)
  rl.draw_circle(int(cx - size * 0.55), int(cy + size * 0.55), th * 0.7, P.BG_DEEP)
  rl.draw_circle(int(cx + size * 0.55), int(cy + size * 0.55), th * 0.7, P.BG_DEEP)


def _bluepilot(cx: float, cy: float, size: float, color: rl.Color) -> None:
  th = max(3, size * 0.10)
  # Steering wheel: outer ring + inner hub + 3 spokes.
  rl.draw_ring(rl.Vector2(cx, cy), size * 0.78, size * 0.92, 0, 360, 64, color)
  rl.draw_circle(int(cx), int(cy), size * 0.18, color)
  for ang in (90, 210, 330):
    a = math.radians(ang)
    rl.draw_line_ex(rl.Vector2(cx, cy),
                    rl.Vector2(cx + math.cos(a) * size * 0.78,
                               cy + math.sin(a) * size * 0.78),
                    th, color)


# ----------------------------------------------------------------
# Public lookup
# ----------------------------------------------------------------
ICON_DRAWERS = {
  "toggles":   _sliders,
  "network":   _wifi,
  "device":    _cpu,
  "firehose":  _upload_cloud,
  "developer": _terminal,
  "vehicle":   _vehicle,
  "bluepilot": _bluepilot,
}


def draw_category_icon(name: str, cx: float, cy: float, size: float = 36,
                        color: rl.Color = P.TEXT, halo_color: rl.Color = None) -> None:
  fn = ICON_DRAWERS.get(name)
  if fn is None:
    return
  _draw_with_halo(fn, cx, cy, size, color, halo_color)
