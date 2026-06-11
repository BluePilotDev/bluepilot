"""Blue/black radial gradient — paints the BP screen backdrop.

Mirrors the mockup CSS:
  radial-gradient(120% 140% at 0% 0%,
    rgba(74,140,255,0.42) 0%,
    rgba(20,40,90,0.28) 28%,
    rgba(2,6,15,0.95) 62%,
    #02060f 100%);

raylib has no rectangle radial gradient, so we paint a deep-navy base then
overlay three concentric soft circles centered at the top-left corner. The
result is visually very close to the CSS radial when viewed on the 536x240
canvas.
"""
import pyray as rl

from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.bp.mici.widgets import bp_palette as P


class BPRadialBackground(Widget):
  def __init__(self):
    super().__init__()

  def _render(self, _):
    r = self._rect

    # Base: solid deep navy / near-black.
    rl.draw_rectangle_rec(r, P.BG_DEEP)

    # Concentric "light" stops centered at the top-left corner. Each call
    # draws inner_color at center fading to fully transparent at radius.
    # Larger radii first so the brighter inner stops paint on top.
    cx, cy = int(r.x), int(r.y)
    diag = (r.width ** 2 + r.height ** 2) ** 0.5

    # Outer fade (62% stop) — barely-perceptible navy lift at the edges of
    # the gradient extent.
    rl.draw_circle_gradient(rl.Vector2(cx, cy), diag * 1.2,
      rl.Color(0x14, 0x28, 0x5A, int(0.50 * 255)),
      rl.Color(0, 0, 0, 0))

    # Mid stop (28%) — navy tint covering most of the upper-left half.
    rl.draw_circle_gradient(rl.Vector2(cx, cy), diag * 0.85,
      rl.Color(0x14, 0x28, 0x5A, int(0.65 * 255)),
      rl.Color(0, 0, 0, 0))

    # Inner glow (0%) — bright blue near the corner.
    rl.draw_circle_gradient(rl.Vector2(cx, cy), diag * 0.55,
      rl.Color(0x4A, 0x8C, 0xFF, int(0.42 * 255)),
      rl.Color(0, 0, 0, 0))
