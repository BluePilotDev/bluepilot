"""Aurora wordmark — "bluepilot" with a breathing radial glow behind it.

Mode is driven by a callable returning 'ready' / 'experimental' / 'offline'.
The aurora is drawn as concentric `draw_circle_gradient` passes for a soft
radial bloom; it scales 0.96 -> 1.05 over a 3.6s sine cycle so the home feels
alive without clutter.
"""
import math
import pyray as rl
from collections.abc import Callable

from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.selfdrive.ui.bp.mici.widgets import bp_palette as P


class AuroraWordmark(Widget):
  AURORA_BASE_RADIUS = 220   # px before breathing scale
  BREATHE_PERIOD     = 3.6   # seconds

  def __init__(self, get_mode: Callable[[], str]):
    super().__init__()
    self._get_mode = get_mode
    self._mode = "ready"

    self._label = UnifiedLabel(
      "bluepilot",
      font_size=P.FS_WORDMARK,
      font_weight=FontWeight.DISPLAY,
      text_color=P.WORDMARK_BLUE,
      max_width=520,
      wrap_text=False,
      letter_spacing=-0.045,
    )

  def _update_state(self):
    self._mode = self._get_mode()
    color = {
      "experimental": P.WORDMARK_AMBER,
      "offline":      P.WORDMARK_GREY,
    }.get(self._mode, P.WORDMARK_BLUE)
    self._label.set_text_color(color)

  def _render(self, _):
    r = self._rect
    cx = r.x + r.width / 2
    cy = r.y + r.height / 2

    # ---- Aurora glow (skipped offline) ----
    if self._mode != "offline":
      t = rl.get_time()
      # 0..1 sine wave at the breathe period
      phase = (math.sin(t * (2 * math.pi / self.BREATHE_PERIOD)) + 1) * 0.5
      scale = 0.96 + phase * 0.09
      radius = self.AURORA_BASE_RADIUS * scale
      inner = {
        "experimental": P.AURORA_AMBER_INNER,
      }.get(self._mode, P.AURORA_BLUE_INNER)
      # Three radial passes give a soft falloff with fixed-cost circle draws.
      for r_mul, alpha_mul in ((1.00, 1.00), (0.62, 0.85), (0.35, 0.70)):
        col = rl.Color(inner.r, inner.g, inner.b, int(inner.a * alpha_mul))
        rl.draw_circle_gradient(rl.Vector2(int(cx), int(cy)), radius * r_mul, col, P.AURORA_OUTER)

    # ---- Wordmark ----
    # UnifiedLabel doesn't expose center alignment cleanly with wrap_text=False,
    # so we measure once and position manually.
    text_w = self._label.text_width
    if text_w <= 0:
      # First render: text_width isn't cached yet. Place at left=0 so the label
      # renders + caches; next frame we'll center.
      self._label.set_position(cx - 240, cy - P.FS_WORDMARK / 2)
    else:
      self._label.set_position(cx - text_w / 2, cy - P.FS_WORDMARK / 2)
    self._label.render()
