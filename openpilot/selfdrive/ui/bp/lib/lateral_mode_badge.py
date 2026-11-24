"""BluePilot: the lateral control mode as a badge at the steering wheel's lower right (3X and C4).

The wheel icon turns with the steering angle, so the mode sits beside it in a pill of its own, drawn level, rather
than over the wheel's center. The pill is sized from the measured label: a single letter makes a circle, "OP" a pill.
"""
import pyray as rl

from openpilot.cereal import custom
from openpilot.system.ui.lib.text_measure import measure_text_cached

LateralMode = custom.ControllerStateBP.LateralMode

# label, fill, text
_CURVATURE = ("C", rl.Color(255, 165, 0, 255), rl.Color(45, 26, 0, 255))  # BluePilot's controller steers
_ANGLE = ("A", rl.Color(50, 100, 255, 255), rl.Color(255, 255, 255, 255))
_STOCK = ("OP", rl.Color(95, 95, 95, 255), rl.Color(255, 255, 255, 255))  # comma's stock command steers
_OUTLINE = rl.Color(0, 0, 0, 220)  # separates the badge from the wheel and the road


def _faded(color: rl.Color, alpha: float) -> rl.Color:
  return rl.Color(color.r, color.g, color.b, int(color.a * alpha / 255))


def _badge(mode) -> tuple[str, rl.Color, rl.Color]:
  # == rather than a dict: the mode read from the message is a capnp enum, which compares equal but hashes differently
  if mode == LateralMode.curvature:
    return _CURVATURE
  if mode == LateralMode.angle:
    return _ANGLE
  return _STOCK


def draw_lateral_mode_badge(font: rl.Font, mode, right: float, bottom: float, text_size: int, alpha: float = 255) -> None:
  """Draw the mode's badge with its outer right and bottom edges at (right, bottom). alpha: 0-255, the wheel's."""
  if alpha <= 0:
    return
  label, fill, text = _badge(mode)
  dims = measure_text_cached(font, label, text_size)
  outline = max(2.0, round(text_size * 0.08))
  height = dims.y + text_size * 0.2
  width = max(dims.x + text_size * 0.6, height)
  x = right - outline - width
  y = bottom - outline - height

  rl.draw_rectangle_rounded(rl.Rectangle(x - outline, y - outline, width + 2 * outline, height + 2 * outline),
                            1.0, 16, _faded(_OUTLINE, alpha))
  rl.draw_rectangle_rounded(rl.Rectangle(x, y, width, height), 1.0, 16, _faded(fill, alpha))
  rl.draw_text_ex(font, label, rl.Vector2(x + (width - dims.x) / 2, y + (height - dims.y) / 2), text_size, 0,
                  _faded(text, alpha))
