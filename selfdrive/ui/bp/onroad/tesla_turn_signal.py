"""Tesla-blue coloring for SunnyPilot's built-in turn-signal lights."""

from __future__ import annotations

import time
from typing import Any

import pyray as rl

from openpilot.selfdrive.ui.bp.lib.tesla_palette import palette_for_dark_fraction
from openpilot.selfdrive.ui.mici.onroad.alert_renderer import IconSide, TURN_SIGNAL_BLINK_PERIOD
from openpilot.selfdrive.ui.sunnypilot.onroad.turn_signal import TurnSignalController, TurnSignalWidget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import GL_VERSION, gui_app
from openpilot.system.ui.lib.shader_polygon import UNIFORM_VEC4, VERTEX_SHADER


TESLA_TURN_SIGNAL_FRAGMENT_SHADER = GL_VERSION + """
in vec2 fragTexCoord;
out vec4 finalColor;

uniform sampler2D texture0;
uniform vec4 monochromeColor;

void main() {
  vec4 source = texture(texture0, fragTexCoord);
  finalColor = vec4(monochromeColor.rgb, monochromeColor.a * source.a);
}
"""


def tesla_turn_signal_state(sm, show_turn_signals: bool) -> tuple[bool, bool]:
  """Return the active vehicle blinkers while honoring SunnyPilot's UI toggle."""
  if not show_turn_signals:
    return False, False
  if not (sm.alive.get("carState", False) and sm.valid.get("carState", False)):
    return False, False

  car_state = sm["carState"]
  return bool(car_state.leftBlinker), bool(car_state.rightBlinker)


def tesla_turn_signal_color(dark_fraction: float, alpha: int = 255) -> rl.Color:
  """Use the same Light/Dark-aware blue as the active Tesla MAX label."""
  active_blue = palette_for_dark_fraction(dark_fraction).max_active
  return rl.Color(active_blue.r, active_blue.g, active_blue.b,
                  round(max(0, min(int(alpha), 255)) * active_blue.a / 255))


class _MonochromeTextureShader:
  _instance: Any = None

  @classmethod
  def get_instance(cls):
    if cls._instance is None:
      cls._instance = cls()
    return cls._instance

  def __init__(self):
    if _MonochromeTextureShader._instance is not None:
      raise RuntimeError("Use get_instance()")
    self.initialized = False
    self.shader = None
    self.color_location = None
    self.mvp_location = None
    self.color = rl.ffi.new("float[4]", [0.0, 0.0, 0.0, 0.0])

  def initialize(self) -> None:
    if self.initialized:
      return
    self.shader = rl.load_shader_from_memory(VERTEX_SHADER, TESLA_TURN_SIGNAL_FRAGMENT_SHADER)
    self.color_location = rl.get_shader_location(self.shader, "monochromeColor")
    self.mvp_location = rl.get_shader_location(self.shader, "mvp")
    projection = rl.matrix_ortho(0, gui_app.width, gui_app.height, 0, -1, 1)
    rl.set_shader_value_matrix(self.shader, self.mvp_location, projection)
    self.initialized = True


def draw_tesla_turn_signal_texture(texture: rl.Texture, position: rl.Vector2,
                                   dark_fraction: float, alpha: int) -> None:
  """Draw a stock turn-signal texture as a monochrome Tesla-blue alpha mask."""
  color = tesla_turn_signal_color(dark_fraction, alpha)
  shader = _MonochromeTextureShader.get_instance()
  shader.initialize()
  shader.color[0:4] = [color.r / 255.0, color.g / 255.0, color.b / 255.0, color.a / 255.0]
  rl.set_shader_value(shader.shader, shader.color_location, shader.color, UNIFORM_VEC4)
  rl.begin_shader_mode(shader.shader)
  rl.draw_texture_ex(texture, position, 0.0, 1.0, rl.WHITE)
  rl.end_shader_mode()


class TeslaBlueTurnSignalWidget(TurnSignalWidget):
  """The stock SunnyPilot light, pulse, and geometry with Tesla-blue pixels."""

  def _render(self, _):
    if not self._active:
      return

    if time.monotonic() - self._turn_signal_timer > TURN_SIGNAL_BLINK_PERIOD:
      self._turn_signal_timer = time.monotonic()
      self._turn_signal_alpha_filter.x = 255 * 2
    else:
      self._turn_signal_alpha_filter.update(255 * 0.2)
    icon_alpha = int(min(self._turn_signal_alpha_filter.x, 255))

    if self._signal_texture:
      pos_x = self._rect.x + (self._rect.width - self._signal_texture.width) / 2
      pos_y = self._rect.y + (self._rect.height - self._signal_texture.height) / 2
      draw_tesla_turn_signal_texture(
        self._signal_texture,
        rl.Vector2(pos_x, pos_y),
        ui_state.tesla_dark_fraction,
        icon_alpha,
      )


class TeslaBlueTurnSignalController(TurnSignalController):
  """Stock C3X signals without reintroducing the retired blind-spot actor."""

  def __init__(self):
    super().__init__()
    self._left_signal = TeslaBlueTurnSignalWidget(direction=IconSide.left)
    self._right_signal = TeslaBlueTurnSignalWidget(direction=IconSide.right)

  def update(self):
    left_active, right_active = tesla_turn_signal_state(ui_state.sm, ui_state.turn_signals)
    if left_active:
      self._left_signal.activate()
    else:
      self._left_signal.deactivate()
    if right_active:
      self._right_signal.activate()
    else:
      self._right_signal.deactivate()
