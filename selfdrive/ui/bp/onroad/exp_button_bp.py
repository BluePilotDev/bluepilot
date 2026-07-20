import pyray as rl

from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.bp.lib.steering_wheel_style import (
  ensure_steering_wheel_icon_style_initialized,
  get_steering_wheel_icon_style,
  SteeringWheelIconStyle,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
# BluePilot: seasonal theme packs (steering wheel icon override)
from openpilot.selfdrive.ui.bp.lib import theme_pack


class ExpButtonBP(ExpButton):
  """BluePilot experimental-mode button with optional steering-wheel animation."""

  def __init__(self, button_size: int, icon_size: int):
    super().__init__(button_size, icon_size)
    self._txt_wheel_comma_4 = gui_app.texture("icons_mici/wheel.png", icon_size, icon_size)
    self._icon_size = icon_size
    self._animate_steering_wheel = self._params.get_bool("BPAnimateSteeringWheel")
    self._wheel_icon_style = ensure_steering_wheel_icon_style_initialized(self._params, SteeringWheelIconStyle.COMMA_3X)
    self._theme_pack = theme_pack.get_active_pack(force=True)
    self._param_counter = 0

  def _update_state(self) -> None:
    super()._update_state()

    # BluePilot: Refresh display params periodically so onroad changes are picked up.
    self._param_counter += 1
    if self._param_counter >= 60:
      self._param_counter = 0
      self._animate_steering_wheel = self._params.get_bool("BPAnimateSteeringWheel")
      self._wheel_icon_style = get_steering_wheel_icon_style(self._params, SteeringWheelIconStyle.COMMA_3X)
      self._theme_pack = theme_pack.get_active_pack()

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(self._rect.x + self._rect.width // 2)
    center_y = int(self._rect.y + self._rect.height // 2)

    self._white_color.a = 180 if self.is_pressed or not self._engageable else 255

    experimental_mode = self._held_or_actual_mode()
    wheel_texture = self._txt_wheel_comma_4 if self._wheel_icon_style == SteeringWheelIconStyle.COMMA_4 else self._txt_wheel
    # BluePilot: theme pack steering wheel icon wins over the built-in styles
    if self._theme_pack is not None:
      pack_wheel = self._theme_pack.wheel_texture(self._icon_size)
      if pack_wheel is not None:
        wheel_texture = pack_wheel
    texture = self._txt_exp if experimental_mode else wheel_texture

    rl.draw_circle(center_x, center_y, self._rect.width / 2, self._black_bg)

    if experimental_mode or not self._animate_steering_wheel:
      position = rl.Vector2(center_x - texture.width / 2, center_y - texture.height / 2)
      rl.draw_texture_ex(texture, position, 0.0, 1.0, self._white_color)
    else:
      rotation = -ui_state.sm['carState'].steeringAngleDeg
      rl.draw_texture_pro(
        texture,
        rl.Rectangle(0, 0, texture.width, texture.height),
        rl.Rectangle(center_x, center_y, texture.width, texture.height),
        rl.Vector2(texture.width / 2, texture.height / 2),
        rotation,
        self._white_color,
      )
