import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.steering_wheel_style import (
  ensure_steering_wheel_icon_style_initialized,
  get_steering_wheel_icon_style,
  SteeringWheelIconStyle,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget
# BluePilot: seasonal theme packs (steering wheel icon override)
from openpilot.selfdrive.ui.bp.lib import theme_pack


class SteeringWheelBP(Widget):
  """BluePilot's 3X onroad wheel icon in a dark circle: the chosen wheel style (or the theme pack's wheel),
  turning with the steering angle when BPAnimateSteeringWheel is on, dimmed while openpilot can't engage.
  Display only: upstream removed experimental mode, so there's no tap action."""

  def __init__(self, icon_size: int):
    super().__init__()
    self._params = Params()
    self._icon_size = icon_size
    self._txt_wheel_comma_3x = gui_app.texture("icons/chffr_wheel.png", icon_size, icon_size)
    self._txt_wheel_comma_4 = gui_app.texture("icons_mici/wheel.png", icon_size, icon_size)
    self._black_bg = rl.Color(0, 0, 0, 166)
    self._engageable = False
    self._animate_steering_wheel = self._params.get_bool("BPAnimateSteeringWheel")
    self._wheel_icon_style = ensure_steering_wheel_icon_style_initialized(self._params, SteeringWheelIconStyle.COMMA_3X)
    self._theme_pack = theme_pack.get_active_pack(force=True)
    self._param_counter = 0

  def _update_state(self) -> None:
    selfdrive_state = ui_state.sm["selfdriveState"]
    self._engageable = selfdrive_state.engageable or selfdrive_state.enabled

    # Refresh display params periodically so onroad changes are picked up.
    self._param_counter += 1
    if self._param_counter >= 60:
      self._param_counter = 0
      self._animate_steering_wheel = self._params.get_bool("BPAnimateSteeringWheel")
      self._wheel_icon_style = get_steering_wheel_icon_style(self._params, SteeringWheelIconStyle.COMMA_3X)
      self._theme_pack = theme_pack.get_active_pack()

  def _wheel_texture(self) -> rl.Texture:
    if self._theme_pack is not None:  # the theme pack's wheel wins over the built-in styles
      pack_wheel = self._theme_pack.wheel_texture(self._icon_size)
      if pack_wheel is not None:
        return pack_wheel
    return self._txt_wheel_comma_4 if self._wheel_icon_style == SteeringWheelIconStyle.COMMA_4 else self._txt_wheel_comma_3x

  def _render(self, rect: rl.Rectangle) -> None:
    center_x = int(rect.x + rect.width // 2)
    center_y = int(rect.y + rect.height // 2)
    color = rl.Color(255, 255, 255, 255 if self._engageable else 180)
    texture = self._wheel_texture()
    rotation = -ui_state.sm["carState"].steeringAngleDeg if self._animate_steering_wheel else 0.0

    rl.draw_circle(center_x, center_y, rect.width / 2, self._black_bg)
    rl.draw_texture_pro(
      texture,
      rl.Rectangle(0, 0, texture.width, texture.height),
      rl.Rectangle(center_x, center_y, texture.width, texture.height),
      rl.Vector2(texture.width / 2, texture.height / 2),
      rotation,
      color,
    )
