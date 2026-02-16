import pyray as rl
from openpilot.common.params import Params
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus


class MiciHudRendererBP(HudRenderer):
  """BluePilot MICI HudRenderer with brake status coloring."""

  def __init__(self):
    super().__init__()
    self._bp_params = Params()
    self._brakes_on = False

  def _update_state(self) -> None:
    super()._update_state()

    if self._bp_params.get_bool("ShowBrakeStatus"):
      sm = ui_state.sm
      try:
        car_state_bp = sm['carStateBP']
        brake_light_status = car_state_bp.brakeLightStatus
        self._brakes_on = brake_light_status.dataAvailable and brake_light_status.brakeLightsOn
      except (KeyError, AttributeError):
        self._brakes_on = False
    else:
      self._brakes_on = False

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    """Override to add brake status coloring to wheel icon."""
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else self._txt_wheel

    if self._show_wheel_critical:
      self._wheel_alpha_filter.update(255)
      self._wheel_y_filter.update(0)
    else:
      if ui_state.status == UIStatus.DISENGAGED:
        self._wheel_alpha_filter.update(0)
        self._wheel_y_filter.update(wheel_txt.height / 2)
      else:
        self._wheel_alpha_filter.update(255 * 0.9)
        self._wheel_y_filter.update(0)

    pos_x = int(rect.x + 21 + wheel_txt.width / 2)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + self._wheel_y_filter.x)
    rotation = -ui_state.sm['carState'].steeringAngleDeg

    turn_intent_margin = 25
    self._turn_intent.render(rl.Rectangle(
      pos_x - wheel_txt.width / 2 - turn_intent_margin,
      pos_y - wheel_txt.height / 2 - turn_intent_margin,
      wheel_txt.width + turn_intent_margin * 2,
      wheel_txt.height + turn_intent_margin * 2,
    ))

    src_rect = rl.Rectangle(0, 0, wheel_txt.width, wheel_txt.height)
    dest_rect = rl.Rectangle(pos_x, pos_y, wheel_txt.width, wheel_txt.height)
    origin = (wheel_txt.width / 2, wheel_txt.height / 2)

    # BP: Red color when braking
    if self._brakes_on:
      color = rl.Color(255, 60, 60, int(self._wheel_alpha_filter.x))
    else:
      color = rl.Color(255, 255, 255, int(self._wheel_alpha_filter.x))
    rl.draw_texture_pro(wheel_txt, src_rect, dest_rect, origin, rotation, color)

    if self._show_wheel_critical:
      EXCLAMATION_POINT_SPACING = 10
      exclamation_pos_x = pos_x - self._txt_exclamation_point.width / 2 + wheel_txt.width / 2 + EXCLAMATION_POINT_SPACING
      exclamation_pos_y = pos_y - self._txt_exclamation_point.height / 2
      rl.draw_texture(self._txt_exclamation_point, int(exclamation_pos_x), int(exclamation_pos_y), rl.WHITE)

    power_flow_radius = self._power_flow.RADIUS
    power_rect = rl.Rectangle(
      int(rect.x + 21) - power_flow_radius,
      int(rect.y + rect.height - wheel_txt.height - 14) - power_flow_radius,
      wheel_txt.width + power_flow_radius * 2,
      wheel_txt.height + power_flow_radius * 2)
    self._power_flow.set_wheel_rect(power_rect)
    self._power_flow.render(rect)
