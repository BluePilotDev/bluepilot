import pyray as rl
from openpilot.common.params import Params
from opendbc.sunnypilot.car.ford.lateral_curv_ext import PrimaryLateralControl
from opendbc.car.structs import ControllerStateBP
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import (
  HudRenderer, FONT_SIZES, KM_TO_MILE, CRUISE_DISABLED_CHAR, SET_SPEED_PERSISTENCE,
)
from openpilot.system.ui.lib.multilang import tr
from openpilot.selfdrive.ui.bp.mici.onroad.powerflow_gauge import MiciPowerflowGauge
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.bp.lib.steering_wheel_style import (
  ensure_steering_wheel_icon_style_initialized,
  get_steering_wheel_icon_style,
  SteeringWheelIconStyle,
)
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log
# BluePilot: seasonal theme packs (steering wheel icon override)
from openpilot.selfdrive.ui.bp.lib import theme_pack
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.application import gui_app
from openpilot.bluepilot.ui.lib.bp_shaders import draw_shader_circle_gradient
# BluePilot: Override upstream Mici torque bar with BP shared state math.
from openpilot.selfdrive.ui.bp.mici.onroad.torque_bar_bp import TorqueBarBP as TorqueBar

LateralMode = ControllerStateBP.LateralMode

class MiciHudRendererBP(HudRenderer):
  """BluePilot MICI HudRenderer with brake status coloring and powerflow gauge."""

  def __init__(self):
    super().__init__()
    # BluePilot: HudRenderer initializes upstream TorqueBar; replace it with ours.
    self._torque_bar = TorqueBar()
    self._bp_params = Params()
    self._brakes_on = False
    self._power_flow = MiciPowerflowGauge()
    self._txt_wheel_comma_3x = gui_app.texture("icons/chffr_wheel.png", self._txt_wheel.width, self._txt_wheel.height)
    self._animate_steering_wheel = self._bp_params.get_bool("BPAnimateSteeringWheel")
    self._wheel_icon_style = ensure_steering_wheel_icon_style_initialized(self._bp_params, SteeringWheelIconStyle.COMMA_4)
    self._theme_pack = theme_pack.get_active_pack(force=True)
    self._animate_wheel_param_counter = 0
    self.show_lateral_control = False
    # BluePilot: actual mode from controllerStateBP (None = not published, e.g. non-Ford)
    self.lateral_mode = None
    # BluePilot: Track overlay hit-area for click-to-toggle
    self._overlay_center_x = 0
    self._overlay_center_y = 0
    self._overlay_size = 0

  def _update_state(self) -> None:
    super()._update_state()

    # BluePilot: Refresh the shared wheel-animation toggle periodically.
    self._animate_wheel_param_counter += 1
    if self._animate_wheel_param_counter >= 60:
      self._animate_wheel_param_counter = 0
      self._animate_steering_wheel = self._bp_params.get_bool("BPAnimateSteeringWheel")
      self._wheel_icon_style = get_steering_wheel_icon_style(self._bp_params, SteeringWheelIconStyle.COMMA_4)
      self._theme_pack = theme_pack.get_active_pack()

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

    self.show_lateral_control = self._bp_params.get_bool("BpShowLateralControl")
    if self.show_lateral_control:
      sm = ui_state.sm
      self.lateral_mode = sm['controllerStateBP'].activeLateralMode if sm.alive['controllerStateBP'] else None

    bp_ui_log.state("MiciHudRenderer", "brakes_on", self._brakes_on)

  def _render(self, rect: rl.Rectangle) -> None:
    """Render HUD elements to the screen."""
    self._torque_bar.render(rect)

    if self.is_cruise_set:
      self._draw_set_speed(rect)

    self._draw_steering_wheel(rect)

  def _draw_steering_wheel(self, rect: rl.Rectangle) -> None:
    """Override to add brake status coloring to wheel icon, powerflow gauge, and lateral control overlay."""
    normal_wheel_txt = self._txt_wheel_comma_3x if self._wheel_icon_style == SteeringWheelIconStyle.COMMA_3X else self._txt_wheel
    # BluePilot: theme pack steering wheel icon wins over the built-in styles
    if self._theme_pack is not None:
      pack_wheel = self._theme_pack.wheel_texture(self._txt_wheel.width)
      if pack_wheel is not None:
        normal_wheel_txt = pack_wheel
    # BluePilot: Preserve the upstream critical-alert wheel regardless of the user's normal wheel style.
    wheel_txt = self._txt_wheel_critical if self._show_wheel_critical else normal_wheel_txt

    bsm_detected = self._has_blind_spot_detected() if hasattr(self, '_has_blind_spot_detected') else False

    show_lateral = True

    if self._show_wheel_critical:
      self._wheel_alpha_filter.update(255)
      self._wheel_y_filter.update(0)
    else:
      if ui_state.status == UIStatus.DISENGAGED or bsm_detected:
        self._wheel_alpha_filter.update(0)
        self._wheel_y_filter.update(wheel_txt.height / 2)
        show_lateral = False
      else:
        self._wheel_alpha_filter.update(255 * 0.9)
        self._wheel_y_filter.update(0)

    pos_x = int(rect.x + 21 + wheel_txt.width / 2)
    pos_y = int(rect.y + rect.height - 14 - wheel_txt.height / 2 + self._wheel_y_filter.x)
    rotation = -ui_state.sm['carState'].steeringAngleDeg if self._animate_steering_wheel else 0.0

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

    # BluePilot: Red color when braking
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

    if show_lateral:
      self._draw_lateral_control_overlay(pos_x, pos_y, wheel_txt.width)

    # BluePilot: Render powerflow gauge around steering wheel
    power_flow_radius = self._power_flow.RADIUS
    power_rect = rl.Rectangle(
      int(rect.x + 21) - power_flow_radius,
      int(rect.y + rect.height - wheel_txt.height - 14) - power_flow_radius,
      wheel_txt.width + power_flow_radius * 2,
      wheel_txt.height + power_flow_radius * 2)
    self._power_flow.set_wheel_rect(power_rect)
    self._power_flow.render(rect)

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    """Upstream set-speed drawing, with the theme pack Accent tinting the value text."""
    accent = self._theme_pack.rl_colors().get("Accent") if self._theme_pack is not None else None
    if accent is None:
      super()._draw_set_speed(rect)
      return

    alpha = self._set_speed_alpha_filter.update(0 < rl.get_time() - self._set_speed_changed_time < SET_SPEED_PERSISTENCE and
                                                self._can_draw_top_icons and self._engaged)
    if alpha < 1e-2:
      return

    x, y = rect.x, rect.y
    circle_radius = 162 // 2
    rl.draw_circle_gradient(rl.Vector2(x + circle_radius, y + circle_radius), circle_radius,
                            rl.Color(0, 0, 0, int(255 / 2 * alpha)), rl.BLANK)

    set_speed_color = rl.Color(accent.r, accent.g, accent.b, int(255 * 0.9 * alpha))
    max_color = rl.Color(255, 255, 255, int(255 * 0.9 * alpha))

    set_speed = self.set_speed
    if self.is_cruise_set and not ui_state.is_metric:
      set_speed *= KM_TO_MILE

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(set_speed))
    rl.draw_text_ex(self._font_display, set_speed_text, rl.Vector2(x + 13 + 4, y + 3 - 8 - 3 + 4),
                    FONT_SIZES.set_speed, 0, set_speed_color)
    rl.draw_text_ex(self._font_semi_bold, tr("MAX"), rl.Vector2(x + 25, y + FONT_SIZES.set_speed - 7 + 4),
                    FONT_SIZES.max_speed, 0, max_color)

  def _draw_lateral_control_overlay(self, center_x: int, center_y: int, wheel_size: int) -> None:
    """Draw a letter overlay indicating current lateral control mode (only when wheel is visible)."""
    if not self.show_lateral_control or self._wheel_alpha_filter.x <= 0 or self.lateral_mode is None:
      self._overlay_size = 0
      return

    text_size = int(wheel_size * 0.65)
    self._overlay_center_x = center_x
    self._overlay_center_y = center_y
    self._overlay_size = text_size

    if self.lateral_mode == LateralMode.angle:
      letter, color = "A", rl.Color(50, 100, 255, 220)  # Blue-ish
    elif self.lateral_mode == LateralMode.curvature:
      letter, color = "C", rl.Color(255, 165, 0, 220)  # Orange
    else:
      letter, color = "OP", rl.Color(100, 100, 100, 220)  # Grey

    text_dims = measure_text_cached(self._font_bold, letter, text_size)
    text_x = center_x - text_dims.x / 2
    text_y = center_y - text_dims.y / 2

    top = rl.Color(250, 250, 250, 200)
    bottom = rl.Color(200, 200, 200, 200)
    draw_shader_circle_gradient(center_x, center_y, text_size / 2, top, bottom)

    rl.draw_text_ex(self._font_bold, letter, rl.Vector2(text_x, text_y), text_size, 0, color)

  def _handle_mouse_press(self, mouse_pos):
    """Toggle FordPrefLateralControl between PrimaryLateralControl.curvature and .angle on overlay click."""
    if self._overlay_size <= 0 or self.lateral_mode not in (LateralMode.curvature, LateralMode.angle):
      return

    hit_rect = rl.Rectangle(
      self._overlay_center_x - self._overlay_size/2,
      self._overlay_center_y - self._overlay_size/2,
      self._overlay_size,
      self._overlay_size,
    )
    if rl.check_collision_point_rec(mouse_pos, hit_rect):
      gui_app._mouse_events.clear()
      current = PrimaryLateralControl(self._bp_params.get("FordPrefLateralControl") or 0)
      new_value = PrimaryLateralControl.curvature if current == PrimaryLateralControl.angle else PrimaryLateralControl.angle
      self._bp_params.put("FordPrefLateralControl", int(new_value))
