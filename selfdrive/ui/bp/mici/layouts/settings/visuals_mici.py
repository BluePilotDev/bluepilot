"""BluePilot MICI: Visuals settings panel — display toggles, rainbow, blindspot, power flow, fade, border, steering wheel."""

from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.steering_wheel_style import (
  ensure_steering_wheel_icon_style_initialized,
  get_steering_wheel_icon_style,
  SteeringWheelIconStyle,
)
from openpilot.selfdrive.ui.bp.lib.dm_icon_style import (
  DMIconStyle,
  ensure_dm_icon_style_initialized,
  get_dm_icon_style,
)
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import (
  BigParamControlBP,
  BigMultiParamToggleBP,
  BigMultiParamToggleBoolBP,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller import NavScroller


class VisualsLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

    self.show_lead_vehicle = BigMultiParamToggleBP(
      "lower right display", "mici_complication",
      ["off", "lead car speed", "speed", "lead car distance", "time to lead car"],
    )
    self.rainbow_mode = BigParamControlBP("rainbow mode", "RainbowMode")
    self.rad_racer_theme = BigParamControlBP("8-bit racer theme", "BPRadRacerTheme")
    self.hide_fade = BigParamControlBP("hide onroad fade", "mici_hide_onroad_fade")
    self.hide_border = BigParamControlBP("hide onroad border", "BPHideOnroadBorder")
    self.hide_camera_view = BigParamControlBP("minimal driving view", "BPHideCameraView")
    self.rainbow_lane_lines = BigParamControlBP("rainbow lane lines", "BPRainbowLines")
    self.show_blindspot_ui = BigParamControlBP("show blindspot overlay", "ShowBlindspotOverlay")
    self.show_brake_status = BigParamControlBP("show brake status", "ShowBrakeStatus")
    self.show_live_delay = BigParamControlBP("show steering lag calibration", "BPShowLiveDelayIndicator")
    self.animate_steering_wheel = BigParamControlBP("animate steering wheel", "BPAnimateSteeringWheel")
    ensure_steering_wheel_icon_style_initialized(Params(), SteeringWheelIconStyle.COMMA_4)
    self.wheel_icon_style = BigMultiParamToggleBP(
      "wheel icon style", "BPSteeringWheelIconStyle", ["comma 4", "comma 3x"],
    )
    ensure_dm_icon_style_initialized(Params(), DMIconStyle.COMMA_4)
    self.dm_icon_style = BigMultiParamToggleBP(
      "dm icon style", "BPDMStylingChoice", ["comma 4", "comma 3x"],
    )
    self.show_hybrid_power_flow = BigParamControlBP("show hybrid power flow", "FordPrefHybridPowerFlow")
    self.hybrid_power_flow_style = BigMultiParamToggleBoolBP(
      "hybrid/ev power flow style", "FordPrefHybridPowerFlowAlternate", ["flat", "round"],
    )

    self._scroller.add_widgets([
      self.show_lead_vehicle,
      self.rainbow_mode,
      self.rad_racer_theme,
      self.hide_fade,
      self.hide_border,
      self.hide_camera_view,
      self.rainbow_lane_lines,
      self.show_blindspot_ui,
      self.show_brake_status,
      self.show_live_delay,
      self.animate_steering_wheel,
      self.wheel_icon_style,
      self.dm_icon_style,
      self.show_hybrid_power_flow,
      self.hybrid_power_flow_style,
    ])

    self._refresh_toggles = (
      ("RainbowMode", self.rainbow_mode),
      ("BPRadRacerTheme", self.rad_racer_theme),
      ("mici_hide_onroad_fade", self.hide_fade),
      ("BPHideOnroadBorder", self.hide_border),
      ("BPHideCameraView", self.hide_camera_view),
      ("BPRainbowLines", self.rainbow_lane_lines),
      ("ShowBlindspotOverlay", self.show_blindspot_ui),
      ("ShowBrakeStatus", self.show_brake_status),
      ("BPShowLiveDelayIndicator", self.show_live_delay),
      ("BPAnimateSteeringWheel", self.animate_steering_wheel),
      ("FordPrefHybridPowerFlow", self.show_hybrid_power_flow),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _update_state(self):
    super()._update_state()
    self.show_lead_vehicle._load_value()
    self.hybrid_power_flow_style._load_value()
    self._update_buttons()

  def show_event(self):
    super().show_event()
    self._update_toggles()
    self._update_buttons()

  def _update_buttons(self):
    ui_state.update_params()
    power_flow_enabled = ui_state.params.get_bool("FordPrefHybridPowerFlow")
    self.hybrid_power_flow_style.set_enabled(power_flow_enabled)

  def _update_toggles(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    wheel_style = get_steering_wheel_icon_style(ui_state.params, SteeringWheelIconStyle.COMMA_4)
    self.wheel_icon_style.set_value(self.wheel_icon_style._options[int(wheel_style)])
    dm_style = get_dm_icon_style(ui_state.params, DMIconStyle.COMMA_4)
    self.dm_icon_style.set_value(self.dm_icon_style._options[int(dm_style)])
