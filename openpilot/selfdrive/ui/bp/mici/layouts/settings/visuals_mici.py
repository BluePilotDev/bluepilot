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
  BigMultiParamToggleStrBP,
)
# BluePilot: seasonal theme packs (discovered on disk at panel build time)
from openpilot.selfdrive.ui.bp.lib import theme_pack
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller import NavScroller


class VisualsLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

    self.show_lead_vehicle = BigMultiParamToggleBP(
      "Lower Right Display", "mici_complication",
      ["off", "lead car speed", "speed", "lead car distance", "time to lead car"],
    )
    self.rainbow_mode = BigParamControlBP("Rainbow Mode", "RainbowMode")
    # BluePilot: one theme selector for everything (8-Bit Racer + seasonal packs),
    # same entries and param as the C3X page — see theme_pack.selector_entries().
    _theme_entries = theme_pack.selector_entries()
    self.theme_pack_sel = BigMultiParamToggleStrBP(
      "Theme", theme_pack.PARAM_KEY,
      [label for label, _ in _theme_entries], values=[value for _, value in _theme_entries],
      value_size=24,
    )
    # Auto seasonal: date-driven pack during holiday weeks; the manual Theme
    # selection above still applies outside those windows.
    self.theme_auto_seasonal = BigParamControlBP("Auto Seasonal Theme", theme_pack.AUTO_PARAM_KEY)
    self.hide_fade = BigParamControlBP("Hide Onroad Fade", "mici_hide_onroad_fade")
    self.hide_border = BigParamControlBP("Hide Onroad Border", "BPHideOnroadBorder")
    self.hide_camera_view = BigParamControlBP("Minimal Driving View", "BPHideCameraView")
    self.rainbow_lane_lines = BigParamControlBP("Rainbow Lane Lines", "BPRainbowLines")
    self.show_blindspot_ui = BigParamControlBP("Show Blindspot Overlay", "ShowBlindspotOverlay")
    self.show_brake_status = BigParamControlBP("Show Brake Status", "ShowBrakeStatus")
    self.animate_steering_wheel = BigParamControlBP("Animate Steering Wheel", "BPAnimateSteeringWheel")
    ensure_steering_wheel_icon_style_initialized(Params(), SteeringWheelIconStyle.COMMA_4)
    self.wheel_icon_style = BigMultiParamToggleBP(
      "Wheel Icon Style", "BPSteeringWheelIconStyle", ["Comma 4", "Comma 3x"],
    )
    ensure_dm_icon_style_initialized(Params(), DMIconStyle.COMMA_4)
    self.dm_icon_style = BigMultiParamToggleBP(
      "DM Icon Style", "BPDMStylingChoice", ["Comma 4", "Comma 3x"],
    )
    self.show_hybrid_power_flow = BigParamControlBP("Show Hybrid Power Flow", "FordPrefHybridPowerFlow")
    self.hybrid_power_flow_style = BigMultiParamToggleBoolBP(
      "Hybrid/EV Power Flow Style", "FordPrefHybridPowerFlowAlternate", ["flat", "round"],
    )

    self._scroller.add_widgets([
      self.show_lead_vehicle,
      self.rainbow_mode,
      self.theme_pack_sel,
      self.theme_auto_seasonal,
      self.hide_fade,
      self.hide_border,
      self.hide_camera_view,
      self.rainbow_lane_lines,
      self.show_blindspot_ui,
      self.show_brake_status,
      self.animate_steering_wheel,
      self.wheel_icon_style,
      self.dm_icon_style,
      self.show_hybrid_power_flow,
      self.hybrid_power_flow_style,
    ])

    self._refresh_toggles = (
      ("RainbowMode", self.rainbow_mode),
      ("mici_hide_onroad_fade", self.hide_fade),
      ("BPHideOnroadBorder", self.hide_border),
      ("BPHideCameraView", self.hide_camera_view),
      ("BPThemeAutoSeasonal", self.theme_auto_seasonal),
      ("BPRainbowLines", self.rainbow_lane_lines),
      ("ShowBlindspotOverlay", self.show_blindspot_ui),
      ("ShowBrakeStatus", self.show_brake_status),
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
