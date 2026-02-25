import pyray as rl
from collections.abc import Callable

from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP, BigParamControlBP, BigMultiToggleBP, BigMultiParamToggleBP
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamFloatControl, BigParamIntControl
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import NavWidget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.mici.widgets.web_server_qr_dialog import WebServerQRDialog

class BluePilotLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)
    self._params = Params()
    self.lane_change_factor_high = float(self._params.get("lane_change_factor_high", return_default=True))

    # ******** Main Scroller ********
    self.enable_web_routes = BigParamControlBP("enable web routes server", "EnableWebRoutesServer")
    self.show_web_routes_qr = BigButtonBP("show QR code", "", "icons_mici/settings/network/wifi_strength_full.png")
    self.show_web_routes_qr.set_click_callback(self._show_qr_dialog)
    self.show_hands_free_ui = BigParamControlBP("show hands-free ui", "send_hands_free_cluster_msg")
    self.show_lead_vehicle = BigMultiParamToggleBP("Lower Right Display", "mici_complication", ["off", "lead car speed", "speed", "lead car distance", "time to lead car"])
    self.show_brake_status = BigParamControlBP("show brake status", "ShowBrakeStatus")
    self.show_blindspot_ui = BigParamControlBP("show blindspot warning", "ShowBlindspotOverlay")
    self.rainbow_mode = BigParamControlBP("rainbow mode", "RainbowMode")
    self.enable_human_turn_detection = BigParamControlBP("enable human turn detection", "enable_human_turn_detection")
    self.lane_change_factor_high = BigParamFloatControl("lane change factor high", "lane_change_factor_high", min=0.5, max=1.0)
    self.enable_lane_positioning = BigParamControlBP("enable lane positioning", "enable_lane_positioning", tint=rl.GREEN)
    self.custom_path_offset = BigParamFloatControl("in-lane offset", "custom_path_offset", is_active_param="enable_lane_positioning", min=-0.5, max=0.5, tint=rl.GREEN)
    self.enable_lane_full_mode = BigParamControlBP("enable lanefull mode", "enable_lane_full_mode", is_active_param="enable_lane_positioning", tint=rl.GREEN)
    self.custom_profile = BigParamControlBP("use custom tuning profile", "custom_profile", tint=rl.BLUE)
    self.pc_blend_ratio_high_C = BigParamFloatControl("predicted curvature blend ratio high", "pc_blend_ratio_high_C_UI", is_active_param="custom_profile", min=0.0, max=1.0, tint=rl.BLUE)
    self.pc_blend_ratio_low_C = BigParamFloatControl("predicted curvature blend ratio low", "pc_blend_ratio_low_C_UI", is_active_param="custom_profile", min=0.0, max=1.0, tint=rl.BLUE)
    self.LC_PID_gain = BigParamFloatControl("low curvature PID gain", "LC_PID_gain_UI", is_active_param="custom_profile", min=0.0, max=5.0, tint=rl.BLUE)
    self.disable_lane_change_under_speed = BigParamControlBP("disable auto lane change under speed", "BlinkerPauseLaneChange")
    self.blinker_min_speed = BigParamIntControl("blinker min lane change speed", "BlinkerMinLateralControlSpeed", min=5, max=50, step=5.0)
    self.animate_steering_wheel = BigParamControlBP("animate steering wheel", "BPAnimateSteeringWheel")
    self.hide_fade = BigParamControlBP("hide onroad fade", "mici_hide_onroad_fade")
    self.hide_border = BigParamControlBP("hide screen border", "mici_hide_onroad_border")
    self.disable_BP_lat = BigParamControlBP("disable BP lateral control", "disable_BP_lat_UI")
    self.disable_BP_long = BigParamControlBP("bypass BP longitudinal control", "disable_BP_long_UI")
    self.ui_debug_log = BigParamControlBP("ui debug logging", "BPUIDebugLog")
    self.vbatt_pause_charging = BigParamFloatControl("12V battery limit", "vbatt_pause_charging", min=11.0, max=14.0, step=0.1)

    def power_flow_callback(value: str):
      match value:
        case "off":
          self._params.put_bool("FordPrefHybridPowerFlow", False)
          self._params.put_bool("FordPrefHybridPowerFlowAlternate", False)
        case "bar":
          self._params.put_bool("FordPrefHybridPowerFlow", True)
          self._params.put_bool("FordPrefHybridPowerFlowAlternate", False)
        case "circular":
          self._params.put_bool("FordPrefHybridPowerFlow", True)
          self._params.put_bool("FordPrefHybridPowerFlowAlternate", True)

    self.show_hybrid_power_flow = BigMultiToggleBP("show hybrid/EV power flow", ["off", "bar", "circular"], select_callback=power_flow_callback)

    #self.charging_btn = BigButton("charging", "", "icons_mici/settings/charge_icon.png")
    #self.charging_btn.set_click_callback(lambda: self._show_charging_view())

    self._scroller = Scroller([
      self.enable_web_routes,
      self.show_web_routes_qr,
      self.show_hands_free_ui,
      self.show_lead_vehicle,
      self.show_brake_status,
      self.show_blindspot_ui,
      self.show_hybrid_power_flow,
      self.rainbow_mode,
      self.enable_human_turn_detection,
      self.lane_change_factor_high,
      self.disable_lane_change_under_speed,
      self.blinker_min_speed,
      self.enable_lane_positioning,
      self.custom_path_offset,
      self.enable_lane_full_mode,
      self.custom_profile,
      self.pc_blend_ratio_high_C,
      self.pc_blend_ratio_low_C,
      self.LC_PID_gain,
      self.animate_steering_wheel,
      self.hide_fade,
      self.hide_border,
      self.vbatt_pause_charging,
      self.disable_BP_lat,
      self.disable_BP_long,
      self.ui_debug_log,
    ], snap_items=False)

    # Toggle lists
    self._refresh_toggles = (
      ("EnableWebRoutesServer", self.enable_web_routes),
      ("send_hands_free_cluster_msg", self.show_hands_free_ui),
      ("FordPrefHybridPowerFlow", self.show_hybrid_power_flow),
      ("ShowBrakeStatus", self.show_brake_status),
      ("ShowBlindspotOverlay", self.show_blindspot_ui),
      ("RainbowMode", self.rainbow_mode),
      ("enable_human_turn_detection", self.enable_human_turn_detection),
      ("BlinkerPauseLaneChange", self.disable_lane_change_under_speed),
      ("enable_lane_positioning", self.enable_lane_positioning),
      ("enable_lane_full_mode", self.enable_lane_full_mode),
      ("custom_profile", self.custom_profile),
      ("disable_BP_lat_UI", self.disable_BP_lat),
      ("disable_BP_long_UI", self.disable_BP_long),
      ("BPAnimateSteeringWheel", self.animate_steering_wheel),
      ("BPUIDebugLog", self.ui_debug_log),
      ("mici_hide_onroad_fade", self.hide_fade),
      ("BPHideOnroadBorder", self.hide_border),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  # def _show_charging_view(self):
  #   dlg = BigChargingDialog()
  #   gui_app.set_modal_overlay(dlg)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()
    self._update_buttons()

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)

  def _show_qr_dialog(self):
    """Show QR code dialog for webserver access."""
    # Only show if server is enabled
    if self._params.get_bool("EnableWebRoutesServer"):
      qr_dialog = WebServerQRDialog(back_callback=lambda: gui_app.set_modal_overlay(None))
      gui_app.set_modal_overlay(qr_dialog)
    # If disabled, could show a message dialog, but for now just do nothing

  def _update_state(self):
    super()._update_state()
    self.show_lead_vehicle._load_value()

  def _update_buttons(self):
    """Update button enabled state based on server status."""
    ui_state.update_params()
    server_enabled = ui_state.params.get_bool("EnableWebRoutesServer")
    self.show_web_routes_qr.set_enabled(server_enabled)

    if self._params.get_bool("FordPrefHybridPowerFlow"):
      if self._params.get_bool("FordPrefHybridPowerFlowAlternate"):
        self.show_hybrid_power_flow.set_value("circular")
      else:
        self.show_hybrid_power_flow.set_value("bar")
    else:
      self.show_hybrid_power_flow.set_value("off")

  def _update_toggles(self):
    ui_state.update_params()

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))

    # Also update button state
    self._update_buttons()

# class BigChargingDialog(BigDialogBase):
#   def __init__(self):
#     super().__init__(None, None)

#     self._watt_label = MiciLabel("120kW", font_size=90)
#     self._watt_label.set_position(150,75)

#   def _render(self, _):
#     self._watt_label.render()
#     return self._ret

#   def _update_state(self):
#     super()._update_state()
#     if self._swiping_away:
#       self._ret = DialogResult.CANCEL
