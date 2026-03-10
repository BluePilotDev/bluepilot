import pyray as rl
from collections.abc import Callable

from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP, BigParamControlBP, BigMultiToggleBP, BigMultiParamToggleBP, BigMultiParamToggleBoolBP
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamFloatControl, BigParamIntControl
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.nav_widget import NavWidget
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.selfdrive.ui.bp.mici.widgets.preferred_network_select import PreferredNetworkSelectMici

class BluePilotLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)
    self._params = Params()
    self.lane_change_factor_high = float(self._params.get("lane_change_factor_high", return_default=True))

    # WifiManager for preferred network selector (same pattern as TICI BluePilotLayout)
    self._wifi_manager = WifiManager()
    self._wifi_manager.set_active(False)  # Don't scan unless menu is shown
    self._saved_networks: list[Network] = []
    self._wifi_manager.add_callbacks(networks_updated=self._on_network_updated)

    # Preferred WiFi network selector (same as TICI - list of saved networks)
    self.preferred_network_btn = BigButtonBP(
      tr("Preferred WiFi Network"),
      "",
      "icons_mici/settings/network/wifi_strength_full.png"
    )
    self.preferred_network_btn.set_click_callback(self._select_preferred_network)

    # ******** Main Scroller ********
    self.enable_web_routes = BigParamControlBP("enable web routes server", "EnableWebRoutesServer")
    self.show_web_routes_qr = BigButtonBP("show QR code", "", "icons_mici/settings/network/wifi_strength_full.png")
    self.show_web_routes_qr.set_click_callback(self._show_qr_dialog)
    self.show_hands_free_ui = BigParamControlBP("show BlueCruise icon on dash", "send_hands_free_cluster_msg")
    self.show_lead_vehicle = BigMultiParamToggleBP("Lower Right Display", "mici_complication", ["off", "lead car speed", "speed", "lead car distance", "time to lead car"])
    self.show_brake_status = BigParamControlBP("show brake status", "ShowBrakeStatus")
    self.show_blindspot_ui = BigParamControlBP("show blindspot overlay", "ShowBlindspotOverlay")
    self.rainbow_mode = BigParamControlBP("rainbow mode", "RainbowMode")
    self.enable_human_turn_detection = BigParamControlBP("enable human turn detection", "enable_human_turn_detection")
    self.lane_change_factor_high = BigParamFloatControl("lane change factor high", "lane_change_factor_high", min=0.5, max=1.0)
    self.enable_lane_positioning = BigParamControlBP("enable lane positioning", "enable_lane_positioning", tint=rl.GREEN)
    self.custom_path_offset = BigParamFloatControl("in-lane offset", "custom_path_offset", is_active_param="enable_lane_positioning", min=-0.5, max=0.5, tint=rl.GREEN)
    self.enable_lane_full_mode = BigParamControlBP("enable lanefull mode", "enable_lane_full_mode", is_active_param="enable_lane_positioning", tint=rl.GREEN)
    self.custom_profile = BigParamControlBP("use custom tuning profile", "custom_profile", tint=rl.BLUE)
    self.pc_blend_ratio_high_C = BigParamFloatControl("predicted curvature blend ratio high", "pc_blend_ratio_high_C_UI", is_active_param="custom_profile", min=0.0, max=1.0, tint=rl.BLUE)
    self.pc_blend_ratio_low_C = BigParamFloatControl("predicted curvature blend ratio low", "pc_blend_ratio_low_C_UI", is_active_param="custom_profile", min=0.0, max=1.0, tint=rl.BLUE)
    self.LC_PID_gain = BigParamFloatControl(
      "low curvature PID gain",
      "LC_PID_gain_UI",
      is_active=lambda: self._params.get_bool("enable_lane_positioning") and self._params.get_bool("custom_profile"),
      min=0.0,
      max=5.0,
      tint=rl.BLUE,
    )
    self.disable_lane_change_under_speed = BigParamControlBP("disable auto lane change under speed", "BlinkerPauseLaneChange")
    self.blinker_min_speed = BigParamIntControl("blinker min lane change speed", "BlinkerMinLateralControlSpeed", min=5, max=50, step=5.0)
    self.animate_steering_wheel = BigParamControlBP("animate steering wheel", "BPAnimateSteeringWheel")
    self.hide_fade = BigParamControlBP("hide onroad fade", "mici_hide_onroad_fade")
    self.hide_border = BigParamControlBP("hide screen border", "mici_hide_onroad_border")
    self.disable_BP_lat = BigParamControlBP("disable BP lateral control", "disable_BP_lat_UI")
    self.disable_BP_long = BigParamControlBP("bypass BP longitudinal control", "disable_BP_long_UI")
    self.disable_dowhill_comp = BigParamControlBP("disable downhill compensation", "disable_downhill_comp_UI")
    self.clear_model_cache = BigButtonBP("clear crashed model", "", "icons_mici/settings/device/reboot.png")
    self.clear_model_cache.set_click_callback(self._clear_model_cache)
    self.ui_debug_log = BigParamControlBP("ui debug logging", "BPUIDebugLog")
    self.vbatt_pause_charging = BigParamFloatControl("12V battery limit", "vbatt_pause_charging", min=11.0, max=14.0, step=0.1)

    # Hybrid/EV power flow: enable toggle (like C3X) + style dropdown Flat/Round (C4), same pattern as Lower Right Display
    self.show_hybrid_power_flow = BigParamControlBP("show hybrid/EV power flow", "FordPrefHybridPowerFlow")

    self.hybrid_power_flow_style = BigMultiParamToggleBoolBP(
      "hybrid/EV power flow style",
      "FordPrefHybridPowerFlowAlternate",
      ["flat", "round"]
    )

    #self.charging_btn = BigButton("charging", "", "icons_mici/settings/charge_icon.png")
    #self.charging_btn.set_click_callback(lambda: self._show_charging_view())

    self._scroller = Scroller(snap_items=False)
    self._scroller._scroller.add_widgets([
      self.enable_web_routes,
      self.show_web_routes_qr,
      self.preferred_network_btn,
      self.show_hands_free_ui,
      self.show_lead_vehicle,
      self.show_brake_status,
      self.show_blindspot_ui,
      self.show_hybrid_power_flow,
      self.hybrid_power_flow_style,
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
      self.disable_dowhill_comp,
      self.clear_model_cache,
      self.ui_debug_log,
    ])

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
      ("disable_downhill_comp_UI", self.disable_dowhill_comp),
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
    # Enable WiFi scanning when BluePilot menu is shown
    self._wifi_manager.set_active(True)
    self.preferred_network_btn.set_value(self._get_preferred_network_display())

  def hide_event(self):
    super().hide_event()
    # Disable WiFi scanning when BluePilot menu is hidden
    self._wifi_manager.set_active(False)

  def _render(self, rect: rl.Rectangle):
    self._wifi_manager.process_callbacks()
    self._scroller.render(rect)

  def _clear_model_cache(self):
    """Clear ModelRunnerTypeCache and ModelManager_ActiveBundle, then reboot."""

    def handle_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        try:
          self._params.remove("ModelRunnerTypeCache")
        except Exception:
          pass
        try:
          self._params.remove("ModelManager_ActiveBundle")
        except Exception:
          pass
        self._params.put_bool_nonblocking("DoReboot", True)
        cloudlog.info("BluePilot: Cleared model cache (ModelRunnerTypeCache, ModelManager_ActiveBundle), triggered reboot")

    dialog = ConfirmDialog(
      tr("Clear crashed model runner cache and reboot? This fixes 'Communication Issue' when modeld fails to start."),
      tr("Clear & Reboot"),
      callback=handle_confirm
    )
    gui_app.push_widget(dialog)

  def _show_qr_dialog(self):
    """Show QR code dialog for webserver access. MICI uses push_widget/pop_widget (no set_modal_overlay)."""
    if not self._params.get_bool("EnableWebRoutesServer"):
      return
    try:
      qr_dialog = WebServerQRDialog(back_callback=gui_app.pop_widget)
      gui_app.push_widget(qr_dialog)
    except Exception as e:
      from openpilot.common.swaglog import cloudlog
      cloudlog.warning(f"Failed to show QR dialog: {e}")

  def _update_state(self):
    super()._update_state()
    self.show_lead_vehicle._load_value()
    self.hybrid_power_flow_style._load_value()
    # Refresh dependent control enabled state (e.g. after toggling enable_lane_positioning)
    self._update_buttons()

  def _update_buttons(self):
    """Update button enabled state based on server status and parameter dependencies (see MICI_MENU.csv)."""
    ui_state.update_params()
    p = self._params

    # Web routes QR: only when server enabled
    server_enabled = ui_state.params.get_bool("EnableWebRoutesServer")
    self.show_web_routes_qr.set_enabled(server_enabled)

    # Hybrid/EV power flow style (flat/round): only when power flow is enabled
    power_flow_enabled = p.get_bool("FordPrefHybridPowerFlow")
    self.hybrid_power_flow_style.set_enabled(power_flow_enabled)

    # Lane positioning–dependent controls (prereq: Enable Advanced Lane Positioning)
    lane_positioning_enabled = p.get_bool("enable_lane_positioning")
    self.custom_path_offset.set_enabled(lane_positioning_enabled)
    self.enable_lane_full_mode.set_enabled(lane_positioning_enabled)

    # Custom profile–dependent controls (prereq: Use Custom Tuning Profile)
    custom_profile_enabled = p.get_bool("custom_profile")
    self.pc_blend_ratio_high_C.set_enabled(custom_profile_enabled)
    self.pc_blend_ratio_low_C.set_enabled(custom_profile_enabled)

    # Low Curvature PID Gain: requires BOTH lane positioning AND custom profile
    self.LC_PID_gain.set_enabled(lane_positioning_enabled and custom_profile_enabled)

    # Preferred WiFi Network: enable when saved networks exist, refresh display value
    self.preferred_network_btn.set_enabled(len(self._saved_networks) > 0)
    self.preferred_network_btn.set_value(self._get_preferred_network_display())

  def _on_network_updated(self, networks: list[Network]):
    """Update saved networks list when WiFi networks are updated (callback from WifiManager)."""
    self._saved_networks = [n for n in networks if self._wifi_manager.is_connection_saved(n.ssid)]
    self.preferred_network_btn.set_enabled(len(self._saved_networks) > 0)
    self.preferred_network_btn.set_value(self._get_preferred_network_display())

    # Clear preferred if network was forgotten in NetworkManager
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      current_favorite = ""
      if favorite_value:
        if isinstance(favorite_value, bytes):
          current_favorite = favorite_value.decode("utf-8", errors="replace").strip("\x00")
        else:
          current_favorite = str(favorite_value).strip("\x00")
      if current_favorite:
        saved_connections = self._wifi_manager._get_connections()
        if current_favorite not in saved_connections:
          self._params.put("WifiFavoriteSSID", "")
          cloudlog.info(f"Cleared preferred network '{current_favorite}' - network no longer saved in NetworkManager")
    except Exception as e:
      cloudlog.debug(f"Error checking preferred network: {e}")

  def _get_preferred_network_display(self) -> str:
    """Get the display text for preferred network."""
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      if favorite_value:
        if isinstance(favorite_value, bytes):
          favorite_ssid = favorite_value.decode("utf-8", errors="replace").strip("\x00")
        else:
          favorite_ssid = str(favorite_value).strip("\x00")
        if favorite_ssid:
          if len(favorite_ssid) > 20:
            return favorite_ssid[:17] + "..."
          return favorite_ssid
    except Exception:
      pass
    return tr("None")

  def _select_preferred_network(self):
    """Open horizontal-scroll panel to select preferred network (same pattern as WiFi network panel)."""
    if len(self._saved_networks) == 0:
      return

    panel = PreferredNetworkSelectMici(
      self._wifi_manager,
      self._saved_networks,
      on_dismiss=lambda: self.preferred_network_btn.set_value(self._get_preferred_network_display())
    )
    gui_app.push_widget(panel)

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
