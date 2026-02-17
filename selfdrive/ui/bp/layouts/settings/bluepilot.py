import pyray as rl
from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.list_view import toggle_item, button_item, text_item, ButtonAction, ListItem
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.bp.lib.favorite_wifi_manager import FavoriteWifiManager
from openpilot.selfdrive.ui.bp.widgets.web_server_qr_dialog_tici import WebServerQRDialogTici
from openpilot.selfdrive.ui.bp.widgets.float_control_item import float_control_item


class BluePilotLayout(Widget):
  """BluePilot settings layout for TICI UI."""

  def __init__(self):
    super().__init__()
    self._params = Params()
    
    # Create WifiManager instance for preferred network selector
    self._wifi_manager = WifiManager()
    self._wifi_manager.set_active(False)  # Don't scan unless needed
    self._saved_networks: list[Network] = []
    self._preferred_network_dialog: MultiOptionDialog | None = None

    # Start standalone favorite-network auto-connect thread (runs in background)
    self._favorite_wifi_manager = FavoriteWifiManager(self._wifi_manager)

    # Register callback to update saved networks list
    self._wifi_manager.add_callbacks(networks_updated=self._on_network_updated)

    # Initialize items
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

    # Toggle refresh list
    self._refresh_toggles = (
      ("BPPortalEnabled", self._enable_web_routes),
      ("send_hands_free_cluster_msg", self._show_hands_free_ui),
      ("BlindSpot", self._show_blindspot),
      ("ShowBrakeStatus", self._show_brake_status),
      ("RoadNameToggle", self._show_road_name),
      ("FordPrefShowRadarLeadOverlay", self._show_ford_radar_overlay),
      ("FordPrefHybridBatteryStatus", self._show_hybrid_battery_status),
      ("FordPrefHybridPowerFlow", self._show_hybrid_power_flow),
      ("enable_human_turn_detection", self._enable_human_turn_detection),
      ("enable_lane_positioning", self._enable_lane_positioning),
      ("enable_lane_full_mode", self._enable_lane_full_mode),
      ("custom_profile", self._custom_profile),
      ("disable_BP_lat_UI", self._disable_BP_lat),
      ("disable_BP_long_UI", self._disable_BP_long),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _initialize_items(self):
    """Initialize all BluePilot menu items."""

    # Web routes server toggle
    self._enable_web_routes = toggle_item(
      lambda: tr("Enable Web Routes Server"),
      lambda: tr("Enable the web routes server for viewing logs and videos over WiFi."),
      initial_state=self._params.get_bool("BPPortalEnabled"),
      callback=lambda state: self._toggle_callback(state, "BPPortalEnabled"),
      icon="chffr_wheel.png"
    )

    # Show QR code button
    self._show_web_routes_qr = button_item(
      lambda: tr("Show QR Code"),
      lambda: tr("SHOW"),
      lambda: tr("Display QR code for connecting to the web routes server."),
      callback=self._show_qr_dialog,
      enabled=lambda: self._params.get_bool("BPPortalEnabled")
    )

    # Hands-free UI toggle
    self._show_hands_free_ui = toggle_item(
      lambda: tr("Show Hands-Free UI"),
      lambda: tr("Display hands-free UI elements."),
      initial_state=self._params.get_bool("send_hands_free_cluster_msg"),
      callback=lambda state: self._toggle_callback(state, "send_hands_free_cluster_msg"),
      icon="monitoring.png"
    )

    # Blindspot overlay toggle
    self._show_blindspot = toggle_item(
      lambda: tr("Show Blindspot Overlay"),
      lambda: tr("Display red overlay when vehicle is detected in blindspot."),
      initial_state=self._params.get_bool("BlindSpot"),
      callback=lambda state: self._toggle_callback(state, "BlindSpot"),
      icon="warning.png"
    )

    # Brake status toggle
    self._show_brake_status = toggle_item(
      lambda: tr("Show Brake Status"),
      lambda: tr("Display speed setpoint in red when vehicle is braking."),
      initial_state=self._params.get_bool("ShowBrakeStatus"),
      callback=lambda state: self._toggle_callback(state, "ShowBrakeStatus"),
      icon="warning.png"
    )

    # Road name display toggle (uses map data from mapd)
    self._show_road_name = toggle_item(
      lambda: tr("Display Road Name"),
      lambda: tr("Show current road name from map data in a pill on the driving screen."),
      initial_state=(self._params.get("RoadNameToggle") or "1") == "1",
      callback=lambda state: self._params.put("RoadNameToggle", "1" if state else "0"),
      icon="speed_limit.png"
    )

    # Ford radar lead overlay toggle
    self._show_ford_radar_overlay = toggle_item(
      lambda: tr("Show Radar Lead Overlay (Ford ACC)"),
      lambda: tr("Display chevron with lead vehicle info when using Ford stock ACC."),
      initial_state=self._params.get_bool("FordPrefShowRadarLeadOverlay"),
      callback=lambda state: self._toggle_callback(state, "FordPrefShowRadarLeadOverlay"),
      icon="speed_limit.png"
    )

    # Hybrid battery status toggle
    self._show_hybrid_battery_status = toggle_item(
      lambda: tr("Show Hybrid/EV Battery Status"),
      lambda: tr("Display hybrid battery gauge with SOC, voltage, and amps."),
      initial_state=self._params.get_bool("FordPrefHybridBatteryStatus"),
      callback=lambda state: self._toggle_callback(state, "FordPrefHybridBatteryStatus"),
      icon="warning.png"
    )

    # Hybrid power flow toggle
    self._show_hybrid_power_flow = toggle_item(
      lambda: tr("Show Hybrid/EV Power Flow"),
      lambda: tr("Display power flow gauge showing throttle demand and regenerative braking."),
      initial_state=self._params.get_bool("FordPrefHybridPowerFlow"),
      callback=lambda state: self._toggle_callback(state, "FordPrefHybridPowerFlow"),
      icon="warning.png"
    )

    # Human turn detection toggle
    self._enable_human_turn_detection = toggle_item(
      lambda: tr("Enable Human Turn Detection"),
      lambda: tr("Enable detection of human-initiated turns."),
      initial_state=self._params.get_bool("enable_human_turn_detection"),
      callback=lambda state: self._toggle_callback(state, "enable_human_turn_detection"),
      icon="warning.png"
    )

    # Lane change factor high (float)
    self._lane_change_factor_high = float_control_item(
      lambda: tr("Lane Change Factor High"),
      lambda: tr("Adjust the high-speed lane change factor (0.5-1.0)."),
      param="lane_change_factor_high",
      min_value=0.5,
      max_value=1.0,
      step=0.05,
      icon="speed_limit.png"
    )

    # Enable lane positioning toggle
    self._enable_lane_positioning = toggle_item(
      lambda: tr("Enable Lane Positioning"),
      lambda: tr("Enable custom lane positioning controls."),
      initial_state=self._params.get_bool("enable_lane_positioning"),
      callback=lambda state: self._toggle_callback(state, "enable_lane_positioning"),
      icon="chffr_wheel.png"
    )

    # Custom path offset (float, conditional on lane positioning)
    self._custom_path_offset = float_control_item(
      lambda: tr("In-Lane Offset"),
      lambda: tr("Adjust the in-lane offset (-0.5 to 0.5)."),
      param="custom_path_offset",
      min_value=-0.5,
      max_value=0.5,
      step=0.05,
      enabled=lambda: self._params.get_bool("enable_lane_positioning"),
      icon="chffr_wheel.png"
    )

    # Enable lanefull mode toggle (conditional on lane positioning)
    self._enable_lane_full_mode = toggle_item(
      lambda: tr("Enable Lanefull Mode"),
      lambda: tr("Enable lanefull mode for lane positioning."),
      initial_state=self._params.get_bool("enable_lane_full_mode"),
      callback=lambda state: self._toggle_callback(state, "enable_lane_full_mode"),
      enabled=lambda: self._params.get_bool("enable_lane_positioning"),
      icon="chffr_wheel.png"
    )

    # Custom profile toggle
    self._custom_profile = toggle_item(
      lambda: tr("Use Custom Tuning Profile"),
      lambda: tr("Enable custom tuning profile settings."),
      initial_state=self._params.get_bool("custom_profile"),
      callback=lambda state: self._toggle_callback(state, "custom_profile"),
      icon="chffr_wheel.png"
    )

    # Predicted curvature blend ratio high (float, conditional on custom profile)
    self._pc_blend_ratio_high_C = float_control_item(
      lambda: tr("Predicted Curvature Blend Ratio High"),
      lambda: tr("Adjust the high curvature blend ratio (0.0-1.0)."),
      param="pc_blend_ratio_high_C_UI",
      min_value=0.0,
      max_value=1.0,
      step=0.05,
      enabled=lambda: self._params.get_bool("custom_profile"),
      icon="chffr_wheel.png"
    )

    # Predicted curvature blend ratio low (float, conditional on custom profile)
    self._pc_blend_ratio_low_C = float_control_item(
      lambda: tr("Predicted Curvature Blend Ratio Low"),
      lambda: tr("Adjust the low curvature blend ratio (0.0-1.0)."),
      param="pc_blend_ratio_low_C_UI",
      min_value=0.0,
      max_value=1.0,
      step=0.05,
      enabled=lambda: self._params.get_bool("custom_profile"),
      icon="chffr_wheel.png"
    )

    # Low curvature PID gain (float, conditional on custom profile)
    self._lc_pid_gain = float_control_item(
      lambda: tr("Low Curvature PID Gain"),
      lambda: tr("Adjust the low curvature PID gain (0.0-5.0)."),
      param="LC_PID_gain_UI",
      min_value=0.0,
      max_value=5.0,
      step=0.1,
      enabled=lambda: self._params.get_bool("custom_profile"),
      icon="chffr_wheel.png"
    )

    # 12V battery limit (float)
    self._vbatt_pause_charging = float_control_item(
      lambda: tr("12V Battery Limit"),
      lambda: tr("Set the 12V battery charging pause limit (11.0-14.0V)."),
      param="vbatt_pause_charging",
      min_value=11.0,
      max_value=14.0,
      step=0.1,
      suffix="V",
      icon="warning.png"
    )

    # Disable BP lateral control toggle
    self._disable_BP_lat = toggle_item(
      lambda: tr("Disable BP Lateral Control"),
      lambda: tr("Disable BluePilot lateral control."),
      initial_state=self._params.get_bool("disable_BP_lat_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_BP_lat_UI"),
      icon="chffr_wheel.png"
    )

    # Bypass BP longitudinal control toggle (use stock long logic)
    self._disable_BP_long = toggle_item(
      lambda: tr("Bypass BP Longitudinal Control"),
      lambda: tr("Use stock longitudinal logic instead of BluePilot TTC/coasting tuning."),
      initial_state=self._params.get_bool("disable_BP_long_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_BP_long_UI"),
      icon="chffr_wheel.png"
    )

    # Preferred WiFi Network selector
    self._preferred_network_action = ButtonAction(lambda: tr("SELECT"))
    self._preferred_network_action.set_value(lambda: self._get_preferred_network_display())
    self._preferred_network_btn = ListItem(
      lambda: tr("Preferred WiFi Network"),
      description=lambda: tr("Automatically connect to this network when available"),
      action_item=self._preferred_network_action,
      callback=self._select_preferred_network
    )

    return [
      self._enable_web_routes,
      self._show_web_routes_qr,
      self._show_hands_free_ui,
      self._show_blindspot,
      self._show_brake_status,
      self._show_road_name,
      self._show_ford_radar_overlay,
      self._show_hybrid_battery_status,
      self._show_hybrid_power_flow,
      self._enable_human_turn_detection,
      self._lane_change_factor_high,
      self._enable_lane_positioning,
      self._custom_path_offset,
      self._enable_lane_full_mode,
      self._custom_profile,
      self._pc_blend_ratio_high_C,
      self._pc_blend_ratio_low_C,
      self._lc_pid_gain,
      self._vbatt_pause_charging,
      self._disable_BP_lat,
      self._disable_BP_long,
      self._preferred_network_btn,
    ]

  def _get_float_param(self, param: str, default: float) -> float:
    """Get float parameter value."""
    try:
      return float(self._params.get(param, return_default=True))
    except (TypeError, ValueError):
      return default

  def _toggle_callback(self, state: bool, param: str):
    """Handle toggle state changes."""
    self._params.put_bool(param, state)
    self._update_toggles()

  def _show_qr_dialog(self):
    """Show QR code dialog for webserver access."""
    if self._params.get_bool("BPPortalEnabled"):
      qr_dialog = WebServerQRDialogTici()
      gui_app.set_modal_overlay(qr_dialog)

  def _update_toggles(self):
    """Update toggle states from params."""
    ui_state.update_params()

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      if key == "RoadNameToggle":
        item.action_item.set_state((ui_state.params.get(key) or "1") == "1")
      else:
        item.action_item.set_state(ui_state.params.get_bool(key))

    # Update button enabled states
    self._show_web_routes_qr.action_item.set_enabled(ui_state.params.get_bool("BPPortalEnabled"))
    self._custom_path_offset.action_item.set_enabled(ui_state.params.get_bool("enable_lane_positioning"))
    self._enable_lane_full_mode.action_item.set_enabled(ui_state.params.get_bool("enable_lane_positioning"))
    self._pc_blend_ratio_high_C.action_item.set_enabled(ui_state.params.get_bool("custom_profile"))
    self._pc_blend_ratio_low_C.action_item.set_enabled(ui_state.params.get_bool("custom_profile"))
    self._lc_pid_gain.action_item.set_enabled(ui_state.params.get_bool("custom_profile"))

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()
    # Enable WiFi scanning when BluePilot menu is shown
    self._wifi_manager.set_active(True)

  def hide_event(self):
    super().hide_event()
    # Disable WiFi scanning when BluePilot menu is hidden
    self._wifi_manager.set_active(False)

  def _on_network_updated(self, networks: list[Network]):
    """Update saved networks list when WiFi networks are updated"""
    self._saved_networks = [n for n in networks if n.is_saved]
    self._preferred_network_action.set_enabled(len(self._saved_networks) > 0)
    
    # Check if preferred network is still saved in NetworkManager
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      current_favorite = ""
      if favorite_value:
        if isinstance(favorite_value, bytes):
          current_favorite = favorite_value.decode('utf-8', errors='replace').strip('\x00')
        else:
          current_favorite = str(favorite_value).strip('\x00')
      if current_favorite:
        # Check NetworkManager's saved connections directly
        saved_connections = self._wifi_manager._get_connections()
        if current_favorite not in saved_connections:
          # Network is no longer saved, clear preferred setting
          self._params.put("WifiFavoriteSSID", "")
          cloudlog.info(f"Cleared preferred network '{current_favorite}' - network no longer saved in NetworkManager")
    except Exception as e:
      cloudlog.debug(f"Error checking preferred network: {e}")

  def _get_preferred_network_display(self) -> str:
    """Get the display text for preferred network"""
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      if favorite_value:
        if isinstance(favorite_value, bytes):
          favorite_ssid = favorite_value.decode('utf-8', errors='replace').strip('\x00')
        else:
          favorite_ssid = str(favorite_value).strip('\x00')
        if favorite_ssid:
          # Truncate if too long
          if len(favorite_ssid) > 20:
            return favorite_ssid[:17] + "..."
          return favorite_ssid
    except Exception:
      pass
    return tr("None")

  def _select_preferred_network(self):
    """Open dialog to select preferred network from saved networks"""
    if len(self._saved_networks) == 0:
      return
    
    # Get current favorite
    current_favorite = ""
    try:
      favorite_value = self._params.get("WifiFavoriteSSID")
      if favorite_value:
        if isinstance(favorite_value, bytes):
          current_favorite = favorite_value.decode('utf-8', errors='replace').strip('\x00')
        else:
          current_favorite = str(favorite_value).strip('\x00')
    except Exception:
      pass
    
    # Build list of network names (add "None" option first)
    network_options = [tr("None")]
    network_options.extend([n.ssid for n in self._saved_networks])
    
    # Create dialog
    self._preferred_network_dialog = MultiOptionDialog(
      tr("Select Preferred Network"),
      network_options,
      current_favorite if current_favorite else tr("None")
    )
    
    def handle_selection(result):
      """Handle selection from dialog"""
      if result == DialogResult.CONFIRM and self._preferred_network_dialog is not None:
        selection = self._preferred_network_dialog.selection
        # Convert "None" back to empty string
        if selection == tr("None"):
          selection = ""
        
        # Save the selection
        self._params.put("WifiFavoriteSSID", selection)
        if selection:
          cloudlog.info(f"Set preferred network: {selection}")
        else:
          cloudlog.info("Cleared preferred network")
        
        # Update button value display
        self._preferred_network_action.set_value(self._get_preferred_network_display())
      
      self._preferred_network_dialog = None
    
    gui_app.set_modal_overlay(self._preferred_network_dialog, callback=handle_selection)

  def _render(self, rect):
    # Process WiFi manager callbacks
    self._wifi_manager.process_callbacks()
    self._scroller.render(rect)
