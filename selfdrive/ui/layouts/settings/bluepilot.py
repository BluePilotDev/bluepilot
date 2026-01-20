import pyray as rl
from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.list_view import toggle_item, button_item, text_item
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.widgets.web_server_qr_dialog_tici import WebServerQRDialogTici
from openpilot.selfdrive.ui.widgets.float_control_item import float_control_item


class BluePilotLayout(Widget):
  """BluePilot settings layout for TICI UI."""
  
  def __init__(self):
    super().__init__()
    self._params = Params()
    
    # Initialize items
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)
    
    # Toggle refresh list
    self._refresh_toggles = (
      ("BPPortalEnabled", self._enable_web_routes),
      ("send_hands_free_cluster_msg", self._show_hands_free_ui),
      ("show_lead_speed", self._show_lead_vehicle),
      ("enable_human_turn_detection", self._enable_human_turn_detection),
      ("enable_lane_positioning", self._enable_lane_positioning),
      ("enable_lane_full_mode", self._enable_lane_full_mode),
      ("custom_profile", self._custom_profile),
      ("disable_BP_lat_UI", self._disable_BP_lat),
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
    
    # Lead vehicle speed toggle
    self._show_lead_vehicle = toggle_item(
      lambda: tr("Show Lead Vehicle Speed"),
      lambda: tr("Display the lead vehicle's speed on the UI."),
      initial_state=self._params.get_bool("show_lead_speed"),
      callback=lambda state: self._toggle_callback(state, "show_lead_speed"),
      icon="speed_limit.png"
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
    
    return [
      self._enable_web_routes,
      self._show_web_routes_qr,
      self._show_hands_free_ui,
      self._show_lead_vehicle,
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
  
  def _render(self, rect):
    self._scroller.render(rect)
