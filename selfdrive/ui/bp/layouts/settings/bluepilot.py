import pyray as rl

from openpilot.common.params import Params
from openpilot.common.params_pyx import UnknownKeyName
from openpilot.common.swaglog import cloudlog
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import toggle_item, multiple_button_item, button_item, ButtonAction, ListItem
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.bp.widgets.float_control_item import float_control_item, int_control_item
from openpilot.selfdrive.ui.bp.widgets.section_header import CollapsibleSectionHeader
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
from openpilot.selfdrive.ui.bp.lib.custom_sound import get_custom_sound_selection
from opendbc.sunnypilot.car.ford.lateral_curv_ext import PrimaryLateralControl
from openpilot.selfdrive.ui.bp.onroad.augmented_road_view_bp import GaugeStyle


class BluePilotLayout(Widget):
  """BluePilot settings layout for TICI UI."""

  @staticmethod
  def _safe_get_bool(params: Params, key: str, default: bool = False) -> bool:
    """Get bool param; return default if key is unknown (e.g. dev environment with reduced params)."""
    try:
      return params.get_bool(key)
    except UnknownKeyName:
      return default

  @staticmethod
  def _safe_get(params: Params, key: str, default=None):
    """Get param; return default if key is unknown (e.g. dev environment with reduced params)."""
    try:
      val = params.get(key, return_default=True)
      return val if val not in (None, b"", "") else default
    except UnknownKeyName:
      return default

  @staticmethod
  def _pinion_yaw_sensor_supported() -> bool:
    """FORD_EDGE_MK2's pinion sensor only reports a relative angle (see
    FORD_PINION_GEOMETRY_INDEX in opendbc/sunnypilot/car/ford/values_ext.py) -- the
    safety/control layers already no-op the toggle there, so grey it out here too rather
    than leaving a live-looking control that silently does nothing."""
    return ui_state.CP is None or ui_state.CP.carFingerprint != "FORD_EDGE_MK2"

  def __init__(self):
    super().__init__()
    self._params = Params()

    # Create WifiManager instance for preferred network selector
    self._wifi_manager = WifiManager()
    self._wifi_manager.set_active(False)  # Don't scan unless needed
    self._saved_networks: list[Network] = []
    self._preferred_network_dialog: MultiOptionDialog | None = None

    # Register callback to update saved networks list
    self._wifi_manager.add_callbacks(networks_updated=self._on_network_updated)

    # Initialize items
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

    # Toggle refresh list
    self._refresh_toggles = (
      ("send_hands_free_cluster_msg", self._show_hands_free_ui),
      ("FordPrefSteerAngleCurvature", self._steer_angle_curvature),
      ("FordAngleAutoCal", self._angle_autocal),
      ("FordAngleAutoCalLock", self._angle_autocal_lock),
      ("FordAngleSmoothing", self._angle_smoothing),
      ("BPDisableLaneLineStatusColor", self._disable_lane_line_status_color),
      ("BPHideCameraView", self._hide_camera_view),
      ("BPRadRacerTheme", self._rad_racer_theme),
      ("BPRainbowLines", self._rainbow_lane_lines),
      ("ShowBlindspotOverlay", self._show_blindspot),
      ("ShowBrakeStatus", self._show_brake_status),
      ("BPHideOnroadBorder", self._hide_onroad_border),
      ("BPShowConfidenceBall", self._show_confidence_ball),
      ("BPAnimateSteeringWheel", self._animate_steering_wheel),
      ("BPUseCustomSounds", self._use_custom_sounds),
      ("FordPrefShowRadarLeadOverlay", self._show_ford_radar_overlay),
      ("FordPrefHybridBatteryStatus", self._show_hybrid_battery_status),
      ("FordPrefHybridPowerFlow", self._show_hybrid_power_flow),
      ("enable_human_turn_detection_curv", self._enable_human_turn_detection),
      ("BlinkerPauseLaneChange", self._disable_lane_change_under_speed),
      ("enable_lane_positioning_curv", self._enable_lane_positioning),
      ("enable_lane_full_mode_curv", self._enable_lane_full_mode),
      ("custom_profile_curv", self._custom_profile),
      ("disable_BP_lat_UI", self._disable_BP_lat),
      ("disable_BP_long_UI", self._disable_BP_long),
      ("disable_downhill_comp_UI", self._disable_dowhill_comp),
      ("disable_ford_radar_UI", self._disable_ford_radar),
      ("BpShowLateralControl", self._show_lateral_control),
      ("BPUIDebugLog", self._ui_debug_log),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _initialize_items(self):
    """Initialize all BluePilot menu items."""

    # BlueCruise icon on dash toggle
    self._show_hands_free_ui = toggle_item(
      lambda: tr("Show BlueCruise UI on Cluster"),
      lambda: tr("Display BlueCruise UI on the cluster for supported vehicles."),
      initial_state=self._safe_get_bool(self._params, "send_hands_free_cluster_msg"),
      callback=lambda state: self._toggle_callback(state, "send_hands_free_cluster_msg"),
      icon="monitoring.png"
    )

    # Ford steering-angle curvature measurement (bad-yaw-sensor workaround). Init-time:
    # card reads it once at car init and mirrors it into the panda safety firmware, so a
    # flip only takes effect after the next restart (safe to toggle any time).
    self._steer_angle_curvature = toggle_item(
      lambda: tr("Use Pinion Yaw Sensor"),
      lambda: tr('Measures how the car is turning from the steering pinion angle sensor instead of a faulty RCM yaw sensor (symptoms: "Turn Exceeds Steering Limit" warnings, weak curve tracking, "Service AdvanceTrac"). Check with tools/ford_yaw_health_check.py. Applies the next time the car starts. Not available on the Edge.'),
      initial_state=self._safe_get_bool(self._params, "FordPrefSteerAngleCurvature"),
      callback=lambda state: self._toggle_callback(state, "FordPrefSteerAngleCurvature"),
      icon="monitoring.png",
      enabled=self._pinion_yaw_sensor_supported,
    )

    # Lane line status color toggle (issue #109: option to keep lane lines grey instead of green when engaged)
    self._disable_lane_line_status_color = toggle_item(
      lambda: tr("Disable Lane Line Status Color"),
      lambda: tr("Keep lane lines grey instead of changing to green when engaged."),
      initial_state=self._safe_get_bool(self._params, "BPDisableLaneLineStatusColor"),
      callback=lambda state: self._toggle_callback(state, "BPDisableLaneLineStatusColor"),
      icon="monitoring.png"
    )

    # Minimal driving view toggle
    self._hide_camera_view = toggle_item(
      lambda: tr("Minimal Driving View"),
      lambda: tr("Disable camera feed & only show lane lines and model path."),
      initial_state=self._safe_get_bool(self._params, "BPHideCameraView"),
      callback=lambda state: self._toggle_callback(state, "BPHideCameraView"),
      icon="chffr_wheel.png"
    )

    # Rad Racer 8-bit theme toggle
    self._rad_racer_theme = toggle_item(
      lambda: tr("8-Bit Racer Theme"),
      lambda: tr("Retro racing game onroad view: hides the camera, draws the road as green game lines, and shows a bottom gauge cluster."),
      initial_state=self._safe_get_bool(self._params, "BPRadRacerTheme"),
      callback=lambda state: self._toggle_callback(state, "BPRadRacerTheme"),
      icon="chffr_wheel.png"
    )

    # Rainbow lane lines toggle
    self._rainbow_lane_lines = toggle_item(
      lambda: tr("Rainbow Lane Lines"),
      lambda: tr("Inner lane lines become rainbow colored when longitudinal control is active."),
      initial_state=self._safe_get_bool(self._params, "BPRainbowLines"),
      callback=lambda state: self._toggle_callback(state, "BPRainbowLines"),
      icon="monitoring.png"
    )

    # Blindspot overlay toggle (BluePilot red edge overlay; SunnyPilot BSM is controlled by Visuals → BlindSpot)
    self._show_blindspot = toggle_item(
      lambda: tr("Show Blindspot Overlay"),
      lambda: tr("Display red overlay when vehicle is detected in blindspot."),
      initial_state=self._safe_get_bool(self._params, "ShowBlindspotOverlay"),
      callback=lambda state: self._toggle_callback(state, "ShowBlindspotOverlay"),
      icon="warning.png"
    )

    # Brake status toggle
    self._show_brake_status = toggle_item(
      lambda: tr("Show Brake Status"),
      lambda: tr("Display speed setpoint in red when vehicle is braking."),
      initial_state=self._safe_get_bool(self._params, "ShowBrakeStatus"),
      callback=lambda state: self._toggle_callback(state, "ShowBrakeStatus"),
      icon="warning.png"
    )

    # Hide onroad border toggle
    self._hide_onroad_border = toggle_item(
      lambda: tr("Hide Onroad Border"),
      lambda: tr("Hide the colored status border around the driving view."),
      initial_state=self._safe_get_bool(self._params, "BPHideOnroadBorder"),
      callback=lambda state: self._toggle_callback(state, "BPHideOnroadBorder"),
      icon="warning.png"
    )

    # Show confidence ball toggle
    self._show_confidence_ball = toggle_item(
      lambda: tr("Show Confidence Ball"),
      lambda: tr("Display the confidence ball on the left side of the driving view."),
      initial_state=self._safe_get_bool(self._params, "BPShowConfidenceBall"),
      callback=lambda state: self._toggle_callback(state, "BPShowConfidenceBall"),
      icon="warning.png"
    )

    # Animate steering wheel toggle
    self._animate_steering_wheel = toggle_item(
      lambda: tr("Animate Steering Wheel"),
      lambda: tr("Rotate the steering wheel icon to match the current steering angle."),
      initial_state=self._safe_get_bool(self._params, "BPAnimateSteeringWheel"),
      callback=lambda state: self._toggle_callback(state, "BPAnimateSteeringWheel"),
      icon="chffr_wheel.png"
    )

    wheel_style_idx = int(ensure_steering_wheel_icon_style_initialized(self._params, SteeringWheelIconStyle.COMMA_3X))
    self._wheel_icon_style_btn = multiple_button_item(
      lambda: tr("Wheel Icon Style"),
      lambda: tr("Toggle wheel icon style between Comma 4 and Comma 3x wheel"),
      buttons=[lambda: tr("Comma 4"), lambda: tr("Comma 3x")],
      button_width=225,
      callback=self._set_wheel_icon_style,
      selected_index=wheel_style_idx,
      icon="chffr_wheel.png"
    )

    dm_style_idx = int(ensure_dm_icon_style_initialized(self._params, DMIconStyle.COMMA_3X))
    self._dm_icon_style_btn = multiple_button_item(
      lambda: tr("DM Icon Style"),
      lambda: tr("Toggle Driver Monitoring icon style between Comma 4 and Comma 3x"),
      buttons=[lambda: tr("Comma 4"), lambda: tr("Comma 3x")],
      button_width=225,
      callback=self._set_dm_icon_style,
      selected_index=dm_style_idx,
      icon="monitoring.png"
    )

    self._use_custom_sounds = toggle_item(
      lambda: tr("Use Custom Engage/Disengage Sounds"),
      lambda: tr("Replace the engage and disengage sounds with the selected sound pack."),
      initial_state=self._safe_get_bool(self._params, "BPUseCustomSounds"),
      callback=self._on_custom_sounds_toggled,
      icon="microphone.png"
    )

    custom_sound_idx = int(get_custom_sound_selection(self._params))
    self._custom_sound_selection_btn = multiple_button_item(
      lambda: tr("Engage/Disengage Sound"),
      lambda: tr("Choose the sound pack played when openpilot engages or disengages."),
      buttons=[lambda: tr("Comma 4"), lambda: tr("Comma 3x"), lambda: tr("Tesla")],
      button_width=225,
      callback=self._set_custom_sound_selection,
      selected_index=custom_sound_idx,
      icon="microphone.png"
    )

    # Ford radar lead overlay toggle
    self._show_ford_radar_overlay = toggle_item(
      lambda: tr("Show Radar Lead Overlay (Ford ACC)"),
      lambda: tr("Display chevron with lead vehicle info when using Ford stock ACC."),
      initial_state=self._safe_get_bool(self._params, "FordPrefShowRadarLeadOverlay"),
      callback=lambda state: self._toggle_callback(state, "FordPrefShowRadarLeadOverlay"),
      icon="speed_limit.png"
    )

    # Ford radar overlay size selector (inline buttons like Driving Personality)
    try:
      overlay_size_idx = int(self._safe_get(self._params, "FordPrefRadarOverlaySize") or 1)
    except (TypeError, ValueError):
      overlay_size_idx = 1
    # Ensure default is persisted so consumers read the correct value on first load
    try:
      if self._safe_get(self._params, "FordPrefRadarOverlaySize") is None:
        self._params.put("FordPrefRadarOverlaySize", str(overlay_size_idx))
    except UnknownKeyName:
      pass
    self._radar_overlay_size_btn = multiple_button_item(
      lambda: tr("Radar Overlay Size"),
      lambda: tr("Set the size of the radar lead overlay chevron and info boxes."),
      buttons=[lambda: tr("Small"), lambda: tr("Medium"), lambda: tr("Large")],
      button_width=225,
      callback=self._set_overlay_size,
      selected_index=overlay_size_idx,
      icon="speed_limit.png"
    )

    # Hybrid battery status toggle
    self._show_hybrid_battery_status = toggle_item(
      lambda: tr("Show Hybrid/EV Battery Status"),
      lambda: tr("Display hybrid battery gauge with SOC, voltage, and amps."),
      initial_state=self._safe_get_bool(self._params, "FordPrefHybridBatteryStatus"),
      callback=lambda state: self._toggle_callback(state, "FordPrefHybridBatteryStatus"),
      icon="warning.png"
    )

    # Hybrid power flow toggle
    self._show_hybrid_power_flow = toggle_item(
      lambda: tr("Show Hybrid/EV Power Flow"),
      lambda: tr("Display power flow gauge showing throttle demand and regenerative braking."),
      initial_state=self._safe_get_bool(self._params, "FordPrefHybridPowerFlow"),
      callback=lambda state: self._toggle_callback(state, "FordPrefHybridPowerFlow"),
      icon="warning.png"
    )

    # Hybrid drive gauge size selector (inline buttons: Small=1, Large=2)
    try:
      gauge_size_idx = int(self._safe_get(self._params, "FordPrefHybridDriveGaugeSize") or 1)
    except (TypeError, ValueError):
      gauge_size_idx = 1
    # Clamp old 3-tier values to new 2-tier range
    gauge_size_idx = min(gauge_size_idx, 2)
    # Ensure default is persisted so consumers read the correct value on first load
    try:
      if self._safe_get(self._params, "FordPrefHybridDriveGaugeSize") is None:
        self._params.put("FordPrefHybridDriveGaugeSize", str(gauge_size_idx))
    except UnknownKeyName:
      pass
    # Map 1/2 to button index 0/1
    self._hybrid_gauge_size_btn = multiple_button_item(
      lambda: tr("Hybrid/EV Gauge Size"),
      lambda: tr("Set the size of the battery and power flow gauges."),
      buttons=[lambda: tr("Small"), lambda: tr("Large")],
      button_width=225,
      callback=self._set_hybrid_gauge_size,
      selected_index=gauge_size_idx - 1,
      icon="warning.png"
    )

    # Hybrid gauge style: Flat (horizontal bar + container) vs Arched (arch above torque bar)
    gauge_style_idx = GaugeStyle(self._params.get("FordPrefGaugeStyle", return_default=True) or 0)
    self._hybrid_gauge_style_btn = multiple_button_item(
      lambda: tr("Hybrid Gauge Style"),
      lambda: tr("Flat: horizontal bar in shared container. Arched: arch above torque bar (older style)."),
      buttons=[lambda: tr("Flat"), lambda: tr("Arched")],
      button_width=225,
      callback=self._set_hybrid_gauge_style,
      selected_index=gauge_style_idx,
      icon="warning.png"
    )

    # Human turn detection toggle
    self._enable_human_turn_detection = toggle_item(
      lambda: tr("Enable Human Turn Detection"),
      lambda: tr("Enable detection of human-initiated turns."),
      initial_state=self._safe_get_bool(self._params, "enable_human_turn_detection_curv"),
      callback=lambda state: self._toggle_callback(state, "enable_human_turn_detection_curv"),
      icon="warning.png"
    )

    # Lane change factor high (float) — curvature-mode and angle-mode tune this independently
    self._lane_change_factor_high_curv = float_control_item(
      lambda: tr("Lane Change Factor High"),
      lambda: tr("Scales steering during a lane change (curvature control). Values <1.0 reduce it."),
      param="lane_change_factor_high_curv",
      min_value=0.5,
      max_value=1.0,
      step=0.05,
      icon="speed_limit.png"
    )
    self._lane_change_factor_high_ang = float_control_item(
      lambda: tr("Lane Change Factor High"),
      lambda: tr("Scales steering during a lane change (angle control). Values >1.0 boost it."),
      param="lane_change_factor_high_ang",
      min_value=0.85,
      max_value=1.50,
      step=0.05,
      icon="speed_limit.png"
    )

    self._disable_lane_change_under_speed = toggle_item(
      lambda: tr("Disable Lane Change Under Speed"),
      lambda: tr("Pause lateral control when blinker is on and below minimum speed."),
      initial_state=self._safe_get_bool(self._params, "BlinkerPauseLaneChange"),
      callback=self._on_blinker_pause_changed,
      icon="chffr_wheel.png"
    )

    self._blinker_min_speed = int_control_item(
      lambda: tr("Minimum Speed to Pause Lane Change"),
      lambda: tr("Below this speed, lateral control is paused when the blinker is active."),
      param="BlinkerMinLateralControlSpeed",
      min_value=5,
      max_value=50,
      step=1,
      icon="chffr_wheel.png"
    )

    # Enable lane positioning toggle
    self._enable_lane_positioning = toggle_item(
      lambda: tr("Enable Lane Positioning"),
      lambda: tr("Enable custom lane positioning controls."),
      initial_state=self._safe_get_bool(self._params, "enable_lane_positioning_curv"),
      callback=lambda state: self._toggle_callback(state, "enable_lane_positioning_curv"),
      icon="chffr_wheel.png"
    )

    # Custom path offset (float, conditional on lane positioning)
    self._custom_path_offset = float_control_item(
      lambda: tr("In-Lane Offset"),
      lambda: tr("Adjust the in-lane offset (-0.5 to 0.5)."),
      param="custom_path_offset_curv",
      min_value=-0.5,
      max_value=0.5,
      step=0.05,
      enabled=lambda: self._safe_get_bool(self._params, "enable_lane_positioning_curv"),
      icon="chffr_wheel.png"
    )

    # Enable lanefull mode toggle (conditional on lane positioning)
    self._enable_lane_full_mode = toggle_item(
      lambda: tr("Enable Lanefull Mode"),
      lambda: tr("Enable lanefull mode for lane positioning."),
      initial_state=self._safe_get_bool(self._params, "enable_lane_full_mode_curv"),
      callback=lambda state: self._toggle_callback(state, "enable_lane_full_mode_curv"),
      enabled=lambda: self._safe_get_bool(self._params, "enable_lane_positioning_curv"),
      icon="chffr_wheel.png"
    )

    # Custom profile toggle
    self._custom_profile = toggle_item(
      lambda: tr("Use Custom Tuning Profile"),
      lambda: tr("Enable custom tuning profile settings."),
      initial_state=self._safe_get_bool(self._params, "custom_profile_curv"),
      callback=lambda state: self._toggle_callback(state, "custom_profile_curv"),
      icon="chffr_wheel.png"
    )

    # Predicted curvature blend ratio high (float, conditional on custom profile)
    self._pc_blend_ratio_high_C = float_control_item(
      lambda: tr("Predicted Curvature Blend Ratio High"),
      lambda: tr("Adjust the high curvature blend ratio (0.0-1.0)."),
      param="pc_blend_ratio_high_C_UI_curv",
      min_value=0.0,
      max_value=1.0,
      step=0.05,
      enabled=lambda: self._safe_get_bool(self._params, "custom_profile_curv"),
      icon="chffr_wheel.png"
    )

    # Predicted curvature blend ratio low (float, conditional on custom profile)
    self._pc_blend_ratio_low_C = float_control_item(
      lambda: tr("Predicted Curvature Blend Ratio Low"),
      lambda: tr("Adjust the low curvature blend ratio (0.0-1.0)."),
      param="pc_blend_ratio_low_C_UI_curv",
      min_value=0.0,
      max_value=1.0,
      step=0.05,
      enabled=lambda: self._safe_get_bool(self._params, "custom_profile_curv"),
      icon="chffr_wheel.png"
    )

    # Centering PID gain — curv-mode lane positioning only (only effective when custom_profile_curv is on).
    # Angle mode's centering trim was removed; see lateral_angle_ext.py docstring.
    self._lc_pid_gain = float_control_item(
      lambda: tr("Centering PID gain"),
      lambda: tr("PID gain for the curvature-mode centering controller (only effective when 'custom profile' is enabled)."),
      param="LC_PID_gain_UI_curv",
      min_value=0.0,
      max_value=50.0,
      step=0.5,
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

    # UI Debug Logging toggle
    self._ui_debug_log = toggle_item(
      lambda: tr("UI Debug Logging"),
      lambda: tr("Log UI state transitions for diagnosing rendering issues on device."),
      initial_state=self._safe_get_bool(self._params, "BPUIDebugLog"),
      callback=lambda state: self._toggle_callback(state, "BPUIDebugLog"),
      icon="warning.png"
    )

    # Connect backend selector (Comma Connect / Konik Stable / Offline Mode)
    self._connect_backend_dialog: MultiOptionDialog | None = None
    self._connect_backend_action = ButtonAction(lambda: tr("SELECT"))
    self._connect_backend_action.set_value(lambda: self._get_connect_backend_display())
    self._connect_backend_btn = ListItem(
      lambda: tr("Connect Backend"),
      description=lambda: tr("Comma Connect uses stock servers. Konik Stable sends routes to stable.konik.ai (dongle ID switches automatically). Offline Mode points at unreachable hosts so uploads never succeed. Reboot to apply."),
      action_item=self._connect_backend_action,
      callback=self._select_connect_backend
    )

    # Restore cached dongle ID — recovery for a device left unregistered by a backend
    # switch (e.g. a pre-existing Konik registration from before this menu existed).
    self._restore_dongle_action = ButtonAction(lambda: tr("RESTORE"), enabled=self._has_recoverable_dongle_id)
    self._restore_dongle_action.set_value(lambda: self._get_recoverable_dongle_id_preview())
    self._restore_dongle_btn = ListItem(
      lambda: tr("Restore Cached Dongle ID"),
      description=lambda: tr("If switching backends left this device unregistered, restore a previously registered ID found cached here. Only enabled when one is found."),
      action_item=self._restore_dongle_action,
      callback=self._restore_dongle_id
    )

    # Lane line feedback trim — toggle + tuning floats
    # Primary lateral actuator: curvature-primary (historical) vs angle-primary (experimental)
    primary_lat_idx = PrimaryLateralControl(self._params.get("FordPrefLateralControl", return_default=True) or 0)
    self._primary_lateral_control_btn = multiple_button_item(
      lambda: tr("Primary Control Variable"),
      lambda: tr("Curvature matches the existing strategy. Angle uses path_angle as the main actuator (in development)."),
      buttons=[lambda: tr("Curvature"), lambda: tr("Angle")],
      button_width=225,
      callback=self._set_primary_lateral_control,
      selected_index=primary_lat_idx,
      icon="chffr_wheel.png"
    )
    self._low_speed_curv_factor = float_control_item(
      lambda: tr("Low Speed Adjustment Factor"),
      lambda: tr("Scales the low-speed steering response in angle mode. Adjust for personal feel. Default 1.0."),
      param="FordLowSpeedFactor_ang",
      min_value=0.5,
      max_value=1.5,
      step=0.01,
      icon="chffr_wheel.png"
    )
    self._high_speed_curv_factor = float_control_item(
      lambda: tr("High Speed Adjustment Factor"),
      lambda: tr("Scales the high-speed steering response in angle mode. Adjust for personal feel. Default 1.0."),
      param="FordHighSpeedFactor_ang",
      min_value=0.5,
      max_value=1.5,
      step=0.01,
      icon="chffr_wheel.png"
    )
    # BluePilot: one-time auto-calibration of the two factors above. Compares requested vs
    # actual turn in steady engaged curves and writes the corrected factors once, then locks.
    self._angle_autocal = toggle_item(
      lambda: tr("Auto-Calibrate Adjustment Factors"),
      lambda: tr("Learns the low/high speed factors automatically by comparing requested and actual "
                 "turn in steady engaged curves, then locks them (one-time, per car). Drive normally "
                 "with lateral engaged; curves at city and highway speeds both needed. Toggle off and "
                 "back on to recalibrate."),
      initial_state=self._safe_get_bool(self._params, "FordAngleAutoCal"),
      callback=self._toggle_angle_autocal,
      icon="chffr_wheel.png"
    )
    # BluePilot: lock behavior for the calibration above. Off = never freeze, keep
    # adapting; flipping it off on a locked car resumes from the saved evidence.
    self._angle_autocal_lock = toggle_item(
      lambda: tr("Calibration Lock"),
      lambda: tr("On (default): auto-calibration freezes once the factors have been stable for "
                 "5 minutes of driving. Off: it never locks and keeps adapting continuously — "
                 "turning this off on an already-locked car resumes calibration from its saved "
                 "evidence without losing anything."),
      initial_state=self._safe_get_bool(self._params, "FordAngleAutoCalLock", default=True),
      callback=lambda state: self._toggle_callback(state, "FordAngleAutoCalLock"),
      icon="chffr_wheel.png"
    )
    # BluePilot: full calibration do-over — evidence, error log and the factors themselves.
    self._angle_autocal_erase = button_item(
      lambda: tr("Erase Calibration Memory"),
      lambda: tr("ERASE"),
      lambda: tr("Wipes all collected calibration evidence, clears any lock, and puts both "
                 "adjustment factors back to 1.00 for a clean retry. Works offroad or mid-drive."),
      callback=self._erase_angle_autocal
    )
    # BluePilot: anti-weave smoothing of the angle command path (gain-schedule filter,
    # wire-quantization hold, blend slew — see lateral_angle_ext.py _SM_* constants).
    self._angle_smoothing = toggle_item(
      lambda: tr("Smooth Steering (Anti-Weave)"),
      lambda: tr("Removes the rhythmic left-right centering motion in angle mode by filtering "
                 "the sources of steering dither on straight roads. No effect in curves. "
                 "Turn off to compare against the unsmoothed behavior."),
      initial_state=self._safe_get_bool(self._params, "FordAngleSmoothing", default=True),
      callback=lambda state: self._toggle_callback(state, "FordAngleSmoothing"),
      icon="chffr_wheel.png"
    )
    # Manual strength for the smoothing above: 0 = minimal, 1.0 = tuned default, 1.5 = strong.
    self._angle_smoothing_strength = float_control_item(
      lambda: tr("Smoothing Strength"),
      lambda: tr("1.0 = stock steering (no smoothing). Step up for more damping of the "
                 "straight-road weave; 2.0 = the log-tuned setting, 2.5 = strongest. "
                 "Curve response is unaffected at any strength."),
      param="FordAngleSmoothStrength",
      min_value=1.0,
      max_value=2.5,
      step=0.1,
      icon="chffr_wheel.png"
    )
    self._high_speed_dampening = float_control_item(
      lambda: tr("High Speed Low Curve Adjustment Factor"),
      lambda: tr("Tune adjustment factor for low curve straightaways (highways) at high speeds. If oversteering, reduce. If understeering, increase"),
      param="FordHighSpeedDampening_ang",
      min_value=0.75,
      max_value=1.25,
      step=0.01,
      icon="chffr_wheel.png"
    )
    # Disable BP lateral control toggle
    self._disable_BP_lat = toggle_item(
      lambda: tr("Disable BP Lateral Control"),
      lambda: tr("Disable BluePilot lateral control."),
      initial_state=self._safe_get_bool(self._params, "disable_BP_lat_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_BP_lat_UI"),
      icon="chffr_wheel.png"
    )

    # Bypass BP longitudinal control toggle (use stock long logic)
    self._disable_BP_long = toggle_item(
      lambda: tr("Bypass BP Longitudinal Control"),
      lambda: tr("Use stock longitudinal logic instead of BluePilot TTC/coasting tuning."),
      initial_state=self._safe_get_bool(self._params, "disable_BP_long_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_BP_long_UI"),
      icon="chffr_wheel.png"
    )

    # Disable downhill compensation toggle
    self._disable_dowhill_comp = toggle_item(
      lambda: tr("Disable Downhill Compensation"),
      lambda: tr("Disable pitch-based brake/gas compensation when going downhill."),
      initial_state=self._safe_get_bool(self._params, "disable_downhill_comp_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_downhill_comp_UI"),
      icon="chffr_wheel.png"
    )

    # Disable Ford radar — vision-only lead detection (requires reboot)
    self._disable_ford_radar = toggle_item(
      lambda: tr("Disable Ford Radar (Vision-Only Leads)"),
      lambda: tr("Ignore the vehicle radar and drive leads exclusively from the vision model. Requires reboot."),
      initial_state=self._safe_get_bool(self._params, "disable_ford_radar_UI"),
      callback=lambda state: self._toggle_callback(state, "disable_ford_radar_UI"),
      icon="chffr_wheel.png"
    )

    # Show lateral control mode overlay toggle
    self._show_lateral_control = toggle_item(
      lambda: tr("Show Lateral Control Mode"),
      lambda: tr("Display the lateral control mode overlay on the steering wheel icon."),
      initial_state=self._safe_get_bool(self._params, "BpShowLateralControl"),
      callback=lambda state: self._toggle_callback(state, "BpShowLateralControl"),
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

    # Clear model runner cache (ModelRunnerTypeCache + ModelManager_ActiveBundle) and reboot
    self._clear_model_cache_btn = button_item(
      lambda: tr("Clear Crashed Model"),
      lambda: tr("CLEAR"),
      lambda: tr("Clear crashed model runner cache and reboot. Fixes 'Communication Issue' if modeld fails to start."),
      callback=self._clear_model_cache
    )

    # BluePilot: reset menu layout — collapses all sections, fixing overlap glitch without reboot.
    self._reset_menu_btn = button_item(
      lambda: tr("Reset Menu Layout"),
      lambda: tr("RESET"),
      lambda: tr("Collapse all sections to fix overlapping items. Use this if the menu looks broken."),
      callback=lambda: self._scroller.show_event()
    )
    # End BluePilot

    # BluePilot: collapsible section groups — all start collapsed when the menu opens.
    def _section(title: str, items: list) -> list:
      header = CollapsibleSectionHeader(title)
      header.set_items(items)
      return [header] + items

    # Angle Tuning: nested collapsible sub-section, angle-mode-only tuning items
    angle_items = [
      self._low_speed_curv_factor,
      self._high_speed_curv_factor,
      self._high_speed_dampening,
      self._angle_autocal,
      self._angle_autocal_lock,
      self._angle_autocal_erase,
      self._angle_smoothing,
      self._angle_smoothing_strength,
      self._lane_change_factor_high_ang,
    ]
    angle_header = CollapsibleSectionHeader(tr("Angle Tuning"))
    angle_header.set_items(angle_items)
    self._angle_header = angle_header

    # Curvature Tuning: nested collapsible sub-section, curvature-mode-only tuning items
    curv_items = [
      self._enable_human_turn_detection,
      self._lane_change_factor_high_curv,
      self._enable_lane_positioning,
      self._custom_path_offset,
      self._enable_lane_full_mode,
      self._custom_profile,
      self._pc_blend_ratio_high_C,
      self._pc_blend_ratio_low_C,
      self._lc_pid_gain,
    ]
    curv_header = CollapsibleSectionHeader(tr("Curvature Tuning"))
    curv_header.set_items(curv_items)
    self._curv_header = curv_header

    # Lateral Tuning: outer section. Disable toggle and mode selector up top, then mode-agnostic
    # lane-change items, then the two nested sub-sections (always visible, greyed by mode above).
    lateral_items = [
      self._disable_BP_lat,
      self._primary_lateral_control_btn,
      self._disable_lane_change_under_speed,
      self._blinker_min_speed,
      self._show_lateral_control,
    ]
    lateral_header = CollapsibleSectionHeader(tr("Lateral Tuning"))
    lateral_header.set_items(lateral_items + [angle_header, curv_header])
    lateral_header.set_nested_headers([angle_header, curv_header])
    self._lateral_header = lateral_header

    lateral_section = [lateral_header] + lateral_items + [angle_header] + angle_items + [curv_header] + curv_items

    return (
      _section(tr("System"), [
        self._preferred_network_btn,
        self._clear_model_cache_btn,
        self._ui_debug_log,
        self._connect_backend_btn,
        self._restore_dongle_btn,
        self._reset_menu_btn,
      ]) +
      _section(tr("Vehicle"), [
        self._show_hands_free_ui,
        self._steer_angle_curvature,
        self._vbatt_pause_charging,
      ]) +
      _section(tr("Audio"), [
        self._use_custom_sounds,
        self._custom_sound_selection_btn,
      ]) +
      _section(tr("Visuals"), [
        self._hide_onroad_border,
        self._disable_lane_line_status_color,
        self._hide_camera_view,
        self._rad_racer_theme,
        self._rainbow_lane_lines,
        self._show_blindspot,
        self._show_brake_status,
        self._show_confidence_ball,
        self._animate_steering_wheel,
        self._wheel_icon_style_btn,
        self._dm_icon_style_btn,
        self._show_ford_radar_overlay,
        self._radar_overlay_size_btn,
        self._show_hybrid_battery_status,
        self._show_hybrid_power_flow,
        self._hybrid_gauge_size_btn,
        self._hybrid_gauge_style_btn,
      ]) +
      _section(tr("Longitudinal Tuning"), [
        self._disable_BP_long,
        self._disable_dowhill_comp,
        self._disable_ford_radar,
      ]) +
      lateral_section
    )
    # End BluePilot

  def _get_float_param(self, param: str, default: float) -> float:
    """Get float parameter value."""
    try:
      return float(self._params.get(param, return_default=True))
    except (TypeError, ValueError):
      return default

  def _toggle_callback(self, state: bool, param: str):
    """Handle toggle state changes."""
    try:
      self._params.put_bool(param, state)
    except UnknownKeyName:
      pass  # Param not available in dev environment
    self._update_toggles(just_toggled={param: state})

  def _get_connect_backend_display(self) -> str:
    try:
      from bluepilot.backend_switch import backend_label, get_connect_backend
      return backend_label(get_connect_backend(self._params))
    except Exception:
      return tr("Comma Connect")

  def _select_connect_backend(self):
    from bluepilot.backend_switch import BACKEND_LABELS, BACKENDS, get_connect_backend

    options = [BACKEND_LABELS[b] for b in BACKENDS]
    current = self._get_connect_backend_display()
    try:
      prev_idx = BACKENDS.index(get_connect_backend(self._params))
    except ValueError:
      prev_idx = 0

    def handle_selection(result):
      if result == DialogResult.CONFIRM and self._connect_backend_dialog is not None:
        selection = self._connect_backend_dialog.selection
        try:
          idx = options.index(selection)
        except ValueError:
          idx = 0
        if idx != prev_idx:
          try:
            self._params.put("BPConnectBackend", idx)
          except UnknownKeyName:
            pass
          self._connect_backend_action.set_value(self._get_connect_backend_display())
          dialog = ConfirmDialog(tr("Server change requires a reboot to take effect. Reboot now?"),
                                 tr("Reboot"), callback=self._handle_connect_backend_reboot)
          gui_app.push_widget(dialog)
        else:
          self._connect_backend_action.set_value(self._get_connect_backend_display())
      self._connect_backend_dialog = None

    self._connect_backend_dialog = MultiOptionDialog(
      tr("Select Connect Backend"),
      options,
      current,
      callback=handle_selection
    )
    gui_app.push_widget(self._connect_backend_dialog)

  def _handle_connect_backend_reboot(self, result):
    if result == DialogResult.CONFIRM:
      self._params.put_bool("DoReboot", True)

  def _get_recoverable_dongle_id(self) -> str | None:
    try:
      from bluepilot.backend_switch import find_recoverable_dongle_id
      return find_recoverable_dongle_id(self._params)
    except Exception:
      return None

  def _has_recoverable_dongle_id(self) -> bool:
    return self._get_recoverable_dongle_id() is not None

  def _get_recoverable_dongle_id_preview(self) -> str:
    candidate = self._get_recoverable_dongle_id()
    if candidate is None:
      return ""
    return f"{candidate[:6]}…{candidate[-4:]}" if len(candidate) > 12 else candidate

  def _restore_dongle_id(self):
    candidate = self._get_recoverable_dongle_id()
    if candidate is None:
      return

    def handle_confirm(result: DialogResult):
      if result == DialogResult.CONFIRM:
        try:
          from bluepilot.backend_switch import restore_cached_dongle_id
          restore_cached_dongle_id(self._params, candidate)
        except Exception:
          cloudlog.exception("bp_dongle_id_recovery_ui failed")
          return
        dialog = ConfirmDialog(tr("Dongle ID restored. Reboot now to apply?"),
                               tr("Reboot"), callback=self._handle_connect_backend_reboot)
        gui_app.push_widget(dialog)

    preview = self._get_recoverable_dongle_id_preview()
    dialog = ConfirmDialog(tr("Restore cached dongle ID {preview}? This overwrites the current (unregistered) device ID.").format(preview=preview),
                           tr("Restore"), callback=handle_confirm)
    gui_app.push_widget(dialog)

  def _on_blinker_pause_changed(self, state: bool) -> None:
    self._toggle_callback(state, "BlinkerPauseLaneChange")
    self._blinker_min_speed.action_item.set_enabled(state)

  def _update_toggles(self, just_toggled: dict | None = None):
    """Update toggle states from params. just_toggled: {param: value} for params we just wrote (avoids refresh race)."""
    ui_state.update_params()
    fresh = just_toggled or {}

    # Refresh toggles from params to mirror external changes (use fresh for params we just wrote)
    for key, item in self._refresh_toggles:
      state = fresh[key] if key in fresh else self._safe_get_bool(ui_state.params, key)
      item.action_item.set_state(state)

    wheel_style_idx = int(get_steering_wheel_icon_style(ui_state.params, SteeringWheelIconStyle.COMMA_3X))
    self._wheel_icon_style_btn.action_item.set_selected_button(wheel_style_idx)
    dm_style_idx = int(get_dm_icon_style(ui_state.params, DMIconStyle.COMMA_3X))
    self._dm_icon_style_btn.action_item.set_selected_button(dm_style_idx)
    custom_sound_idx = int(get_custom_sound_selection(ui_state.params))
    self._custom_sound_selection_btn.action_item.set_selected_button(custom_sound_idx)
    custom_sounds_enabled = fresh.get(
      "BPUseCustomSounds", self._safe_get_bool(ui_state.params, "BPUseCustomSounds")
    )
    self._custom_sound_selection_btn.action_item.set_enabled(
      custom_sounds_enabled
    )

    # Update button enabled states
    self._radar_overlay_size_btn.action_item.set_enabled(self._safe_get_bool(ui_state.params, "FordPrefShowRadarLeadOverlay"))
    try:
      overlay_idx = int(self._safe_get(ui_state.params, "FordPrefRadarOverlaySize") or 1)
    except (TypeError, ValueError):
      overlay_idx = 1
    self._radar_overlay_size_btn.action_item.set_selected_button(overlay_idx)
    # Hybrid gauge size and style: enable only when power flow gauge is enabled (NOT battery status)
    self._hybrid_gauge_size_btn.action_item.set_enabled(
      lambda: self._safe_get_bool(ui_state.params, "FordPrefHybridPowerFlow")
    )
    self._hybrid_gauge_style_btn.action_item.set_enabled(
      lambda: self._safe_get_bool(ui_state.params, "FordPrefHybridPowerFlow")
    )
    try:
      gauge_size = int(self._safe_get(ui_state.params, "FordPrefHybridDriveGaugeSize") or 1)
    except (TypeError, ValueError):
      gauge_size = 1
    gauge_size = min(gauge_size, 2)  # Clamp old 3-tier values
    self._hybrid_gauge_size_btn.action_item.set_selected_button(gauge_size - 1)
    style_idx = GaugeStyle(ui_state.params.get("FordPrefGaugeStyle", return_default=True) or 0)
    self._hybrid_gauge_style_btn.action_item.set_selected_button(style_idx)
    plat_idx = PrimaryLateralControl(ui_state.params.get("FordPrefLateralControl", return_default=True) or 0)
    self._primary_lateral_control_btn.action_item.set_selected_button(plat_idx)
    custom_prof = fresh.get("custom_profile_curv") if "custom_profile_curv" in fresh else self._safe_get_bool(ui_state.params, "custom_profile_curv")
    lane_pos = fresh.get("enable_lane_positioning_curv") if "enable_lane_positioning_curv" in fresh else self._safe_get_bool(ui_state.params, "enable_lane_positioning_curv")
    pause_lc = fresh.get("BlinkerPauseLaneChange") if "BlinkerPauseLaneChange" in fresh else self._safe_get_bool(ui_state.params, "BlinkerPauseLaneChange")
    is_angle = (plat_idx == PrimaryLateralControl.angle)
    is_curv = not is_angle
    # Conditional on BlinkerPauseLaneChange
    self._blinker_min_speed.action_item.set_enabled(pause_lc)
    # Angle-mode items: always visible (Angle Tuning section), greyed out when curvature mode is active
    self._low_speed_curv_factor.action_item.set_enabled(is_angle)
    self._high_speed_curv_factor.action_item.set_enabled(is_angle)
    self._high_speed_dampening.action_item.set_enabled(is_angle)
    self._angle_autocal.action_item.set_enabled(is_angle)
    self._angle_autocal_lock.action_item.set_enabled(is_angle)
    self._angle_autocal_erase.action_item.set_enabled(is_angle)
    self._angle_smoothing.action_item.set_enabled(is_angle)
    self._angle_smoothing_strength.action_item.set_enabled(is_angle)
    self._lane_change_factor_high_ang.action_item.set_enabled(is_angle)
    # Curvature-mode items: always visible (Curvature Tuning section), greyed out when angle mode is active
    self._lane_change_factor_high_curv.action_item.set_enabled(is_curv)
    self._enable_human_turn_detection.action_item.set_enabled(is_curv)
    self._enable_lane_positioning.action_item.set_enabled(is_curv)
    self._custom_path_offset.action_item.set_enabled(is_curv and lane_pos)
    self._enable_lane_full_mode.action_item.set_enabled(is_curv and lane_pos)
    self._custom_profile.action_item.set_enabled(is_curv)
    self._pc_blend_ratio_high_C.action_item.set_enabled(is_curv and custom_prof)
    self._pc_blend_ratio_low_C.action_item.set_enabled(is_curv and custom_prof)
    self._lc_pid_gain.action_item.set_enabled(is_curv and lane_pos and custom_prof)

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
    self._saved_networks = [n for n in networks if self._wifi_manager.is_connection_saved(n.ssid)]
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
        saved_connections = self._wifi_manager._connections
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

    # Create dialog with callback; MultiOptionDialog calls pop_widget and callback internally
    self._preferred_network_dialog = MultiOptionDialog(
      tr("Select Preferred Network"),
      network_options,
      current_favorite if current_favorite else tr("None"),
      callback=handle_selection
    )
    gui_app.push_widget(self._preferred_network_dialog)

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
        self._params.put_bool("DoReboot", True, block=False)
        cloudlog.info("BluePilot: Cleared model cache (ModelRunnerTypeCache, ModelManager_ActiveBundle), triggered reboot")

    dialog = ConfirmDialog(
      tr("Clear crashed model runner cache and reboot? This fixes 'Communication Issue' when modeld fails to start."),
      tr("Clear & Reboot"),
      callback=handle_confirm
    )
    gui_app.push_widget(dialog)

  def _set_overlay_size(self, button_index: int):
    """Handle overlay size button selection."""
    self._params.put("FordPrefRadarOverlaySize", button_index)

  def _toggle_angle_autocal(self, state: bool):
    """Arm/disarm the one-time factor auto-calibration; disarming clears a finished
    calibration's lock so re-enabling starts a fresh collection."""
    self._toggle_callback(state, "FordAngleAutoCal")
    if not state:
      try:
        self._params.put("FordAngleAutoCalState", "")
      except UnknownKeyName:
        pass

  def _erase_angle_autocal(self):
    """Erase calibration memory: evidence, error log, any lock, and the factors back to
    1.00. The params are cleared here for immediate offroad visibility; the onroad
    controller consumes FordAngleAutoCalReset so a mid-drive erase lands within a second."""
    try:
      self._params.put_bool("FordAngleAutoCalReset", True)
      self._params.put("FordAngleAutoCalState", "")
      self._params.put("FordAngleAutoCalError", "")
      self._params.put("FordLowSpeedFactor_ang", 1.0)
      self._params.put("FordHighSpeedFactor_ang", 1.0)
    except UnknownKeyName:
      pass

  def _set_wheel_icon_style(self, button_index: int):
    """Handle wheel icon style: 0 = comma 4, 1 = comma 3X."""
    self._params.put("BPSteeringWheelIconStyle", button_index)

  def _set_dm_icon_style(self, button_index: int):
    """Handle DM icon style: 0 = comma 4, 1 = comma 3X."""
    self._params.put("BPDMStylingChoice", button_index)

  def _set_custom_sound_selection(self, button_index: int):
    """Handle engagement sound selection: 0 = Comma 4, 1 = Comma 3x, 2 = Tesla."""
    previous = int(get_custom_sound_selection(self._params))
    if button_index == previous:
      return
    self._params.put("BPCustSoundsSelection", button_index)
    self._prompt_sound_reboot()

  def _on_custom_sounds_toggled(self, state: bool) -> None:
    self._toggle_callback(state, "BPUseCustomSounds")
    self._custom_sound_selection_btn.action_item.set_enabled(state)

  def _prompt_sound_reboot(self) -> None:
    dialog = ConfirmDialog(
      tr("For these sound changes to take effect, you will need to reboot your device.\n\nTHIS REBOOT WILL DISENGAGE OPENPILOT."),
      tr("Reboot now"),
      cancel_text=tr("Reboot later"),
      callback=self._handle_sound_reboot,
    )
    gui_app.push_widget(dialog)

  def _handle_sound_reboot(self, result):
    if result == DialogResult.CONFIRM:
      self._params.put_bool("DoReboot", True)

  def _set_hybrid_gauge_size(self, button_index: int):
    """Handle hybrid gauge size button selection. Buttons are 0/1/2, param stores 1/2/3."""
    self._params.put("FordPrefHybridDriveGaugeSize", button_index + 1)

  def _set_hybrid_gauge_style(self, button_index: int):
    """Handle hybrid gauge style: 0 = Flat, 1 = Arched."""
    try:
      self._params.put("FordPrefGaugeStyle", int(GaugeStyle(button_index)))
    except UnknownKeyName:
      pass

  def _set_primary_lateral_control(self, button_index: int):
    try:
      self._params.put("FordPrefLateralControl", int(PrimaryLateralControl(button_index)))
    except UnknownKeyName:
      pass
    self._update_toggles()

  def _render(self, rect):
    # Process WiFi manager callbacks
    self._wifi_manager.process_callbacks()
    self._scroller.render(rect)
