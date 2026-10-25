"""BluePilot MICI: Lateral tuning panel — control variable, factors, lane change, offset, mode display."""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigParamControlBP
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamFloatControl, BigParamIntControl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller import NavScroller
from opendbc.sunnypilot.car.ford.lateral_curv_ext import PrimaryLateralControl


class LateralLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

    # --- Angle-mode-only items ---
    self.low_speed_factor = BigParamFloatControl(
      "Low Speed Adjustment Factor", "FordLowSpeedFactor_ang", min=0.5, max=1.5, step=0.01,
    )
    self.high_speed_factor = BigParamFloatControl(
      "High Speed Adjustment Factor", "FordHighSpeedFactor_ang", min=0.5, max=1.5, step=0.01,
    )
    self.high_speed_dampening = BigParamFloatControl(
      "High Speed Low Curve Adjustment Factor", "FordHighSpeedDampening_ang", min=0.25, max=1.25, step=0.01,
    )
    self.lane_change_factor_high_ang = BigParamFloatControl(
      "Lane Change Factor High", "lane_change_factor_high_ang", min=0.85, max=1.50,
    )
    # Lane centering trim — angle mode's "advanced lane positioning" (curvature-domain trim,
    # see opendbc/sunnypilot/car/ford/lane_center_trim.py).
    self.enable_lane_positioning_ang = BigParamControlBP(
      "Enable Lane Positioning", "enable_lane_positioning_ang",
    )
    self.custom_path_offset_ang = BigParamFloatControl(
      "In-Lane Offset", "custom_path_offset_ang", min=-0.5, max=0.5, step=0.01,
    )
    self.lane_centering_strength_ang = BigParamFloatControl(
      "Lane Centering Strength", "lane_centering_strength_ang", min=0.0, max=1.0, step=0.05,
    )

    # --- Always-visible items ---
    self.disable_BP_lat = BigParamControlBP("Disable BP Lateral Control", "disable_BP_lat_UI")
    self.disable_lane_change_under_speed = BigParamControlBP(
      "Disable Auto Lane Change Under Speed", "BlinkerPauseLaneChange",
      toggle_callback=lambda state: self.blinker_min_speed.set_enabled(state),
    )
    self.blinker_min_speed = BigParamIntControl(
      "Minimum Speed to Pause Lane Change", "BlinkerMinLateralControlSpeed", min=5, max=50, step=5,
    )
    self.show_lateral_control = BigParamControlBP("Show Lateral Control Mode", "BpShowLateralControl")

    # --- Curvature-mode-only items ---
    self.lane_change_factor_high_curv = BigParamFloatControl(
      "Lane Change Factor High", "lane_change_factor_high_curv", min=0.5, max=1.0,
    )
    self.custom_path_offset = BigParamFloatControl(
      "In-Lane Offset", "custom_path_offset_curv", min=-0.5, max=0.5,
    )
    self.enable_human_turn_detection = BigParamControlBP(
      "Enable Human Turn Detection", "enable_human_turn_detection_curv",
    )
    self.enable_lane_positioning = BigParamControlBP(
      "Enable Lane Positioning", "enable_lane_positioning_curv",
    )
    self.enable_lane_full_mode = BigParamControlBP(
      "Enable Lanefull Mode", "enable_lane_full_mode_curv",
    )
    self.custom_profile = BigParamControlBP(
      "Use Custom Tuning Profile", "custom_profile_curv",
    )
    self.pc_blend_ratio_high_C = BigParamFloatControl(
      "Predicted Curvature Blend Ratio High", "pc_blend_ratio_high_C_UI_curv", min=0.0, max=1.0, step=0.05,
    )
    self.pc_blend_ratio_low_C = BigParamFloatControl(
      "Predicted Curvature Blend Ratio Low", "pc_blend_ratio_low_C_UI_curv", min=0.0, max=1.0, step=0.05,
    )
    self.lc_pid_gain = BigParamFloatControl(
      "Centering PID Gain", "LC_PID_gain_UI_curv", min=0.0, max=50.0, step=0.5,
    )

    self._scroller.add_widgets([
      self.low_speed_factor,
      self.high_speed_factor,
      self.high_speed_dampening,
      self.lane_change_factor_high_ang,
      self.enable_lane_positioning_ang,
      self.custom_path_offset_ang,
      self.lane_centering_strength_ang,
      self.disable_lane_change_under_speed,
      self.blinker_min_speed,
      self.lane_change_factor_high_curv,
      self.enable_human_turn_detection,
      self.custom_path_offset,
      self.enable_lane_positioning,
      self.enable_lane_full_mode,
      self.custom_profile,
      self.pc_blend_ratio_high_C,
      self.pc_blend_ratio_low_C,
      self.lc_pid_gain,
      self.show_lateral_control,
      self.disable_BP_lat,
    ])

    self._refresh_toggles = (
      ("disable_BP_lat_UI", self.disable_BP_lat),
      ("BlinkerPauseLaneChange", self.disable_lane_change_under_speed),
      ("enable_human_turn_detection_curv", self.enable_human_turn_detection),
      ("enable_lane_positioning_curv", self.enable_lane_positioning),
      ("enable_lane_full_mode_curv", self.enable_lane_full_mode),
      ("custom_profile_curv", self.custom_profile),
      ("enable_lane_positioning_ang", self.enable_lane_positioning_ang),
      ("BpShowLateralControl", self.show_lateral_control),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    plat_idx = PrimaryLateralControl(ui_state.params.get("FordPrefLateralControl", return_default=True) or 0)
    is_angle = (plat_idx == PrimaryLateralControl.angle)
    is_curv = not is_angle
    self.low_speed_factor.set_visible(is_angle)
    self.high_speed_factor.set_visible(is_angle)
    self.high_speed_dampening.set_visible(is_angle)
    self.lane_change_factor_high_ang.set_visible(is_angle)
    self.enable_lane_positioning_ang.set_visible(is_angle)
    lane_pos_ang = ui_state.params.get_bool("enable_lane_positioning_ang")
    self.custom_path_offset_ang.set_visible(is_angle)
    self.custom_path_offset_ang.set_enabled(lane_pos_ang)
    self.lane_centering_strength_ang.set_visible(is_angle)
    self.lane_centering_strength_ang.set_enabled(lane_pos_ang)
    self.blinker_min_speed.set_enabled(ui_state.params.get_bool("BlinkerPauseLaneChange"))
    for item in (
      self.lane_change_factor_high_curv,
      self.enable_human_turn_detection,
      self.enable_lane_positioning,
      self.enable_lane_full_mode,
      self.custom_profile,
      self.pc_blend_ratio_high_C,
      self.pc_blend_ratio_low_C,
      self.lc_pid_gain,
    ):
      item.set_visible(is_curv)
