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
      "low speed adjustment factor", "FordLowSpeedFactor_ang", min=0.5, max=1.5, step=0.01,
    )
    self.high_speed_factor = BigParamFloatControl(
      "high speed adjustment factor", "FordHighSpeedFactor_ang", min=0.5, max=1.5, step=0.01,
    )
    self.high_speed_dampening = BigParamFloatControl(
      "high speed low curve adjustment factor", "FordHighSpeedDampening_ang", min=0.75, max=1.25, step=0.01,
    )
    self.lane_change_factor_high_ang = BigParamFloatControl(
      "lane change factor high", "lane_change_factor_high_ang", min=0.85, max=1.50,
    )

    # --- Always-visible items ---
    self.disable_BP_lat = BigParamControlBP("disable bp lateral control", "disable_BP_lat_UI")
    self.disable_lane_change_under_speed = BigParamControlBP(
      "disable auto lane change under speed", "BlinkerPauseLaneChange",
      toggle_callback=lambda state: self.blinker_min_speed.set_enabled(state),
    )
    self.blinker_min_speed = BigParamIntControl(
      "minimum speed to pause lane change", "BlinkerMinLateralControlSpeed", min=5, max=50, step=5,
    )
    self.show_lateral_control = BigParamControlBP("show lateral control mode", "BpShowLateralControl")

    # --- Curvature-mode-only items ---
    self.lane_change_factor_high_curv = BigParamFloatControl(
      "lane change factor high", "lane_change_factor_high_curv", min=0.5, max=1.0,
    )
    self.custom_path_offset = BigParamFloatControl(
      "in-lane offset", "custom_path_offset_curv", min=-0.5, max=0.5,
    )
    self.enable_human_turn_detection = BigParamControlBP(
      "enable human turn detection", "enable_human_turn_detection_curv",
    )
    self.enable_lane_positioning = BigParamControlBP(
      "enable lane positioning", "enable_lane_positioning_curv",
    )
    self.enable_lane_full_mode = BigParamControlBP(
      "enable lanefull mode", "enable_lane_full_mode_curv",
    )
    self.custom_profile = BigParamControlBP(
      "use custom tuning profile", "custom_profile_curv",
    )
    self.pc_blend_ratio_high_C = BigParamFloatControl(
      "predicted curvature blend ratio high", "pc_blend_ratio_high_C_UI_curv", min=0.0, max=1.0, step=0.05,
    )
    self.pc_blend_ratio_low_C = BigParamFloatControl(
      "predicted curvature blend ratio low", "pc_blend_ratio_low_C_UI_curv", min=0.0, max=1.0, step=0.05,
    )
    self.lc_pid_gain = BigParamFloatControl(
      "centering pid gain", "LC_PID_gain_UI_curv", min=0.0, max=50.0, step=0.5,
    )

    self._scroller.add_widgets([
      self.low_speed_factor,
      self.high_speed_factor,
      self.high_speed_dampening,
      self.lane_change_factor_high_ang,
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
