"""BluePilot MICI: Lateral tuning panel — control variable, factors, lane change, offset, mode display."""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP, BigParamControlBP
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamFloatControl, BigParamIntControl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller import NavScroller
from opendbc.sunnypilot.car.ford.lateral_curv_ext import PrimaryLateralControl


class _EraseAutoCalButton(BigButtonBP):
  """One-tap 'erase calibration memory': evidence, the error log and the factors
  themselves go back to neutral so a calibration run can simply be retried. The params
  are cleared here for immediate offroad visibility (the factor steppers show 1.00 on
  next refresh); the onroad controller consumes FordAngleAutoCalReset to drop its
  in-memory pipeline too, so a mid-drive erase takes effect within a second."""

  def __init__(self):
    super().__init__("Erase Calibration Memory")

  def _handle_mouse_release(self, mouse_pos):
    super()._handle_mouse_release(mouse_pos)
    ui_state.params.put_bool("FordAngleAutoCalReset", True)
    ui_state.params.put("FordAngleAutoCalState", "")
    ui_state.params.put("FordAngleAutoCalError", "")
    ui_state.params.put("FordLowSpeedFactor_ang", 1.0)
    ui_state.params.put("FordHighSpeedFactor_ang", 1.0)


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
      "High Speed Low Curve Adjustment Factor", "FordHighSpeedDampening_ang", min=0.75, max=1.25, step=0.01,
    )
    # One-time auto-calibration of the two factors above; toggling off clears the lock
    # so re-enabling starts a fresh collection.
    self.angle_autocal = BigParamControlBP(
      "Auto-Calibrate Factors", "FordAngleAutoCal",
      toggle_callback=self._on_autocal_toggled,
    )
    # Full retry: wipes evidence AND puts both factors back to 1.00 (the toggle above
    # only clears the lock; it leaves the factors wherever the calibrator walked them).
    self.angle_autocal_erase = _EraseAutoCalButton()
    # On (default): calibration freezes once stable. Off: never locks — keeps adapting;
    # turning it off on an already-locked car resumes from the saved evidence.
    self.angle_autocal_lock = BigParamControlBP(
      "Calibration Lock", "FordAngleAutoCalLock",
    )
    # Anti-weave smoothing of the angle command path (see lateral_angle_ext.py _SM_*).
    self.angle_smoothing = BigParamControlBP(
      "Smooth Steering (Anti-Weave)", "FordAngleSmoothing",
    )
    self.angle_smoothing_strength = BigParamFloatControl(
      "Smoothing Strength", "FordAngleSmoothStrength", min=1.0, max=2.5, step=0.1,
    )
    self.lane_change_factor_high_ang = BigParamFloatControl(
      "Lane Change Factor High", "lane_change_factor_high_ang", min=0.85, max=1.50,
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
      self.angle_autocal,
      self.angle_autocal_lock,
      self.angle_autocal_erase,
      self.angle_smoothing,
      self.angle_smoothing_strength,
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
      ("FordAngleAutoCal", self.angle_autocal),
      ("FordAngleAutoCalLock", self.angle_autocal_lock),
      ("FordAngleSmoothing", self.angle_smoothing),
      ("disable_BP_lat_UI", self.disable_BP_lat),
      ("BlinkerPauseLaneChange", self.disable_lane_change_under_speed),
      ("enable_human_turn_detection_curv", self.enable_human_turn_detection),
      ("enable_lane_positioning_curv", self.enable_lane_positioning),
      ("enable_lane_full_mode_curv", self.enable_lane_full_mode),
      ("custom_profile_curv", self.custom_profile),
      ("BpShowLateralControl", self.show_lateral_control),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _on_autocal_toggled(self, state: bool):
    """Disarming clears a finished calibration's lock so re-enabling starts fresh."""
    if not state:
      ui_state.params.put("FordAngleAutoCalState", "")

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
    self.angle_autocal.set_visible(is_angle)
    self.angle_autocal_lock.set_visible(is_angle)
    self.angle_autocal_erase.set_visible(is_angle)
    self.angle_smoothing.set_visible(is_angle)
    self.angle_smoothing_strength.set_visible(is_angle)
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
