"""
BluePilot Ford lateral curvature extension.

Implements full 4-signal lateral control (curvature, curvature_rate, path_offset, path_angle)
using predicted curvature from modelV2, PID-based lane centering, and laneline-aware path offset.
Mixed into CarController as LateralCurvExt.

Ford uses four signals to control steering:
  - curvature: primary steering command (also used in upstream, limited to 0.02 m^-1)
  - curvature_rate: derivative of curvature for smoother entry/exit of curves
  - path_offset: lateral offset for lane centering (blends model + laneline data)
  - path_angle: heading angle correction via PID controller

A detailed explanation of Ford's control protocol:
https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
"""

from collections import namedtuple, deque
from enum import IntEnum

import cereal.messaging as messaging
import numpy as np
from numpy import clip, interp

from common.pid import PIDController
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.ford.values import CarControllerParams, FordFlags
from opendbc.sunnypilot.car.ford.values_ext import BP_ANGLE_LIMITS, CURVATURE_MAX
from opendbc.sunnypilot.car.ford.human_turn import HumanTurnDetector
from selfdrive.modeld.constants import ModelConstants


class PrimaryLateralControl(IntEnum):
  curvature = 0
  angle = 1


# CAN FD lateral-accel cap (match opendbc/car/ford/carcontroller.py apply_ford_curvature_limits)
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL - (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)


# Result namedtuple returned by LateralCurvExt.update()
LateralResult = namedtuple('LateralResult', [
  'apply_curvature',
  'curvature_rate',
  'path_offset',
  'path_angle',
  'ramp_type',
  'precision_type',
  'lateralUncertainty',
])


def apply_ford_curvature_limits_ext(apply_curvature, apply_curvature_last, current_curvature,
                                     v_ego_raw, steering_angle, lat_active, CP):
  """Extended version of apply_ford_curvature_limits that returns
  (apply_curvature, max_curvature, curvature_deviation_limited).

  The max_curvature value is used by _calculate_lateral_uncertainty() for the steering
  torque bar visualization. Stock version returns only apply_curvature.

  curvature_deviation_limited (BluePilot): True when the current_curvature +- CURVATURE_ERROR
  clip below actually constrained the commanded curvature this frame -- i.e. the desired
  maneuver was limited by deviation from measured current curvature, not by rate-of-change
  (apply_std_steer_angle_limits below is a separate limiter with its own telemetry).
  """
  max_curvature = 1  # large initial value
  curvature_deviation_limited = False

  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature_pre_error_clip = apply_curvature
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)
    curvature_deviation_limited = bool(abs(apply_curvature - apply_curvature_pre_error_clip) > 1e-9)
    max_curvature = abs(current_curvature) + CarControllerParams.CURVATURE_ERROR

  # Curvature rate limit after driver torque limit (same inputs/order as apply_ford_curvature_limits)
  apply_curvature_before_std = apply_curvature
  apply_curvature = apply_std_steer_angle_limits(apply_curvature_before_std, apply_curvature_last, v_ego_raw,
                                                  steering_angle, lat_active, BP_ANGLE_LIMITS)

  # Max one-step envelope for UI: steer_up must use pre-limit curvature (see apply_std_steer_angle_limits)
  steer_up = (apply_curvature_last * apply_curvature_before_std >= 0.0
              and abs(apply_curvature_before_std) > abs(apply_curvature_last))
  rate_limits = BP_ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else BP_ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN
  std_steer_angle_rate_limit = np.interp(v_ego_raw, rate_limits[0], rate_limits[1])
  std_steer_angle_limit = abs(apply_curvature_last) + abs(std_steer_angle_rate_limit)
  max_curvature = np.minimum(max_curvature, std_steer_angle_limit)

  # Ford CAN FD lateral acceleration limit (more torque available than CAN)
  if CP.flags & FordFlags.CANFD:
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))
    max_curvature = np.minimum(max_curvature, abs(curvature_accel_limit))

  return apply_curvature, max_curvature, curvature_deviation_limited


class LateralCurvExt:
  """
  BluePilot lateral control extension for Ford vehicles.

  Mixed into CarController via multiple inheritance. CarController calls
  LateralCurvExt.update() during the lateral control loop (20Hz) to get the full
  4-signal steering command instead of the upstream curvature-only approach.
  """

  def __init__(self, CP, CP_SP):
    # SubMaster for model data, live parameters, and selfdrive state
    # liveDelay is consumed by LateralAngleExt (variable lookup time); harmless for curvature mode.
    self.sm = messaging.SubMaster(['modelV2', 'liveParameters', 'selfdriveState', 'radarState', 'liveDelay'])
    self.VM = VehicleModel(CP)
    self.model = None
    self.lp = None
    self.ss = None

    # Primary lateral control variable: consumed by CarController's lateral dispatch.
    self.primary_lateral_control = PrimaryLateralControl.curvature

    # Toggles (updated from Params each frame)
    self.enable_human_turn_detection_curv = True
    self.enable_lane_positioning_curv = False
    self.custom_profile_curv = 0

    # Precision/ramp control
    self.precision_type = 1  # 1=Precise, 0=Comfortable
    self.lateralUncertainty = 0.0
    # BluePilot: deviation-from-current-curvature telemetry (see carcontroller.py)
    self.bp_curvature_deviation_limited = False

    # Predicted curvature blending
    self.curvature_lookup_time = 0.2  # seconds into the future for curvature extraction
    self.pc_blend_ratio = 0.5
    self.pc_blend_ratio_bp = [0.0, 0.001]  # curvature breakpoints (1/m)
    self.pc_blend_ratio_low = 0.40
    self.pc_blend_ratio_high = 0.40
    self.pc_blend_ratio_low_C = 0.40   # from UI when custom_profile_curv == 1
    self.pc_blend_ratio_high_C = 0.40  # from UI when custom_profile_curv == 1
    self.pc_blend_ratio_low_C_UI_curv = 0.40   # set by update_lateral_params()
    self.pc_blend_ratio_high_C_UI_curv = 0.40  # set by update_lateral_params()

    # Lane change smoothing
    self.lane_change_factor_bp = [4.4, 40.23]  # speed breakpoints (m/s)
    self.lane_change_factor_low = 0.95
    self.lane_change_factor_high_curv = 0.85  # updated from UI
    self.lane_change = False
    self.lane_change_last = False

    # Post lane change transition
    self.post_lane_change_timer = 0
    self.post_lane_change_active = False
    self.pre_lane_change_values = {'path_angle': 0.0, 'path_offset': 0.0, 'desired_curvature_rate': 0.0}
    self.max_path_angle_change = 0.00125
    self.max_path_offset_change = 0.00125
    # Lane-change smoothing for curvature_rate; keep >= tightest BP rate step (0.00008 at 25 m/s)
    self.max_curvature_rate_change = 0.00025

    # Human turn detection (shared with angle mode — see human_turn.HumanTurnDetector)
    self.human_turn_detector = HumanTurnDetector()
    self.human_turn = False
    self.post_reset_ramp_active = False
    self.reset_steering_last = False

    # Curvature rate computation
    self.curvature_rate_delta_t = 0.3  # seconds for derivative window
    _dt_lat = CarControllerParams.STEER_STEP * DT_CTRL
    self.curvature_rate_deque = deque(maxlen=int(round(self.curvature_rate_delta_t / _dt_lat)))
    self.curvature_rate_speed_bp = [0.0, 14.5, 15.5]  # m/s
    self.curvature_rate_speed_v = [1.0, 1.0, 0.0]
    self.curvature_rate_PC_bp = [0.0, 0.008, 0.01]  # 1/m
    self.curvature_rate_PC_v = [0.0, 0.0, 1.0]
    self.large_curve_factor_low = 1.0
    self.large_curve_factor_high = 0.80
    self.large_curve_factor_bp = [0.001, 0.02]  # 1/m
    self.large_curve_factor_v = [self.large_curve_factor_low, self.large_curve_factor_high]

    # Path offset
    self.custom_path_offset_curv = 0.0  # from UI
    self.path_offset_lookup_time = 0.2  # seconds
    self.min_laneline_confidence_bp = [0.6, 0.8]
    self.enable_lane_full_mode_curv = True

    # PID-based path angle for lane centering
    self.path_angle_filter_samples = 3
    self.path_angle_deque = deque(maxlen=self.path_angle_filter_samples)
    self.LC_PID_gain_UI_curv = 0.0
    self.LC_PID_gain = 3.0
    self.LC_PID_k_p = 0.25
    self.LC_PID_k_i = 0.05
    self.LC_PID_controller = PIDController(k_p=self.LC_PID_k_p, k_i=self.LC_PID_k_i, rate=20)
    self.LC_PID_speed_bp = [0.0, 9.0, 15.0]  # m/s
    self.LC_PID_speed_v = [0.0, 0.0, 1.0]
    self.LC_path_angle_ROC_bp = [5, 15, 25]  # m/s
    self.LC_path_angle_ROC_v = [0.003, 0.0015, 0.002]  # match panda limits
    self.LC_path_angle_reset_counter = 0
    self.LC_path_angle_reset_duration = 1.5  # seconds

    # DBC signal max limits
    self.path_angle_max = 0.5
    self.path_offset_max = 2.0
    self.curvature_max = CURVATURE_MAX  # 0.02
    self.curvature_rate_max = 0.001023

    # Previous frame values
    self.curvature_rate_last = 0.0
    self.path_offset_last = 0.0
    self.path_angle_last = 0.0
    self.curvature_rate = 0

  def update_lateral_params(self, params):
    """Read lateral-related Params from the UI. Called each frame."""
    self.enable_human_turn_detection_curv = params.get_bool("enable_human_turn_detection_curv")
    self.lane_change_factor_high_curv = float(params.get("lane_change_factor_high_curv", return_default=True))
    self.pc_blend_ratio_high_C_UI_curv = float(params.get("pc_blend_ratio_high_C_UI_curv", return_default=True))
    self.pc_blend_ratio_low_C_UI_curv = float(params.get("pc_blend_ratio_low_C_UI_curv", return_default=True))
    self.enable_lane_positioning_curv = params.get_bool("enable_lane_positioning_curv")
    self.custom_path_offset_curv = float(params.get("custom_path_offset_curv", return_default=True))
    self.enable_lane_full_mode_curv = params.get_bool("enable_lane_full_mode_curv")
    self.custom_profile_curv = int(params.get("custom_profile_curv", return_default=True))
    self.LC_PID_gain_UI_curv = float(params.get("LC_PID_gain_UI_curv", return_default=True))

    self.primary_lateral_control = PrimaryLateralControl(params.get("FordPrefLateralControl", return_default=True) or 0)

  def _ensure_lateral_curv_initialized(self, CP):
    # Compatibility shim for LateralAngleExt, which calls this as a lazy-init guard. In this
    # branch LateralCurvExt state is initialized eagerly in __init__, so nothing to do here.
    pass

  def get_current_curvature(self, CS):
    """Measured curvature of the car right now (OP sign convention).

    The single measurement source for every BluePilot lateral consumer: the deviation
    clip, the stall detector, and the shadow curvature published to ford.h's angle-mode
    deviation check. Sourced from the RCM yaw rate -- the same family ford.h derives its
    angle_meas from. The shadow value judged against that check must always come from
    the same measurement as the check's own reference, so route all reads through here.
    """
    return -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

  def update_sm(self):
    """Update SubMaster and vehicle model. Called each frame before lateral/long update."""
    self.sm.update(0)

    if self.sm.updated['modelV2']:
      self.model = self.sm['modelV2']
    if self.sm.updated['liveParameters']:
      self.lp = self.sm['liveParameters']
    if self.sm.updated['selfdriveState']:
      self.ss = self.sm['selfdriveState']

    if self.lp is not None:
      x = max(self.lp.stiffnessFactor, 0.1)
      sr = max(self.lp.steerRatio, 0.1)
      self.VM.update_params(x, sr)

  def update(self, CC, CS, actuators, apply_curvature_last, CP):
    """
    Compute lateral steering signals for the current frame.

    Called at 20Hz from CarController.update() when inside the STEER_STEP block.

    Args:
      CC: CarControl with latActive, hudControl
      CS: CarState with vEgoRaw, yawRate, steeringPressed, steeringAngleDeg
      actuators: CC.actuators with desired curvature from planner
      apply_curvature_last: previous frame's applied curvature
      CP: CarParams with flags for CANFD etc.

    Returns:
      LateralResult namedtuple with all signals needed for CAN message construction.
    """
    apply_curvature = 0.0
    desired_curvature_rate = 0.0
    path_offset = 0.0
    path_angle = 0.0
    reset_steering = 0
    ramp_type = 2
    lateralUncertainty = 0.0
    curvature_deviation_limited = False

    if CC.latActive:
      self.precision_type = 1
      steeringPressed = CS.out.steeringPressed
      steeringAngleDeg_PV = CS.out.steeringAngleDeg

      # Select tuning profile
      if self.custom_profile_curv == 1:
        self.pc_blend_ratio_low_C = self.pc_blend_ratio_low_C_UI_curv
        self.pc_blend_ratio_high_C = self.pc_blend_ratio_high_C_UI_curv
        self.LC_PID_gain = self.LC_PID_gain_UI_curv

      self.pc_blend_ratio_v = [self.pc_blend_ratio_low_C, self.pc_blend_ratio_high_C]

      # Current and desired curvature
      current_curvature = self.get_current_curvature(CS)
      desired_curvature = actuators.curvature

      # Extract predicted curvature from modelV2
      if self.model is not None and len(self.model.orientation.x) >= 17:
        curvatures = np.array(self.model.orientationRate.z) / max(0.01, CS.out.vEgoRaw)
        predicted_curvature = interp(self.curvature_lookup_time, ModelConstants.T_IDXS, curvatures)
      else:
        predicted_curvature = 0.0

      # Blend predicted and desired curvature
      self.pc_blend_ratio = interp(abs(desired_curvature), self.pc_blend_ratio_bp, self.pc_blend_ratio_v)
      requested_curvature = (predicted_curvature * self.pc_blend_ratio) + (desired_curvature * (1 - self.pc_blend_ratio))

      # Lane change detection
      if self.model is not None:
        self.lane_change = self.model.meta.laneChangeState in (1, 2, 3)
      else:
        self.lane_change = False

      # Lane change curvature smoothing
      lane_change_factor = interp(CS.out.vEgoRaw, self.lane_change_factor_bp,
                                   [self.lane_change_factor_low, self.lane_change_factor_high_curv])

      if self.lane_change and self.model is not None:
        if self.model.meta.laneChangeDirection == 1 and requested_curvature < 0:
          requested_curvature *= lane_change_factor
          self.precision_type = 0
        elif self.model.meta.laneChangeDirection == 2 and requested_curvature > 0:
          requested_curvature *= lane_change_factor
          self.precision_type = 0

      # Human turn: sustained hands-on + large angle (not just a nudge) — see HumanTurnDetector
      self.human_turn = self.human_turn_detector.update(
        self.enable_human_turn_detection_curv, steeringPressed, steeringAngleDeg_PV)

      # Steering reset logic
      if self.human_turn or (CS.out.vEgoRaw < 0.1):
        reset_steering = 1
      if reset_steering == 1:
        requested_curvature = 0.0
        # BluePilot: keep deque from accumulating model curvature while commanding reset; avoids
        # large desired_curvature_rate on the bus when κ/path_angle are zero (panda curvature_rate_cmd_checks).
        self.curvature_rate_deque.clear()

      # Apply curvature limits (extended version returning max_curvature)
      apply_curvature, max_curvature, curvature_deviation_limited = apply_ford_curvature_limits_ext(
        requested_curvature, apply_curvature_last, current_curvature,
        CS.out.vEgoRaw, 0, CC.latActive, CP)

      # Lateral uncertainty for torque bar visualization
      lateralUncertainty = self._calculate_lateral_uncertainty(requested_curvature, apply_curvature, max_curvature)

      # Human turn / standstill reset: after apply_ford_curvature_limits_ext (for torque-bar max_curvature),
      # force κ=0 and (below) path_angle=0 on the bus. ford.h reset latch treats desired_curvature==0 &&
      # desired_path_angle==0 as the neutral reset frame and clears rate violations for that TX, then
      # holds ~3s bypass (see ford.h reset_bypass_latch_counter on LateralMotionControl / LateralMotionControl2).
      if reset_steering == 1:
        apply_curvature = 0.0
        self.post_reset_ramp_active = False
      else:
        # Detect transition from reset to normal
        if self.reset_steering_last and not reset_steering:
          self.post_reset_ramp_active = True
          apply_curvature_last = 0.0  # Reset to ensure clean ramp from 0

      # Post-reset ramp: gradually ramp from 0 to avoid safety limit trips
      if self.post_reset_ramp_active:
        apply_curvature = apply_std_steer_angle_limits(
          requested_curvature, apply_curvature_last,
          CS.out.vEgoRaw, 0, CC.latActive, BP_ANGLE_LIMITS)

        curvature_error = abs(requested_curvature - apply_curvature)
        curvature_threshold = max(abs(requested_curvature) * 0.1, 0.001)
        if curvature_error < curvature_threshold:
          self.post_reset_ramp_active = False

      self.reset_steering_last = (reset_steering == 1)

      # Curvature rate (derivative of predicted curvature), 20Hz-aligned with LateralMotionControl STEER_STEP
      if reset_steering != 1:
        self.curvature_rate_deque.append(predicted_curvature)
        if len(self.curvature_rate_deque) > 1:
          dt_frame = CarControllerParams.STEER_STEP * DT_CTRL
          delta_t = (self.curvature_rate_delta_t
                     if len(self.curvature_rate_deque) == self.curvature_rate_deque.maxlen
                     else (len(self.curvature_rate_deque) - 1) * dt_frame)
          desired_curvature_rate = ((self.curvature_rate_deque[-1] - self.curvature_rate_deque[0])
                                     / delta_t / max(0.01, CS.out.vEgoRaw))
        else:
          desired_curvature_rate = 0.0
      else:
        desired_curvature_rate = 0.0

      # Curvature rate factors
      curvature_rate_PC_factor = interp(abs(predicted_curvature), self.curvature_rate_PC_bp, self.curvature_rate_PC_v)
      desired_curvature_rate *= curvature_rate_PC_factor

      curvature_rate_speed_factor = interp(CS.out.vEgoRaw, self.curvature_rate_speed_bp, self.curvature_rate_speed_v)
      desired_curvature_rate *= curvature_rate_speed_factor

      large_curve_factor = interp(abs(requested_curvature), self.large_curve_factor_bp, self.large_curve_factor_v)
      desired_curvature_rate *= large_curve_factor

      # Zero curvature rate during lane changes
      if self.lane_change:
        desired_curvature_rate = 0.0

      # Path offset: blend model position with laneline data
      if self.model is not None:
        path_offset_position = interp(self.path_offset_lookup_time, ModelConstants.T_IDXS, self.model.position.y)
        path_offset_lanelines = (self.model.laneLines[1].y[0] + self.model.laneLines[2].y[0]) / 2

        # Laneline width tolerance (prevents jumps when lanes merge/diverge)
        laneline_width = self.model.laneLines[2].y[0] + (-self.model.laneLines[1].y[0])
        laneline_width_tolerance = interp(laneline_width, [3.75, 4.25], [0.81, 0.59])

        # Laneline confidence
        laneline_confidence = min(self.model.laneLineProbs[1], self.model.laneLineProbs[2], laneline_width_tolerance)
        if not self.enable_lane_full_mode_curv:
          laneline_confidence = 0.0

        laneline_path_offset_scale = interp(laneline_confidence, self.min_laneline_confidence_bp, [0.0, 1.0])
        path_offset = ((path_offset_position * (1 - laneline_path_offset_scale)) +
                       (path_offset_lanelines * laneline_path_offset_scale)) + self.custom_path_offset_curv

      # No path offset during lane changes
      if self.lane_change:
        path_offset = 0

      # PID-based path angle for lane centering
      path_offset_error = path_offset * (self.LC_PID_gain_UI_curv / 100)
      LC_PID_speed_factor = interp(CS.out.vEgoRaw, self.LC_PID_speed_bp, self.LC_PID_speed_v)
      path_offset_error_adj = path_offset_error * LC_PID_speed_factor

      if not self.enable_lane_positioning_curv:
        path_offset_error_adj = 0.0

      path_angle_low_c = self.LC_PID_controller.update(path_offset_error_adj)

      if not self.enable_lane_positioning_curv:
        path_angle_low_c = 0.0
      if reset_steering == 1:
        path_angle_low_c = 0.0

      # Rate limit path angle for comfort
      path_angle_roc = interp(abs(CS.out.vEgoRaw), self.LC_path_angle_ROC_bp, self.LC_path_angle_ROC_v)
      path_angle_low_c = clip(path_angle_low_c, self.path_angle_last - path_angle_roc, self.path_angle_last + path_angle_roc)

      # Reset PID if driver applies consistent steering pressure
      if steeringPressed:
        self.LC_path_angle_reset_counter += 1
      else:
        self.LC_path_angle_reset_counter = 0
      if self.LC_path_angle_reset_counter > self.LC_path_angle_reset_duration * 20:
        self.LC_PID_controller.reset()

      path_angle = path_angle_low_c  # path_angle_high_c not used currently

      # Post lane change transition smoothing
      path_angle, path_offset, desired_curvature_rate = self._handle_post_lane_change_transition(
        path_angle, path_offset, desired_curvature_rate)

      # Final reset handling
      if reset_steering == 1:
        path_angle = 0.0

      # Clip all signals to DBC limits
      apply_curvature = clip(apply_curvature, -self.curvature_max, self.curvature_max)
      desired_curvature_rate = clip(desired_curvature_rate, -self.curvature_rate_max, self.curvature_rate_max)
      path_offset = clip(path_offset, -self.path_offset_max, self.path_offset_max)
      path_angle = clip(path_angle, -self.path_angle_max, self.path_angle_max)

      # Zero path_offset before CAN send (path_offset and path_angle can conflict, causing discomfort)
      path_offset = 0.0

      # Ramp type selection
      if reset_steering == 1:
        ramp_type = 3  # Immediate
        self.path_angle_deque.clear()
        self.LC_PID_controller.reset()
      else:
        ramp_type = 2  # Fast

    else:
      # Lateral control off — zero everything
      apply_curvature = 0.0
      desired_curvature_rate = 0.0
      path_offset = 0.0
      path_angle = 0.0
      self.path_angle_deque.clear()
      self.LC_PID_controller.reset()
      ramp_type = 0
      lateralUncertainty = 0.0

    # Update state for next frame
    self.lateralUncertainty = lateralUncertainty
    self.curvature_rate_last = desired_curvature_rate
    self.path_offset_last = path_offset
    self.path_angle_last = path_angle
    # BluePilot: did the current_curvature +- CURVATURE_ERROR clip constrain the commanded
    # curvature this frame (deviation from measured, not rate-of-change)? See carcontroller.py.
    self.bp_curvature_deviation_limited = curvature_deviation_limited

    return LateralResult(
      apply_curvature=apply_curvature,
      curvature_rate=desired_curvature_rate,
      path_offset=path_offset,
      path_angle=path_angle,
      ramp_type=ramp_type,
      precision_type=self.precision_type,
      lateralUncertainty=lateralUncertainty,
    )

  def _handle_post_lane_change_transition(self, path_angle, path_offset, desired_curvature_rate):
    """Smooth transition of control variables after lane change completes.

    Rate-limits path_angle, path_offset, and curvature_rate back to target values
    over 160 frames (~8 seconds at 20Hz).
    """
    # Detect lane change completion (True → False transition)
    if self.lane_change_last and not self.lane_change:
      self.post_lane_change_active = True
      self.post_lane_change_timer = 0
      self.pre_lane_change_values = {'path_angle': 0.0, 'path_offset': 0.0, 'desired_curvature_rate': 0.0}

    self.lane_change_last = self.lane_change

    if self.post_lane_change_active:
      self.post_lane_change_timer += 1

      new_path_angle = clip(path_angle,
                            self.pre_lane_change_values['path_angle'] - self.max_path_angle_change,
                            self.pre_lane_change_values['path_angle'] + self.max_path_angle_change)
      new_path_offset = clip(path_offset,
                             self.pre_lane_change_values['path_offset'] - self.max_path_offset_change,
                             self.pre_lane_change_values['path_offset'] + self.max_path_offset_change)
      new_curvature_rate = clip(desired_curvature_rate,
                                self.pre_lane_change_values['desired_curvature_rate'] - self.max_curvature_rate_change,
                                self.pre_lane_change_values['desired_curvature_rate'] + self.max_curvature_rate_change)

      self.pre_lane_change_values = {
        'path_angle': new_path_angle,
        'path_offset': new_path_offset,
        'desired_curvature_rate': new_curvature_rate,
      }

      if self.post_lane_change_timer >= 160:
        self.post_lane_change_active = False

      return (new_path_angle, new_path_offset, new_curvature_rate)

    return (path_angle, path_offset, desired_curvature_rate)

  def _calculate_lateral_uncertainty(self, requested_curvature, apply_curvature, max_curvature):
    """Compute ratio of requested to max achievable curvature for the torque bar UI."""
    max_curvature = np.clip(max_curvature, apply_curvature, self.curvature_max)
    return float(requested_curvature / max_curvature)
