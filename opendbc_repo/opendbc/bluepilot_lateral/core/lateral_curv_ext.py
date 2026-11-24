"""
BluePilot Ford lateral curvature extension.

Implements Ford's 4-signal lateral control (curvature, curvature_rate, path_offset, path_angle)
using predicted curvature from modelV2, PID-based lane centering, and laneline-aware path offset.
Held by BluePilotLateral (carcontroller_ext.py) as LateralCurvExt.

Ford uses four signals to control steering:
  - curvature: primary steering command (also used in upstream, limited to 0.02 m^-1)
  - curvature_rate: always 0, like upstream and stock Ford
  - path_offset: lateral offset for lane centering (blends model + laneline data)
  - path_angle: heading angle correction via PID controller

A detailed explanation of Ford's control protocol:
https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
"""

from collections import namedtuple, deque
from enum import IntEnum

import numpy as np
from numpy import clip, interp

from ...car.common.pid import PIDController
from ...car import ACCELERATION_DUE_TO_GRAVITY
from ...car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from ...car.ford.values import CarControllerParams, FordFlags
from .values_ext import BP_ANGLE_LIMITS, CURVATURE_MAX, LANE_CHANGE_FACTOR_BP, LANE_CHANGE_FACTOR_LOW
from .values_ext import MODEL_T_IDXS
from ..params.manifest import default, value


# the settings curvature mode reads (params/manifest.py), each kept as an attribute of the same name
CURV_SETTINGS = ("enable_human_turn_detection_curv", "lane_change_factor_high_curv", "pc_blend_ratio_high_C_UI_curv",
                 "pc_blend_ratio_low_C_UI_curv", "enable_lane_positioning_curv", "custom_path_offset_curv",
                 "enable_lane_full_mode_curv", "custom_profile_curv", "LC_PID_gain_UI_curv")


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

  Held by BluePilotLateral (carcontroller_ext.py), which calls
  update() during the lateral control loop (20Hz) to get the full
  4-signal steering command instead of the upstream curvature-only approach.
  """

  def __init__(self, inputs, shared):
    # BluePilotLateral's LateralInputs (read) and SharedLateralState (read and written), common to both strategies
    self.inputs = inputs
    self.shared = shared

    # Primary lateral control variable: consumed by BluePilotLateral's lateral dispatch.
    self.primary_lateral_control = PrimaryLateralControl.curvature

    # Settings (updated from the host's params each frame), at their defaults until then
    for key in CURV_SETTINGS:
      setattr(self, key, default(key))

    # Precision/ramp control (precision_type is in the shared state)
    self.lateralUncertainty = 0.0

    # Predicted curvature blending
    self.curvature_lookup_time = 0.2  # seconds into the future for curvature extraction
    self.pc_blend_ratio = 0.5
    self.pc_blend_ratio_bp = [0.0, 0.001]  # curvature breakpoints (1/m)
    self.pc_blend_ratio_low = 0.40
    self.pc_blend_ratio_high = 0.40
    self.pc_blend_ratio_low_C = 0.40   # from UI when custom_profile_curv == 1
    self.pc_blend_ratio_high_C = 0.40  # from UI when custom_profile_curv == 1

    # Lane change smoothing
    self.lane_change = False
    self.lane_change_last = False

    # Post lane change transition
    self.post_lane_change_timer = 0
    self.post_lane_change_active = False
    self.pre_lane_change_values = {'path_angle': 0.0, 'path_offset': 0.0}
    self.max_path_angle_change = 0.00125
    self.max_path_offset_change = 0.00125

    # Human turn detection (the detector is in the shared state, used by both modes)
    self.human_turn = False
    self.post_reset_ramp_active = False
    self.reset_steering_last = False

    # Path offset
    self.path_offset_lookup_time = 0.2  # seconds
    self.min_laneline_confidence_bp = [0.6, 0.8]

    # PID-based path angle for lane centering
    self.path_angle_filter_samples = 3
    self.path_angle_deque = deque(maxlen=self.path_angle_filter_samples)
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

    # Previous frame values
    self.path_offset_last = 0.0

  def update_lateral_params(self, params):
    """Read lateral-related Params from the UI. Called each frame."""
    for key in CURV_SETTINGS:
      setattr(self, key, value(params, key))

    try:
      self.primary_lateral_control = PrimaryLateralControl(value(params, "FordPrefLateralControl"))
    except ValueError:  # a mode other than 0 or 1: curvature mode, the default (#66)
      self.primary_lateral_control = PrimaryLateralControl.curvature

  def update(self, CC, CS, actuators, apply_curvature_last, CP):
    """
    Compute lateral steering signals for the current frame.

    Called at 20Hz from BluePilotLateral.step_lateral() when inside the STEER_STEP block.

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
    path_offset = 0.0
    path_angle = 0.0
    reset_steering = 0
    ramp_type = 2
    lateralUncertainty = 0.0
    curvature_deviation_limited = False

    if CC.latActive:
      self.shared.precision_type = 1
      steeringPressed = CS.out.steeringPressed
      steeringAngleDeg_PV = CS.out.steeringAngleDeg

      # Select tuning profile
      if self.custom_profile_curv == 1:
        self.pc_blend_ratio_low_C = self.pc_blend_ratio_low_C_UI_curv
        self.pc_blend_ratio_high_C = self.pc_blend_ratio_high_C_UI_curv

      self.pc_blend_ratio_v = [self.pc_blend_ratio_low_C, self.pc_blend_ratio_high_C]

      # Current and desired curvature
      current_curvature = self.inputs.get_current_curvature(CS)
      desired_curvature = actuators.curvature

      # Extract predicted curvature from modelV2
      if self.inputs.model is not None and len(self.inputs.model.orientationRate.z) >= 17:
        curvatures = np.array(self.inputs.model.orientationRate.z) / max(0.01, CS.out.vEgoRaw)
        predicted_curvature = interp(self.curvature_lookup_time, MODEL_T_IDXS, curvatures)
      else:
        predicted_curvature = 0.0

      # Blend predicted and desired curvature
      self.pc_blend_ratio = interp(abs(desired_curvature), self.pc_blend_ratio_bp, self.pc_blend_ratio_v)
      requested_curvature = (predicted_curvature * self.pc_blend_ratio) + (desired_curvature * (1 - self.pc_blend_ratio))

      # Lane change detection
      if self.inputs.model is not None:
        self.lane_change = self.inputs.model.meta.laneChangeState in (1, 2, 3)
      else:
        self.lane_change = False

      # Lane change curvature smoothing
      lane_change_factor = interp(CS.out.vEgoRaw, LANE_CHANGE_FACTOR_BP,
                                   [LANE_CHANGE_FACTOR_LOW, self.lane_change_factor_high_curv])

      if self.lane_change and self.inputs.model is not None:
        if self.inputs.model.meta.laneChangeDirection == 1 and requested_curvature < 0:
          requested_curvature *= lane_change_factor
          self.shared.precision_type = 0
        elif self.inputs.model.meta.laneChangeDirection == 2 and requested_curvature > 0:
          requested_curvature *= lane_change_factor
          self.shared.precision_type = 0

      # Human turn: sustained hands-on + large angle (not just a nudge) — see HumanTurnDetector
      self.human_turn = self.shared.human_turn_detector.update(
        self.enable_human_turn_detection_curv, steeringPressed, steeringAngleDeg_PV)

      # Steering reset logic
      if self.human_turn or (CS.out.vEgoRaw < 0.1):
        reset_steering = 1
      if reset_steering == 1:
        requested_curvature = 0.0

      # Apply curvature limits (extended version returning max_curvature)
      apply_curvature, max_curvature, curvature_deviation_limited = apply_ford_curvature_limits_ext(
        requested_curvature, apply_curvature_last, current_curvature,
        CS.out.vEgoRaw, 0, CC.latActive, CP)

      # Lateral uncertainty for torque bar visualization
      lateralUncertainty = self._calculate_lateral_uncertainty(requested_curvature, apply_curvature, max_curvature)

      # Human turn / standstill reset: after apply_ford_curvature_limits_ext (for torque-bar max_curvature),
      # force κ=0 and (below) path_angle=0 on the bus. ford.h accepts that frame (curvature 0 skips the
      # curvature checks), but checks the frames after it as usual: a ramp back that exceeds the rate
      # limit or sits outside the curvature error band is blocked.
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

      # Path offset: blend model position with laneline data
      if self.inputs.model is not None:
        path_offset_position = interp(self.path_offset_lookup_time, MODEL_T_IDXS, self.inputs.model.position.y)
        path_offset_lanelines = (self.inputs.model.laneLines[1].y[0] + self.inputs.model.laneLines[2].y[0]) / 2

        # Laneline width tolerance (prevents jumps when lanes merge/diverge)
        laneline_width = self.inputs.model.laneLines[2].y[0] + (-self.inputs.model.laneLines[1].y[0])
        laneline_width_tolerance = interp(laneline_width, [3.75, 4.25], [0.81, 0.59])

        # Laneline confidence
        laneline_confidence = min(self.inputs.model.laneLineProbs[1], self.inputs.model.laneLineProbs[2], laneline_width_tolerance)
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
      path_angle_low_c = clip(path_angle_low_c, self.shared.path_angle_last - path_angle_roc, self.shared.path_angle_last + path_angle_roc)

      # Reset PID if driver applies consistent steering pressure
      if steeringPressed:
        self.LC_path_angle_reset_counter += 1
      else:
        self.LC_path_angle_reset_counter = 0
      if self.LC_path_angle_reset_counter > self.LC_path_angle_reset_duration * 20:
        self.LC_PID_controller.reset()

      path_angle = path_angle_low_c  # path_angle_high_c not used currently

      # Post lane change transition smoothing
      path_angle, path_offset = self._handle_post_lane_change_transition(path_angle, path_offset)

      # Final reset handling
      if reset_steering == 1:
        path_angle = 0.0

      # Clip all signals to DBC limits
      # float() is required: numpy's clip returns np.float64, and capnp rejects that type when
      # apply_curvature reaches actuators.curvature via apply_curvature_last in carcontroller.py.
      # The cast is exact (both are IEEE 754 doubles), so no output value changes.
      apply_curvature = float(clip(apply_curvature, -self.curvature_max, self.curvature_max))
      path_offset = float(clip(path_offset, -self.path_offset_max, self.path_offset_max))
      path_angle = float(clip(path_angle, -self.path_angle_max, self.path_angle_max))

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
      path_offset = 0.0
      path_angle = 0.0
      self.path_angle_deque.clear()
      self.LC_PID_controller.reset()
      ramp_type = 0
      lateralUncertainty = 0.0

    # Update state for next frame
    self.lateralUncertainty = lateralUncertainty
    self.path_offset_last = path_offset
    self.shared.path_angle_last = path_angle
    # BluePilot: did the current_curvature +- CURVATURE_ERROR clip constrain the commanded
    # curvature this frame (deviation from measured, not rate-of-change)? The BluePilot UI's deviation flag.
    self.shared.bp_curvature_deviation_limited = curvature_deviation_limited

    return LateralResult(
      apply_curvature=apply_curvature,
      curvature_rate=0.0,  # always 0, like stock Ford
      path_offset=path_offset,
      path_angle=path_angle,
      ramp_type=ramp_type,
      precision_type=self.shared.precision_type,
      lateralUncertainty=lateralUncertainty,
    )

  def _handle_post_lane_change_transition(self, path_angle, path_offset):
    """Smooth transition of control variables after lane change completes.

    Rate-limits path_angle and path_offset back to target values
    over 160 frames (~8 seconds at 20Hz).
    """
    # Detect lane change completion (True → False transition)
    if self.lane_change_last and not self.lane_change:
      self.post_lane_change_active = True
      self.post_lane_change_timer = 0
      self.pre_lane_change_values = {'path_angle': 0.0, 'path_offset': 0.0}

    self.lane_change_last = self.lane_change

    if self.post_lane_change_active:
      self.post_lane_change_timer += 1

      new_path_angle = clip(path_angle,
                            self.pre_lane_change_values['path_angle'] - self.max_path_angle_change,
                            self.pre_lane_change_values['path_angle'] + self.max_path_angle_change)
      new_path_offset = clip(path_offset,
                             self.pre_lane_change_values['path_offset'] - self.max_path_offset_change,
                             self.pre_lane_change_values['path_offset'] + self.max_path_offset_change)

      self.pre_lane_change_values = {
        'path_angle': new_path_angle,
        'path_offset': new_path_offset,
      }

      if self.post_lane_change_timer >= 160:
        self.post_lane_change_active = False

      return (new_path_angle, new_path_offset)

    return (path_angle, path_offset)

  def _calculate_lateral_uncertainty(self, requested_curvature, apply_curvature, max_curvature):
    """Compute ratio of requested to max achievable curvature for the torque bar UI."""
    max_curvature = np.clip(max_curvature, apply_curvature, self.curvature_max)
    return float(requested_curvature / max_curvature)
