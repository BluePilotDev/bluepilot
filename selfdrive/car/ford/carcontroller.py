import numpy as np
from cereal import car, log
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.car import apply_std_steer_angle_limits
from openpilot.selfdrive.car.ford import fordcan
from openpilot.selfdrive.car.ford.values import CarControllerParams, FordFlags, FordFlagsSP
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N
from openpilot.selfdrive.modeld.constants import ModelConstants

LongCtrlState = car.CarControl.Actuators.LongControlState
VisualAlert = car.CarControl.HUDControl.VisualAlert
LaneChangeState = log.LaneChangeState # is lane change active


def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                           current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, CarControllerParams)

  return clip(apply_curvature, -CarControllerParams.CURVATURE_MAX, CarControllerParams.CURVATURE_MAX)

def hysteresis(current_value, old_value, target, stdDevLow: float, stdDevHigh: float):
  if target - stdDevLow < current_value < target + stdDevHigh:
    result = old_value
  elif current_value <= target - stdDevLow:
    result = 1
  elif current_value >= target + stdDevHigh:
    result = 0

  return result

def actuators_calc(self, brake):
  ts = self.frame * DT_CTRL
  brake_actuate = hysteresis(brake, self.brake_actuate_last, self.brake_actutator_target, self.brake_actutator_stdDevLow, self.brake_actutator_stdDevHigh)
  self.brake_actuate_last = brake_actuate

  precharge_actuate = hysteresis(brake, self.precharge_actuate_last, self.precharge_actutator_target, self.precharge_actutator_stdDevLow, self.precharge_actutator_stdDevHigh)
  if precharge_actuate and not self.precharge_actuate_last:
    self.precharge_actuate_ts = ts
  elif not precharge_actuate:
    self.precharge_actuate_ts = 0

  if (
      precharge_actuate and
      not brake_actuate and
      self.precharge_actuate_ts > 0 and
      brake > (self.precharge_actutator_target - self.precharge_actutator_stdDevLow) and
      (ts - self.precharge_actuate_ts) > (200 * DT_CTRL)
    ):
    precharge_actuate = False

  self.precharge_actuate_last = precharge_actuate

  return precharge_actuate, brake_actuate

class CarController:
  def __init__(self, dbc_name, CP, VM):
    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.CAN = fordcan.CanBus(CP)
    self.frame = 0

    self.precision_type = 1
    self.apply_curvature_last = 0
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.gac_tr_cluster_last = -1
    self.gac_tr_cluster_last_ts = 0
    self.brake_actuate_last = 0
    self.precharge_actuate_last = 0
    self.precharge_actuate_ts = 0
    self.lead_distance_bars_last = None
    # self.path_angle = 0.
    # self.path_offset = 0.
    # self.curvature_rate = 0.

    self.brake_actuate_last = 0
    self.precharge_actuate_last = 0
    self.precharge_actuate_ts = 0

    # Anti ping-pong parameters
    self.t_diffs = np.diff(ModelConstants.T_IDXS)
    # self.desired_curvature_rate_scale = -0.07 # determined in plotjuggler to best match with `LatCtlCrv_NoRate2_Actl`
    self.future_lookup_time_diff = 0.5
    self.future_lookup_time = CP.steerActuatorDelay
    self.future_curvature_time_v = [self.future_lookup_time, self.future_lookup_time_diff + self.future_lookup_time] # how many seconds in the future to use predicted curvature
    self.future_curvature_time_bp = [5.0, 30.0] # corresponding speeds in m/s in [0, ~40] in 1.0 increments
    self.max_app_curvature = 0.00028 # maximum curvature to still be considered a straightaway (for anti ping-pong purposes)
    # self.app_filter_factor = 0.45 # how much to allow current signals for anti ping-pong
    # self.app_damp_factor = 0.85 # how much to mute all signals for anti ping-pong
    self.app_PC_percentage = 0.4 # what percentage of apply_curvature is derived from predicted curvature for straight aways
    # self.lc_PC_percentage = 0.4 # what percentage of apply_curvature is derived from predicted curvature for lane changes
    self.right_lc_modifier = 0.80 # how much to reduce curvature of right lane changes
    self.lc1_modifier = 0.80 # how much to reduce curvature during "starting lane change"
    self.lc2_modifier = 0.95 # how much to reduce curvature during "chanigng lanes"
    self.lane_change = False # initialize variable for capturing lane change status

    # Activates at self.brake_actutator_target - self.brake_actutator_stdDevLow
    self.brake_actutator_target = -0.1 # Default: -0.5
    self.brake_actutator_stdDevLow = 0.05 # Default: -0.5

    # Deactivates at self.brake_actutator_target + self.brake_actutator_stdDevHigh
    self.brake_actutator_stdDevHigh = 0.08 # Default: 0

    # Activates at self.precharge_actutator_target - self.precharge_actutator_stdDevLow
    self.precharge_actutator_stdDevLow = 0.03 # Default: -0.25

    # Deactivates at self.precharge_actutator_target + self.precharge_actutator_stdDevHigh
    self.precharge_actutator_stdDevHigh = 0.05 # Default: 0

    self.precharge_actutator_target = -0.07
    self.brake_0_point = 0
    self.brake_converge_at = -1.5
    self.testing_active = False

    # Deactivates at self.precharge_actutator_target + self.precharge_actutator_stdDevHigh
    self.target_speed_multiplier = 1 # Default: 0

    # model specific tuning
    print(f'CarFingerprint: {self.CP.carFingerprint}')
    if self.CP.flags & FordFlags.CANFD:
      self.testing_active = True

      if self.CP.carFingerprint == "FORD F-150 14TH GEN":
        print(f'Matched carFingerprint: {self.CP.carFingerprint}')
        self.brake_actutator_target = -0.1
        self.brake_actutator_stdDevLow = 0.00
        self.brake_actutator_stdDevHigh = 0.05
        self.precharge_actutator_target = -0.1
        self.precharge_actutator_stdDevLow = 0.0
        self.precharge_actutator_stdDevHigh = 0.05
        self.app_PC_percentage = 0.4 # what percentage of apply_curvature is derived from predicted curvature

      elif self.CP.carFingerprint == "FORD F-150 LIGHTNING 1ST GEN":
        print(f'Matched carFingerprint: {self.CP.carFingerprint}')
        self.brake_actutator_target = -0.1
        self.brake_actutator_stdDevLow = 0.00
        self.brake_actutator_stdDevHigh = 0.05
        self.precharge_actutator_target = -0.1
        self.precharge_actutator_stdDevLow = 0.0
        self.precharge_actutator_stdDevHigh = 0.05
        self.app_PC_percentage = 0.4 # what percentage of apply_curvature is derived from predicted curvature

      elif self.CP.carFingerprint == "FORD MUSTANG MACH-E 1ST GEN":
        print(f'Matched carFingerprint: {self.CP.carFingerprint}')
        self.brake_actutator_target = -0.1
        self.brake_actutator_stdDevLow = 0.00
        self.brake_actutator_stdDevHigh = 0.05
        self.precharge_actutator_target = -0.1
        self.precharge_actutator_stdDevLow = 0.0
        self.precharge_actutator_stdDevHigh = 0.05
        self.app_PC_percentage = 0.5 # what percentage of apply_curvature is derived from predicted curvature

      elif self.CP.carFingerprint == "FORD ESCAPE 2023 REFRESH":
        print(f'Matched carFingerprint: {self.CP.carFingerprint}')
        self.brake_actutator_target = -0.1
        self.brake_actutator_stdDevLow = 0.00
        self.brake_actutator_stdDevHigh = 0.05
        self.precharge_actutator_target = -0.1
        self.precharge_actutator_stdDevLow = 0.0
        self.precharge_actutator_stdDevHigh = 0.05
        self.app_PC_percentage = 0.5 # what percentage of apply_curvature is derived from predicted curvature
    else:
      self.brake_actutator_target = -0.1
      self.brake_actutator_stdDevLow = 0.00
      self.brake_actutator_stdDevHigh = 0.05
      self.precharge_actutator_target = -0.1
      self.precharge_actutator_stdDevLow = 0.0
      self.precharge_actutator_stdDevHigh = 0.05
      self.app_PC_percentage = 0.5 # what percentage of apply_curvature is derived from predicted curvature

    self.brake_clip = self.brake_actutator_target - self.brake_actutator_stdDevLow

  def update(self, CC, CS, now_nanos, model_data=None):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        # apply rate limits, curvature error limit, and clip to signal range
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
        desired_curvature = actuators.curvature
        apply_curvature = desired_curvature
        immediate_curvature = apply_ford_curvature_limits(desired_curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw)
        self.precision_type = 1 #precise by default
        # equate velocity
        vEgoRaw = CS.out.vEgoRaw

        if model_data is not None and len(model_data.orientation.x) >= CONTROL_N:
          # compute curvature from model predicted orientation
          future_time = 0.2 + self.future_lookup_time # 0.2 + SteerActutatorDelay
          predicted_curvature = interp(future_time, ModelConstants.T_IDXS, model_data.orientationRate.z) / vEgoRaw
         
          # build an array to hold future curvatures, to help with straight away detection
          curvatures = np.array(model_data.acceleration.y) / (CS.out.vEgo ** 2)
          # extract predicted curvature for 1.0 seconds, 2.0 seconds, and 3.0 seconds into the future
          curvature_1 = abs(interp(1, ModelConstants.T_IDXS, curvatures))
          curvature_2 = abs(interp(2, ModelConstants.T_IDXS, curvatures))
          curvature_3 = abs(interp(3, ModelConstants.T_IDXS, curvatures))

        # determine if a lane change is active
        if model_data.meta.laneChangeState == 1 or model_data.meta.laneChangeState == 2:
          self.lane_change = True
        else:
          self.lane_change = False

        # if at highway speeds, check for straight aways and apply anti ping pong logic
        if vEgoRaw > 24.56:
          if abs(immediate_curvature) < self.max_app_curvature and curvature_1 < self.max_app_curvature and curvature_2 < self.max_app_curvature and curvature_3 < self.max_app_curvature:
            apply_curvature = ((predicted_curvature * self.app_PC_percentage) + (desired_curvature * (1- self.app_PC_percentage)))
            self.precision_type = 0 # comfort for straight aways

        # apply ford cuvature safety limits
        apply_curvature = apply_ford_curvature_limits(apply_curvature, self.apply_curvature_last, current_curvature, vEgoRaw)
        
        # if changing lanes, blend PC and DC to smooth out the lane change.
        if self.lane_change:
          if apply_curvature > 0 and model_data.meta.laneChangeState == 1: # initial stages of a right lane change (positive in comma, negative when sent to Ford)
            apply_curvature = apply_curvature * self.right_lc_modifier
          if abs(apply_curvature) < self.max_app_curvature: # if we are not changing lanes in a curve
            if model_data.meta.laneChangeState == 1:
              apply_curvature = apply_curvature * self.lc1_modifier
            if model_data.meta.laneChangeState == 2:
              apply_curvature = apply_curvature * self.lc2_modifier
          self.precision_type = 0 # comfort for lane change

      else:
        apply_curvature = 0.

      # human turn detection
      steeringPressed = CS.out.steeringPressed
      steeringAngleDeg = CS.out.steeringAngleDeg

      # if a human turn is active, reset steering to prevent windup
      if steeringPressed and abs(steeringAngleDeg) > 45:
        apply_curvature = 0
        ramp_type = 3
      else:
        ramp_type = 0

      self.apply_curvature_last = apply_curvature

      if self.CP.flags & FordFlags.CANFD:
        # TODO: extended mode
        mode = 1 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        if self.CP.spFlags & FordFlagsSP.SP_ENHANCED_LAT_CONTROL.value:
          can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, ramp_type, self.precision_type, 0., 0., -apply_curvature, 0., counter))
        else:
          can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, ramp_type, self.precision_type, 0., 0., -apply_curvature, 0., counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, ramp_type, self.precision_type, 0., 0., -apply_curvature, 0.))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      # Both gas and accel are in m/s^2, accel is used solely for braking
      accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
      gas = accel
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS
      stopping = CC.actuators.longControlState == LongCtrlState.stopping

      precharge_actuate, brake_actuate = actuators_calc(self, accel)
      brake = accel
      if brake < 0 and brake_actuate:
        brake = interp(accel, [ CarControllerParams.ACCEL_MIN, self.brake_converge_at, self.brake_clip], [CarControllerParams.ACCEL_MIN, self.brake_converge_at, self.brake_0_point])

      # Calculate targetSpeed
      targetSpeed = clip(actuators.speed * self.target_speed_multiplier, 0, V_CRUISE_MAX)
      if not CC.longActive and hud_control.setSpeed:
        targetSpeed = hud_control.setSpeed

      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, brake, stopping, brake_actuate, precharge_actuate, v_ego_kph=targetSpeed))

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, hud_control,
                                                 CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.copy()
    new_actuators.curvature = self.apply_curvature_last

    self.frame += 1
    return new_actuators, can_sends
