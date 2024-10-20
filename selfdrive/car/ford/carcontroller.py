# Plan to add interpolated scaling factors
from collections import deque  # used for moving averages
import numpy as np  # used for calculationg predicted curvatures
from cereal import car  # needed for pretty much everything in comma
from openpilot.common.filter_simple import FirstOrderFilter
from opendbc.can.packer import CANPacker  # used for canbus messages
from openpilot.common.numpy_fast import clip, interp  # used for lots of interpolations and clipping
from openpilot.selfdrive.car import DT_CTRL, apply_std_steer_angle_limits  # for applying safety limits
from openpilot.selfdrive.car.ford import fordcan  # ford specific canbus messages
from openpilot.selfdrive.car.ford.values import CarControllerParams, FordFlags, FordFlagsSP
from openpilot.common.params import Params
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N  # for control libraries
from openpilot.selfdrive.modeld.constants import ModelConstants  # for calculations
from openpilot.selfdrive.car.ford.helpers import (
  initialize_param_defaults,
  update_settings_params,
  load_initial_cc_pref_params,
  get_ford_vehicle_tuning_carcontroller,
  actuators_calc,
  compute_dm_msg_values,
  logDebug,
)

LongCtrlState = car.CarControl.Actuators.LongControlState  # is long control engaged
VisualAlert = car.CarControl.HUDControl.VisualAlert  # alerts on screen
EventName = car.CarEvent.EventName


# define the fuction to calculate curvature limits for Ford vehicles
def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                           current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, CarControllerParams)

  return clip(apply_curvature, -CarControllerParams.CURVATURE_MAX, CarControllerParams.CURVATURE_MAX)


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    super().__init__(dbc_name, CP, VM)

    self.params = Params()  # create a shortcut for params object

    self.CP = CP
    self.VM = VM
    self.packer = CANPacker(dbc_name)
    self.CAN = fordcan.CanBus(CP)
    self.frame = 0

    # Load the initial preference settings parameters like send_hands_free_cluster_msg, enable_human_turn_detection, and anti_ping_pong_value, curvature_max, and curvature_error from the params
    load_initial_cc_pref_params(self)

    self.apply_curvature_last = 0  # previous value of apply_curvature
    self.main_on_last = False  # previous main cruise control state
    self.lkas_enabled_last = False  # previous lkas state
    self.steer_alert_last = False  # previous status of steering alert
    self.send_ui_last = False  # previous state of ui elements
    self.send_ui_ts_last = 0  # prevous state of ui elements
    self.brake_actuate_last = 0  # previous state of brake actuator
    self.precharge_actuate_last = 0  # previous state of pre-charge actuator
    self.precharge_actuate_ts = 0  # previous state of pre-charge actuator
    self.lead_distance_bars_last = None
    self.last_log_frame = 0
    self.log_frames = 0

    # Activates at self.brake_actuator_target - self.brake_actuator_stdDevLow
    self.brake_actuator_target = -0.5  # Default: -0.5
    self.brake_actuator_stdDevLow = 0.2  # Default: -0.5

    # Deactivates at self.brake_actuator_target + self.brake_actuator_stdDevHigh
    self.brake_actuator_stdDevHigh = 0.1  # Default: 0

    # Activates at self.precharge_actuator_target - self.precharge_actuator_stdDevLow
    self.precharge_actuator_stdDevLow = 0.1  # Default: -0.25

    # Deactivates at self.precharge_actuator_target + self.precharge_actuator_stdDevHigh
    self.precharge_actuator_stdDevHigh = 0.1  # Default: 0

    self.precharge_actuator_target = -0.1
    self.brake_0_point = 0
    self.brake_converge_at = -1.5
    self.testing_active = False

    # Deactivates at self.precharge_actuator_target + self.precharge_actuator_stdDevHigh
    self.target_speed_multiplier = 1  # Default: 0

    ################################## lateral control parameters ##############################################

    # Variables to initialize (these get updated every scan as part of the control code)
    self.lat_mode = 1  # lateral control mode. 0 = None, 1 = PathFollowingLimitedMode 2 = PathFollowingExtendedMode
    self.precision_type = 1  # precise or comfort
    self.human_turn_frames = 0  # how many scans has a human been turning the wheel
    self.human_turn = False  # have we detected a human override in a turn
    self.steering_limited = False # was steering limited by safety limits
    self.steer_warning = False # flag to tirgger steering warning on screen
    self.steer_warning_count = 0 # how many scan cycles has the steering warning been clear

    # Curvature variables
    self.requested_curvature_filtered = FirstOrderFilter(0.0, 0.3, 0.05)  # filter for apply_curvature
    self.curvature_lookup_time = CP.steerActuatorDelay # + 0.3  # how far into the future do we need to look for curvature signal
    self.lane_change_factor = 0.65  # updated from UI
    self.low_curv_pc_factor = 0.6 # updated from UI. Amount of predicted curvature at city speed
    self.high_curv_pc_factor = 0.4 # updated from UI.  Amount of predicted curvature at highway speed
    self.high_curvature_factor = 0.95 # updated from UI.  reduces curvature to eliminate corner cutting

    # Curvature rate variables
    self.curvature_rate_delta_t = 0.3  # [s] used in denominator for curvature rate calculation
    self.curvature_rate_deque = deque(maxlen=int(round(self.curvature_rate_delta_t / 0.05)))  # 0.3 seconds at 20Hz

    # max absolute values for all four signals
    self.path_angle_max = 0.5  # from dbc files
    self.path_offset_max = 5.11  # from dbc files
    self.curvature_max = 0.01  # adjust limit to avoid windup
    self.curvature_rate_max = 0.001023  # from dbc files

    # human turn detection parameters
    self.human_turn_dur_threshold = 0.5  # how long does a human need to be turning the wheel before we consider it an over-ride instead of an adjustment
    self.human_turn_dur_frames = int(self.human_turn_dur_threshold / (DT_CTRL * 5))  # convert time to scans

    # values from previous frame
    self.curvature_rate_last = 0.0
    self.path_offset_last = 0.0
    self.path_angle_last = 0.0
    self.curvature_rate = 0  # initialize curvature_rate

    # Count cycles for logging
    self.cycle_count = 0

    # set braking tuning based on which vehicle is being driven
    logDebug(f'Car Fingerprint (CarController): {CP.carFingerprint}')

    # Ford Model Specific Tuning
    if CP.flags & FordFlags.CANFD:
      self.testing_active = True

      # Check FORD_VEHICLE_TUNINGS has a key for the carFingerprint
      ford_tuning = get_ford_vehicle_tuning_carcontroller(CP.carFingerprint)
      if ford_tuning:
        # loop throught each key in ford_tuning and set the value to the corresponding key in the CarController object
        for key in ford_tuning:
          logDebug(f'Ford Tuning (carcontroller.py) Key: {key} | Value: {ford_tuning[key]}')
          if ford_tuning[key] is not None:
            setattr(self, key, ford_tuning[key])

    # check each param in helpers.SETTINGS_PARAMS to make sure they are set and if not sets them to the default values based on the car being driven
    initialize_param_defaults(self)

    # Define the Ford Variables object
    self.fordVariables = None

  def update(self, CC, CS, now_nanos, model_data=None):
    can_sends = []

    # Trigger the update of the settings params defined in helpers.SETTINGS_PARAMS
    update_settings_params(self)

    # print(f'FordPrefSendHandsFreeCanMsg: {self.send_hands_free_cluster_msg} | BlueCruise Cluster Present: {CS.bluecruise_cluster_present}')
    # print(f'FordPrefHumanTurnDetectionEnable: {self.enable_human_turn_detection}')
    # logDebug(f'FordLongTuningBrakeActuatorTarget: {self.brake_actuator_target}')
    # logDebug(f'FordLongTuningBrakeActuatorStdDevLow: {self.brake_actuator_stdDevLow}')
    # logDebug(f'FordLongTuningBrakeActuatorStdDevHigh: {self.brake_actuator_stdDevHigh}')
    # logDebug(f'FordLongTuningPrechargeActuatorTarget: {self.precharge_actuator_target}')
    # logDebug(f'FordLongTuningPrechargeActuatorStdDevLow: {self.precharge_actuator_stdDevLow}')
    # logDebug(f'FordLongTuningPrechargeActuatorStdDevHigh: {self.precharge_actuator_stdDevHigh}')
    # logDebug(f'FordLatTuningPathLookupTime: {self.path_lookup_time}')
    # logDebug(f'FordLatTuningResetLookupTime: {self.reset_lookup_time}')
    # logDebug(f'FordLimitsCurvatureError: {self.curvature_error}')
    # logDebug(f'FordLimitsCurvatureMax: {self.curvature_max}')

    self.brake_clip = self.brake_actuator_target - self.brake_actuator_stdDevLow

    actuators = CC.actuators  # create a shortcut for actuators
    hud_control = CC.hudControl  # create a shortcut for hud
    if self.fordVariables is None:
      act = actuators.as_builder()
      self.fordVariables = act.fordVariables

    main_on = CS.out.cruiseState.available  # main cruise control button status
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)  # alerts on screen and dashboard
    fcw_alert = 0  # alert on screen and dashboard
    tja_msg = 0
    tja_warn = 0
    if self.send_driver_monitor_can_msg:
      if self.send_hands_free_cluster_msg:
        # print(f'HudControl: {hud_control}')
        # print(f'tja_msg: {tja_msg} | tja_warn: {tja_warn}')
        tja_msg, tja_warn = compute_dm_msg_values(hud_control, self.send_hands_free_cluster_msg)
    else:
      steer_alert = hud_control.visualAlert == VisualAlert.fcw

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
    #############################################################################################################################
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        self.precision_type = 1
        steeringPressed = CS.out.steeringPressed
        steeringAngleDeg = CS.out.steeringAngleDeg

        # calculate current curvature and model desired curvature
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)  # use canbus data to calculate current_curvature
        self.fordVariables.currentCurvature01 = float(current_curvature)
        desired_curvature = actuators.curvature  # get desired curvature from model
        self.fordVariables.desiredCurvature01 = float(desired_curvature)

        # use the model position data to calculate the predicted path curvature at different time intervals
        if model_data is not None and len(model_data.orientation.x) >= CONTROL_N:
          # compute curvature from model predicted orientationRate, and blend with desired curvature based on max predicted curvature magnitude
          curvatures = np.array(model_data.orientationRate.z) / max(0.01, CS.out.vEgoRaw)
          max_abs_predicted_curvature = max(np.abs(curvatures[:CONTROL_N]))  # max curvature magnitude over next 2.5s
          self.fordVariables.maxAbsPredictedCurvature01 = float(max_abs_predicted_curvature)

          # calculate predicted curvature used for the curvature and curvature_rate variables
          predicted_curvature = interp(self.curvature_lookup_time, ModelConstants.T_IDXS, curvatures)
          self.fordVariables.predictedCurvature01 = float(predicted_curvature)

          # equate apply_curvature to a blend of desired and predicted_curvature and apply curvature limits
          self.pc_blend_ratio_bp = [0, 0.001]  # At what desired curvature do we blend PC and DC? (0/m to 0.001/m)
          self.pc_blend_ratio_v = [self.high_curv_pc_factor, self.low_curv_pc_factor]
          pc_blend_ratio = interp(abs(desired_curvature), self.pc_blend_ratio_bp, self.pc_blend_ratio_v)
          requested_curvature = (predicted_curvature * pc_blend_ratio) + (desired_curvature * (1 - pc_blend_ratio))

          # filter curvature before calculating rate
          requested_curvature = self.requested_curvature_filtered.update(requested_curvature)
          self.fordVariables.requestedCurvature01 = float(requested_curvature)

          # determine if a lane change is active
          if model_data.meta.laneChangeState == 1 or model_data.meta.laneChangeState == 2:
            self.lane_change = True
          else:
            self.lane_change = False

          # if changing lanes, reduce apply_curvature by a factor smooth out the lane change.
          if self.lane_change:
            requested_curvature = requested_curvature * self.lane_change_factor
            self.precision_type = 0  # comfort for lane change

          # if high curvature situation, apply reduction factor
          if abs(requested_curvature) > 0.001 and not self.lane_change:
              requested_curvature = requested_curvature * self.high_curvature_factor

          # apply ford cuvature safety limits
          apply_curvature = apply_ford_curvature_limits(requested_curvature, self.apply_curvature_last, current_curvature, CS.out.vEgoRaw)

          # detect if steering was limited
          if (requested_curvature != apply_curvature) and (not steeringPressed):
            self.steering_limited = True
          else:
            self.steering_limited =False

          # if steering was limited turn on steer_warning if above 15mph
          if self.steering_limited and CS.out.vEgoRaw > 7:
              self.steer_warning = True

          # latch steer_warning and count cycles before clearing
          if self.steer_warning and not self.steering_limited:
              self.steer_warning_count = self.steer_warning_count + 1

          # clear steer_warning after 10 counts of no steering limited
          if self.steer_warning_count > 10:
            self.steer_warning = False
            self.steer_warning_count = 0

          self.fordVariables.steeringLimited01 = float(self.steer_warning)

          # compute curvature rate
          self.curvature_rate_deque.append(apply_curvature)
          if len(self.curvature_rate_deque) > 1:
            delta_t = (
              self.curvature_rate_delta_t if len(self.curvature_rate_deque) == self.curvature_rate_deque.maxlen else (len(self.curvature_rate_deque) - 1) * 0.05
            )
            desired_curvature_rate = (self.curvature_rate_deque[-1] - self.curvature_rate_deque[0]) / delta_t / max(0.01, CS.out.vEgoRaw)
          else:
            desired_curvature_rate = 0.0
          self.fordVariables.desiredCurvatureRate01 = float(desired_curvature_rate)

          #############################################################################################################################################################

          path_offset = 0
          path_angle = 0

          # clip all values
          apply_curvature = clip(apply_curvature, -self.curvature_max, self.curvature_max)
          desired_curvature_rate = clip(desired_curvature_rate, -self.curvature_rate_max, self.curvature_rate_max)
          path_offset = clip(path_offset, -self.path_offset_max, self.path_offset_max)
          path_angle = clip(path_angle, -self.path_angle_max, self.path_angle_max)

          self.fordVariables.applyCurvature01 = float(apply_curvature)
          self.fordVariables.desiredCurvatureRate02 = float(desired_curvature_rate)
          self.fordVariables.pathOffset02 = float(path_offset)
          self.fordVariables.pathAngle02 = float(path_angle)

          # human turn detection
          steeringPressed = CS.out.steeringPressed
          steeringAngleDeg = CS.out.steeringAngleDeg

           # if a human turn is active, reset steering to prevent windup
          if steeringPressed and abs(steeringAngleDeg) > 45:
            self.human_turn = True
          else:
            self.human_turn = False

          # Determine when to reset steering
          if (self.human_turn) and self.enable_human_turn_detection:
            reset_steering = 1
          else:
            reset_steering = 0

          # reset steering by setting all values to 0 and ramp_type to immediate
          if reset_steering == 1:
            apply_curvature = 0
            path_offset = 0
            path_angle = 0
            desired_curvature_rate = 0
            ramp_type = 3
            self.requested_curvature_filtered.x = 0.0
          else:
            ramp_type = 2
        else:
          desired_curvature_rate = 0.0
          path_offset = 0.0
          path_angle = 0.0
          self.requested_curvature_filtered.x = 0.0
          ramp_type = 0
      else:
        apply_curvature = 0.0
        desired_curvature_rate = 0.0
        path_offset = 0.0
        path_angle = 0.0
        self.requested_curvature_filtered.x = 0.0
        ramp_type = 0

      self.apply_curvature_last = apply_curvature
      self.curvature_rate_last = desired_curvature_rate
      self.path_offset_last = path_offset
      self.path_angle_last = path_angle

      if self.CP.flags & FordFlags.CANFD:
        mode = 1 if CC.latActive else 0
        if self.lat_mode == 0:
          mode = 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        if self.CP.spFlags & FordFlagsSP.SP_ENHANCED_LAT_CONTROL.value:
          can_sends.append(
            fordcan.create_lat_ctl2_msg(
              self.packer, self.CAN, mode, ramp_type, self.precision_type, -path_offset, -path_angle, -apply_curvature, -desired_curvature_rate, counter
            )
          )
        else:
          can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, ramp_type, self.precision_type, 0.0, 0.0, -apply_curvature, -desired_curvature_rate, counter))

      else:
        can_sends.append(
          fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, ramp_type, 1, -path_offset, -path_angle, -apply_curvature, -desired_curvature_rate)
        )

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN, CC.latActive, hud_control if self.send_lane_depart_can_msg else None))

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
      # if brake < 0 and brake_actuate:
      #   brake = interp(
      #     accel,
      #     [CarControllerParams.ACCEL_MIN, self.brake_converge_at, self.brake_clip],
      #     [CarControllerParams.ACCEL_MIN, self.brake_converge_at, self.brake_0_point],
      #   )

      self.fordVariables.brakeActive = True if brake_actuate == 1 else False
      self.fordVariables.preChargeActive = True if precharge_actuate == 1 else False

      # Calculate targetSpeed
      targetSpeed = clip(actuators.speed * self.target_speed_multiplier, 0, V_CRUISE_MAX)
      if not CC.longActive and hud_control.setSpeed:
        targetSpeed = hud_control.setSpeed

      can_sends.append(
        fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, brake, stopping, brake_actuate, precharge_actuate, v_ego_kph=targetSpeed)
      )

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True

    # Logic to keep sending the UI for 4 seconds
    if not self.send_ui_last and send_ui:
      # Save the frame # for the last flip from False to True
      self.send_ui_ts_last = self.frame

    # keep sending the UI for 4 seconds (400 at 100Hz)
    if ( self.send_ui_ts_last > 0 and (self.frame - self.send_ui_ts_last) <= (400)):
      send_ui = True

    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(
        fordcan.create_acc_ui_msg(
          self.packer,
          self.CAN,
          self.CP,
          main_on,
          CC.latActive,
          fcw_alert,
          CS.out.cruiseState.standstill,
          hud_control,
          CS.acc_tja_status_stock_values,
          self.send_hands_free_cluster_msg,
          send_ui,
          tja_warn,
          tja_msg
        )
      )

    self.main_on_last = main_on
    self.send_ui_last = send_ui
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.fordVariables = self.fordVariables

    self.frame += 1
    return new_actuators, can_sends
