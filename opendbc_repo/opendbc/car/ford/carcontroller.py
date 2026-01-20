import math
import cereal.messaging as messaging
import numpy as np
from numpy import clip, interp
from collections import deque
from common.filter_simple import FirstOrderFilter
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from selfdrive.modeld.constants import ModelConstants  # for calculations
from common.pid import PIDController # PID control of lateral
from opendbc.car.ford.helpers import compute_dm_msg_values
from openpilot.common.params import Params
#from opendbc.sunnypilot.car.ford.icbm import IntelligentCruiseButtonManagementInterface

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def index_function(idx, max_val=192, max_idx=32):
  return (max_val) * ((idx/max_idx)**2)

# ISO 11270
ISO_LATERAL_ACCEL = 3.0  # m/s^2  # TODO: import from test lateral limits file?

# Limit to average banked road since safety doesn't have the roll
EARTH_G = 9.81
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL - (EARTH_G * AVERAGE_ROAD_ROLL)  # ~2.4 m/s^2

# Model prediction variables
CONTROL_N = 17
IDX_N = 33
T_IDXS = [index_function(idx, max_val=10.0) for idx in range(IDX_N)]

def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5  # 5s smooths over the overshoot
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))

def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, steering_angle, lat_active, CP):
  max_curvature = 1 # large initial value
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)
    max_curvature = abs(current_curvature) + CarControllerParams.CURVATURE_ERROR

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, steering_angle, lat_active, CarControllerParams.ANGLE_LIMITS)

  steer_up = apply_curvature_last * apply_curvature >= 0. and abs(apply_curvature) > abs(apply_curvature_last)
  rate_limits = CarControllerParams.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else CarControllerParams.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN
  std_steer_angle_rate_limit = np.interp(v_ego_raw, rate_limits[0], rate_limits[1])
  std_steer_angle_limit = abs(apply_curvature_last) + abs(std_steer_angle_rate_limit)
  max_curvature = np.minimum(max_curvature, std_steer_angle_limit)

  # Ford Q4/CAN FD has more torque available compared to Q3/CAN so we limit it based on lateral acceleration.
  # Safety is not aware of the road roll so we subtract a conservative amount at all times
  if CP.flags & FordFlags.CANFD:
    # Limit curvature to conservative max lateral acceleration
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))
    max_curvature = np.minimum(max_curvature, abs(curvature_accel_limit))

  return apply_curvature, max_curvature


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


class CarController(CarControllerBase): #, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    #IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)

    self.params = Params()

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    # Initialize control variables
    self.apply_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.brake_request = False
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.fcw_alert_last = False  # previous status of collision alert
    self.send_ui_last = False  # previous state of ui elements
    self.send_bars_ts_last = 0  # previous state of ui elements
    self.send_bars_last = False  # previous state of ACC Gap elements
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.accel_pitch_compensated = 0.0
    self.steering_wheel_delta_adjusted = 0.0
    self.last_button_frame = 0  # Track last ICBM button press frame
    self.lateralUncertainty = 0.0

    ################################## lateral control parameters ##############################################

    # Toggles
    self.enable_human_turn_detection = True
    self.enable_lane_positioning = True

    # Variables to initialize (these get updated every scan as part of the control code)
    self.precision_type = 1  # precise or comfort
    self.human_turn = False  # have we detected a human override in a turn
    self.post_reset_ramp_active = False  # track if we're ramping after a steering reset
    self.reset_steering_last = False  # track previous reset_steering state
    self.enable_lane_positioning = False # Updated from UI: enable Advanced Lane Positioning
    self.enable_high_curvature_mode = False # Updated from UI: enable High Curvature Mode
    self.custom_profile = 0 # updated from UI
    self.pc_blend_ratio = 0.5
    self.steer_warning = False # warning for steering limits exceeded
    self.steer_warning_count = 0 # count how many cycles the warning has existed
    self.steering_limited = 0 # count how many cycles the steering was limited
    self.disable_BP_lat_UI = False # updated from UI: disable BP lateral control
    self.anti_overshoot_curvature_last = 0.0 # initialize anti_overshoot_curvature_last

    # Curvature variables
    self.curvature_lookup_time = 0.42 #from lagd
    self.lane_change_factor_bp = [4.4, 40.23] # what speed to adjust lane_change_factor
    self.lane_change_factor_low = 0.95 # lane_change_factor at 4.4 m/s
    self.lane_change_factor_high = 0.85 # updated from UI: lane_change_factor at 40.23 m/s
    self.pc_blend_ratio_low_C_CAN = 0.40 # %-Predicted Curvature
    self.pc_blend_ratio_high_C_CAN = 0.40 # %-Predicted Curvature
    self.pc_blend_ratio_low_C_CANFD = 0.40 # %-Predicted Curvature
    self.pc_blend_ratio_high_C_CANFD = 0.40 # %-Predicted Curvature
    self.pc_blend_ratio_bp = [0.0, 0.001] # curvature breakpoints in 1/m
    self.large_curve_factor_low = 1.0 # factor to reduce curvature for small curves
    self.large_curve_factor_high = 0.80 # factor to reduce curvature for large curves
    self.large_curve_factor_bp = [0.001, 0.02] # curvature breakpoints in 1/m
    self.large_curve_factor_v = [self.large_curve_factor_low, self.large_curve_factor_high] #  determine factor to reduce cu

    # Curvature rate variables
    self.curvature_rate_delta_t = 0.3  # [s] used in denominator for curvature rate calculation
    self.curvature_rate_deque = deque(maxlen=int(round(self.curvature_rate_delta_t / 0.05)))  # 0.3 seconds at 20Hz
    self.curvature_rate_speed_bp = [0.0, 14.5, 15.5]  # speed breakpoints in m/s
    self.curvature_rate_speed_v = [1.0, 1.0, 0.0]  # corresponding k_p values
    self.curvature_rate_PC_bp = [0.0, 0.008,0.01] # curvature breakpoints in 1/m
    self.curvature_rate_PC_v = [0.0, 0.0, 1.0] # corresponding k_p values

    # path offset variables
    self.custom_path_offset = 0.0 # updated from UI: applies a custom offset to help with in-lane positioning
    self.path_offset_lookup_time = 0.2 # in seconds (from bp-2.1)
    self.lane_width_tolerance_factor = 0.75
    self.min_laneline_confidence_bp = [0.6, 0.8]
    self.enable_lanefull_mode = True

    #path angle shared variables
    self.path_angle_filter_samples = 3 # number of samples to use for the moving average filter
    self.path_angle_deque = deque(maxlen=self.path_angle_filter_samples) # deque to hold the samples
    self.path_angle_wheel_angle_conversion = (np.pi/180) # degrees to radians

    # path angle low curvature variables
    self.LC_PID_GAIN_CAN = 5.0
    self.LC_PID_GAIN_CANFD_SMALL_VEHICLE = 3.0
    self.LC_PID_GAIN_CANFD_LARGE_VEHICLE = 3.0
    self.LC_PID_GAIN_UI = 0.0 # gain for UI tuning
    self.LC_PID_GAIN = 0.0
    self.LC_PID_k_p = 0.25
    self.LC_PID_k_i = 0.05
    self.LC_PID_controller = PIDController(k_p=self.LC_PID_k_p, k_i=self.LC_PID_k_i, rate=20)
    self.LC_PID_speed_bp = [0.0, 9.0, 15.0]  # speed breakpoints in m/s
    self.LC_PID_speed_v = [0.0, 0.0, 1.0]  # corresponding k_p values
    self.LC_path_angle_ROC_bp = [5, 15, 25]  # speed breakpoints in m/s
    self.LC_path_angle_ROC_v = [0.003, 0.0015, 0.002]  # match panda limits
    self.LC_path_angle_reset_counter = 0
    self.LC_path_angle_reset_duration = 1.5 # in seconds

    # path angle high curvature variables
    self.HC_PID_gain_UI = 0.5 # gain for UI tuning
    self.HC_PID_k_p = 1.0
    self.HC_PID_k_i = 0.05
    self.HC_PID_controller = PIDController(k_p=self.HC_PID_k_p, k_i=self.HC_PID_k_i, rate=20)
    self.wheel_angle_lookup_time = 0.05
    self.HC_PID_curvature_bp = [0.0, 0.008, 0.01, 0.02]  # curvature breakpoints in 1/m
    self.HC_PID_curvature_v = [0.0, 0.0, 1.0, 1.0]  # corresponding k_p values
    self.HC_PID_speed_bp = [0.0, 20.00, 22.00, 25.00]  # what speeds to adjust path_angle_speed_factor over.
    self.HC_PID_speed_v = [1.0, 1.0, 0.0, 0.0]
    self.pswa_blend_ratio = 1.0

    # max absolute values for all four signals
    self.path_angle_max = 0.5  # from dbc files
    self.path_offset_max = 2.0  # too much path offset causes issues
    self.curvature_max = 0.0115  # 0.02 is max from dbc files, but more than 0.012 can cause windup in big curves
    self.curvature_rate_max = 0.001023  # from dbc files

    # values from previous frame
    self.curvature_rate_last = 0.0
    self.path_offset_last = 0.0
    self.path_angle_last = 0.0
    self.curvature_rate = 0  # initialize curvature_rate

    # Logging variables
    #debug(f'Car Fingerprint (CarController): {CP.carFingerprint}', True)

    # Lane change transition tracking
    self.post_lane_change_timer = 0
    self.post_lane_change_active = False
    self.lane_change_last = False  # Track previous lane change state
    self.pre_lane_change_values = {
        'path_angle': 0.0,
        'path_offset': 0.0,
        'desired_curvature_rate': 0.0
    }

    # Maximum allowed changes per frame
    self.max_path_angle_change = 0.00125
    self.max_path_offset_change = 0.00125
    self.max_curvature_rate_change = 0.0001

    self.sm = messaging.SubMaster(['modelV2', 'liveParameters', 'selfdriveState'])
    self.VM = VehicleModel(self.CP)
    self.curvature_lookup_time = 0.2

    self.model = None
    self.lp = None
    self.ss = None
    self.send_driver_monitor_can_msg = False
    self.send_lane_depart_can_msg = False
    self.tja_msg = 0
    self.tja_warn = 0
    self.hands = 0
    self.predictedSteeringAngleDeg_SP = 0.0
    self._update_params()

  def _update_params(self):
    self.send_hands_free_cluster_msg = self.params.get_bool("send_hands_free_cluster_msg")
    self.enable_human_turn_detection = self.params.get_bool("enable_human_turn_detection")
    # updated from UI: lane_change_factor at 40.23 m/s
    self.lane_change_factor_high = float(self.params.get("lane_change_factor_high", return_default=True))
    self.pc_blend_ratio_high_C_UI = float(self.params.get("pc_blend_ratio_high_C_UI", return_default=True))
    self.pc_blend_ratio_low_C_UI = float(self.params.get("pc_blend_ratio_low_C_UI", return_default=True))
    self.enable_lane_positioning = self.params.get_bool("enable_lane_positioning")
    # updated from UI: applies a custom offset to help with in-lane positioning
    self.custom_path_offset = float(self.params.get("custom_path_offset", return_default=True))
    self.enable_lanefull_mode = self.params.get_bool("enable_lane_full_mode")
    self.custom_profile = int(self.params.get("custom_profile", return_default=True))
    self.LC_PID_gain_UI = float(self.params.get("LC_PID_gain_UI", return_default=True))

  def handle_post_lane_change_transition(self, path_angle, path_offset, desired_curvature_rate):
    """
    Manages smooth transition of control variables after lane change
    Returns: Tuple of (path_angle, path_offset, desired_curvature_rate)
    """
    # Detect lane change completion (transition from True to False)
    if self.lane_change_last and not self.lane_change:
        self.post_lane_change_active = True
        self.post_lane_change_timer = 0
        # Store current values as starting point
        self.pre_lane_change_values = {
            'path_angle': 0.0,  # Start from zero since we're coming out of lane change
            'path_offset': 0.0,
            'desired_curvature_rate': 0.0
        }

    # Update previous lane change state
    self.lane_change_last = self.lane_change

    # If we're in post-lane change state
    if self.post_lane_change_active:
        self.post_lane_change_timer += 1

        # Apply smooth transition using rate limiting
        new_path_angle = clip(
            path_angle,
            self.pre_lane_change_values['path_angle'] - self.max_path_angle_change,
            self.pre_lane_change_values['path_angle'] + self.max_path_angle_change
        )

        new_path_offset = clip(
            path_offset,
            self.pre_lane_change_values['path_offset'] - self.max_path_offset_change,
            self.pre_lane_change_values['path_offset'] + self.max_path_offset_change
        )

        new_curvature_rate = clip(
            desired_curvature_rate,
            self.pre_lane_change_values['desired_curvature_rate'] - self.max_curvature_rate_change,
            self.pre_lane_change_values['desired_curvature_rate'] + self.max_curvature_rate_change
        )

        # Update stored values
        self.pre_lane_change_values = {
            'path_angle': new_path_angle,
            'path_offset': new_path_offset,
            'desired_curvature_rate': new_curvature_rate
        }

        # Exit transition state after 40 frames
        if self.post_lane_change_timer >= 160:
            self.post_lane_change_active = False

        return (new_path_angle, new_path_offset, new_curvature_rate)

    return (path_angle, path_offset, desired_curvature_rate)

  def calculate_lateral_uncertainty(self, requested_curvature, apply_curvature, max_curvature):
    max_curvature = np.clip(max_curvature, apply_curvature, self.curvature_max)  # ensure max_curvature is within reasonable bounds
    return float(requested_curvature / max_curvature)

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    self.sm.update(0)

    if self.sm.updated['modelV2']:
      self.model = self.sm["modelV2"]

    if self.sm.updated['liveParameters']:
      self.lp = self.sm["liveParameters"]

    if self.sm.updated['selfdriveState']:
      self.ss = self.sm['selfdriveState']

    if self.lp is not None:
      x = max(self.lp.stiffnessFactor, 0.1)
      sr = max(self.lp.steerRatio, 0.1)
      self.VM.update_params(x, sr)

    self._update_params()

    actuators = CC.actuators
    hud_control = CC.hudControl
    main_on = CS.out.cruiseState.available
    # if self.fordVariables is None:
      # act = actuators.as_builder()
      # self.fordVariables = act.fordVariables

    # Calculate steer_alert and fcw_alert
    steer_alert = False
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    # Compute the DM message values
    if self.send_driver_monitor_can_msg:
      # print(f'HudControl: {hud_control}')
      # print(f'tja_msg: {self.tja_msg} | tja_warn: {self.tja_warn}')
      if (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
        self.tja_msg, self.tja_warn, self.hands = compute_dm_msg_values(self.ss, hud_control, self.send_hands_free_cluster_msg, main_on, CS.out.cruiseState.standstill)
    else:
      steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
      if steer_alert:
        self.hands = 1
      else:
        self.hands = 0

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

    # Intelligent Cruise Button Management (ICBM)
    #icbm_can_sends, self.last_button_frame = IntelligentCruiseButtonManagementInterface.update(
    #  self, CC_SP, CS, self.packer, self.CAN, self.frame, self.last_button_frame
    #)
    #can_sends.extend(icbm_can_sends)

    ### lateral control ###

    apply_curvature = 0.0 # initialize apply_curvature
    desired_curvature_rate = 0.0 # initialize desired_curvature_rate
    path_offset = 0.0 # initialize path_offset
    path_angle = 0.0 # initialize path_angle
    reset_steering = 0 # initialize reset_steering
    ramp_type = 2 # initialize ramp_type

    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        self.precision_type = 1
        steeringPressed = CS.out.steeringPressed
        steeringAngleDeg_PV = CS.out.steeringAngleDeg

        # determine tuning profile
        if self.custom_profile == 1: # custom tuning profile
          self.pc_blend_ratio_low_C =  self.pc_blend_ratio_low_C_UI
          self.pc_blend_ratio_high_C =  self.pc_blend_ratio_high_C_UI
          self.LC_PID_GAIN = self.LC_PID_gain_UI

        elif self.CP.flags & FordFlags.CANFD:
          self.pc_blend_ratio_low_C = self.pc_blend_ratio_low_C_CANFD
          self.pc_blend_ratio_high_C = self.pc_blend_ratio_high_C_CANFD
          if (self.CP.carFingerprint == CAR.FORD_ESCAPE_MK4_5 or self.CP.carFingerprint == CAR.FORD_MUSTANG_MACH_E_MK1):
            self.LC_PID_gain = self.LC_PID_GAIN_CANFD_SMALL_VEHICLE
          else:
            self.LC_PID_gain = self.LC_PID_GAIN_CANFD_LARGE_VEHICLE
        else:
          self.pc_blend_ratio_low_C = self.pc_blend_ratio_low_C_CAN
          self.pc_blend_ratio_high_C = self.pc_blend_ratio_high_C_CAN
          self.LC_PID_gain = self.LC_PID_GAIN_CAN

        self.pc_blend_ratio_v = [self.pc_blend_ratio_low_C, self.pc_blend_ratio_high_C] # %-Predicted Curvature

        # calculate current curvature and model desired curvature
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)  # use canbus data to calculate current_curvature
        desired_curvature = actuators.curvature  # get desired curvature from model

        # extract predicted curvature from modelV2
        if self.model is not None and len(self.model.orientation.x) >= 17:
          # compute curvature from model predicted orientationRate, and blend with desired curvature based on max predicted curvature magnitude
          curvatures = np.array(self.model.orientationRate.z) / max(0.01, CS.out.vEgoRaw)
          predicted_steering_angle_curvature = interp(self.wheel_angle_lookup_time, ModelConstants.T_IDXS, curvatures)
          predicted_curvature = interp(self.curvature_lookup_time, ModelConstants.T_IDXS, curvatures)
        else:
          predicted_curvature = 0.0

        # calculate predicted steering angle
        self.predictedSteeringAngleDeg_SP = math.degrees(self.VM.get_steer_from_curvature(-predicted_steering_angle_curvature, CS.out.vEgoRaw, 0))
        self.predictedSteeringAngleDeg_SP += self.lp.angleOffsetDeg

        # calculate blend ratio
        self.pc_blend_ratio = interp(abs(desired_curvature), self.pc_blend_ratio_bp, self.pc_blend_ratio_v)

        # equate requested_curvature to a blend of desired and predicted_curvature and apply curvature limits
        requested_curvature = (predicted_curvature * self.pc_blend_ratio) + (desired_curvature * (1 - self.pc_blend_ratio))

        # determine if a lane change is active
        if (self.model.meta.laneChangeState == 1 or self.model.meta.laneChangeState == 2 or self.model.meta.laneChangeState == 3):
            self.lane_change = True
        else:
            self.lane_change = False

        # determine lane_change_factor based on speed
        lane_change_factor = interp(CS.out.vEgoRaw, self.lane_change_factor_bp, [self.lane_change_factor_low, self.lane_change_factor_high])

        # if changing lanes, modify curvature to smooth out the lane change
        if self.lane_change and (self.model.meta.laneChangeDirection == 1): # if we are changing lanes to the left
          if requested_curvature < 0: # and the curvature is taking us to the left
              requested_curvature = requested_curvature * lane_change_factor # reduce the curvature to smooth out the lane change
          else:
              requested_curvature = requested_curvature # if we are moving back right to correct for over travel, do not reduce curvature

          self.precision_type = 0 # use comfort mode

        if self.lane_change and (self.model.meta.laneChangeDirection == 2): # if we are changing lanes to the right
          if requested_curvature > 0: # and the curvature is taking us to the right
              requested_curvature = requested_curvature * lane_change_factor # reduce the curvature to smooth out the lane change
          else:
              requested_curvature = requested_curvature # if we are moving back left to correct for over travel, do not reduce curvature

          self.precision_type = 0 # use comfort mode

        # Determine if a human is making a turn and trap the value
        # if a human turn is active, reset steering to prevent windup
        if steeringPressed and abs(steeringAngleDeg_PV) > 45:
          self.human_turn = True
        else:
          self.human_turn = False

        # Determine when to reset steering
        if ((self.human_turn) and self.enable_human_turn_detection) or (CS.out.vEgoRaw < 0.1):
          reset_steering = 1
        else:
          reset_steering = 0

        #if reset_steering is 1, set requested_curvature to 0
        if reset_steering == 1:
          requested_curvature = 0.0

        # apply curvature limits
        apply_curvature, max_curvature = apply_ford_curvature_limits(requested_curvature,
                                                                self.apply_curvature_last,
                                                                current_curvature,
                                                                CS.out.vEgoRaw,
                                                                0,
                                                                CC.latActive,
                                                                self.CP)
        lateralUncertainty = self.calculate_lateral_uncertainty(requested_curvature, apply_curvature, max_curvature)

        #if reset_steering is 1, set apply_curvature to 0
        if reset_steering == 1:
          apply_curvature = 0.0
          self.post_reset_ramp_active = False  # Cancel any active ramp when resetting
        else:
          # Detect transition from reset to normal (reset_steering goes from 1 to 0)
          if self.reset_steering_last and not reset_steering:
            # Just came out of reset, start post-reset ramp
            self.post_reset_ramp_active = True
            self.apply_curvature_last = 0.0  # Reset to ensure clean ramp from 0

        # Post-reset ramp logic: gradually ramp from 0 to requested curvature
        # Keep path_angle = 0 during ramp to maintain bypass in ford.h
        if self.post_reset_ramp_active:
          # Use rate limits to gradually ramp up from 0 towards requested_curvature
          # This prevents blocked messages when transitioning out of reset
          apply_curvature = apply_std_steer_angle_limits(requested_curvature, self.apply_curvature_last,
                                                         CS.out.vEgoRaw, 0, CC.latActive, CarControllerParams.ANGLE_LIMITS)

          # Check if we've ramped close enough to requested curvature (within 10% or 0.001, whichever is larger)
          curvature_error = abs(requested_curvature - apply_curvature)
          curvature_threshold = max(abs(requested_curvature) * 0.1, 0.001)

          if curvature_error < curvature_threshold:
            # Ramp complete, exit post-reset mode
            self.post_reset_ramp_active = False

        # Update reset_steering_last for next frame
        self.reset_steering_last = (reset_steering == 1)

        # detect if steering was limited (lanes changes always trigger, but complete just fine)
        if (requested_curvature != apply_curvature) and (not steeringPressed) and (not self.lane_change):
          self.steering_limited = self.steering_limited + 1
        else:
          self.steering_limited = 0

        # if steering was limited for 10 scans turn on steer_warning if above 15mph
        if self.steering_limited > 10 and CS.out.vEgoRaw > 7:
            self.steer_warning = True

        # latch steer_warning and count cycles before clearing
        if self.steer_warning and not self.steering_limited:
            self.steer_warning_count = self.steer_warning_count + 1

        # clear steer_warning after 10 counts of no steering limited
        if self.steer_warning_count > 10:
          self.steer_warning = False
          self.steer_warning_count = 0

        # compute curvature rate
        self.curvature_rate_deque.append(predicted_curvature)
        if len(self.curvature_rate_deque) > 1:
          delta_t = (
            self.curvature_rate_delta_t if len(self.curvature_rate_deque) == self.curvature_rate_deque.maxlen else (len(self.curvature_rate_deque) - 1) * 0.05
          )
          desired_curvature_rate = (self.curvature_rate_deque[-1] - self.curvature_rate_deque[0]) / delta_t / max(0.01, CS.out.vEgoRaw)
        else:
          desired_curvature_rate = 0.0

        # calculate curvature rate PC factor
        curvature_rate_PC_factor = interp(abs(predicted_curvature), self.curvature_rate_PC_bp, self.curvature_rate_PC_v)
        desired_curvature_rate = desired_curvature_rate * curvature_rate_PC_factor

        # calcualte curvature rate speed factor
        curvature_rate_speed_factor = interp(CS.out.vEgoRaw, self.curvature_rate_speed_bp, self.curvature_rate_speed_v)
        desired_curvature_rate = desired_curvature_rate * curvature_rate_speed_factor

        # determine large curve factor
        large_curve_factor = interp(abs(requested_curvature), self.large_curve_factor_bp, self.large_curve_factor_v)

        # apply large curve factor to desired_curvature_rate
        desired_curvature_rate = desired_curvature_rate * large_curve_factor

        #no large curve factor in lane changes
        if self.lane_change:
          large_curve_factor = 1.0

        # if we are in a lane change, set the desired_curvature_rate to 0
        if self.lane_change:
          desired_curvature_rate = 0.0

        # get path offset from model.position.y
        path_offset_position = interp(self.path_offset_lookup_time, ModelConstants.T_IDXS, self.model.position.y)

        # now get path offset from lanelines
        path_offset_lanelines = (self.model.laneLines[1].y[0] + self.model.laneLines[2].y[0]) / 2

        # determinie laneline width tolerance scaling factor
        laneline_width = self.model.laneLines[2].y[0] + (-self.model.laneLines[1].y[0]) # laneLines[1] is a negative value because it is left of the vehicle.
        laneline_width_tolerance = interp(laneline_width, [3.75,4.25], [0.81, 0.59]) # 3.7 is the width of standard US lane in meters

        # determine laneline confidence
        laneline_confidence = min(self.model.laneLineProbs[1], self.model.laneLineProbs[2], laneline_width_tolerance)
        if not self.enable_lanefull_mode:
          laneline_confidence = 0.0

        # determine laneline path offset scale
        laneline_path_offset_scale = interp(laneline_confidence, self.min_laneline_confidence_bp, [0.0, 1.0])

        # get the total path_offset combining model and lanelines
        path_offset = (path_offset_position * (1-laneline_path_offset_scale) + (path_offset_lanelines * laneline_path_offset_scale)) + self.custom_path_offset

        # no path_offset during lane changes (it will fight you until it swaps to new lane if you don't set to zero)
        if self.lane_change:
          path_offset = 0

        # Use the UI variable for adjustable Gain and set the PID gain to a fixed number, UI variable divided by 100 to make UI variable more closely match the 2.1 logic tuning.
        path_offset_error = (path_offset * (self.LC_PID_gain_UI/100))

        # determine speed factor
        LC_PID_speed_factor = interp(CS.out.vEgoRaw, self.LC_PID_speed_bp, self.LC_PID_speed_v)

        # apply speed factor to path_offset_error
        path_offset_error_adj = path_offset_error * LC_PID_speed_factor

        # if not using lane positioning, zero out path_offset_error_adj
        if not self.enable_lane_positioning:
          path_offset_error_adj = 0.0

        # Use path_angle to help with centering vehicle in lane, Use PID controller to calculate path_angle
        path_angle_low_c = self.LC_PID_controller.update(path_offset_error_adj)

        # if not using lane positioning, zero out path_angle_low_c (should be zeroed out in the PID controller, but just in case)
        if not self.enable_lane_positioning:
          path_angle_low_c = 0.0

        # reset path angle if steering reset is active
        # During post-reset ramp, path_angle can ramp normally (latch in ford.h handles bypass)
        if reset_steering == 1:
          path_angle_low_c = 0.0

        # rate limit path_angle_low_c for comfort
        path_angle_roc = interp(abs(CS.out.vEgoRaw), self.LC_path_angle_ROC_bp, self.LC_path_angle_ROC_v)
        path_angle_low_c = clip(path_angle_low_c, self.path_angle_last - path_angle_roc, self.path_angle_last + path_angle_roc)

        # if the driver is applying consistent pressure to the steering wheel, reset the path_angle_low_c PID controller
        if steeringPressed:
          self.LC_path_angle_reset_counter = self.LC_path_angle_reset_counter + 1
        else:
          self.LC_path_angle_reset_counter = 0
        if self.LC_path_angle_reset_counter > self.LC_path_angle_reset_duration * 20: #20 scans per second
          self.LC_PID_controller.reset()

        # path_angle_high_c is not used in the current implementation
        path_angle_high_c = 0.0

        # sum path_angle_low_c and path_angle_high_c
        path_angle = path_angle_low_c + path_angle_high_c

        # Apply post lane change transition logic
        path_angle, path_offset, desired_curvature_rate = self.handle_post_lane_change_transition(
            path_angle, path_offset, desired_curvature_rate
        )

        # reset path angle if steering reset is active
        # During post-reset ramp, path_angle can ramp normally (latch in ford.h handles bypass)
        if reset_steering == 1:
          path_angle = 0.0

        # clip all values to max.
        apply_curvature = clip(apply_curvature, -self.curvature_max, self.curvature_max)
        desired_curvature_rate = clip(desired_curvature_rate, -self.curvature_rate_max, self.curvature_rate_max)
        path_offset = clip(path_offset, -self.path_offset_max, self.path_offset_max)
        path_angle = clip(path_angle, -self.path_angle_max, self.path_angle_max)


        # if path_offset and path_angle disagree, it can result in a very uncomortable ride, since path_angle is so strong, zero out path_offset signal before it is sent over canbus
        path_offset = 0.0

        if self.disable_BP_lat_UI:
          reset_steering = 0
          path_offset = 0
          path_angle = 0
          desired_curvature_rate = 0
          ramp_type = 1

          self.anti_overshoot_curvature_last = anti_overshoot(desired_curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
          apply_curvature = self.anti_overshoot_curvature_last

          current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

          self.apply_curvature_last, max_curvature = apply_ford_curvature_limits(apply_curvature, self.apply_curvature_last, current_curvature,
                                                              CS.out.vEgoRaw, 0., CC.latActive, self.CP)

          #rem bluepilot sends apply_curvature, and at some point openpilot swapped to sending apply_curvature_last.
          apply_curvature = self.apply_curvature_last

          lateralUncertainty = self.calculate_lateral_uncertainty(requested_curvature, apply_curvature, max_curvature)

        # reset steering by setting all values to 0 and ramp_type to immediate
        if reset_steering == 1:
          ramp_type = 3
          self.path_angle_deque.clear()
          self.HC_PID_controller.reset()
          self.LC_PID_controller.reset()
        else:
          ramp_type = 2
      else:
        apply_curvature = 0.0
        desired_curvature_rate = 0.0
        path_offset = 0.0
        path_angle = 0.0
        self.path_angle_deque.clear()
        self.HC_PID_controller.reset()
        self.LC_PID_controller.reset()
        ramp_type = 0
        lateralUncertainty = 0.0

      self.lateralUncertainty = lateralUncertainty
      self.apply_curvature_last = apply_curvature
      self.curvature_rate_last = desired_curvature_rate
      self.path_offset_last = path_offset
      self.path_angle_last = path_angle


      # set lat_active to the value of CC.latActive
      lat_active = CC.latActive

      if self.CP.flags & FordFlags.CANFD:
        # TODO: extended mode
        # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02m/s^2)
        # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
        # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
        # A detailed explanation on ford control can be found here:
        # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
        mode = 1 if lat_active else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(
          self.packer, self.CAN, mode, ramp_type, self.precision_type, -path_offset, -path_angle,
          -apply_curvature, -desired_curvature_rate, counter
        ))
      else:
        # Ford non-CANFD lateral control
        can_sends.append(fordcan.create_lat_ctl_msg(
          self.packer, self.CAN, lat_active, ramp_type, self.precision_type,
          -path_offset, -path_angle, -apply_curvature, -desired_curvature_rate
        ))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      lka_hud_control = None
      if self.send_lane_depart_can_msg:
        lka_hud_control = hud_control
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN, CC.latActive, lka_hud_control))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = actuators.accel
      gas = accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction is very slow
        # TODO: verify this applies to EV/hybrid
        accel = apply_creep_compensation(accel, CS.out.vEgo)

        # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
        # however even 3.5 m/s^3 causes some overshoot with a step response.
        accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Both gas and accel are in m/s^2, accel is used solely for braking
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      accel_pitch_compensated = accel + accel_due_to_pitch
      if accel_pitch_compensated > 0.3 or not CC.longActive:
        self.brake_request = False
      elif accel_pitch_compensated < 0.0:
        self.brake_request = True

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      # TODO: look into using the actuators packet to send the desired speed
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, self.brake_request, v_ego_kph=V_CRUISE_MAX))

      self.accel = accel
      self.gas = gas
      self.accel_pitch_compensated = accel_pitch_compensated

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, self.hands, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    send_bars = False
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      send_bars = True

    # Logic to keep sending the bars for 4 seconds
    if not self.send_bars_last and send_bars:
      # Save the frame # for the last flip from False to True
      self.send_bars_ts_last = self.frame
      self.distance_bar_frame = self.frame

    # keep sending the bars for 4 seconds (400 at 100Hz)
    if (self.send_bars_ts_last > 0 and (self.frame - self.send_bars_ts_last) <= 400):
      send_ui = True
      send_bars = True

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
          send_bars,
          self.tja_warn,
          self.tja_msg,
        )
      )

    self.main_on_last = main_on
    self.send_ui_last = send_ui
    self.send_bars_last = send_bars
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.fcw_alert_last = fcw_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    new_actuators.torqueOutputCan = float(self.steer_warning)
    new_actuators.curvature = float(apply_curvature)
    new_actuators.accel = float(self.accel)
    new_actuators.gas = float(self.gas)
    new_actuators.steeringAngleDeg = float(self.predictedSteeringAngleDeg_SP)
    self.frame += 1
    return new_actuators, can_sends