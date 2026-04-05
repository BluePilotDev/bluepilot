import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from openpilot.common.params import Params

# BluePilot: extension imports for lateral, longitudinal, and HUD control
from opendbc.sunnypilot.car.ford.lateral_ext import LateralExt
from opendbc.sunnypilot.car.ford.longitudinal_ext import LongitudinalExt
from opendbc.sunnypilot.car.ford.hud_ext import HudExt
from opendbc.sunnypilot.car.ford import fordcan_ext
from opendbc.sunnypilot.car.ford.icbm import IntelligentCruiseButtonManagementInterface

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# CAN FD limits:
# Limit to average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll raises lateral acceleration
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL - (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~2.4 m/s^2


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
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, steering_angle, lat_active, CarControllerParams.ANGLE_LIMITS)

  # Ford Q4/CAN FD has more torque available compared to Q3/CAN so we limit it based on lateral acceleration.
  # Safety is not aware of the road roll so we subtract a conservative amount at all times
  if CP.flags & FordFlags.CANFD:
    # Limit curvature to conservative max lateral acceleration
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))

  return apply_curvature


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


# BluePilot: CarController inherits from LateralExt, LongitudinalExt, HudExt, and ICBM
# for full 4-signal lateral control, follow-aware longitudinal, and enhanced HUD messaging.
# Init order: CarControllerBase first (sets self.CP, self.frame), then ext classes.
class CarController(CarControllerBase, LateralExt, LongitudinalExt, HudExt,
                    IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    # BluePilot: initialize extension classes
    LateralExt.__init__(self, CP, CP_SP)
    LongitudinalExt.__init__(self, CP, CP_SP)
    HudExt.__init__(self, CP, CP_SP)
    # ICBM: base class sets state used at runtime, init for robustness
    # IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)

    self.params = Params()
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.apply_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.last_button_frame = 0  # BluePilot: ICBM button press tracking
    # Note: main_on_last, lkas_enabled_last, steer_alert_last, lead_distance_bars_last,
    # distance_bar_frame are initialized by HudExt.__init__() above

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    # BluePilot: update SubMaster (modelV2, liveParameters, selfdriveState, radarState) and vehicle model
    LateralExt.update_sm(self)

    # BluePilot: read runtime params from UI
    LateralExt.update_lateral_params(self, self.params)
    LongitudinalExt.update_long_params(self, self.params)
    HudExt.update_hud_params(self, self.params, self.CP)

    actuators = CC.actuators
    hud_control = CC.hudControl
    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    # BluePilot: compute DM state (TJA message, warning, hands level)
    HudExt.update_dm(self, hud_control, main_on, CS.out.cruiseState.standstill, self.frame)

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

    # BluePilot: Intelligent Cruise Button Management (ICBM)
    icbm_can_sends, self.last_button_frame = IntelligentCruiseButtonManagementInterface.update(
      self, CC_SP, CS, self.packer, self.CAN, self.frame, self.last_button_frame
    )
    can_sends.extend(icbm_can_sends)

    ### lateral control ###
    # BluePilot: full 4-signal lateral control via LateralExt
    # Computes curvature, curvature_rate, path_offset, path_angle using predicted curvature,
    # PID lane centering, laneline-aware offset, and lane change smoothing.
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      lat = LateralExt.update(self, CC, CS, actuators, self.apply_curvature_last, self.CP)
      self.apply_curvature_last = lat.apply_curvature
      self.lateralUncertainty = lat.lateralUncertainty

      lat_active = CC.latActive
      if self.CP.flags & FordFlags.CANFD:
        mode = 1 if lat_active else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan_ext.create_lat_ctl2_msg(
          self.packer, self.CAN, mode, lat.ramp_type, lat.precision_type,
          -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate, counter
        ))
      else:
        can_sends.append(fordcan_ext.create_lat_ctl_msg(
          self.packer, self.CAN, lat_active, lat.ramp_type, lat.precision_type,
          -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate
        ))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      # Stock creep compensation and rate limiting (upstream-identical)
      op_accel = actuators.accel
      op_gas = op_accel

      if CC.longActive:
        op_accel = apply_creep_compensation(op_accel, CS.out.vEgo)
        op_accel = max(op_accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      op_accel = float(np.clip(op_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      op_gas = float(np.clip(op_gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      if not CC.longActive or op_gas < CarControllerParams.MIN_GAS:
        op_gas = CarControllerParams.INACTIVE_GAS

      # Pitch compensation (upstream-identical)
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      # BluePilot: downhill compensation disable
      if self.disable_downhill_comp_UI:
        if accel_due_to_pitch < 0:
          accel_due_to_pitch = 0

      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      target_speed = V_CRUISE_MAX
      v_ego_mph = CS.out.vEgo * 2.23694

      # BluePilot: longitudinal follow control via LongitudinalExt
      # Classifies lead vehicle state (gaining/pacing/trailing) and applies gas/accel limits,
      # rate-limited braking, and split brake/precharge hysteresis.
      lng = LongitudinalExt.update(self, CC, CS, op_accel, op_gas, accel_due_to_pitch,
                                    v_ego_mph, stopping, target_speed)

      can_sends.append(fordcan_ext.create_acc_msg(
        self.packer, self.CAN, CC.longActive, lng.gas, lng.accel, lng.accel_pred_send,
        lng.stopping, lng.brake_actuate, lng.precharge_actuate, v_ego_kph=lng.target_speed
      ))

      self.accel = lng.accel
      self.gas = lng.gas

    ### ui ###
    # BluePilot: HUD message generation via HudExt
    # Handles LKAS UI (1Hz), ACC UI (5Hz), bar persistence, and TJA/hands-free messaging.
    hud_can_sends = HudExt.update_hud(self, CC, CS, hud_control, main_on, fcw_alert,
                                       self.frame, self.packer, self.CAN, self.CP)
    can_sends.extend(hud_can_sends)

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.accel = float(self.accel)
    new_actuators.gas = float(self.gas)

    self.frame += 1
    return new_actuators, can_sends
