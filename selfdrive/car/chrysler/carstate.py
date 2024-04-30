from cereal import car
from openpilot.common.conversions import Conversions as CV
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from openpilot.selfdrive.car.interfaces import CarStateBase
from openpilot.selfdrive.car.chrysler.values import DBC, STEER_THRESHOLD, RAM_CARS, BUTTON_STATES


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])

    self.auto_high_beam = 0
    self.button_counter = 0
    self.lkas_car_model = -1

    if CP.carFingerprint in RAM_CARS:
      self.shifter_values = can_define.dv["Transmission_Status"]["Gear_State"]
    else:
      self.shifter_values = can_define.dv["GEAR"]["PRNDL"]

    self.lkas_enabled = False
    self.prev_lkas_enabled = False
    self.lkas_heartbit = None
    self.lkas_disabled = False

    self.buttonStates = BUTTON_STATES.copy()
    self.buttonStatesPrev = BUTTON_STATES.copy()

  def update(self, cp, cp_cam):

    ret = car.CarState.new_message()

    self.prev_mads_enabled = self.mads_enabled
    self.prev_lkas_enabled = self.lkas_enabled
    self.buttonStatesPrev = self.buttonStates.copy()

    # lock info
    ret.doorOpen = any([cp.vl["BCM_1"]["DOOR_OPEN_FL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_FR"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RL"],
                        cp.vl["BCM_1"]["DOOR_OPEN_RR"]])
    ret.seatbeltUnlatched = cp.vl["ORC_1"]["SEATBELT_DRIVER_UNLATCHED"] == 1

    # brake pedal
    ret.brake = 0
    ret.brakePressed = cp.vl["ESP_1"]['Brake_Pedal_State'] == 1  # Physical brake pedal switch
    ret.brakeLightsDEPRECATED = bool(cp.vl["ESP_1"]["BRAKE_PRESSED_ACC"])

    # gas pedal
    ret.gas = cp.vl["ECM_5"]["Accelerator_Position"]
    ret.gasPressed = ret.gas > 1e-5

    # car speed
    if self.CP.carFingerprint in RAM_CARS:
      ret.vEgoRaw = cp.vl["ESP_8"]["Vehicle_Speed"] * CV.KPH_TO_MS
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["Transmission_Status"]["Gear_State"], None))
    else:
      ret.vEgoRaw = (cp.vl["SPEED_1"]["SPEED_LEFT"] + cp.vl["SPEED_1"]["SPEED_RIGHT"]) / 2.
      ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(cp.vl["GEAR"]["PRNDL"], None))
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = not ret.vEgoRaw > 0.001
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["ESP_6"]["WHEEL_SPEED_FL"],
      cp.vl["ESP_6"]["WHEEL_SPEED_FR"],
      cp.vl["ESP_6"]["WHEEL_SPEED_RL"],
      cp.vl["ESP_6"]["WHEEL_SPEED_RR"],
      unit=1,
    )

    # button presses
    ret.leftBlinker, ret.rightBlinker = ret.leftBlinkerOn, ret.rightBlinkerOn = self.update_blinker_from_stalk(200, cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 1,
                                                                                                                    cp.vl["STEERING_LEVERS"]["TURN_SIGNALS"] == 2)
    ret.genericToggle = cp.vl["STEERING_LEVERS"]["HIGH_BEAM_PRESSED"] == 1

    # steering wheel
    ret.steeringAngleDeg = cp.vl["STEERING"]["STEERING_ANGLE"] + cp.vl["STEERING"]["STEERING_ANGLE_HP"]
    ret.steeringRateDeg = cp.vl["STEERING"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["EPS_2"]["COLUMN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["EPS_2"]["EPS_TORQUE_MOTOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > STEER_THRESHOLD

    # cruise state
    cp_cruise = cp_cam if self.CP.carFingerprint in RAM_CARS else cp

    ret.cruiseState.available = cp_cruise.vl["DAS_3"]["ACC_AVAILABLE"] == 1
    ret.cruiseState.enabled = cp_cruise.vl["DAS_3"]["ACC_ACTIVE"] == 1
    ret.cruiseState.speed = cp_cruise.vl["DAS_4"]["ACC_SET_SPEED_KPH"] * CV.KPH_TO_MS
    ret.cruiseState.nonAdaptive = cp_cruise.vl["DAS_4"]["ACC_STATE"] in (1, 2)  # 1 NormalCCOn and 2 NormalCCSet
    ret.cruiseState.standstill = cp_cruise.vl["DAS_3"]["ACC_STANDSTILL"] == 1
    ret.accFaulted = cp_cruise.vl["DAS_3"]["ACC_FAULTED"] != 0

    if self.CP.carFingerprint in RAM_CARS:
      # Auto High Beam isn't Located in this message on chrysler or jeep currently located in 729 message
      self.auto_high_beam = cp_cam.vl["DAS_6"]['AUTO_HIGH_BEAM_ON']
      self.lkas_enabled = cp.vl["Center_Stack_2"]["LKAS_Button"] == 1 or cp.vl["Center_Stack_1"]["LKAS_Button"] == 1
      ret.steerFaultTemporary = cp.vl["EPS_3"]["DASM_FAULT"] == 1
    else:
      self.lkas_enabled = cp.vl["TRACTION_BUTTON"]["TOGGLE_LKAS"] == 1
      self.lkas_heartbit = cp_cam.vl["LKAS_HEARTBIT"]
      if not self.control_initialized:
        self.lkas_disabled = bool(cp_cam.vl["LKAS_HEARTBIT"]["LKAS_DISABLED"])
      ret.steerFaultTemporary = cp.vl["EPS_2"]["LKAS_TEMPORARY_FAULT"] == 1
      ret.steerFaultPermanent = cp.vl["EPS_2"]["LKAS_STATE"] == 4

    # blindspot sensors
    if self.CP.enableBsm:
      ret.leftBlindspot = cp.vl["BSM_1"]["LEFT_STATUS"] == 1
      ret.rightBlindspot = cp.vl["BSM_1"]["RIGHT_STATUS"] == 1

    self.buttonStates["accelCruise"] = bool(cp.vl["CRUISE_BUTTONS"]["ACC_Accel"])
    self.buttonStates["decelCruise"] = bool(cp.vl["CRUISE_BUTTONS"]["ACC_Decel"])
    self.buttonStates["cancel"] = bool(cp.vl["CRUISE_BUTTONS"]["ACC_Cancel"])
    self.buttonStates["resumeCruise"] = bool(cp.vl["CRUISE_BUTTONS"]["ACC_Resume"])

    self.lkas_car_model = cp_cam.vl["DAS_6"]["CAR_MODEL"]
    self.button_counter = cp.vl["CRUISE_BUTTONS"]["COUNTER"]
    self.cruise_buttons = cp.vl["CRUISE_BUTTONS"]

    return ret

  @staticmethod
  def get_cruise_messages():
    messages = [
      ("DAS_3", 50),
      ("DAS_4", 50),
    ]
    return messages

  @staticmethod
  def get_can_parser(CP):
    messages = [
      # sig_address, frequency
      ("ESP_1", 50),
      ("EPS_2", 100),
      ("ESP_6", 50),
      ("STEERING", 100),
      ("ECM_5", 50),
      ("CRUISE_BUTTONS", 50),
      ("STEERING_LEVERS", 10),
      ("ORC_1", 2),
      ("BCM_1", 1),
    ]

    if CP.enableBsm:
      messages.append(("BSM_1", 2))

    if CP.carFingerprint in RAM_CARS:
      messages += [
        ("ESP_8", 50),
        ("EPS_3", 50),
        ("Transmission_Status", 50),
        ("Center_Stack_1", 1),
        ("Center_Stack_2", 1),
      ]
    else:
      messages += [
        ("GEAR", 50),
        ("SPEED_1", 100),
        ("TRACTION_BUTTON", 1),
      ]
      messages += CarState.get_cruise_messages()

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
      ("DAS_6", 4),
    ]

    if CP.carFingerprint in RAM_CARS:
      messages += CarState.get_cruise_messages()
    else:
      messages.append(("LKAS_HEARTBIT", 10))

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)
