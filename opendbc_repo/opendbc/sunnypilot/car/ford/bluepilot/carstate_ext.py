"""
BluePilot (full BluePilot only): Ford car state extras, from BluePilot 7.

- Cruise button events for the steering-wheel combo buttons (Set/+, Set/-, Cancel/Resume) and main
  cruise, so cruise-button management and speed limit assist can track the set speed.
- The posted speed limit from the camera's traffic sign recognition (CarStateSP.speedLimit).
- Hybrid/EV drive and battery values and the brake light state, as `car_state_bp` (the fields of
  cereal's CarStateBP); the host publishes them as carStateBP.
- CAN FD: the set speed's unit and the gear from the shifter, as BP7 reads them; every Ford: BP7's dash speed
  (vEgoCluster) floor and hysteresis.

Called from opendbc/car/ford/carstate.py; the hybrid flags are set from the fingerprint in
interfaces_ext.py.
"""
from collections import namedtuple

from opendbc.can import CANDefine
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.values import CAR, DBC, FordFlags

ButtonType = structs.CarState.ButtonEvent.Type
TransmissionType = structs.CarParams.TransmissionType
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

# CarParams.flags bits, BluePilot's own (FordFlags has CANFD = 1, HAS_BSM = 2; 4 is kept for the Edge)
FORD_FLAG_BP_HEV_CLUSTER_DATA = 8   # Cluster_HEV_Data2 present
FORD_FLAG_BP_HEV_BATTERY_DATA = 16  # Battery_Traction_1/3/4_FD1 present

# the dash speed (vEgoCluster) reads 0 below this and moves only by more than the gap (applied by the base interface)
CLUSTER_MIN_SPEED = CV.KPH_TO_MS * 1.5
CLUSTER_SPEED_HYST_GAP = CV.KPH_TO_MS / 2.

# Combo buttons are listed twice, once per event they can emit (handled in CarStateExt.update)
BUTTONS = [
  Button(ButtonType.accelCruise, "Steering_Data_FD1", "CcAslButtnSetIncPress", [1]),
  Button(ButtonType.setCruise, "Steering_Data_FD1", "CcAslButtnSetIncPress", [1]),
  Button(ButtonType.decelCruise, "Steering_Data_FD1", "CcAslButtnSetDecPress", [1]),
  Button(ButtonType.setCruise, "Steering_Data_FD1", "CcAslButtnSetDecPress", [1]),
  Button(ButtonType.cancel, "Steering_Data_FD1", "CcAslButtnCnclResPress", [1]),
  Button(ButtonType.resumeCruise, "Steering_Data_FD1", "CcAslButtnCnclResPress", [1]),
  Button(ButtonType.mainCruise, "Steering_Data_FD1", "CcButtnOnOffPress", [1]),
]

# combo signal: (event while cruise is enabled, event while disabled)
COMBO_BUTTONS = {
  "CcAslButtnSetIncPress": (ButtonType.accelCruise, ButtonType.setCruise),
  "CcAslButtnSetDecPress": (ButtonType.decelCruise, ButtonType.setCruise),
  "CcAslButtnCnclResPress": (ButtonType.cancel, ButtonType.resumeCruise),
}

HEV_POWER_FLOW_MODES = {
  0: "", 1: "Hybrid Drive", 2: "Charging HV Battery", 3: "Idle", 4: "Idle with Charging", 5: "Electric Drive",
  6: "Engine Drive", 7: "Remote Start", 8: "Charge Complete", 9: "Fast Charge Complete", 10: "Fast Charging",
  11: "Regen Braking", 12: "Not Used", 13: "Not Used", 14: "Not Used", 15: "Not Used",
}
HEV_ENGINE_ON_REASONS = {
  0: "", 1: "Acceleration", 2: "High Speed", 3: "Heater Setting", 4: "Neutral Gear", 5: "Engine Cold",
  6: "Battery Charging", 7: "Low Gear", 8: "Normal Operation", 9: "Oil Maintenance", 10: "Fuel Maintenance",
  11: "Hill Descent Control", 12: "Battery Temperature", 13: "Drive Mode",
}


def _canfd_automatic(CP) -> bool:
  """The cars whose gear comes from Gear_Shift_by_Wire_FD1: what can_messages() registers and CarStateExt reads."""
  return bool(CP.flags & FordFlags.CANFD) and CP.transmissionType == TransmissionType.automatic


def can_messages(CP) -> tuple[list, list]:
  """(powertrain, camera) messages to register with BP7's frequencies before anything reads them:
  a message first read without one is required, and a car without it would lose CAN validity.
  nan = optional (never required to arrive)."""
  pt = [("BCM_Lamp_Stat_FD1", float('nan'))]
  if CP.flags & FORD_FLAG_BP_HEV_CLUSTER_DATA:
    pt.append(("Cluster_HEV_Data2", float('nan')))
  if CP.flags & FORD_FLAG_BP_HEV_BATTERY_DATA:
    pt += [(m, float('nan')) for m in ("Battery_Traction_1_FD1", "Battery_Traction_3_FD1", "Battery_Traction_4_FD1", "MtrTracData_1_FD1")]
  # the posted limit: optional on every Ford (BP7 required it on CAN FD, so a CAN FD Ford without sign recognition lost CAN validity)
  cam = [("Traffic_RecognitnData", float('nan'))]
  # what the CAN FD reads below need, required at BP7's frequencies
  if _canfd_automatic(CP):
    # BP7 still requires PowertrainData_10 on every automatic but the Edge, though CAN FD no longer reads it
    pt += [("Gear_Shift_by_Wire_FD1", 10), ("PowertrainData_10", 10)]
  if CP.flags & FordFlags.CANFD:
    cam.append(("IPMA_Data2", 1))
  return pt, cam


class CarStateExt:
  def __init__(self, CP):
    self.CP = CP
    self.button_events: list[structs.CarState.ButtonEvent] = []
    self.button_states = {button.event_type: False for button in BUTTONS}
    self.last_emitted_event: dict[str, int] = {}  # combo signal -> the event its press emitted
    # each combo signal's own previous raw state: comparing against the shared button_states slots
    # let one combo handler's writes look like another signal's edge, a press/release storm on any
    # sustained hold (BluePilotDev/bluepilot#193)
    self.prev_button_signal: dict[str, bool] = {}
    self.cruise_enabled_prev = False
    self.main_cruise_pressed_recently = False
    self.car_state_bp: dict = {}
    if _canfd_automatic(CP):
      self.canfd_shifter_values = CANDefine(DBC[CP.carFingerprint][Bus.pt]).dv["Gear_Shift_by_Wire_FD1"]["TrnRng_D_RqGsm"]

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, cp, cp_cam) -> None:
    ret_sp.speedLimit = self.traffic_sign_speed_limit(cp_cam)
    self.update_buttons(ret, cp)
    self.car_state_bp = self.hybrid_and_brake_lights(cp, cp_cam)

  def update_buttons(self, ret: structs.CarState, cp) -> None:
    button_events = []
    cruise_enabled = ret.cruiseState.enabled
    cruise_just_enabled = cruise_enabled and not self.cruise_enabled_prev
    main_cruise_just_pressed = False

    processed_signals = set()  # once per call: combo signals are listed twice in BUTTONS (#193)
    for button in BUTTONS:
      state = cp.vl[button.can_addr][button.can_msg] in button.values
      if button.can_msg in processed_signals:
        continue

      if button.can_msg in COMBO_BUTTONS:
        processed_signals.add(button.can_msg)
        transitioned = state != self.prev_button_signal.get(button.can_msg, False)
        self.prev_button_signal[button.can_msg] = state
        enabled_event, disabled_event = COMBO_BUTTONS[button.can_msg]
        if state and transitioned:
          event_type = enabled_event if cruise_enabled else disabled_event
          button_events.append(structs.CarState.ButtonEvent(pressed=True, type=event_type))
          self.last_emitted_event[button.can_msg] = event_type
        elif not state and transitioned:
          # release only the event the press emitted
          last_emitted = self.last_emitted_event.pop(button.can_msg, None)
          if last_emitted is not None:
            button_events.append(structs.CarState.ButtonEvent(pressed=False, type=last_emitted))
        if transitioned:
          self.button_states[enabled_event] = self.button_states[disabled_event] = state
        continue

      # regular buttons: an event on each edge
      if self.button_states.get(button.event_type, False) != state:
        button_events.append(structs.CarState.ButtonEvent(pressed=state, type=button.event_type))
        if button.event_type == ButtonType.mainCruise and state:
          main_cruise_just_pressed = True
      self.button_states[button.event_type] = state

    if main_cruise_just_pressed:
      self.main_cruise_pressed_recently = True

    # main cruise turning cruise on also sets it to the current speed (same frame or the next)
    if cruise_just_enabled and self.main_cruise_pressed_recently:
      button_events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.setCruise))
      button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.setCruise))
      self.main_cruise_pressed_recently = False

    if cruise_enabled and not cruise_just_enabled:
      self.main_cruise_pressed_recently = False

    self.cruise_enabled_prev = cruise_enabled
    self.button_events = button_events

  @staticmethod
  def canfd_is_metric(cp_cam) -> bool:
    """The set speed's unit on CAN FD: the camera's speed limit unit (1 = km/h, 2 = mph)."""
    return cp_cam.vl["IPMA_Data2"]["IsaVLimUnit_D_Rq"] == 1

  def canfd_gear(self, cp) -> str | None:
    """The CAN FD gear: the shifter's request (Gear_Shift_by_Wire_FD1), as BP7 reads it."""
    return self.canfd_shifter_values.get(cp.vl["Gear_Shift_by_Wire_FD1"]["TrnRng_D_RqGsm"])

  @staticmethod
  def traffic_sign_speed_limit(cp_cam) -> float:
    """The posted limit in m/s from the camera's traffic sign recognition, 0 if none. The unit is in
    the message; 0 and 255 mean no limit (also what a car without the feature reads)."""
    v_limit = cp_cam.vl["Traffic_RecognitnData"]["TsrVLim1MsgTxt_D_Rq"]
    v_limit_unit = cp_cam.vl["Traffic_RecognitnData"]["TsrVlUnitMsgTxt_D_Rq"]
    speed_factor = CV.MPH_TO_MS if v_limit_unit == 2 else CV.KPH_TO_MS if v_limit_unit == 1 else 0
    return v_limit * speed_factor if v_limit not in (0, 255) else 0

  def hybrid_and_brake_lights(self, cp, cp_cam) -> dict:
    """The fields of cereal's CarStateBP."""
    drive = dict(dataAvailable=False, throttleDemandPercent=0.0, throttleThresholdPercent=0.0, powerFlowMode="",
                 powerFlowModeValue=0, engineOnReason="", engineOnReasonValue=0)
    battery = dict(dataAvailable=False, voltHighLimit=0.0, voltLowLimit=0.0, voltActual=0.0, ampsActual=0.0,
                   socMinPerc=0.0, socMaxPerc=0.0, socActual=0.0)

    # brake lights: the body control module's lamp status
    lights_on = bool(cp.vl["BCM_Lamp_Stat_FD1"]["StopLghtOn_B_Stat"])
    # openpilot longitudinal braking also lights them
    if self.CP.openpilotLongitudinalControl:
      acc = cp_cam.vl["ACCDATA"]
      lights_on = lights_on or acc["AccBrkPrchg_B_Rq"] == 1 or acc["AccBrkDecel_B_Rq"] == 1
    lights = dict(dataAvailable=True, brakeLightsOn=lights_on)

    if self.CP.flags & FORD_FLAG_BP_HEV_CLUSTER_DATA:
      hev = cp.vl["Cluster_HEV_Data2"]
      power_flow, engine_reason = int(hev["PwrFlowTxt_D_Dsply"]), int(hev["EngOnMsg1_D_Dsply"])
      drive.update(dataAvailable=True, throttleDemandPercent=hev["EffWhlLvl2_Pc_Dsply"],
                   throttleThresholdPercent=hev["EffWhlThres_Pc_Dsply"],
                   powerFlowMode=HEV_POWER_FLOW_MODES.get(power_flow, "Unknown"), powerFlowModeValue=power_flow,
                   engineOnReason=HEV_ENGINE_ON_REASONS.get(engine_reason, "Unknown"), engineOnReasonValue=engine_reason)

    if self.CP.flags & FORD_FLAG_BP_HEV_BATTERY_DATA:
      batt1, batt3, batt4 = cp.vl["Battery_Traction_1_FD1"], cp.vl["Battery_Traction_3_FD1"], cp.vl["Battery_Traction_4_FD1"]
      # the Mach-E's Battery_Traction_1 current reads a constant -750 A: use the motor current, negated
      # so positive = charging
      amps = -cp.vl["MtrTracData_1_FD1"]["MtrTrac2_I_Actl"] if self.CP.carFingerprint == CAR.FORD_MUSTANG_MACH_E_MK1 \
             else batt1["BattTrac_I_Actl"]
      battery.update(dataAvailable=True, voltHighLimit=batt1["BattTrac_U_LimHi"], voltLowLimit=batt1["BattTrac_U_LimLo"],
                     voltActual=batt1["BattTrac_U_Actl"], ampsActual=amps, socMinPerc=batt3["BattTracSoc_Pc_MnPrtct"],
                     socMaxPerc=batt3["BattTracSoc_Pc_MxPrtct"], socActual=batt4["BattTracSoc2_Pc_Actl"])

    return dict(hybridDrive=drive, hybridBattery=battery, brakeLightStatus=lights)
