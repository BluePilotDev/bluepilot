from cereal import car
from panda import Panda
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import create_button_events, get_safety_config
from openpilot.selfdrive.car.ford.fordcan import CanBus
from openpilot.common.params import Params
from openpilot.selfdrive.car.ford.values import Ecu, FordFlags, FordFlagsSP
from openpilot.selfdrive.car.interfaces import CarInterfaceBase

ButtonType = car.CarState.ButtonEvent.Type
TransmissionType = car.CarParams.TransmissionType
GearShifter = car.CarState.GearShifter


class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    ret.carName = "ford"

    ret.radarUnavailable = True
    ret.steerControlType = car.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.2
    ret.steerLimitTimer = 1.0

    if Params().get("DongleId", encoding='utf8') in ("4fde83db16dc0802", "112e4d6e0cad05e1", "e36b272d5679115f", "24574459dd7fb3e0", "83a4e056c7072678"):
      ret.spFlags |= FordFlagsSP.SP_ENHANCED_LAT_CONTROL.value

    CAN = CanBus(fingerprint=fingerprint)
    cfgs = [get_safety_config(car.CarParams.SafetyModel.ford)]
    if CAN.main >= 4:
      cfgs.insert(0, get_safety_config(car.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = cfgs

    ret.experimentalLongitudinalAvailable = True
    if experimental_long:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_FORD_LONG_CONTROL
      ret.openpilotLongitudinalControl = True

    if ret.flags & FordFlags.CANFD:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_FORD_CANFD
    else:
      # Lock out if the car does not have needed lateral and longitudinal control APIs.
      # Note that we also check CAN for adaptive cruise, but no known signal for LCA exists
      pscm_config = next((fw for fw in car_fw if fw.ecu == Ecu.eps and b'\x22\xDE\x01' in fw.request), None)
      if pscm_config:
        if len(pscm_config.fwVersion) != 24:
          ret.dashcamOnly = True
        else:
          config_tja = pscm_config.fwVersion[7]  # Traffic Jam Assist
          config_lca = pscm_config.fwVersion[8]  # Lane Centering Assist
          if config_tja != 0xFF or config_lca != 0xFF:
            ret.dashcamOnly = True

    if ret.spFlags & FordFlagsSP.SP_ENHANCED_LAT_CONTROL:
      ret.safetyConfigs[-1].safetyParam |= Panda.FLAG_FORD_ENHANCED_LAT_CONTROL

    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [0.5]
    ret.longitudinalTuning.kiV = [0.]

    # Auto Transmission: 0x732 ECU or Gear_Shift_by_Wire_FD1
    found_ecus = [fw.ecu for fw in car_fw]
    if Ecu.shiftByWire in found_ecus or 0x5A in fingerprint[CAN.main] or docs:
      ret.transmissionType = TransmissionType.automatic
    else:
      ret.transmissionType = TransmissionType.manual
      ret.minEnableSpeed = 20.0 * CV.MPH_TO_MS

    # BSM: Side_Detect_L_Stat, Side_Detect_R_Stat
    # TODO: detect bsm in car_fw?
    ret.enableBsm = 0x3A6 in fingerprint[CAN.main] and 0x3A7 in fingerprint[CAN.main]

    # LCA can steer down to zero
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1.
    ret.centerToFront = ret.wheelbase * 0.44
    return ret

  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)
    self.sp_update_params()

    self.CS.button_events.extend(create_button_events(self.CS.distance_button, self.CS.prev_distance_button, {1: ButtonType.gapAdjustCruise}))

    self.CS.mads_enabled = self.get_sp_cruise_main_state(ret, self.CS)

    self.CS.accEnabled = self.get_sp_v_cruise_non_pcm_state(ret, self.CS.accEnabled,
                                                            self.CS.button_events, c.vCruise)

    if ret.cruiseState.available:
      if self.enable_mads:
        if not self.CS.prev_mads_enabled and self.CS.mads_enabled:
          self.CS.madsEnabled = True
        if not self.CS.prev_lkas_enabled and self.CS.lkas_enabled:
          self.CS.madsEnabled = not self.CS.madsEnabled
        self.CS.madsEnabled = self.get_acc_mads(ret.cruiseState.enabled, self.CS.accEnabled, self.CS.madsEnabled)
    else:
      self.CS.madsEnabled = False

    if not self.CP.pcmCruise or (self.CP.pcmCruise and self.CP.minEnableSpeed > 0):
      if any(b.type == ButtonType.cancel for b in self.CS.button_events):
        self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
    if self.get_sp_pedal_disengage(ret):
      self.CS.madsEnabled, self.CS.accEnabled = self.get_sp_cancel_cruise_state(self.CS.madsEnabled)
      ret.cruiseState.enabled = ret.cruiseState.enabled if not self.enable_mads else False if self.CP.pcmCruise else self.CS.accEnabled

    if self.CP.pcmCruise and self.CP.minEnableSpeed > 0 and self.CP.pcmCruiseSpeed:
      if ret.gasPressed and not ret.cruiseState.enabled:
        self.CS.accEnabled = False
      self.CS.accEnabled = ret.cruiseState.enabled or self.CS.accEnabled

    ret, self.CS = self.get_sp_common_state(ret, self.CS, gap_button=bool(self.CS.distance_button))

    ret.buttonEvents = [
      *self.CS.button_events,
      *self.button_events.create_mads_event(self.CS.madsEnabled, self.CS.out.madsEnabled)  # MADS BUTTON
    ]

    events = self.create_common_events(ret, c, extra_gears=[GearShifter.manumatic], pcm_enable=False)

    events, ret = self.create_sp_events(self.CS, ret, events)

    if not self.CS.vehicle_sensors_valid:
      events.add(car.CarEvent.EventName.vehicleSensorsInvalid)

    ret.events = events.to_msg()

    return ret
