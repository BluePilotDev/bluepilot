import numpy as np
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.carcontroller import CarController
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.radar_interface import RadarInterface
from opendbc.car.ford.values import CarControllerParams, DBC, Ecu, FordFlags, FordConfig, RADAR, FordSafetyFlags
from opendbc.car.interfaces import CarInterfaceBase
from bluepilot.params.bp_params import apply_interface_params
from bluepilot.logger.bp_logger import debug, info, warning, error, critical

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    # PCM doesn't allow acceleration near cruise_speed,
    # so limit limits of pid to prevent windup
    ACCEL_MAX_VALS = [CarControllerParams.ACCEL_MAX, 0.2]
    ACCEL_MAX_BP = [cruise_speed - 2., cruise_speed - .4]
    return CarControllerParams.ACCEL_MIN, np.interp(current_speed, ACCEL_MAX_BP, ACCEL_MAX_VALS)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, experimental_long, docs) -> structs.CarParams:
    print("| CarParams Debug")
    debug(f'| Candidate (interface): {candidate}', True)
    ret.brand = "ford"
    ret.dashcamOnly = False # not (ret.flags & FordFlags.CANFD)
    info(f'| Dashcam Only Mode: {ret.dashcamOnly}', True)
    ret.radarUnavailable = Bus.radar not in DBC[candidate]
    info(f'| Radar Unavailable: {ret.radarUnavailable}', True)

    FordConfig.BLUECRUISE_CLUSTER_PRESENT = any(fw.ecu == Ecu.hud for fw in car_fw) # Check for blue cruise cluster
    info(f'| Blue Cruise Cluster Present: {FordConfig.BLUECRUISE_CLUSTER_PRESENT}', True)

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.steerActuatorDelay = 0.22
    ret.steerLimitTimer = 1.0
    ret.steerAtStandstill = True

    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kpV = [0.]
    ret.longitudinalTuning.kiV = [0.5]

    if not ret.radarUnavailable and DBC[candidate][Bus.radar] == RADAR.DELPHI_MRR:
      # average of 33.3 Hz radar timestep / 4 scan modes = 60 ms
      # MRR_Header_Timestamps->CAN_DET_TIME_SINCE_MEAS reports 61.3 ms
      ret.radarDelay = 0.06

    if not ret.radarUnavailable and DBC[candidate][Bus.radar] == RADAR.DELPHI_MRR_64:
      # average of 20 Hz radar timestep / 4 scan modes = 100 ms
      ret.radarDelay = 0.1

    CAN = CanBus(fingerprint=fingerprint)
    cfgs = [get_safety_config(structs.CarParams.SafetyModel.ford)]
    if CAN.main >= 4:
      cfgs.insert(0, get_safety_config(structs.CarParams.SafetyModel.noOutput))
    ret.safetyConfigs = cfgs

    ret.experimentalLongitudinalAvailable = (bool)(ret.flags & FordFlags.CANFD)
    info(f"| experimentalLongAvailable: {ret.experimentalLongitudinalAvailable}", True)
    info(f"| experimental_long: {experimental_long}", True)
    info(f"| ret.flags & FordFlags.CANFD: {ret.flags & FordFlags.CANFD}", True)
    if experimental_long or not ret.flags & FordFlags.CANFD:
      ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.LONG_CONTROL.value
      ret.openpilotLongitudinalControl = True

    if ret.flags & FordFlags.CANFD:
      ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.CANFD.value

    # for fw in car_fw:
    #  debug(f'ECU: {fw.ecu}, FW Version: {fw.fwVersion}', True)

      # TRON (SecOC) platforms are not supported
      # LateralMotionControl2, ACCDATA are 16 bytes on these platforms
      if len(fingerprint[CAN.camera]):
        if fingerprint[CAN.camera].get(0x3d6) != 8 or fingerprint[CAN.camera].get(0x186) != 8:
          carlog.error('dashcamOnly: SecOC is unsupported')
          error('dashcamOnly: SecOC is unsupported', True)
          ret.dashcamOnly = True
    else:
      # Lock out if the car does not have needed lateral and longitudinal control APIs.
      # Note that we also check CAN for adaptive cruise, but no known signal for LCA exists
      pscm_config = next((fw for fw in car_fw if fw.ecu == Ecu.eps and b'\x22\xDE\x01' in fw.request), None)
      if pscm_config:
        if len(pscm_config.fwVersion) != 24:
          carlog.error('dashcamOnly: Invalid EPS FW version')
          error("dashcamOnly: Invalid EPS FW version", True)
          ret.dashcamOnly = True
        else:
          config_tja = pscm_config.fwVersion[7]  # Traffic Jam Assist
          config_lca = pscm_config.fwVersion[8]  # Lane Centering Assist
          if config_tja != 0xFF or config_lca != 0xFF:
            carlog.error('dashcamOnly: Car lacks required lateral control APIs')
            error("dashcamOnly: Car lacks required lateral control APIs", True)
            ret.dashcamOnly = True

    # Apply custom parameters from params.json
    apply_interface_params(ret, "interface")

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

    if 0x365 in fingerprint[CAN.main]:  # F150 HEV Cluster_HEV_Data2 signal (869 = 0x365)
      ret.flags |= int(FordFlags.HEV_CLUSTER_DATA)
      info('HEV_CLUSTER_DATA signal detected (interface.py)', True)
    # Check for HEV battery data signals
    if 0x07A in fingerprint[CAN.main] and 0x24B in fingerprint[CAN.main] and 0x24C in fingerprint[CAN.main]:  # 122, 587, 588
      ret.flags |= int(FordFlags.HEV_BATTERY_DATA)
      info('HEV_BATTERY_DATA signal detected (interface.py)', True)

    # LCA can steer down to zero
    ret.minSteerSpeed = 0.

    ret.autoResumeSng = ret.minEnableSpeed == -1.
    ret.centerToFront = ret.wheelbase * 0.44
    return ret
