from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from openpilot.common.params import Params
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, CarControllerParams, FordFlags
from opendbc.car.interfaces import CarStateBase
from cereal import messaging
# from bluepilot.logger.bp_logger import debug, info, warning, error, critical
from opendbc.sunnypilot.car.ford.mads import MadsCarState
from opendbc.sunnypilot.car.ford.carstate_ext import CarStateExt

# from opendbc.car.ford.fordcanparser import FordCanParser
from opendbc.car.ford.helpers import get_hev_power_flow_text, get_hev_engine_on_reason_text

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase, MadsCarState, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    CarStateExt.__init__(self, CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.params = Params()
    # self.ford_can_parser = FordCanParser(CP)

    # self.bluecruise_cluster_present = FordConfig.BLUECRUISE_CLUSTER_PRESENT # Sets the value of whether the car has the blue cruise cluster
    if CP.transmissionType == TransmissionType.automatic:
      if CP.flags & FordFlags.CANFD:
        self.shifter_values = can_define.dv["Gear_Shift_by_Wire_FD1"]["TrnRng_D_RqGsm"]
      elif CP.flags & FordFlags.ALT_STEER_ANGLE:
        self.shifter_values = can_define.dv["TransGearData"]["GearLvrPos_D_Actl"]
      else:
        self.shifter_values = can_define.dv["PowertrainData_10"]["TrnRng_D_Rq"]

    self.cluster_min_speed = CV.KPH_TO_MS * 1.5
    self.cluster_speed_hyst_gap = CV.KPH_TO_MS / 2.
    self.distance_button = 0
    self.lc_button = 0

    # Save the HEV data available flag to a param
    self.params.put_bool("FordPrefHevDataAvailable", True if CP.flags & FordFlags.HEV_CLUSTER_DATA else False)
    self.params.put_bool("FordPrefHevBattDataAvailable", True if CP.flags & FordFlags.HEV_BATTERY_DATA else False)
    self.hev_data_available = CP.flags & FordFlags.HEV_CLUSTER_DATA

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

	# Publish CAN data first so any parsing errors don't affect critical CarState updates
    # if(self.params.get_bool("FordPrefStreamCanData")):
    #   try:
    #     self.ford_can_parser.publish_can_data(cp, cp_cam, self.CP.carFingerprint)
    #   except Exception as e:
    #     print(f"Error publishing Ford CAN data: {e}")

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    if self.CP.flags & FordFlags.ALT_STEER_ANGLE:
      self.vehicle_sensors_valid = (
        int((cp.vl["ParkAid_Data"]["ExtSteeringAngleReq2"] + 1000) * 10) not in (32766, 32767)
        and cp.vl["ParkAid_Data"]["EPASExtAngleStatReq"] == 0
        and cp.vl["ParkAid_Data"]["ApaSys_D_Stat"] in (0, 1)
      )
    else:
   	  # Occasionally on startup, the ABS module recalibrates the steering pinion offset, so we need to block engagement
      # The vehicle usually recovers out of this state within a minute of normal driving
      ret.vehicleSensorsInvalid = cp.vl["SteeringPinion_Data"]["StePinCompAnEst_D_Qf"] != 3

    # car speed
    ret.vEgoRaw = cp.vl["BrakeSysFeatures"]["Veh_V_ActlBrk"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    if self.CP.flags & FordFlags.CANFD:
      ret.vEgoCluster = ((cp.vl["Cluster_Info_3_FD1"]["DISPLAY_SPEED_SCALING"]/100) * cp.vl["EngVehicleSpThrottle2"]["Veh_V_ActlEng"] +
                         cp.vl["Cluster_Info_3_FD1"]["DISPLAY_SPEED_OFFSET"]) * CV.KPH_TO_MS

    ret.yawRate = cp.vl["Yaw_Data_FD1"]["VehYaw_W_Actl"]
    ret.standstill = cp.vl["DesiredTorqBrk"]["VehStop_D_Stat"] == 1

    # gas pedal
    ret.gasPressed = cp.vl["EngVehicleSpThrottle"]["ApedPos_Pc_ActlArb"] / 100. > 1e-6

    # brake pedal
    ret.brake = cp.vl["BrakeSnData_4"]["BrkTot_Tq_Actl"] / 32756.  # torque in Nm
    ret.brakePressed = cp.vl["EngBrakeData"]["BpedDrvAppl_D_Actl"] == 2
    ret.parkingBrake = cp.vl["DesiredTorqBrk"]["PrkBrkStatus"] in (1, 2)

    # steering wheel
    if self.CP.flags & FordFlags.ALT_STEER_ANGLE:
      steering_angle_init = cp.vl["SteeringPinion_Data_Alt"]["StePinRelInit_An_Sns"]
      if self.vehicle_sensors_valid:
        steering_angle_est = cp.vl["ParkAid_Data"]["ExtSteeringAngleReq2"]
        self.steering_angle_offset_deg = steering_angle_est - steering_angle_init
      ret.steeringAngleDeg = steering_angle_init + self.steering_angle_offset_deg
    else:
      ret.steeringAngleDeg = cp.vl["SteeringPinion_Data"]["StePinComp_An_Est"]
    ret.steeringTorque = cp.vl["EPAS_INFO"]["SteeringColumnTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    ret.steerFaultTemporary = cp.vl["EPAS_INFO"]["EPAS_Failure"] == 1
    ret.steerFaultPermanent = cp.vl["EPAS_INFO"]["EPAS_Failure"] in (2, 3)
    ret.espDisabled = cp.vl["Cluster_Info1_FD1"]["DrvSlipCtlMde_D_Rq"] != 0  # 0 is default mode

    if self.CP.flags & FordFlags.CANFD:
      # this signal is always 0 on non-CAN FD cars
      ret.steerFaultTemporary |= cp.vl["Lane_Assist_Data3_FD1"]["LatCtlSte_D_Stat"] not in (1, 2, 3)

    # cruise state
    is_metric = cp.vl["INSTRUMENT_PANEL"]["METRIC_UNITS"] == 1 if not self.CP.flags & FordFlags.CANFD else cp_cam.vl["IPMA_Data2"]["IsaVLimUnit_D_Rq"] == 1
    ret.cruiseState.speed = cp.vl["EngBrakeData"]["Veh_V_DsplyCcSet"] * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    ret.cruiseState.speedCluster = ret.cruiseState.speed  # ICBM needs speedCluster to read current cruise setpoint
    ret.cruiseState.enabled = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (4, 5)
    ret.cruiseState.available = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (3, 4, 5)
    ret.cruiseState.nonAdaptive = cp.vl["Cluster_Info1_FD1"]["AccEnbl_B_RqDrv"] == 0
    ret.cruiseState.standstill = cp.vl["EngBrakeData"]["AccStopMde_D_Rq"] == 3
    ret.accFaulted = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (1, 2)

    if self.CP.flags & FordFlags.CANFD:
      ret_sp.speedLimit = self.update_traffic_signals(cp_cam)

    if not self.CP.openpilotLongitudinalControl:
      ret.accFaulted = ret.accFaulted or cp_cam.vl["ACCDATA"]["CmbbDeny_B_Actl"] == 1

    # gear
    if self.CP.transmissionType == TransmissionType.automatic:
      if self.CP.flags & FordFlags.CANFD:
        gear = self.shifter_values.get(cp.vl["Gear_Shift_by_Wire_FD1"]["TrnRng_D_RqGsm"])
      elif self.CP.flags & FordFlags.ALT_STEER_ANGLE:
          gear = self.shifter_values.get(cp.vl["TransGearData"]["GearLvrPos_D_Actl"])
      else:
        gear = self.shifter_values.get(cp.vl["PowertrainData_10"]["TrnRng_D_Rq"])

      ret.gearShifter = self.parse_gear_shifter(gear)
    elif self.CP.transmissionType == TransmissionType.manual:
      if bool(cp.vl["BCM_Lamp_Stat_FD1"]["RvrseLghtOn_B_Stat"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # safety
    ret.stockFcw = bool(cp_cam.vl["ACCDATA_3"]["FcwVisblWarn_B_Rq"])
    ret.stockAeb = bool(cp_cam.vl["ACCDATA_2"]["CmbbBrkDecel_B_Rq"])

    # button presses
    ret.leftBlinker = cp.vl["Steering_Data_FD1"]["TurnLghtSwtch_D_Stat"] == 1
    ret.rightBlinker = cp.vl["Steering_Data_FD1"]["TurnLghtSwtch_D_Stat"] == 2
    # TODO: block this going to the camera otherwise it will enable stock TJA
    ret.genericToggle = bool(cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"])
    prev_distance_button = self.distance_button
    prev_lc_button = self.lc_button
    self.distance_button = cp.vl["Steering_Data_FD1"]["AccButtnGapTogglePress"]
    self.lc_button = bool(cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"])

    # lock info
    ret.doorOpen = any([cp.vl["BodyInfo_3_FD1"]["DrStatDrv_B_Actl"], cp.vl["BodyInfo_3_FD1"]["DrStatPsngr_B_Actl"],
                        cp.vl["BodyInfo_3_FD1"]["DrStatRl_B_Actl"], cp.vl["BodyInfo_3_FD1"]["DrStatRr_B_Actl"]])
    ret.seatbeltUnlatched = cp.vl["RCMStatusMessage2_FD1"]["FirstRowBuckleDriver"] == 2

    # blindspot sensors
    if self.CP.enableBsm:
      cp_bsm = cp_cam if self.CP.flags & FordFlags.CANFD else cp
      ret.leftBlindspot = cp_bsm.vl["Side_Detect_L_Stat"]["SodDetctLeft_D_Stat"] != 0
      ret.rightBlindspot = cp_bsm.vl["Side_Detect_R_Stat"]["SodDetctRight_D_Stat"] != 0

    # Stock steering buttons so that we can passthru blinkers etc.
    self.buttons_stock_values = cp.vl["Steering_Data_FD1"]
    # Stock values from IPMA so that we can retain some stock functionality
    self.acc_tja_status_stock_values = cp_cam.vl["ACCDATA_3"]
    self.lkas_status_stock_values = cp_cam.vl["IPMA_Data"]

    MadsCarState.update_mads(self, ret, can_parsers)
    CarStateExt.update(self, ret, ret_sp, can_parsers)

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.lc_button, prev_lc_button, {1: ButtonType.lkas}),
      *self.button_events,
    ]

    self.car_state_bp_msg = self.update_car_state_bp(cp, cp_cam)
    return ret, ret_sp

  def update_car_state_bp(self, cp, cp_cam):
    """Update the CarStateBP message for HEV/PHEV data

    Args:
        cp: Powertrain bus CAN parser
        cp_cam: Camera bus CAN parser
    """
    # Create a new message
    dat = messaging.new_message("carStateBP")
    dat.valid = True

    # Get handles to the message structures
    hybrid_drive = dat.carStateBP.hybridDrive
    hybrid_battery = dat.carStateBP.hybridBattery
    brake_light_status = dat.carStateBP.brakeLightStatus

    # Initialize with default values
    hybrid_drive.dataAvailable = False
    hybrid_drive.throttleDemandPercent = 0.0
    hybrid_drive.throttleThresholdPercent = 0.0
    hybrid_drive.powerFlowMode = ""
    hybrid_drive.powerFlowModeValue = 0
    hybrid_drive.engineOnReason = ""
    hybrid_drive.engineOnReasonValue = 0

    hybrid_battery.dataAvailable = False
    hybrid_battery.voltHighLimit = 0.0
    hybrid_battery.voltLowLimit = 0.0
    hybrid_battery.voltActual = 0.0
    hybrid_battery.ampsActual = 0.0
    hybrid_battery.socMinPerc = 0.0
    hybrid_battery.socMaxPerc = 0.0
    hybrid_battery.socActual = 0.0

    # Initialize brake light status
    brake_light_status.dataAvailable = False
    brake_light_status.brakeLightsOn = False

    # Brake light status - try BCM message first (more reliable), then fallback to BrakeSysFeatures_2
    brake_lights_detected = False

    # Primary: BCM_Lamp_Stat_FD1 message (actual brake light status from Body Control Module)
    try:
      bcm_data = cp.vl["BCM_Lamp_Stat_FD1"]
      if bcm_data is not None:
        brake_light_status.dataAvailable = True

        # Try StopLghtOn_B_Stat signal first (standard DBC signal)
        if "StopLghtOn_B_Stat" in bcm_data:
          brake_light_status.brakeLightsOn = bool(bcm_data["StopLghtOn_B_Stat"])
          brake_lights_detected = True
        # Fallback: Try other BCM lamp signals that might indicate brake lights
        elif "RvrseLghtOn_B_Stat" in bcm_data:
          # Some vehicles may use reverse light status as brake indicator
          brake_light_status.brakeLightsOn = bcm_data["RvrseLghtOn_B_Stat"] == 1
          brake_lights_detected = True
        else:
          # If no known signals work, mark as unavailable and fall back
          brake_light_status.dataAvailable = False
    except (KeyError, AttributeError):
      pass  # BCM message not available, try fallback

    # Fallback: BrakeSysFeatures_2 message (brake light request signal)
    if not brake_lights_detected:
      try:
        brake_data = cp.vl["BrakeSysFeatures_2"]
        if brake_data is not None:
          brake_light_status.dataAvailable = True
          # BrkLamp_B_Rq indicates when brake lights should be on
          brake_light_status.brakeLightsOn = brake_data["BrkLamp_B_Rq"] == 1
          brake_lights_detected = True
      except (KeyError, AttributeError):
        pass  # BrakeSysFeatures_2 not available

    # ACC brake light logic (applies to both sources)
    if brake_lights_detected and self.CP.openpilotLongitudinalControl:
      try:
        acc_data = cp_cam.vl["ACCDATA"]  # ACCDATA is on camera bus
        # Check if openpilot is actively requesting braking via ACC
        acc_brake_active = (acc_data["AccBrkPrchg_B_Rq"] == 1 or
                           acc_data["AccBrkDecel_B_Rq"] == 1)
        brake_light_status.brakeLightsOn = (brake_light_status.brakeLightsOn or
                                           acc_brake_active)
      except (KeyError, AttributeError):
        pass  # ACCDATA not available, use original brake light status

    # HEV cluster data
    try:
        if self.CP.flags & FordFlags.HEV_CLUSTER_DATA:
          hev_data = cp.vl["Cluster_HEV_Data2"]
          if hev_data is not None:
            hybrid_drive.dataAvailable = True
            hybrid_drive.throttleDemandPercent = hev_data["EffWhlLvl2_Pc_Dsply"]
            hybrid_drive.throttleThresholdPercent = hev_data[
                "EffWhlThres_Pc_Dsply"
            ]
            power_flow_value = int(hev_data["PwrFlowTxt_D_Dsply"])
            engine_reason_value = int(hev_data["EngOnMsg1_D_Dsply"])
            hybrid_drive.powerFlowMode = get_hev_power_flow_text(power_flow_value)
            hybrid_drive.powerFlowModeValue = power_flow_value
            hybrid_drive.engineOnReason = get_hev_engine_on_reason_text(engine_reason_value)
            hybrid_drive.engineOnReasonValue = engine_reason_value
    except (KeyError, AttributeError):
      pass

    # HEV battery data
    try:
      if self.CP.flags & FordFlags.HEV_BATTERY_DATA:
        batt_data1 = cp.vl["Battery_Traction_1_FD1"]
        batt_data3 = cp.vl["Battery_Traction_3_FD1"]
        batt_data4 = cp.vl["Battery_Traction_4_FD1"]

        if all(x is not None for x in [batt_data1, batt_data3, batt_data4]):
          hybrid_battery.dataAvailable = True
          hybrid_battery.voltHighLimit = batt_data1["BattTrac_U_LimHi"]
          hybrid_battery.voltLowLimit = batt_data1["BattTrac_U_LimLo"]
          hybrid_battery.voltActual = batt_data1["BattTrac_U_Actl"]
          hybrid_battery.ampsActual = batt_data1["BattTrac_I_Actl"]
          hybrid_battery.socMinPerc = batt_data3["BattTracSoc_Pc_MnPrtct"]
          hybrid_battery.socMaxPerc = batt_data3["BattTracSoc_Pc_MxPrtct"]
          hybrid_battery.socActual = batt_data4["BattTracSoc2_Pc_Actl"]
    except (KeyError, AttributeError):
        pass

    return dat

  def update_traffic_signals(self, cp_cam):
     # TODO: Check if CAN platforms have the same signals
     if self.CP.flags & FordFlags.CANFD:
       self.v_limit = cp_cam.vl["Traffic_RecognitnData"]["TsrVLim1MsgTxt_D_Rq"]
       v_limit_unit = cp_cam.vl["Traffic_RecognitnData"]["TsrVlUnitMsgTxt_D_Rq"]

       speed_factor = CV.MPH_TO_MS if v_limit_unit == 2 else CV.KPH_TO_MS if v_limit_unit == 1 else 0

       return self.v_limit * speed_factor if self.v_limit not in (0, 255) else 0

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      # sig_address, frequency
      ("VehicleOperatingModes", 100),
      ("BrakeSysFeatures", 50),
      ("BrakeSysFeatures_2", 50),
      ("BCM_Lamp_Stat_FD1", float('nan')),  # Brake light status - non-critical, ignore for CAN validity
      ("Yaw_Data_FD1", 100),
      ("DesiredTorqBrk", 50),
      ("EngVehicleSpThrottle", 100),
      ("EngVehicleSpThrottle2", 50),
      ("BrakeSnData_4", 50),
      ("EngBrakeData", 10),
      ("Cluster_Info1_FD1", 10),
      ("EPAS_INFO", 50),
      ("Steering_Data_FD1", 10),
      ("BodyInfo_3_FD1", 2),
      ("RCMStatusMessage2_FD1", 10),
    ]

    # HEV overlay messages - use float('nan') to mark as non-critical for CAN validity
    # These messages may arrive at irregular intervals depending on vehicle state
    if CP.flags & FordFlags.HEV_CLUSTER_DATA:
      pt_messages.append(("Cluster_HEV_Data2", float('nan')))

    if CP.flags & FordFlags.HEV_BATTERY_DATA:
      pt_messages.append(("Battery_Traction_1_FD1", float('nan')))
      pt_messages.append(("Battery_Traction_3_FD1", float('nan')))
      pt_messages.append(("Battery_Traction_4_FD1", float('nan')))

    if CP.flags & FordFlags.ALT_STEER_ANGLE:
      pt_messages += [
        ("SteeringPinion_Data_Alt", 100),
        ("ParkAid_Data", 50),
        ("TransGearData",10),
      ]
    else:
      pt_messages += [
        ("SteeringPinion_Data", 100),
      ]
      if CP.transmissionType == TransmissionType.automatic:
        pt_messages += [
          ("PowertrainData_10",10)
        ]

    if CP.flags & FordFlags.CANFD:
      pt_messages += [
        ("Lane_Assist_Data3_FD1", 33),
        ("Cluster_Info_3_FD1", 10),
      ]
    else:
      pt_messages += [
        ("INSTRUMENT_PANEL", 1),
      ]

    if CP.transmissionType == TransmissionType.automatic:
      # CANFD_PSCM: shift-by-wire module is on CANFD bus, not CAN main; gear read via PowertrainData_10
      if not (CP.flags & (FordFlags.CANFD_PSCM | FordFlags.CANFD_PSCM_RP)):
        pt_messages += [
          ("Gear_Shift_by_Wire_FD1", 10),
        ]
    elif CP.transmissionType == TransmissionType.manual:
      pt_messages += [
        ("Engine_Clutch_Data", 33),
      ]

    if CP.enableBsm and not (CP.flags & FordFlags.CANFD):
      pt_messages += [
        ("Side_Detect_L_Stat", 5),
        ("Side_Detect_R_Stat", 5),
      ]

    cam_messages = [
      # sig_address, frequency
      ("ACCDATA", 50),
      ("ACCDATA_2", 50),
      ("ACCDATA_3", 5),
      ("IPMA_Data", 1),
    ]

    if CP.flags & FordFlags.CANFD:
      cam_messages += [
        ("Traffic_RecognitnData", 1),
        ("IPMA_Data2", 1),
      ]

    if CP.enableBsm and CP.flags & FordFlags.CANFD:
      cam_messages += [
        ("Side_Detect_L_Stat", 5),
        ("Side_Detect_R_Stat", 5),
      ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus(CP).main),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus(CP).camera),
    }
