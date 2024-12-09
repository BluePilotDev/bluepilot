from cereal import car
from openpilot.selfdrive.car import CanBusBase

HUDControl = car.CarControl.HUDControl


class CanBus(CanBusBase):
  def __init__(self, CP=None, fingerprint=None) -> None:
    super().__init__(CP, fingerprint)

  @property
  def main(self) -> int:
    return self.offset

  @property
  def radar(self) -> int:
    return self.offset + 1

  @property
  def camera(self) -> int:
    return self.offset + 2


def calculate_lat_ctl2_checksum(mode: int, counter: int, dat: bytearray) -> int:
  curvature = (dat[2] << 3) | ((dat[3]) >> 5)
  curvature_rate = (dat[6] << 3) | ((dat[7]) >> 5)
  path_angle = ((dat[3] & 0x1F) << 6) | ((dat[4]) >> 2)
  path_offset = ((dat[4] & 0x3) << 8) | dat[5]

  checksum = mode + counter
  for sig_val in (curvature, curvature_rate, path_angle, path_offset):
    checksum += sig_val + (sig_val >> 8)

  return 0xFF - (checksum & 0xFF)


def create_lka_msg(packer, CAN: CanBus, lat_active: bool, hud_control):
  """
  Creates an empty CAN message for the Ford LKA Command.

  This command can apply "Lane Keeping Aid" manoeuvres, which are subject to the PSCM lockout.

  Frequency is 33Hz.

  # Example hud_control data:
  hud_control: (
    speedVisible = false,
    setSpeed = 12.96416,
    lanesVisible = true,
    leadVisible = false,
    visualAlert = none,
    audibleAlert = none,
    rightLaneVisible = true,
    leftLaneVisible = true,
    rightLaneDepart = false,
    leftLaneDepart = false,
    leadDistanceBars = 3
  )

  """
  # values = {}

  # if hud_control is not None:
  #   actvStats = 7 # LDW_Suppress_Right_Left
  #   actvStatsD2 = 0 # LkaNoInterv

  #   if lat_active and hud_control.lanesVisible:
  #     actvStats = 0 # LDW_Idle
  #     actvStatsD2 = 0 # LkaNoInterv

  #     if hud_control.leftLaneVisible and hud_control.rightLaneVisible:
  #       if(hud_control.leftLaneDepart):
  #         actvStats = 2 # LDW_Warning_Left
  #         actvStatsD2 = 2 # LkaIncrIntervLeft
  #       elif(hud_control.rightLaneDepart):
  #         actvStats = 4 # LDW_Warning_Right
  #         actvStatsD2 = 4 # LkaStandIntervRight
  #       else:
  #         actvStats = 0 # LDW_Idle
  #         actvStatsD2 = 0 # LkaNoInterv
  #     elif hud_control.leftLaneVisible and not hud_control.rightLaneVisible:
  #       if(hud_control.leftLaneDepart):
  #         actvStats = 2 # LDW_Warning_Left
  #         actvStatsD2 = 2 # LkaIncrIntervLeft
  #       else:
  #         actvStats = 5 # LDW_Suppress_Right
  #         actvStatsD2 = 5 # LkaSupprRight
  #     elif not hud_control.leftLaneVisible and hud_control.rightLaneVisible:
  #       if(hud_control.rightLaneDepart):
  #         actvStats = 4 # LDW_Warning_Right
  #         actvStatsD2 = 4 # LkaStandIntervRight
  #       else:
  #         actvStats = 3 # LDW_Suppress_Left
  #         actvStatsD2 = 3 # LkaSupprLeft

  #   values.update({
  #     "LdwActvStats_D_Req": actvStats,  # LKA status: 0=LDW_Idle, 1=LDW_DemoVibration, 2=LDW_Warning_Left, 3=LDW_Suppress_Left, 4=LDW_Warning_Right, 5=LDW_Suppress_Right, 6=Not_Used2, 7=LDW_Suppress_Right_Left
  #     "LkwActvStats_D2_Req": actvStatsD2, # 7=Not Used, 6=LkaIncrIntervRight, 5=LkaSupprRight, 4=LkaStandIntervRight, 3=LkaSupprLeft, 2=LkaStandIntervLeft, 1=LkaIncrIntervLeft, 0=LkaNoInterv
  #     "LdwActvIntns_D_Req": 2,  # LKA intervention: 0=None, 1=Low, 2=Medium, 3=High
  #   })

  #write the values to the console as well as lat_active, and hud_control
  # print(f'values: {values}')
  # print(f'lat_active: {lat_active}')
  # print(f'hud_control: {hud_control}')
  return packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})


def create_lka3_msg(packer, CAN: CanBus):
  """
  Creates an empty CAN message for the Ford LKA Command.

  This command can apply "Lane Keeping Aid" manoeuvres, which are subject to the PSCM lockout.

  Frequency is 33Hz.
  """
  values = {
    "LatCtlCpblty_D_Stat": 2,                   # Lateral Control Capability: 0=NoModeAvailable, 1=LimitedModeAvailable,
                                                #            2=ExtendedModeAvailable, 3=Faulty [0|3]
    "LaActAvail_D_Actl": 3,                      # Lane Keeping Aid Availability: 0=No, 1=Yes, 2=Faulty, 3=NotAvailable [0|3]
  }
  return packer.make_can_msg("Lane_Assist_Data3", CAN.main, values)


def create_lat_ctl_msg(packer, CAN: CanBus, lat_active: bool, ramp_type: int, precision_type: int, path_offset: float, path_angle: float,
                       curvature: float, curvature_rate: float):
  """
  Creates a CAN message for the Ford TJA/LCA Command.

  This command can apply "Lane Centering" manoeuvres: continuous lane centering for traffic jam assist and highway
  driving. It is not subject to the PSCM lockout.

  Ford lane centering command uses a third order polynomial to describe the road centerline. The polynomial is defined
  by the following coefficients:
    c0: lateral offset between the vehicle and the centerline (positive is right)
    c1: heading angle between the vehicle and the centerline (positive is right)
    c2: curvature of the centerline (positive is left)
    c3: rate of change of curvature of the centerline
  As the PSCM combines this information with other sensor data, such as the vehicle's yaw rate and speed, the steering
  angle cannot be easily controlled.

  The PSCM should be configured to accept TJA/LCA commands before these commands will be processed. This can be done
  using tools such as Forscan.

  Frequency is 20Hz.
  """

  values = {
    "LatCtlRng_L_Max": 0,                       # Unknown [0|126] meter
    "HandsOffCnfm_B_Rq": 0,                     # Unknown: 0=Inactive, 1=Active [0|1]
    "LatCtl_D_Rq": 1 if lat_active else 0,      # Mode: 0=None, 1=ContinuousPathFollowing, 2=InterventionLeft,
                                                #       3=InterventionRight, 4-7=NotUsed [0|7]
    "LatCtlRampType_D_Rq": ramp_type,           # Ramp speed: 0=Slow, 1=Medium, 2=Fast, 3=Immediate [0|3]
                                                #             Makes no difference with curvature control
    "LatCtlPrecision_D_Rq": precision_type,     # Precision: 0=Comfortable, 1=Precise, 2/3=NotUsed [0|3]
                                                #            The stock system always uses comfortable
    "LatCtlPathOffst_L_Actl": path_offset,      # Path offset [-5.12|5.11] meter
    "LatCtlPath_An_Actl": path_angle,           # Path angle [-0.5|0.5235] radians
    "LatCtlCurv_NoRate_Actl": curvature_rate,   # Curvature rate [-0.001024|0.00102375] 1/meter^2
    "LatCtlCurv_No_Actl": curvature,            # Curvature [-0.02|0.02094] 1/meter
  }
  return packer.make_can_msg("LateralMotionControl", CAN.main, values)


def create_lat_ctl2_msg(packer, CAN: CanBus, mode: int, ramp_type: int, precision_type: int, path_offset: float, path_angle: float, curvature: float,
                        curvature_rate: float, counter: int):
  """
  Create a CAN message for the new Ford Lane Centering command.

  This message is used on the CAN FD platform and replaces the old LateralMotionControl message. It is similar but has
  additional signals for a counter and checksum.

  Frequency is 20Hz.
  """

  values = {
    "LatCtl_D2_Rq": mode,                       # Mode: 0=None, 1=PathFollowingLimitedMode, 2=PathFollowingExtendedMode,
                                                #       3=SafeRampOut, 4-7=NotUsed [0|7]
    "LatCtlRampType_D_Rq": ramp_type,           # 0=Slow, 1=Medium, 2=Fast, 3=Immediate [0|3]
    "LatCtlPrecision_D_Rq": precision_type,     # 0=Comfortable, 1=Precise, 2/3=NotUsed [0|3]
    "LatCtlPathOffst_L_Actl": path_offset,      # [-5.12|5.11] meter
    "LatCtlPath_An_Actl": path_angle,           # [-0.5|0.5235] radians
    "LatCtlCurv_No_Actl": curvature,            # [-0.02|0.02094] 1/meter
    "LatCtlCrv_NoRate2_Actl": curvature_rate,   # [-0.001024|0.001023] 1/meter^2
    "HandsOffCnfm_B_Rq": 0,                     # 0=Inactive, 1=Active [0|1]
    "LatCtlPath_No_Cnt": counter,               # [0|15]
    "LatCtlPath_No_Cs": 0,                      # [0|255]
  }

  # calculate checksum
  dat = packer.make_can_msg("LateralMotionControl2", 0, values)[1]
  values["LatCtlPath_No_Cs"] = calculate_lat_ctl2_checksum(mode, counter, dat)

  return packer.make_can_msg("LateralMotionControl2", CAN.main, values)


def create_acc_msg(packer, CAN: CanBus, long_active: bool, gas: float, accel: float, stopping: bool, brake_actuator: bool, precharge_actuator: bool, v_ego_kph: float):
  """
  Creates a CAN message for the Ford ACC Command.

  This command can be used to enable ACC, to set the ACC gas/brake/decel values
  and to disable ACC.

  Frequency is 50Hz.
  """
  # decel = accel < 0 and long_active
  gas_pred = -5.0 if gas == -5.0 else accel
  values = {
    "AccBrkTot_A_Rq": accel,                          # Brake total accel request: [-20|11.9449] m/s^2
    "Cmbb_B_Enbl": 1 if long_active else 0,           # Enabled: 0=No, 1=Yes
    "AccPrpl_A_Rq": gas,                              # Acceleration request: [-5|5.23] m/s^2
    "AccPrpl_A_Pred": gas_pred,                           # Acceleration request: [-5|5.23] m/s^2
    "AccResumEnbl_B_Rq": 1 if long_active else 0,
    "AccVeh_V_Trg": v_ego_kph,                        # Target speed: [0|255] km/h
    # TODO: we may be able to improve braking response by utilizing pre-charging better
    "AccBrkPrchg_B_Rq": 1 if precharge_actuator else 0,            # Pre-charge brake request: 0=No, 1=Yes
    "AccBrkDecel_B_Rq": 1 if brake_actuator else 0,            # Deceleration request: 0=Inactive, 1=Active
    "AccStopStat_B_Rq": 1 if stopping else 0,
  }
  return packer.make_can_msg("ACCDATA", CAN.main, values)


def create_acc_ui_msg(packer, CAN: CanBus, CP, main_on: bool, enabled: bool, fcw_alert: bool, standstill: bool,
                      hud_control, stock_values: dict, send_hands_free_msg: bool, send_ui: bool, send_bars: bool, tja_warn: int, tja_msg: int):
  """
  Creates a CAN message for the Ford IPC adaptive cruise, forward collision warning and traffic jam
  assist status.

  Stock functionality is maintained by passing through unmodified signals.

  Frequency is 5Hz.
  """

  # Tja_D_Stat
  if enabled:
    if hud_control.leftLaneDepart:
      status = 3  # ActiveInterventionLeft
    elif hud_control.rightLaneDepart:
      status = 4  # ActiveInterventionRight
    elif send_hands_free_msg:
      status = 7 # Show BlueCruise UI in the Cluster
    else:
      status = 2  # Active
  elif main_on:
    if hud_control.leftLaneDepart:
      status = 5  # ActiveWarningLeft
    elif hud_control.rightLaneDepart:
      status = 6  # ActiveWarningRight
    else:
      status = 1  # Standby
  elif standstill:
    status = 0  # Off
  else:
    status = 1    # Standby

  values = {s: stock_values[s] for s in [
    "HaDsply_No_Cs",
    "HaDsply_No_Cnt",
    "AccStopStat_D_Dsply",       # ACC stopped status message
    "AccTrgDist2_D_Dsply",       # ACC target distance
    "AccStopRes_B_Dsply",
    #"TjaWarn_D_Rq",              # TJA warning
    #"TjaMsgTxt_D_Dsply",         # TJA text
    "IaccLamp_D_Rq",             # iACC status icon
    "AccMsgTxt_D2_Rq",           # ACC text
    "FcwDeny_B_Dsply",           # FCW disabled
    "FcwMemStat_B_Actl",         # FCW enabled setting
    "AccTGap_B_Dsply",           # ACC time gap display setting
    "CadsAlignIncplt_B_Actl",
    "AccFllwMde_B_Dsply",        # ACC follow mode display setting
    "CadsRadrBlck_B_Actl",
    "CmbbPostEvnt_B_Dsply",      # AEB event status
    "AccStopMde_B_Dsply",        # ACC stop mode display setting
    "FcwMemSens_D_Actl",         # FCW sensitivity setting
    "FcwMsgTxt_D_Rq",            # FCW text
    "AccWarn_D_Dsply",           # ACC warning
    "FcwVisblWarn_B_Rq",         # FCW visible alert
    "FcwAudioWarn_B_Rq",         # FCW audio alert
    "AccTGap_D_Dsply",           # ACC time gap
    "AccMemEnbl_B_RqDrv",        # ACC adaptive/normal setting
    "FdaMem_B_Stat",             # FDA enabled setting
  ]}

  values.update({
    "Tja_D_Stat": status,        # TJA status
    "TjaWarn_D_Rq": tja_warn,    # TJA warning
    "TjaMsgTxt_D_Dsply": tja_msg,# TJA text
  })

  if CP.openpilotLongitudinalControl:
    values.update({
      "AccStopStat_D_Dsply": 2 if standstill else 0,              # Stopping status text
      "AccMsgTxt_D2_Rq": 0,                                       # ACC text
      "AccTGap_B_Dsply": 1 if send_bars else 0,      	            # Show time gap control UI
      "AccFllwMde_B_Dsply": 1 if hud_control.leadVisible else 0,  # Lead indicator
      "AccStopMde_B_Dsply": 1 if standstill else 0,
      "AccWarn_D_Dsply": 0,                                       # ACC warning
      "AccTGap_D_Dsply": hud_control.leadDistanceBars,            # Time gap
    })

  # Forwards FCW alert from IPMA
  if fcw_alert:
    values["FcwVisblWarn_B_Rq"] = 1  # FCW visible alert
    values["FcwAudioWarn_B_Rq"] = 1  # FCW audio alert

  return packer.make_can_msg("ACCDATA_3", CAN.main, values)


def create_lkas_ui_msg(packer, CAN: CanBus, main_on: bool, lat_active: bool, steer_alert: int, hud_control,
                       stock_values: dict):
  """
  Creates a CAN message for the Ford IPC IPMA/LKAS status.

  Show the LKAS status with the "driver assist" lines in the IPC.

  Stock functionality is maintained by passing through unmodified signals.

  Frequency is 1Hz.

  # Example hud_control data:
  hud_control: (
    speedVisible = false,
    setSpeed = 12.96416,
    lanesVisible = true,
    leadVisible = false,
    visualAlert = none,
    audibleAlert = none,
    rightLaneVisible = true,
    leftLaneVisible = true,
    rightLaneDepart = false,
    leftLaneDepart = false,
    leadDistanceBars = 3
  )

  # LaActvStats_D_Dsply Value Table
  # Left \\ Right  | Intervene | Warning | Suppress | Available | None
  # ---------------------------------------------------------------
  # Intervene     | 24        | 19      | 14       | 9         | 4
  # Warning       | 23        | 18      | 13       | 8         | 3
  # Suppress      | 22        | 17      | 12       | 7         | 2
  # Available     | 21        | 16      | 11       | 6         | 1
  # None          | 20        | 15      | 10       | 5         | 0

  """

  # TODO: test suppress state
  lines = 0

  if hud_control is not None:
    left_status = 2  # Default to Suppress
    right_status = 10  # Default to Suppress

    # Determine left lane status
    if hud_control.leftLaneDepart:
      left_status = 4 # Intervene (Yellow)
      #left_status = 3 # Warning (Red)
    elif hud_control.leftLaneVisible:
      left_status = 1 # Available
    else:
      left_status = 2 # Suppress

    # Determine right lane status
    if hud_control.rightLaneDepart:
      right_status = 20 # Intervene (Yellow)
      #right_status = 15 # Warning (Red)
    elif hud_control.rightLaneVisible:
      right_status = 5
    else:
      right_status = 10 # Suppress

    # Combine left and right lane status
    lines = left_status + right_status
  else:
    #Set this to 0 if hud_control is None
    lines = 0

  hands_on_wheel_dsply = 1 if steer_alert else 0

  values = {s: stock_values[s] for s in [
    "FeatConfigIpmaActl",
    "FeatNoIpmaActl",
    "PersIndexIpma_D_Actl",
    "AhbcRampingV_D_Rq",     # AHB ramping
    "LaDenyStats_B_Dsply",   # LKAS error
    "CamraDefog_B_Req",      # Windshield heater?
    "CamraStats_D_Dsply",    # Camera status
    "DasAlrtLvl_D_Dsply",    # DAS alert level
    "DasStats_D_Dsply",      # DAS status
    "DasWarn_D_Dsply",       # DAS warning
    "AhbHiBeam_D_Rq",        # AHB status
    "Passthru_63",
    "Passthru_48",
  ]}

  values.update({
    "LaActvStats_D_Dsply": lines,                 # LKAS status (lines) [0|31]
    "LaHandsOff_D_Dsply": hands_on_wheel_dsply,   # 0=HandsOn, 1=Level1 (w/o chime), 2=Level2 (w/ chime), 3=Suppressed
  })
  return packer.make_can_msg("IPMA_Data", CAN.main, values)


def create_button_msg(packer, bus: int, stock_values: dict, cancel=False, resume=False, tja_toggle=False):
  """
  Creates a CAN message for the Ford SCCM buttons/switches.

  Includes cruise control buttons, turn lights and more.

  Frequency is 10Hz.
  """

  values = {s: stock_values[s] for s in [
    "HeadLghtHiFlash_D_Stat",  # SCCM Passthrough the remaining buttons
    "TurnLghtSwtch_D_Stat",    # SCCM Turn signal switch
    "WiprFront_D_Stat",
    "LghtAmb_D_Sns",
    "AccButtnGapDecPress",
    "AccButtnGapIncPress",
    "AslButtnOnOffCnclPress",
    "AslButtnOnOffPress",
    "LaSwtchPos_D_Stat",
    "CcAslButtnCnclResPress",
    "CcAslButtnDeny_B_Actl",
    "CcAslButtnIndxDecPress",
    "CcAslButtnIndxIncPress",
    "CcAslButtnOffCnclPress",
    "CcAslButtnOnOffCncl",
    "CcAslButtnOnPress",
    "CcAslButtnResDecPress",
    "CcAslButtnResIncPress",
    "CcAslButtnSetDecPress",
    "CcAslButtnSetIncPress",
    "CcAslButtnSetPress",
    "CcButtnOffPress",
    "CcButtnOnOffCnclPress",
    "CcButtnOnOffPress",
    "CcButtnOnPress",
    "HeadLghtHiFlash_D_Actl",
    "HeadLghtHiOn_B_StatAhb",
    "AhbStat_B_Dsply",
    "AccButtnGapTogglePress",
    "WiprFrontSwtch_D_Stat",
    "HeadLghtHiCtrl_D_RqAhb",
  ]}

  values.update({
    "CcAslButtnCnclPress": 1 if cancel else 0,      # CC cancel button
    "CcAsllButtnResPress": 1 if resume else 0,      # CC resume button
    "TjaButtnOnOffPress": 1 if tja_toggle else 0,   # LCA/TJA toggle button
  })
  return packer.make_can_msg("Steering_Data_FD1", bus, values)
