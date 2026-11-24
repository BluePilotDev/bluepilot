"""
BluePilot (full BluePilot only): Ford CAN messages BluePilot changed from upstream's
opendbc/car/ford/fordcan.py, from BluePilot 7.
"""
from opendbc.car.ford.fordcan import CanBus


def create_acc_msg(packer, CAN: CanBus, long_active: bool, gas: float, accel: float, stopping: bool,
                   brake_actuate: bool, precharge_actuate: bool, v_ego_kph: float):
  """Upstream's ACC command (ACCDATA, 50 Hz) with the pre-charge and deceleration requests set
  separately, each with its own hysteresis (longitudinal_ext.py), instead of one brake request."""
  values = {
    "AccBrkTot_A_Rq": accel,                            # Brake total accel request: [-20|11.9449] m/s^2
    "Cmbb_B_Enbl": 1 if long_active else 0,             # Enabled: 0=No, 1=Yes
    "AccPrpl_A_Rq": gas,                                # Acceleration request: [-5|5.23] m/s^2
    "AccPrpl_A_Pred": -5.0,                             # Acceleration request: [-5|5.23] m/s^2
    "AccResumEnbl_B_Rq": 1 if long_active else 0,
    "AccVeh_V_Trg": v_ego_kph,                          # Target speed: [0|255] km/h
    "AccBrkPrchg_B_Rq": 1 if precharge_actuate else 0,  # Pre-charge brake request: 0=No, 1=Yes
    "AccBrkDecel_B_Rq": 1 if brake_actuate else 0,      # Deceleration request: 0=Inactive, 1=Active
    "AccStopStat_B_Rq": 1 if stopping else 0,
  }
  return packer.make_can_msg("ACCDATA", CAN.main, values)


def create_acc_ui_msg(packer, CAN: CanBus, CP, main_on: bool, enabled: bool, fcw_alert: bool, standstill: bool,
                      hud_control, stock_values: dict, hands_free: bool, show_distance_bars: bool):
  """Upstream's ACC/FCW/TJA cluster status (ACCDATA_3) as BP7 sends it: BlueCruise display (Tja_D_Stat 7)
  when hands_free, Standby rather than Off while moving with main off, FCW audio as well as the visible
  alert, and the camera's TJA warning and text cleared (BP7's driver-monitoring path, which set them,
  never runs)."""
  if enabled:
    if hud_control.leftLaneDepart:
      status = 3  # ActiveInterventionLeft
    elif hud_control.rightLaneDepart:
      status = 4  # ActiveInterventionRight
    elif hands_free:
      status = 7  # BlueCruise UI in the cluster
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
    status = 1  # Standby

  values = {s: stock_values[s] for s in [
    "HaDsply_No_Cs",
    "HaDsply_No_Cnt",
    "AccStopStat_D_Dsply",       # ACC stopped status message
    "AccTrgDist2_D_Dsply",       # ACC target distance
    "AccStopRes_B_Dsply",
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
    "Tja_D_Stat": status,   # TJA status
    "TjaWarn_D_Rq": 0,      # TJA warning: cleared, not the camera's
    "TjaMsgTxt_D_Dsply": 0, # TJA text: cleared, not the camera's
  })

  if CP.openpilotLongitudinalControl:
    values.update({
      "AccStopStat_D_Dsply": 2 if standstill else 0,              # Stopping status text
      "AccMsgTxt_D2_Rq": 0,                                       # ACC text
      "AccTGap_B_Dsply": 1 if show_distance_bars else 0,          # Show time gap control UI
      "AccFllwMde_B_Dsply": 1 if hud_control.leadVisible else 0,  # Lead indicator
      "AccStopMde_B_Dsply": 1 if standstill else 0,
      "AccWarn_D_Dsply": 0,                                       # ACC warning
      "AccTGap_D_Dsply": hud_control.leadDistanceBars,            # Time gap
    })

  if fcw_alert:
    values["FcwVisblWarn_B_Rq"] = 1  # FCW visible alert
    values["FcwAudioWarn_B_Rq"] = 1  # FCW audio alert

  return packer.make_can_msg("ACCDATA_3", CAN.main, values)


def create_lkas_ui_msg(packer, CAN: CanBus, steer_alert: bool, hud_control, stock_values: dict):
  """Upstream's IPMA/LKAS cluster status (IPMA_Data) as BP7 sends it: each lane line on its own
  (departure, visible, else Suppress), whether or not openpilot is engaged.

  LaActvStats_D_Dsply (left \\ right):
    Right ->   | Intervene | Warning | Suppress | Available | None
    Intervene  | 24        | 19      | 14       | 9         | 4
    Warning    | 23        | 18      | 13       | 8         | 3
    Suppress   | 22        | 17      | 12       | 7         | 2
    Available  | 21        | 16      | 11       | 6         | 1
    None       | 20        | 15      | 10       | 5         | 0
  """
  left = 4 if hud_control.leftLaneDepart else 1 if hud_control.leftLaneVisible else 2       # Intervene, Available, Suppress
  right = 20 if hud_control.rightLaneDepart else 5 if hud_control.rightLaneVisible else 10  # Intervene, Available, Suppress

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
    "LaActvStats_D_Dsply": left + right,        # LKAS status (lane lines) [0|31]
    "LaHandsOff_D_Dsply": 1 if steer_alert else 0,  # 0=HandsOn, 1=Level1 (w/o chime)
  })
  return packer.make_can_msg("IPMA_Data", CAN.main, values)
