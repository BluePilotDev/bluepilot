"""
BluePilot Ford CAN message builder extensions (lateral only).

Extended versions of the stock fordcan.py lateral builders:
  - Dynamic ramp_type and precision_type for the 4-signal lateral commands
  - Lane_Assist_Data1 carries the angle-mode side channel read by bluepilot_lateral/safety/ford.h

Trimmed for the lateral proof-of-concept: the longitudinal, HUD and ICBM builders from
BluePilot's fordcan_ext.py are out of scope here.
"""

from ...car.ford.fordcan import CanBus


# BluePilot: the Lane_Assist_Data1 side channel, see LANE_ASSIST_DATA1.md. ford_declarations.h has the
# same values (tests/test_lane_assist_data1.py).
BP_LKA_ANGLE_MODE_BYTE = 4
BP_LKA_ANGLE_MODE_MASK = 0x1
BP_LKA_SHADOW_MSB_BYTE = 5  # shadow curvature: int16, big-endian, bytes 5-6
BP_LKA_SHADOW_LSB_BYTE = 6
BP_LKA_SHADOW_CURVATURE_SCALE = 1e-6  # 1/meter per raw unit


def create_lka_msg(packer, CAN: CanBus, lat_active: bool, hud_control,
                    angle_mode_engaged: bool = False, shadow_curvature: float = 0.0):
  """
  Creates a CAN message for the Ford LKA Command.

  BluePilot extension: accepts lat_active and hud_control parameters for future
  lane departure warning integration (unused so far).

  Also carries angle_mode_engaged + shadow_curvature (Ford sign) in bits with no DBC signal: the
  layout and what the panda does with them are in LANE_ASSIST_DATA1.md.

  Frequency is 33Hz.
  """
  addr, dat, bus = packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})
  dat = bytearray(dat)

  shadow_curvature_raw = int(round(shadow_curvature / BP_LKA_SHADOW_CURVATURE_SCALE))
  shadow_curvature_raw = max(-32768, min(32767, shadow_curvature_raw)) & 0xFFFF

  dat[BP_LKA_ANGLE_MODE_BYTE] |= BP_LKA_ANGLE_MODE_MASK if angle_mode_engaged else 0
  dat[BP_LKA_SHADOW_MSB_BYTE] = (shadow_curvature_raw >> 8) & 0xFF
  dat[BP_LKA_SHADOW_LSB_BYTE] = shadow_curvature_raw & 0xFF

  return addr, bytes(dat), bus


def create_lat_ctl_msg(packer, CAN: CanBus, lat_active: bool, ramp_type: int, precision_type: int,
                       path_offset: float, path_angle: float, curvature: float, curvature_rate: float):
  """
  Creates a CAN message for the Ford TJA/LCA Command (non-CAN FD).

  BluePilot extension: dynamic ramp_type and precision_type parameters.
  Stock hardcodes ramp_type=0 (Slow) and precision_type=1 (Precise).

  Ford lane centering uses a third-order polynomial to describe the road centerline:
    c0 (path_offset): lateral offset between vehicle and centerline (positive is right)
    c1 (path_angle): heading angle between vehicle and centerline (positive is right)
    c2 (curvature): curvature of the centerline (positive is left)
    c3 (curvature_rate): rate of change of curvature

  Frequency is 20Hz.
  """
  values = {
    "LatCtlRng_L_Max": 0,                       # Unknown [0|126] meter
    "HandsOffCnfm_B_Rq": 0,                     # Unknown: 0=Inactive, 1=Active [0|1]
    "LatCtl_D_Rq": 1 if lat_active else 0,      # Mode: 0=None, 1=ContinuousPathFollowing, 2=InterventionLeft,
                                                 #       3=InterventionRight, 4-7=NotUsed [0|7]
    "LatCtlRampType_D_Rq": ramp_type,           # Ramp speed: 0=Slow, 1=Medium, 2=Fast, 3=Immediate [0|3]
    "LatCtlPrecision_D_Rq": precision_type,     # Precision: 0=Comfortable, 1=Precise, 2/3=NotUsed [0|3]
    "LatCtlPathOffst_L_Actl": path_offset,      # Path offset [-5.12|5.11] meter
    "LatCtlPath_An_Actl": path_angle,           # Path angle [-0.5|0.5235] radians
    "LatCtlCurv_NoRate_Actl": curvature_rate,   # Curvature rate [-0.001024|0.00102375] 1/meter^2
    "LatCtlCurv_No_Actl": curvature,            # Curvature [-0.02|0.02094] 1/meter
  }
  return packer.make_can_msg("LateralMotionControl", CAN.main, values)


def create_lat_ctl2_msg(packer, CAN: CanBus, mode: int, ramp_type: int, precision_type: int,
                        path_offset: float, path_angle: float, curvature: float,
                        curvature_rate: float, counter: int):
  """
  Creates a CAN message for the Ford Lane Centering command (CAN FD).

  BluePilot extension: dynamic ramp_type and precision_type parameters.
  Stock hardcodes ramp_type=0 (Slow) and precision_type=1 (Precise).

  This message replaces LateralMotionControl on CAN FD platforms and includes
  counter and checksum fields.

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

  # Computed here, not left to the packer: upstream's packer fills Ford checksums itself (and overwrites
  # this with the same value), but a host's packer may not (IQ.Pilot's doesn't), and the PSCM drops a
  # frame whose checksum is wrong.
  dat = packer.make_can_msg("LateralMotionControl2", CAN.main, values)[1]
  values["LatCtlPath_No_Cs"] = lat_ctl2_checksum(mode, counter, dat)
  return packer.make_can_msg("LateralMotionControl2", CAN.main, values)


def lat_ctl2_checksum(mode: int, counter: int, dat: bytes) -> int:
  """LatCtlPath_No_Cs of a packed LateralMotionControl2: upstream's ford_checksum over the message's
  checksum fields (mode, counter, curvature, curvature rate, path angle, path offset)."""
  curvature = (dat[2] << 3) | (dat[3] >> 5)
  curvature_rate = (dat[6] << 3) | (dat[7] >> 5)
  path_angle = ((dat[3] & 0x1F) << 6) | (dat[4] >> 2)
  path_offset = ((dat[4] & 0x3) << 8) | dat[5]

  checksum = mode + counter
  for sig_val in (curvature, curvature_rate, path_angle, path_offset):
    checksum += sig_val + (sig_val >> 8)
  return 0xFF - (checksum & 0xFF)
