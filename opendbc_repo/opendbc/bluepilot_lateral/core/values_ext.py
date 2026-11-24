from ...car import structs
from ...car.lateral import AngleSteeringLimits
from .. import host_types
from ..params.manifest import PER_FRAME_KEYS, startup_value


class FordSafetyFlagsSP:
  """BluePilot's Ford safety flags: the bits of the second safety param a host sends the panda
  (current_safety_param_sp, USB control 0xdf), not the main safetyConfigs[].safetyParam (HOSTS.md).
  ford_bp_init reads it with GET_FLAG(current_safety_param_sp, ...), same pattern as Subaru
  STOP_AND_GO (subaru_common.h). Plain int constants, not IntFlag: the param must stay a plain int
  through the host's serialization.
  """
  # Bit layout of the SP safety param (mirrored in ford_bp_init, bluepilot_lateral/safety/ford.h; pinned by
  # tests/test_safety_param_bits.py):
  #   bit 0    STEER_ANGLE_CURVATURE (pinion-sourced curvature measurement)
  #   bits 1-4 pinion geometry index (FORD_PINION_GEOMETRY_SHIFT / FORD_PINION_GEOMETRY_INDEX)
  #   bit 5    BP_LATERAL: the panda runs the 4-signal checks (ford_lmc_checks) instead of
  #            upstream's stock curvature-only checks. Clear = upstream safety, verbatim.
  STEER_ANGLE_CURVATURE = 1
  BP_LATERAL = 1 << 5


# Geometry-table index for the steering-angle curvature measurement, packed into
# the startup safety bits 1-4 when STEER_ANGLE_CURVATURE is set. Must match the
# ford_pinion_geometry table in bluepilot_lateral/safety/ford.h row for row.
# Index 0 is reserved as invalid: the firmware treats flag-set-but-no-index as feature
# off, so a half-configured param can never select the wrong geometry silently.
# Platforms with no row are in FORD_NO_PINION_PLATFORMS. Indices stay aligned with the C table.
# BluePilot: the per-platform tables key on the platform name (CP.carFingerprint), not the CAR enum,
# so a row for a platform some host lacks never breaks import there; it just never matches.
# FORD_HOST_ONLY_PLATFORMS lists the names with rows here that upstream opendbc's CAR doesn't have; any
# name a host's CAR lacks must be on it (tests/test_platform_tables.py).
FORD_HOST_ONLY_PLATFORMS: frozenset[str] = frozenset({
  "FORD_EDGE_MK2",     # StarPilot
  "FORD_MONDEO_MK5",   # BP7, StarPilot
  "FORD_TRANSIT_MK5",  # StarPilot
})
# No pinion geometry row, so no STEER_ANGLE_CURVATURE (the setting no-ops and the yaw source is kept):
FORD_NO_PINION_PLATFORMS: frozenset[str] = frozenset({
  # ALT_STEER_ANGLE: reads a RELATIVE pinion angle (SteeringPinion_Data_Alt + learned offset) and lacks the
  # absolute measurement this needs
  "FORD_EDGE_MK2",
  # LKA steering through Lane_Assist_Data1 alone: no BluePilot lateral at all
  "FORD_TRANSIT_MK5",
})

FORD_PINION_GEOMETRY_SHIFT = 1
FORD_PINION_GEOMETRY_INDEX = {
  "FORD_BRONCO_SPORT_MK1": 1,
  "FORD_ESCAPE_MK4": 2,
  "FORD_ESCAPE_MK4_5": 3,
  "FORD_EXPEDITION_MK4": 4,
  "FORD_EXPLORER_MK6": 5,
  "FORD_FOCUS_MK4": 6,
  "FORD_F_150_LIGHTNING_MK1": 7,
  "FORD_F_150_MK14": 8,
  "FORD_MAVERICK_MK1": 9,
  "FORD_MONDEO_MK5": 10,
  "FORD_MUSTANG_MACH_E_MK1": 11,
  "FORD_RANGER_MK2": 12,
}


def ford_safety_param(CP: structs.CarParams, params_dict: dict[str, str]) -> int:
  """The startup safety bits for this car, from the startup settings (HOSTS.md). A host ORs them into the
  second safety param it sends the panda (current_safety_param_sp, USB 0xdf) at car init."""
  # Steering-angle curvature measurement (bad-yaw-sensor workaround): the STEER_ANGLE_CURVATURE flag +
  # the platform geometry-table index; the control side mirrors the same flag
  # (carcontroller_ext.InputTracker.get_current_curvature). Platforms without a geometry row silently
  # keep stock yaw behavior -- the toggle no-ops rather than half-configuring.
  bits = 0
  if CP.brand == 'ford':
    # BluePilot 4-signal lateral: tell the panda to run the BP checks. Read once here, at init;
    # the carcontroller sends 4-signal messages only when this bit is set. Changing the setting
    # while driving does not change the panda: stock messages are then judged by the BP
    # envelope (which is not identical to the stock one), so the setting takes full effect at
    # the next init. Same property as openpilot longitudinal.
    bp_lateral = not startup_value(params_dict, "disable_BP_lat_UI")
    if bp_lateral:
      bits |= FordSafetyFlagsSP.BP_LATERAL

    # The pinion-sourced measurement is a BP lateral feature: its wider error band and the
    # Python side's yaw-vs-pinion handling only line up on the BP path, so it is off whenever
    # BP lateral is off (ford_bp_init enforces the same on the panda).
    steer_angle_curvature = bp_lateral and startup_value(params_dict, "FordPrefSteerAngleCurvature")
    if steer_angle_curvature:
      geometry_index = FORD_PINION_GEOMETRY_INDEX.get(CP.carFingerprint)
      if geometry_index is not None:
        bits |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE | (geometry_index << FORD_PINION_GEOMETRY_SHIFT)
  return bits


# Pinion geometry wheelbase (m) where BluePilot deliberately differs from CarSpecs. Used by both the
# C table (ford_pinion_geometry in bluepilot_lateral/safety/ford.h) and the Python pinion path
# (carcontroller_ext.pinion_vehicle_model), so the two layers convert pinion angle the same way.
# FORD_F_150_MK14: the trucks with lane centering come in 3.68 and 3.99 m wheelbases and nothing on
# the car tells them apart; the average keeps either within about 4% (#21).
FORD_PINION_WHEELBASE = {
  "FORD_F_150_MK14": 3.84,
}


# BluePilot: Max curvature for steering command (m^-1), from DBC file limits
CURVATURE_MAX = 0.02

# BluePilot: Curvature rate limits — 3-point breakpoints for smoother lateral control.
# Upstream opendbc uses 2-point ([5, 25]) with more conservative values.
# These allow higher rates at low speed for responsiveness, lower rates at mid-speed
# for comfort, and very low rates at highway speed for stability.
#
# Control (Python) uses stricter windup than unwind so OP stays inside panda when apply_std
# picks the wrong table vs ford_bp_curvature_cmd_checks. Safety firmware is looser than both: its
# up table is the control's down table, and its down table is 1% looser again — see
# bluepilot_lateral/safety/ford.h FORD_LIMITS.
_BP_ANGLE_RATE_UP = ([5, 16, 25], [0.0025, 0.0012, 0.00008])
_BP_ANGLE_RATE_DOWN = ([5, 16, 25], [0.0025, 0.0014, 0.00018])
BP_ANGLE_LIMITS = AngleSteeringLimits(
  0.02,  # Max curvature for steering command, m^-1
  _BP_ANGLE_RATE_UP,
  _BP_ANGLE_RATE_DOWN,
)


# The model's prediction times, part of the host contract (host_types.py); the strategies read it from here.
MODEL_T_IDXS = host_types.MODEL_T_IDXS


# The per-frame settings a host publishes every frame (HOSTS.md; hosts import this name from here). The
# settings, their defaults and how they are read are in params/manifest.py.
BP_LATERAL_PARAMS = PER_FRAME_KEYS

# Lane change smoothing, both modes: speed breakpoints (m/s) and the low-speed factor
LANE_CHANGE_FACTOR_BP = (4.4, 40.23)
LANE_CHANGE_FACTOR_LOW = 0.95


# Angle mode's per-platform high-speed gain defaults (low curvature, high curvature); any other
# platform gets ANGLE_GAIN_CAN. Read by lateral_angle_ext.update_angle_params.
# CAN vehicles (Escape MK4, Bronco Sport, Explorer, Maverick, Edge)
ANGLE_GAIN_CAN = (1.00, 1.15)
# CAN-FD body-on-frame trucks (F-150, Lightning, Ranger)
ANGLE_GAIN_CANFD_BOF = (0.95, 0.95)
# Expedition MK4: its own base, 1.5x the body-on-frame values, as BluePilot 7 has since
# BluePilotDev/bluepilot#220 (an owner had FordHighSpeedFactor_ang at its 1.5 ceiling and wanted more).
# Provisional there too; kept so a truck moving from BP7 steers the same with the same settings.
ANGLE_GAIN_EXPEDITION = (ANGLE_GAIN_CANFD_BOF[0] * 1.5, ANGLE_GAIN_CANFD_BOF[1] * 1.5)
# CAN-FD unibody SUVs (Mustang Mach-E, Escape MK4.5)
ANGLE_GAIN_CANFD_SUV = (1.00, 1.05)

ANGLE_GAIN_CANFD_BOF_CARS = frozenset({
  "FORD_F_150_MK14",
  "FORD_F_150_LIGHTNING_MK1",
  "FORD_RANGER_MK2",
})
ANGLE_GAIN_EXPEDITION_CARS = frozenset({
  "FORD_EXPEDITION_MK4",
})
ANGLE_GAIN_CANFD_SUV_CARS = frozenset({
  "FORD_MUSTANG_MACH_E_MK1",
  "FORD_ESCAPE_MK4_5",
})
