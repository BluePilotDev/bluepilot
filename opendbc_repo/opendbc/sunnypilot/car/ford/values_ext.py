"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections import namedtuple

from opendbc.car import structs
from opendbc.car.docs_definitions import CarParts, Device
from opendbc.car.ford.values import CAR
from opendbc.car.lateral import AngleSteeringLimits

ButtonType = structs.CarState.ButtonEvent.Type
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])

# Ford cruise control buttons are in the Steering_Data_FD1 message (CAN ID 131)
# These signals are 1-bit flags: 1 = pressed, 0 = not pressed
#
# Note: Some buttons are combo buttons that emit multiple ButtonEvent types:
# - CcAslButtnSetIncPress emits both accelCruise (type 3) and setCruise (type 9)
# - CcAslButtnCnclResPress emits both cancel (type 5) and resumeCruise (type 10)
#
# There is also a separate CcAslButtnSetPress signal for the standalone "Set" button,
# but based on user mapping, setCruise is mapped to the combo button instead.
BUTTONS = [
  # Combo button: Set + Increase (emits accelCruise when enabled, setCruise when disabled)
  Button(ButtonType.accelCruise, "Steering_Data_FD1", "CcAslButtnSetIncPress", [1]),
  Button(ButtonType.setCruise, "Steering_Data_FD1", "CcAslButtnSetIncPress", [1]),

  # Combo button: Set + Decrease (emits decelCruise when enabled, setCruise when disabled)
  Button(ButtonType.decelCruise, "Steering_Data_FD1", "CcAslButtnSetDecPress", [1]),
  Button(ButtonType.setCruise, "Steering_Data_FD1", "CcAslButtnSetDecPress", [1]),

  # Combo button: Cancel/Resume (emits cancel when enabled, resumeCruise when disabled)
  Button(ButtonType.cancel, "Steering_Data_FD1", "CcAslButtnCnclResPress", [1]),
  Button(ButtonType.resumeCruise, "Steering_Data_FD1", "CcAslButtnCnclResPress", [1]),

  # Main cruise button (on/off toggle)
  Button(ButtonType.mainCruise, "Steering_Data_FD1", "CcButtnOnOffPress", [1]),
]


class FordSafetyFlagsSP:
  """Sunnypilot-level safety flags for Ford.

  Carried in CP_SP.safetyParam and delivered to the safety firmware as
  current_safety_param_sp (the separate SP uint16, USB control 0xdf) -- NOT the main
  safetyConfigs[].safetyParam. ford_init reads it with GET_FLAG(current_safety_param_sp,
  ...), same pattern as Subaru STOP_AND_GO (subaru_common.h). Plain int constants, not
  IntFlag: CP_SP.safetyParam must stay a plain int through capnp serialization in card.
  """
  STEER_ANGLE_CURVATURE = 1


# Geometry-table index for the steering-angle curvature measurement, packed into
# CP_SP.safetyParam bits 1-4 when STEER_ANGLE_CURVATURE is set. Must match the
# ford_pinion_geometry table in safety/modes/ford.h row for row (enforced by
# test_ford.py's geometry-consistency test against CarSpecs + calc_slip_factor).
# Index 0 is reserved as invalid: the firmware treats flag-set-but-no-index as feature
# off, so a half-configured param can never select the wrong geometry silently.
# FORD_EDGE_MK2 is deliberately absent: ALT_STEER_ANGLE platforms read a RELATIVE pinion
# angle (SteeringPinion_Data_Alt + learned offset) and lack the absolute measurement
# this feature needs -- the toggle no-ops there and yaw behavior is kept.
FORD_PINION_GEOMETRY_SHIFT = 1
FORD_PINION_GEOMETRY_INDEX = {
  CAR.FORD_BRONCO_SPORT_MK1: 1,
  CAR.FORD_ESCAPE_MK4: 2,
  CAR.FORD_ESCAPE_MK4_5: 3,
  CAR.FORD_EXPEDITION_MK4: 4,
  CAR.FORD_EXPLORER_MK6: 5,
  CAR.FORD_FOCUS_MK4: 6,
  CAR.FORD_F_150_LIGHTNING_MK1: 7,
  CAR.FORD_F_150_MK14: 8,
  CAR.FORD_MAVERICK_MK1: 9,
  CAR.FORD_MONDEO_MK5: 10,
  CAR.FORD_MUSTANG_MACH_E_MK1: 11,
  CAR.FORD_RANGER_MK2: 12,
}


# BluePilot: Max curvature for steering command (m^-1), from DBC file limits
CURVATURE_MAX = 0.02

# BluePilot: Curvature rate limits — 3-point breakpoints for smoother lateral control.
# Upstream opendbc uses 2-point ([5, 25]) with more conservative values.
# These allow higher rates at low speed for responsiveness, lower rates at mid-speed
# for comfort, and very low rates at highway speed for stability.
#
# Control (Python) uses stricter windup than unwind so OP stays inside panda when apply_std
# picks the wrong table vs steer_angle_cmd_checks. Safety firmware uses looser symmetric ROCs
# (former “down” table for both up/down) — see ford.h FORD_LIMITS.
# Tests: test_ford.py ANGLE_RATE_* match ford.h, not the stricter BP_ANGLE_LIMITS up row.
_BP_ANGLE_RATE_UP = ([5, 16, 25], [0.0025, 0.0012, 0.00008])
_BP_ANGLE_RATE_DOWN = ([5, 16, 25], [0.0025, 0.0014, 0.00018])
BP_ANGLE_LIMITS = AngleSteeringLimits(
  0.02,  # Max curvature for steering command, m^-1
  _BP_ANGLE_RATE_UP,
  _BP_ANGLE_RATE_DOWN,
)


def apply_bp_device_mount(car_docs, CP):
  """BluePilot: Select comma3 mount type per vehicle.

  Most Ford vehicles use the angled mount due to windshield angle.
  Vehicles not in this list use the standard mount.
  """
  from opendbc.car.ford.values import CAR, CarHarness, FordFlags
  harness = CarHarness.ford_q4 if CP.flags & FordFlags.CANFD else CarHarness.ford_q3
  if CP.carFingerprint in (
    CAR.FORD_BRONCO_SPORT_MK1,
    CAR.FORD_MAVERICK_MK1,
    CAR.FORD_F_150_MK14,
    CAR.FORD_F_150_LIGHTNING_MK1,
    CAR.FORD_ESCAPE_MK4_5,
    CAR.FORD_MUSTANG_MACH_E_MK1,
    CAR.FORD_RANGER_MK2,
    CAR.FORD_EDGE_MK2,
  ):
    car_docs.car_parts = CarParts([Device.threex_angled_mount, harness])
  else:
    car_docs.car_parts = CarParts([Device.threex, harness])



# ---------------------------------------------------------------------------------------
# Angle-mode gain model — owned by the strategy (lateral_angle_ext), consumed by the
# auto-calibrator (angle_autocal) and the offline analyzer. Single source of truth here
# so the strategy never imports its own gain model back out of the calibrator.
#
# Hard-coded per-platform gain defaults (not user-tunable):
GAIN_CAN = (1.00, 1.15)        # CAN vehicles (Escape MK4, Bronco Sport, Explorer, Maverick, Edge)
GAIN_CANFD_BOF = (0.95, 0.95)  # CAN-FD body-on-frame trucks (F-150, Lightning, Expedition, Ranger)
GAIN_CANFD_SUV = (1.00, 1.05)  # CAN-FD unibody SUVs (Mustang Mach-E, Escape MK4.5)

CANFD_BOF_CARS = frozenset({
  CAR.FORD_F_150_MK14,
  CAR.FORD_F_150_LIGHTNING_MK1,
  CAR.FORD_EXPEDITION_MK4,
  CAR.FORD_RANGER_MK2,
})
CANFD_SUV_CARS = frozenset({
  CAR.FORD_MUSTANG_MACH_E_MK1,
  CAR.FORD_ESCAPE_MK4_5,
})


def platform_gains(fingerprint: str) -> tuple[float, float]:
  """(lowC_highV, highC_highV) platform gain pair for a car fingerprint."""
  if fingerprint in CANFD_BOF_CARS:
    return GAIN_CANFD_BOF
  if fingerprint in CANFD_SUV_CARS:
    return GAIN_CANFD_SUV
  return GAIN_CAN


# Speed anchors of the strategy's gain interpolation (m/s: ~30 mph and ~60 mph), and the
# fixed multiplier on the low-speed anchor. The auto-calibrator's fit is expressed
# against these — if the strategy's interp changes, they must move together.
V_LOW = 13.5
V_HIGH = 26.82
LOW_ANCHOR_BASE = 1.30
