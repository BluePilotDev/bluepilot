"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections import namedtuple

from opendbc.car import structs
from opendbc.car.docs_definitions import CarParts, Device
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


# BluePilot: Max curvature for steering command (m^-1), from DBC file limits
CURVATURE_MAX = 0.02

# BluePilot: Curvature rate limits — 3-point breakpoints for smoother lateral control.
# Upstream sunnypilot uses 2-point ([5, 25]) with more conservative values.
# These allow higher rates at low speed for responsiveness, lower rates at mid-speed
# for comfort, and very low rates at highway speed for stability.
BP_ANGLE_LIMITS = AngleSteeringLimits(
  0.02,  # Max curvature for steering command, m^-1
  ([5, 16, 25], [0.0025, 0.0012, 0.00008]),   # up (windup) rate limits
  ([5, 16, 25], [0.0025, 0.0014, 0.00018]),   # down (unwind) rate limits
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

