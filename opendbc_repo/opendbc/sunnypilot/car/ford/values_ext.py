"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from collections import namedtuple

from opendbc.car import structs

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

