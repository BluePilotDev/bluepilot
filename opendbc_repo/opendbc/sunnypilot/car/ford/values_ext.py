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
BUTTONS = [
  Button(ButtonType.accelCruise, "Steering_Data_FD1", "CcAslButtnSetIncPress", [1]),
  Button(ButtonType.decelCruise, "Steering_Data_FD1", "CcAslButtnSetDecPress", [1]),
  Button(ButtonType.cancel, "Steering_Data_FD1", "CcAslButtnCnclPress", [1]),
  Button(ButtonType.resumeCruise, "Steering_Data_FD1", "CcAsllButtnResPress", [1]),
]

