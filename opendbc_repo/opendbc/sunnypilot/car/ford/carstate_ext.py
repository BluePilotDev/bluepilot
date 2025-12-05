"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.ford.values_ext import BUTTONS


class CarStateExt:
  """
  Extension class for Ford CarState to parse cruise control buttons.

  This class tracks button state transitions and emits ButtonEvent messages
  when buttons are pressed or released. This is required for ICBM and Speed
  Limit Assist to work correctly, as they need to track manual button presses
  to set vCruiseCluster.
  """

  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.button_events = []
    self.button_states = {button.event_type: False for button in BUTTONS}

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP, can_parsers: dict[StrEnum, CANParser]):
    """
    Update button state tracking and emit ButtonEvent messages.

    This handles combo buttons correctly - when a single CAN signal maps to multiple
    ButtonEvent types (e.g., CcAslButtnSetIncPress -> accelCruise + setCruise), both
    events will be emitted when the signal changes state.

    Args:
      ret: CarState structure to update
      ret_sp: CarStateSP structure (unused but required for interface compatibility)
      can_parsers: Dictionary of CAN parsers by bus
    """
    cp = can_parsers[Bus.pt]

    button_events = []
    for button in BUTTONS:
      # Check if button signal is in the pressed state (value == 1)
      # Note: Multiple Button entries can reference the same CAN signal (combo buttons)
      state = (cp.vl[button.can_addr][button.can_msg] in button.values)

      # Emit event on state transition (pressed or released)
      # Each ButtonEvent type is tracked separately, so combo buttons will emit
      # multiple events when the same CAN signal changes state
      if self.button_states[button.event_type] != state:
        event = structs.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)

      # Update stored state for this ButtonEvent type
      self.button_states[button.event_type] = state

    self.button_events = button_events

