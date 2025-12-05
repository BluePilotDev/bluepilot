"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.sunnypilot.car.ford.values_ext import BUTTONS
from openpilot.common.swaglog import cloudlog


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

    For combo buttons, we emit the appropriate event based on cruise state:
    - CcAslButtnSetIncPress: emits setCruise if cruise disabled, accelCruise if enabled
    - CcAslButtnCnclResPress: emits cancel if cruise enabled, resumeCruise if disabled

    Args:
      ret: CarState structure to update
      ret_sp: CarStateSP structure (unused but required for interface compatibility)
      can_parsers: Dictionary of CAN parsers by bus
    """
    cp = can_parsers[Bus.pt]

    button_events = []
    cruise_enabled = ret.cruiseState.enabled

    for button in BUTTONS:
      # Check if button signal is in the pressed state (value == 1)
      try:
        signal_value = cp.vl[button.can_addr][button.can_msg]
        state = (signal_value in button.values)
      except (KeyError, AttributeError):
        # Signal not available in this frame, skip
        continue

      # Handle combo buttons: emit only the appropriate event based on cruise state
      # CcAslButtnSetIncPress: setCruise (9) when disabled, accelCruise (3) when enabled
      if button.can_msg == "CcAslButtnSetIncPress":
        if state and self.button_states.get(button.event_type, False) != state:
          # Choose event type based on cruise state
          if cruise_enabled:
            # Cruise is enabled, so this is an increase request
            event_type = 3  # accelCruise
          else:
            # Cruise is disabled, so this is a set request
            event_type = 9  # setCruise

          # Only emit if this is the correct event type for current state
          if button.event_type == event_type:
            if self.button_states.get(button.event_type, False) != state:
              event = structs.CarState.ButtonEvent.new_message()
              event.type = button.event_type
              event.pressed = state
              button_events.append(event)
              cloudlog.warning(f"Ford ButtonEvent: type={button.event_type}, pressed={state}, signal={button.can_msg}={signal_value}, cruise_enabled={cruise_enabled}")
          # Update state for both event types to track transitions
          self.button_states[button.event_type] = state
        elif not state:
          # Button released - update state for both event types
          if self.button_states.get(button.event_type, False) != state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = button.event_type
            event.pressed = state
            button_events.append(event)
          self.button_states[button.event_type] = state
        continue

      # CcAslButtnCnclResPress: cancel (5) when enabled, resumeCruise (10) when disabled
      if button.can_msg == "CcAslButtnCnclResPress":
        if state and self.button_states.get(button.event_type, False) != state:
          # Choose event type based on cruise state
          if cruise_enabled:
            # Cruise is enabled, so this is a cancel request
            event_type = 5  # cancel
          else:
            # Cruise is disabled, so this is a resume request
            event_type = 10  # resumeCruise

          # Only emit if this is the correct event type for current state
          if button.event_type == event_type:
            if self.button_states.get(button.event_type, False) != state:
              event = structs.CarState.ButtonEvent.new_message()
              event.type = button.event_type
              event.pressed = state
              button_events.append(event)
              cloudlog.warning(f"Ford ButtonEvent: type={button.event_type}, pressed={state}, signal={button.can_msg}={signal_value}, cruise_enabled={cruise_enabled}")
          # Update state for both event types to track transitions
          self.button_states[button.event_type] = state
        elif not state:
          # Button released - update state for both event types
          if self.button_states.get(button.event_type, False) != state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = button.event_type
            event.pressed = state
            button_events.append(event)
          self.button_states[button.event_type] = state
        continue

      # Regular buttons (non-combo): emit event on state transition
      if self.button_states.get(button.event_type, False) != state:
        event = structs.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        button_events.append(event)
        # Debug: log button events
        if button.event_type in (3, 9, 5, 10):  # accelCruise, setCruise, cancel, resumeCruise
          cloudlog.warning(f"Ford ButtonEvent: type={button.event_type}, pressed={state}, signal={button.can_msg}={signal_value}")

      # Update stored state for this ButtonEvent type
      self.button_states[button.event_type] = state

    self.button_events = button_events

