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
      # Track which signals we've already processed to avoid duplicate events
      processed_signals = set()

      # CcAslButtnSetIncPress: setCruise (9) when disabled, accelCruise (3) when enabled
      if button.can_msg == "CcAslButtnSetIncPress" and button.can_msg not in processed_signals:
        processed_signals.add(button.can_msg)
        signal_state = state
        prev_accel_state = self.button_states.get(3, False)  # accelCruise
        prev_set_state = self.button_states.get(9, False)  # setCruise

        if signal_state and (prev_accel_state != signal_state or prev_set_state != signal_state):
          # Choose event type based on cruise state
          if cruise_enabled:
            event_type = 3  # accelCruise
          else:
            event_type = 9  # setCruise

          # Emit the appropriate event
          if self.button_states.get(event_type, False) != signal_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = event_type
            event.pressed = signal_state
            button_events.append(event)
            cloudlog.warning(f"Ford ButtonEvent: type={event_type}, pressed={signal_state}, signal={button.can_msg}={signal_value}, cruise_enabled={cruise_enabled}")

          # Update state for both event types
          self.button_states[3] = signal_state  # accelCruise
          self.button_states[9] = signal_state  # setCruise
        elif not signal_state and (prev_accel_state != signal_state or prev_set_state != signal_state):
          # Button released - emit release for the event type that was previously active
          if prev_accel_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = 3  # accelCruise
            event.pressed = False
            button_events.append(event)
          if prev_set_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = 9  # setCruise
            event.pressed = False
            button_events.append(event)
          self.button_states[3] = False  # accelCruise
          self.button_states[9] = False  # setCruise
        continue

      # CcAslButtnSetDecPress: setCruise (9) when disabled, decelCruise (4) when enabled
      if button.can_msg == "CcAslButtnSetDecPress" and button.can_msg not in processed_signals:
        processed_signals.add(button.can_msg)
        signal_state = state
        prev_decel_state = self.button_states.get(4, False)  # decelCruise
        prev_set_state = self.button_states.get(9, False)  # setCruise

        if signal_state and (prev_decel_state != signal_state or prev_set_state != signal_state):
          # Choose event type based on cruise state
          if cruise_enabled:
            event_type = 4  # decelCruise
          else:
            event_type = 9  # setCruise

          # Emit the appropriate event
          if self.button_states.get(event_type, False) != signal_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = event_type
            event.pressed = signal_state
            button_events.append(event)
            cloudlog.warning(f"Ford ButtonEvent: type={event_type}, pressed={signal_state}, signal={button.can_msg}={signal_value}, cruise_enabled={cruise_enabled}")

          # Update state for both event types
          self.button_states[4] = signal_state  # decelCruise
          self.button_states[9] = signal_state  # setCruise
        elif not signal_state and (prev_decel_state != signal_state or prev_set_state != signal_state):
          # Button released - emit release for the event type that was previously active
          if prev_decel_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = 4  # decelCruise
            event.pressed = False
            button_events.append(event)
          if prev_set_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = 9  # setCruise
            event.pressed = False
            button_events.append(event)
          self.button_states[4] = False  # decelCruise
          self.button_states[9] = False  # setCruise
        continue

      # CcAslButtnCnclResPress: cancel (5) when enabled, resumeCruise (10) when disabled
      if button.can_msg == "CcAslButtnCnclResPress" and button.can_msg not in processed_signals:
        processed_signals.add(button.can_msg)
        signal_state = state
        prev_cancel_state = self.button_states.get(5, False)  # cancel
        prev_resume_state = self.button_states.get(10, False)  # resumeCruise

        if signal_state and (prev_cancel_state != signal_state or prev_resume_state != signal_state):
          # Choose event type based on cruise state
          if cruise_enabled:
            event_type = 5  # cancel
          else:
            event_type = 10  # resumeCruise

          # Emit the appropriate event
          if self.button_states.get(event_type, False) != signal_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = event_type
            event.pressed = signal_state
            button_events.append(event)
            cloudlog.warning(f"Ford ButtonEvent: type={event_type}, pressed={signal_state}, signal={button.can_msg}={signal_value}, cruise_enabled={cruise_enabled}")

          # Update state for both event types
          self.button_states[5] = signal_state  # cancel
          self.button_states[10] = signal_state  # resumeCruise
        elif not signal_state and (prev_cancel_state != signal_state or prev_resume_state != signal_state):
          # Button released - emit release for the event type that was previously active
          if prev_cancel_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = 5  # cancel
            event.pressed = False
            button_events.append(event)
          if prev_resume_state:
            event = structs.CarState.ButtonEvent.new_message()
            event.type = 10  # resumeCruise
            event.pressed = False
            button_events.append(event)
          self.button_states[5] = False  # cancel
          self.button_states[10] = False  # resumeCruise
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

