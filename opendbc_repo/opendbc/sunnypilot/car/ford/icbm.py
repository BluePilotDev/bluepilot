"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford ICBM (Intelligent Cruise Button Management) implementation.
"""

from opendbc.car import structs, DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.ford import fordcan
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import IntelligentCruiseButtonManagementInterfaceBase
from openpilot.common.swaglog import cloudlog

ButtonType = structs.CarState.ButtonEvent.Type
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# Ford cruise control button signals for speed adjustment
# These correspond to the signals in the Steering_Data_FD1 CAN message (ID 131)
BUTTON_SIGNALS = {
  SendButtonState.increase: "CcAslButtnSetIncPress",  # Set + Increase button (speed up)
  SendButtonState.decrease: "CcAslButtnSetDecPress",  # Set + Decrease button (speed down)
}


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC_SP, CS, packer, CAN, frame, last_button_frame) -> tuple[list[CanData], int]:
    """
    Update ICBM state and generate button press messages.

    Args:
      CC_SP: CarControlSP structure with ICBM commands
      CS: CarState with stock button values
      packer: CAN message packer
      CAN: Ford CAN bus configuration
      frame: Current frame number
      last_button_frame: Frame number of last button press

    Returns:
      Tuple of (can_sends list, updated last_button_frame)
    """
    can_sends = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    # loudlog.warning(f"XXXXXXXXXXXXXXXXXXX ICBM: {self.ICBM.sendButton}")
    if self.ICBM.sendButton != SendButtonState.none:
      button_signal = BUTTON_SIGNALS[self.ICBM.sendButton]

      # Ford sends button messages at 10Hz (every 0.1s), but we send at 20Hz (every 0.05s) per CarControllerParams.BUTTONS_STEP
      # Only send if enough time has passed since last button press
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.05:
        cloudlog.warning(f"XXXXXXXXXXXXXXXXXXX Sending button press: {button_signal}")
        # Send button press to both camera and main bus (same as cancel/resume)
        can_sends.append(fordcan.create_button_msg(packer, CAN.camera, CS.buttons_stock_values,
                                                     icbm_button=button_signal))
        can_sends.append(fordcan.create_button_msg(packer, CAN.main, CS.buttons_stock_values,
                                                     icbm_button=button_signal))
        self.last_button_frame = self.frame

    return can_sends, self.last_button_frame
