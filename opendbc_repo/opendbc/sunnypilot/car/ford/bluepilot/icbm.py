"""BluePilot (bp-full): Ford ICBM (Intelligent Cruise Button Management), from BluePilot 7.

While openpilot asks for a set-speed change (CC_SP.intelligentCruiseButtonManagement.sendButton),
press Set+ or Set- on Steering_Data_FD1 on the camera and main buses, the way cancel/resume are sent.
Stock ACC only: upstream turns ICBM off with openpilot longitudinal. The panda allows these presses
on both buses (ford.h checks only cancel and resume).

Called from CarController.update() in opendbc/car/ford/carcontroller.py.
"""
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.ford import fordcan

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# the Steering_Data_FD1 signal each request presses
BUTTON_SIGNALS = {
  SendButtonState.increase: "CcAslButtnSetIncPress",
  SendButtonState.decrease: "CcAslButtnSetDecPress",
}
ICBM_STEP = 6  # frames between presses (about 17 Hz): BP7's (frame - last press) * DT_CTRL > 0.05


class IcbmExt:
  def __init__(self):
    self.last_button_frame = 0

  def update(self, CC_SP, CS, packer, CAN, frame: int) -> list[CanData]:
    button = CC_SP.intelligentCruiseButtonManagement.sendButton
    if button == SendButtonState.none or frame - self.last_button_frame < ICBM_STEP:
      return []

    # upstream's builder passes the stock values through: hand it the stock values with the button pressed
    stock = {**CS.buttons_stock_values, BUTTON_SIGNALS[button]: 1}
    self.last_button_frame = frame
    return [fordcan.create_button_msg(packer, CAN.camera, stock),
            fordcan.create_button_msg(packer, CAN.main, stock)]
