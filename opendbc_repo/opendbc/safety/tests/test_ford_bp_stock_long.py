#!/usr/bin/env python3
# BluePilot (bp-full): a CAN Ford with the BluePilot stock-ACC bit runs the car's own ACC: no ACCDATA
# from openpilot, and the camera's ACCDATA is forwarded to the car. Without the bit a CAN Ford is
# upstream's (TestFordLongitudinalSafety in test_ford.py).
import unittest

import opendbc.safety.tests.test_ford as ford
from opendbc.sunnypilot.car.ford.bluepilot.interfaces_ext import FORD_SAFETY_BP_STOCK_LONG


class TestFordCANStockSafety(ford.TestFordSafetyBase):
  SAFETY_PARAM = FORD_SAFETY_BP_STOCK_LONG

  STEER_MESSAGE = ford.MSG_LateralMotionControl

  TX_MSGS = [
    [ford.MSG_Steering_Data_FD1, 0], [ford.MSG_Steering_Data_FD1, 2], [ford.MSG_ACCDATA_3, 0], [ford.MSG_Lane_Assist_Data1, 0],
    [ford.MSG_LateralMotionControl, 0], [ford.MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (ford.MSG_ACCDATA_3, ford.MSG_Lane_Assist_Data1, ford.MSG_LateralMotionControl,
                                 ford.MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [ford.MSG_ACCDATA_3, ford.MSG_Lane_Assist_Data1, ford.MSG_LateralMotionControl,
                               ford.MSG_IPMA_Data]}

  def test_no_accdata(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("ACCDATA", 0, {})))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, ford.MSG_ACCDATA))  # the camera's ACC reaches the car


if __name__ == "__main__":
  unittest.main()
