#!/usr/bin/env python3
# BluePilot (bp-full): openpilot's Set+/Set- presses (ICBM, sunnypilot/car/ford/bluepilot/icbm.py) only while
# controls are allowed, like resume: on a Ford with ACC on but not engaged, Set engages ACC at the current
# speed. Tested on the two stock-ACC configurations, where ICBM runs.
import unittest

import opendbc.safety.tests.test_ford as ford
import opendbc.safety.tests.test_ford_bp_stock_long as bp_stock_long

SET_BUTTONS = ("CcAslButtnSetIncPress", "CcAslButtnSetDecPress")


class SetButtonsMixin:
  def test_set_buttons_need_controls_allowed(self):
    for allowed in (False, True):
      self.safety.set_controls_allowed(allowed)
      for button in SET_BUTTONS:
        for bus in (0, 2):
          with self.subTest(allowed=allowed, button=button, bus=bus):
            self.assertEqual(allowed, self._tx(self.packer.make_can_msg_safety("Steering_Data_FD1", bus, {button: 1})))


class TestFordCANFDStockSetButtons(SetButtonsMixin, ford.TestFordCANFDStockSafety):
  pass


class TestFordCANStockSetButtons(SetButtonsMixin, bp_stock_long.TestFordCANStockSafety):
  pass


if __name__ == "__main__":
  unittest.main()
