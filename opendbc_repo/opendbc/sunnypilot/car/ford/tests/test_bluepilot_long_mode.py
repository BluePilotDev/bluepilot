"""BluePilot (bp-full): the alpha toggle decides openpilot longitudinal vs stock ACC on every Ford,
and a CAN FD Ford gets Steer_Assist_Data as its radar."""
import unittest

from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.ford.interface import CarInterface
from opendbc.car.ford.values import CAR, DBC, FordFlags, FordSafetyFlags, RADAR
from opendbc.sunnypilot.car.ford.bluepilot.interfaces_ext import FORD_SAFETY_BP_STOCK_LONG


class TestFordLongMode(unittest.TestCase):
  def test_every_platform(self):
    for car in CAR:
      for alpha_long in (False, True):
        with self.subTest(car=car, alpha_long=alpha_long):
          CP = CarInterface.get_params(car, gen_empty_fingerprint(), [], alpha_long, False, False)
          param = CP.safetyConfigs[-1].safetyParam
          canfd = bool(CP.flags & FordFlags.CANFD)

          self.assertTrue(CP.alphaLongitudinalAvailable)
          self.assertEqual(CP.openpilotLongitudinalControl, alpha_long)
          self.assertEqual(bool(param & FordSafetyFlags.LONG_CONTROL), alpha_long)
          # the stock-ACC bit only ever restricts a CAN Ford with the toggle off
          self.assertEqual(bool(param & FORD_SAFETY_BP_STOCK_LONG), not canfd and not alpha_long)

          self.assertFalse(CP.radarUnavailable)
          if canfd:
            self.assertEqual(DBC[car][Bus.radar], RADAR.STEER_ASSIST_DATA)


if __name__ == '__main__':
  unittest.main()
