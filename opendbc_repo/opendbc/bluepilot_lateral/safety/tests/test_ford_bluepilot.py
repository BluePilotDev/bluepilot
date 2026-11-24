#!/usr/bin/env python3
"""
BluePilot: the BP_LATERAL safety-param bit selects which Ford lateral checks the panda runs.

This pins the switch itself, not the BluePilot limits: with the bit set a modest non-zero
path_angle is accepted, and with it clear (upstream's stock checks) the same frame is rejected.
The BluePilot limits are covered separately.
"""
import unittest

from ....car.structs import CarParams
from ....car.ford.values import FordSafetyFlags
from ...core.values_ext import FordSafetyFlagsSP
from ....safety.tests.libsafety import libsafety_py
from ....safety.tests.common import CANPackerSafety
from ....safety.tests import test_ford  # module import: keeps its test classes out of this module's discovery
from ...tests import host

SPEED = 15.0             # m/s, above the curvature-error gate, inside the path_angle ROC table
SMALL_PATH_ANGLE = 0.01  # rad, well inside the per-call ROC at this speed (~0.043)
LARGE_PATH_ANGLE = 0.30  # rad, outside both the ROC step and the ±0.25 curvature-mode range


class TestFordBPLateralBit(unittest.TestCase):
  """Uses upstream's CAN FD stock test class only for its message helpers; its tests are not inherited."""

  def setUp(self):
    self.h = test_ford.TestFordCANFDStockSafety("test_steer_allowed")
    self.h.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.h.safety = libsafety_py.libsafety

  def tearDown(self):
    host.set_safety_param(self.h.safety, 0)
    super().tearDown()

  def _init(self, param_sp: int):
    h = self.h
    host.set_safety_param(h.safety, param_sp)
    h.safety.set_safety_hooks(CarParams.SafetyModel.ford, host.ford_param(FordSafetyFlags.CANFD, param_sp))
    h.safety.init_tests()
    h.safety.set_controls_allowed(True)
    h._reset_curvature_measurement(0, SPEED)
    h._set_prev_desired_angle(0.001)

  def test_bit_set_runs_bluepilot_checks(self):
    self._init(FordSafetyFlagsSP.BP_LATERAL)
    self.assertTrue(self.h._tx(self.h._lat_ctl_msg(True, 0, SMALL_PATH_ANGLE, 0.001, 0)))
    self.assertFalse(self.h._tx(self.h._lat_ctl_msg(True, 0, LARGE_PATH_ANGLE, 0.001, 0)))

  def test_bit_clear_runs_stock_checks(self):
    self._init(0)
    # upstream: any non-zero path_angle is a violation
    self.assertFalse(self.h._tx(self.h._lat_ctl_msg(True, 0, SMALL_PATH_ANGLE, 0.001, 0)))
    self.assertTrue(self.h._tx(self.h._lat_ctl_msg(True, 0, 0, 0.001, 0)))


if __name__ == "__main__":
  unittest.main()
