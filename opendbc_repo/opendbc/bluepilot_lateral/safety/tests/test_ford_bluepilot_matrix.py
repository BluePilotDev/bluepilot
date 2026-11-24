#!/usr/bin/env python3
"""
BluePilot Ford lateral: upstream's whole Ford safety matrix with the BP_LATERAL bit set.

Subclasses the three stock test classes so relay-malfunction, forwarding, button, longitudinal,
rx-hook and every other upstream test runs against the BluePilot tx hook. Only the tests whose
stock assertions the 4-signal lateral changes on purpose are replaced:

  * test_steer_allowed, test_curvature_rate_limits: assert upstream's single-signal limits; the
    BluePilot equivalents are in test_ford_bluepilot_limits.py (every signal, exact boundaries).
  * test_max_lateral_acceleration: BluePilot caps at ISO_LATERAL_ACCEL - g*roll (~2.41 m/s^2)
    with a speed fudge, not upstream's ~3.6 m/s^2; CAN FD only.
  * test_rt_limits runs unchanged: it sends curvature-0 frames, which still get the message-rate
    check in ford_bp_curvature_cmd_checks (#24, #60). test_rt_limits_nonzero_curvature pins it for
    curvature mode.
"""
import unittest

from ....safety.tests import common
from ...core.values_ext import FordSafetyFlagsSP
from ....safety.tests import test_ford
from ....safety.tests.libsafety import libsafety_py
from .ford_bluepilot_common import (
  BPFordLateralModel, CURVATURE_TO_CAN, MAX_CURVATURE_ERROR_CAN, CURVATURE_SIGNAL_MAX_CAN,
)
from ...tests import host

SKIP_REASON = "BluePilot 4-signal limits are asserted in test_ford_bluepilot_limits.py"


class BPLateralMatrixMixin:
  """Set the bit before ford_init runs, clear it after, and swap the host's stock lateral tests
  (skipped below, or replaced).

  Class names must start with 'TestFord': common.py's cross-mode TX sweep only skips
  Ford-vs-Ford comparisons by that prefix."""

  CANFD = True
  PARAM_SP = int(FordSafetyFlagsSP.BP_LATERAL)
  MAX_CURVATURE_ERROR_CAN_BP = MAX_CURVATURE_ERROR_CAN  # FORD_LIMITS(_, 100); the pinion classes use MAX_CURVATURE_ERROR_CAN_PINION

  def setUp(self):
    self.safety = libsafety_py.libsafety
    host.set_safety_param(self.safety, self.PARAM_SP)
    # StarPilot: the bits ride the Ford param. Only the upstream bases that define SAFETY_PARAM
    # (sunnypilot, StarPilot) need this; IQ.Pilot's iqdbc base doesn't define it, and ford_param
    # is a no-op for non-StarPilot hosts anyway.
    if hasattr(self, "SAFETY_PARAM"):
      self.SAFETY_PARAM = host.ford_param(self.SAFETY_PARAM, self.PARAM_SP)
    super().setUp()  # stock setUp: packer, set_safety_hooks, init_tests
    self.model = BPFordLateralModel(self.CANFD, self.MAX_CURVATURE_ERROR_CAN_BP)

  def tearDown(self):
    host.set_safety_param(self.safety, 0)
    super().tearDown()  # the host's own resets (IQ.Pilot's AOL state)

  @unittest.skip(SKIP_REASON)
  def test_steer_allowed(self):
    pass

  @unittest.skip(SKIP_REASON)
  def test_curvature_rate_limits(self):
    pass

  # IQ.Pilot's test_ford: its stock controller (CURVATURE_LIMITS.apply_limits) against the stock checks.
  # With BP_LATERAL set the car controller sends the BluePilot commands instead, never these.
  @unittest.skip(SKIP_REASON)
  def test_controller_limits_pass_safety(self):
    pass

  # IQ.Pilot's test_ford (from beta b589c894, upstream's error-band rework): the stock error band's relaxed
  # wind-down, and the stock controller against it with a measured-curvature offset. BluePilot's band is its
  # own (test_ford_bluepilot_limits: test_curvature_error_band_convergence).
  @unittest.skip(SKIP_REASON)
  def test_curvature_error_relaxed_rate_limits(self):
    pass

  @unittest.skip(SKIP_REASON)
  def test_controller_limits_pass_safety_with_curvature_offset(self):
    pass

  def test_max_lateral_acceleration(self):
    """BluePilot's cap (CAN FD only) at ISO_LATERAL_ACCEL - g*roll over the fudged speed squared."""
    for speed in (12.0, 15.0, 20.0, 25.0, 30.0, 35.0):
      self.safety.set_controls_allowed(True)
      self._reset_curvature_measurement(0, speed)
      v_min = float(self.safety.get_vehicle_speed_min())
      cap = self.model.cap(v_min)
      for sign in (1, -1):
        for offset in (-5, -1, 0, 1, 5):
          curvature_can = sign * (cap + offset)
          if abs(curvature_can) > CURVATURE_SIGNAL_MAX_CAN:
            continue
          self._reset_curvature_measurement(curvature_can / CURVATURE_TO_CAN, speed)
          self._set_prev_desired_angle(curvature_can / CURVATURE_TO_CAN)
          should_tx = abs(curvature_can) <= cap
          with self.subTest(speed=speed, curvature_can=curvature_can):
            self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, curvature_can / CURVATURE_TO_CAN, 0)))

  def test_rt_limits_nonzero_curvature(self):
    """Upstream's rolling-window message-rate check, driven with a non-zero curvature (curvature mode)."""
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 0)
    curvature = 1 / CURVATURE_TO_CAN
    max_rt_msgs = int(self.LATERAL_FREQUENCY * common.RT_INTERVAL / 1e6 * 1.2 + 1)
    half = common.RT_INTERVAL // 2

    def send():
      self._set_prev_desired_angle(curvature)
      return self._tx(self._lat_ctl_msg(True, 0, 0, curvature, 0, increment_timer=False))

    self.safety.set_timer(0)
    for i in range(max_rt_msgs * 2):
      self.assertEqual(i <= max_rt_msgs, send())
    self.safety.set_timer(half)
    self.assertFalse(send())
    self.safety.set_timer(half + 2 * common.RT_INTERVAL // 5)
    self.assertFalse(send())
    self.assertFalse(send())
    self.safety.set_timer(half + common.RT_INTERVAL)
    self.assertFalse(send())
    self.safety.set_timer(half + 2 * common.RT_INTERVAL)
    for _ in range(max_rt_msgs):
      self.assertTrue(send())


class TestFordBPCANFDStockSafety(BPLateralMatrixMixin, test_ford.TestFordCANFDStockSafety):
  CANFD = True


class TestFordBPCANFDLongitudinalSafety(BPLateralMatrixMixin, test_ford.TestFordCANFDLongitudinalSafety):
  CANFD = True


class TestFordBPLongitudinalSafety(BPLateralMatrixMixin, test_ford.TestFordLongitudinalSafety):
  CANFD = False


if __name__ == "__main__":
  unittest.main()
