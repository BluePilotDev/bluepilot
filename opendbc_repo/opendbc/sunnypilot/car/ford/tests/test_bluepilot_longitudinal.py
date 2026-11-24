"""BluePilot (bp-full): Ford highway following (sunnypilot/car/ford/bluepilot/longitudinal_ext.py)."""
import unittest
from types import SimpleNamespace as NS

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.bluepilot.longitudinal_ext import LongitudinalExt

MPH = 0.44704


def cc_sp(lead=None, **params):
  lead_ns = NS(**{"status": lead is not None, "dRel": 0.0, "vRel": 0.0, "vLead": 0.0, **(lead or {})})
  return NS(leadOne=lead_ns, params=[NS(key=k, value=v) for k, v in params.items()])


def cs(mph, gas=False, brake=False):
  return NS(out=NS(vEgo=mph * MPH, gasPressed=gas, brakePressed=brake))


ACTIVE = NS(longActive=True)


class TestLongitudinalExt(unittest.TestCase):
  def setUp(self):
    self.ext = LongitudinalExt()

  def run_at(self, mph, op_accel, op_gas, lead=None, pitch=0.0, frames=1, **params):
    for _ in range(frames):
      out = self.ext.update(ACTIVE, cc_sp(lead, **params), cs(mph), op_accel, op_gas, pitch)
    return out

  def test_below_50_mph_is_upstream_accel(self):
    accel, gas, brake, precharge = self.run_at(40, -1.0, CarControllerParams.INACTIVE_GAS)
    self.assertEqual(accel, -1.0)
    self.assertTrue(brake and precharge)

  def test_speed_deadband(self):
    self.run_at(51, 0.5, 0.5)
    self.assertTrue(self.ext.bp_speed_allow)
    self.run_at(47, 0.5, 0.5)
    self.assertTrue(self.ext.bp_speed_allow)  # stays on until below 45
    self.run_at(44, 0.5, 0.5)
    self.assertFalse(self.ext.bp_speed_allow)

  def test_gaining_close_cuts_gas(self):
    lead = dict(dRel=30.0, vRel=-2.0, vLead=28.0)  # 1.2 s at 55 mph, closing
    accel, gas, _, _ = self.run_at(55, 0.3, 0.3, lead)
    self.assertEqual(gas, 0.0)

  def test_gaining_gas_cut_ends_at_1_5_s(self):
    # 55 mph = 24.5872 m/s: 35.65 m is 1.45 s, 36.8808 m exactly 1.5 s, 38.11 m 1.55 s
    self.assertEqual(36.8808 / cs(55).out.vEgo, 1.5)  # else the exact-boundary case goes soft
    for d_rel, expected_gas in ((35.65, 0.0), (36.8808, 0.3), (38.11, 0.3)):
      with self.subTest(d_rel=d_rel):
        self.ext = LongitudinalExt()
        _, gas, _, _ = self.run_at(55, 0.3, 0.3, dict(dRel=d_rel, vRel=-2.0, vLead=28.0))
        self.assertEqual(gas, expected_gas)

  def test_gaining_starts_below_minus_0_1(self):
    # 1.2 s gap: gaining cuts gas to 0, pacing caps it at 0.2
    for v_rel, expected_gas in ((-0.15, 0.0), (-0.1, 0.2)):
      with self.subTest(v_rel=v_rel):
        self.ext = LongitudinalExt()
        _, gas, _, _ = self.run_at(55, 0.3, 0.3, dict(dRel=30.0, vRel=v_rel, vLead=28.0))
        self.assertAlmostEqual(gas, expected_gas)

  def test_pacing_ends_at_plus_0_1(self):
    # 1.2 s gap: pacing caps gas at 0.2, a trailing lead lets it through
    for v_rel, expected_gas in ((0.1, 0.2), (0.15, 0.3)):
      with self.subTest(v_rel=v_rel):
        self.ext = LongitudinalExt()
        _, gas, _, _ = self.run_at(55, 0.3, 0.3, dict(dRel=30.0, vRel=v_rel, vLead=28.0))
        self.assertAlmostEqual(gas, expected_gas)

  def test_pacing_caps_gas(self):
    lead = dict(dRel=50.0, vRel=0.0, vLead=25.0)
    _, gas, _, _ = self.run_at(55, 1.0, 1.0, lead)
    self.assertAlmostEqual(gas, 0.2)

  def test_slow_lead_is_upstream(self):
    lead = dict(dRel=50.0, vRel=0.0, vLead=15.0)  # lead under 40 mph
    _, gas, _, _ = self.run_at(55, 1.0, 1.0, lead)
    self.assertEqual(gas, 1.0)

  def test_braking_ramps_in(self):
    lead = dict(dRel=60.0, vRel=-1.0, vLead=24.0)  # far, slow closing: rate limit applies
    accel, _, _, _ = self.run_at(55, -1.0, CarControllerParams.INACTIVE_GAS, lead)
    self.assertAlmostEqual(accel, -0.002)

  def test_no_lead_holds_accel(self):
    # BP7 behavior, ported as-is (#16 follow-up)
    accel, _, brake, _ = self.run_at(55, -1.0, CarControllerParams.INACTIVE_GAS, frames=3)
    self.assertEqual(accel, 0.0)
    self.assertFalse(brake)

  def test_disabled_is_upstream_accel(self):
    lead = dict(dRel=30.0, vRel=-2.0, vLead=28.0)
    accel, gas, _, _ = self.run_at(55, 0.3, 0.3, lead, disable_BP_long_UI=b"1")
    self.assertEqual((accel, gas), (0.3, 0.3))

  def test_downhill_compensation_off(self):
    # -0.1 accel on a downhill: with compensation the pitch pushes it into braking; without, it doesn't
    _, _, brake, _ = self.run_at(40, -0.1, CarControllerParams.INACTIVE_GAS, pitch=-0.5)
    self.assertTrue(brake)
    self.ext = LongitudinalExt()
    _, _, brake, _ = self.run_at(40, -0.1, CarControllerParams.INACTIVE_GAS, pitch=-0.5, disable_downhill_comp_UI=b"1")
    self.assertFalse(brake)

  def test_no_gas_while_braking(self):
    _, gas, brake, _ = self.run_at(40, -1.0, 0.4)
    self.assertTrue(brake)
    self.assertEqual(gas, CarControllerParams.INACTIVE_GAS)


if __name__ == '__main__':
  unittest.main()
