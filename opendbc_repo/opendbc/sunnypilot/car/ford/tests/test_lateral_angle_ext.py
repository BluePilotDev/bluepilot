"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Unit tests for angle-mode shadow-curvature publishing (bp_kappa_cmd).
#
# The shadow value is consumed by carcontroller as the input to ford.h's angle-mode
# deviation check (Lane_Assist_Data1 bytes 5-6, judged against angle_meas). These tests
# pin the truthfulness contract: whenever the planner kappa cannot honestly describe the
# car's steering -- inactive, human-turn override, stall blip, driver pressing -- the
# published shadow must equal the measured curvature, so the panda-latched value always
# stays inside the check's band and re-engage frames never compare a stale zero against
# real measured curvature.

import math
import unittest
from dataclasses import dataclass
from unittest import mock

from opendbc.car import structs
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.car.interfaces import scale_tire_stiffness
from opendbc.sunnypilot.car.ford import lateral_curv_ext
from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsSP
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralCurvExt
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt


def _explorer_cp():
  CP = structs.CarParams()
  CP.mass = 2050.
  CP.wheelbase = 3.025
  CP.steerRatio = 16.8
  CP.centerToFront = CP.wheelbase * 0.44
  CP.tireStiffnessFactor = 0.82
  CP.tireStiffnessFront, CP.tireStiffnessRear = scale_tire_stiffness(
    CP.mass, CP.wheelbase, CP.centerToFront, CP.tireStiffnessFactor)
  return CP


class _FakeLiveDelay:
  lateralDelay = 0.2


class _FakeSubMaster:
  def __init__(self, *args, **kwargs):
    self.updated = {s: False for s in ('modelV2', 'liveParameters', 'selfdriveState', 'radarState', 'liveDelay')}

  def update(self, timeout=0):
    pass

  def __getitem__(self, key):
    if key == 'liveDelay':
      return _FakeLiveDelay()
    raise KeyError(key)


class _ForcedDetector:
  def __init__(self, active):
    self.active = active

  def update(self, *_args):
    return self.active

  def reset(self):
    pass


class _FakeParams:
  def __init__(self, values):
    self.values = values

  def get(self, key, return_default=False):
    return self.values.get(key)


@dataclass
class _CSOut:
  vEgoRaw: float = 15.0
  vEgo: float = 15.0
  steeringPressed: bool = False
  steeringAngleDeg: float = 0.0
  yawRate: float = 0.0


class _CS:
  def __init__(self, **kwargs):
    self.out = _CSOut(**kwargs)
    self.lat_ctl_lim_stat = 0


@dataclass
class _CC:
  latActive: bool = True


@dataclass
class _Actuators:
  curvature: float = 0.0


class _Harness(LateralCurvExt, LateralAngleExt):
  """Mirrors CarController's mixin composition (see carcontroller.py)."""

  def __init__(self, CP, CP_SP=None):
    self.CP = CP  # CarControllerBase initializes this before the lateral mixins.
    with mock.patch.object(lateral_curv_ext.messaging, 'SubMaster', _FakeSubMaster):
      LateralCurvExt.__init__(self, CP, CP_SP)
    LateralAngleExt.__init__(self, CP, CP_SP)


def _pinion_harness(flag):
  """Harness with the STEER_ANGLE_CURVATURE flag set (or not) on CP_SP, detector stubbed."""
  CP = _explorer_cp()
  CP_SP = structs.CarParamsSP()
  if flag:
    CP_SP.safetyParam |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE
  ext = _Harness(CP, CP_SP)
  ext.human_turn_detector = _ForcedDetector(False)
  return ext, CP


class TestShadowCurvaturePublishing(unittest.TestCase):
  V_EGO = 15.0
  YAW_RATE = 0.75  # rad/s -> measured curvature = -0.75 / 15 = -0.05 (OP convention)

  def setUp(self):
    self.CP = _explorer_cp()
    self.ext = _Harness(self.CP)
    self.ext.human_turn_detector = _ForcedDetector(False)
    self.cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=self.YAW_RATE)
    self.measured = -self.YAW_RATE / self.V_EGO

  def _update(self, lat_active=True):
    return self.ext.update_angle_strategy(_CC(latActive=lat_active), self.cs, _Actuators(curvature=0.01), self.CP)

  def test_inactive_publishes_measured(self):
    result = self._update(lat_active=False)
    self.assertEqual(result.path_angle, 0.0)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_human_turn_override_publishes_measured(self):
    self.ext.human_turn_detector = _ForcedDetector(True)
    result = self._update()
    self.assertTrue(self.ext.angle_human_turn_active)
    self.assertEqual(result.path_angle, 0.0)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_stall_blip_publishes_measured(self):
    self.ext.stall_blip_frames_left = 3
    result = self._update()
    self.assertTrue(self.ext.angle_stall_blip_active)
    self.assertEqual(result.path_angle, 0.0)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_pressed_publishes_measured(self):
    self.cs.out.steeringPressed = True
    self._update()
    self.assertFalse(self.ext.angle_human_turn_active)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_hands_off_publishes_clipped_planner_kappa(self):
    # planner wants +0.01 while measured is -0.05: the deviation clip (active above 9 m/s)
    # bounds the shadow to measured + CURVATURE_ERROR, not measured itself -- hands-off
    # behavior is unchanged by the truthful-shadow sites.
    self._update()
    expected = self.measured + CarControllerParams.CURVATURE_ERROR
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, expected)
    self.assertNotAlmostEqual(self.ext.bp_kappa_cmd, self.measured)
    self.assertTrue(self.ext.bp_curvature_deviation_limited)


class TestMeasurementSelection(unittest.TestCase):
  """get_current_curvature must select by the CP_SP STEER_ANGLE_CURVATURE flag: yaw rate
  by default (stock ford.h angle_meas family), pinion angle via the vehicle model when
  the steering-angle curvature measurement is enabled (pinion ford.h angle_meas family).
  """

  V_EGO = 15.0

  def test_default_is_yaw_rate(self):
    ext, _ = _pinion_harness(flag=False)
    cs = _CS(vEgoRaw=self.V_EGO, yawRate=0.75, steeringAngleDeg=30.0)
    self.assertFalse(ext.bp_pinion_curvature_enabled)
    self.assertAlmostEqual(ext.get_current_curvature(cs), -0.75 / self.V_EGO)

  def test_flag_selects_pinion_vehicle_model(self):
    from opendbc.car.vehicle_model import VehicleModel
    ext, CP = _pinion_harness(flag=True)
    cs = _CS(vEgoRaw=self.V_EGO, yawRate=0.75, steeringAngleDeg=30.0)
    self.assertTrue(ext.bp_pinion_curvature_enabled)
    expected = -VehicleModel(CP).calc_curvature(math.radians(30.0), self.V_EGO, 0.0)
    self.assertAlmostEqual(ext.get_current_curvature(cs), expected)
    self.assertNotAlmostEqual(ext.get_current_curvature(cs), -0.75 / self.V_EGO)


class TestAngleParams(unittest.TestCase):
  def setUp(self):
    self.ext = _Harness(_explorer_cp())

  def test_high_speed_dampening_preserves_platform_gain(self):
    CP = _explorer_cp()
    CP.carFingerprint = CAR.FORD_F_150_MK14
    ext = _Harness(CP)
    ext.update_angle_params(_FakeParams({"FordHighSpeedDampening_ang": b"1.12"}))
    self.assertAlmostEqual(ext.path_angle_gain_lowC_highV, 0.95)
    self.assertAlmostEqual(ext.user_dampening_factor, 1.12)

  def test_high_speed_dampening_multiplies_low_curvature_high_speed_gain(self):
    self.ext.update_angle_params(_FakeParams({"FordHighSpeedDampening_ang": b"1.12"}))
    cs = _CS(vEgoRaw=26.82, vEgo=26.82)
    self.ext.update_angle_strategy(_CC(), cs, _Actuators(), self.ext.CP)
    self.assertAlmostEqual(
      self.ext.low_gain_calc,
      self.ext.path_angle_gain_lowC_highV * self.ext.user_dampening_factor,
    )

  def test_high_speed_dampening_is_clamped(self):
    for raw_value, expected in ((b"0.50", 0.75), (b"1.50", 1.25)):
      with self.subTest(raw_value=raw_value):
        self.ext.update_angle_params(_FakeParams({"FordHighSpeedDampening_ang": raw_value}))
        self.assertAlmostEqual(self.ext.user_dampening_factor, expected)


class TestInitializeFord(unittest.TestCase):
  def test_safety_param_stays_a_plain_int(self):
    """card serializes CP_SP to capnp, which rejects enum subclasses of int -- an
    IntFlag-typed safetyParam crashed card on-device. Pin the exact type."""
    from opendbc.sunnypilot.car.interfaces import _initialize_ford
    CP = structs.CarParams()
    CP.brand = 'ford'
    CP.carFingerprint = 'FORD_EXPLORER_MK6'
    CP_SP = structs.CarParamsSP()
    _initialize_ford(CP, CP_SP, {"FordPrefSteerAngleCurvature": True})
    self.assertEqual(CP_SP.safetyParam, 0xb)  # flag | (explorer index 5 << 1)
    self.assertIs(type(CP_SP.safetyParam), int)


class TestStallFractionalGate(unittest.TestCase):
  """The stall pulse must arm on a FRACTIONAL delivery deficit, not absolute gap alone:
  an honest deep-curve entry tracks at 0.7-0.85x of a large demand and clears
  _STALL_GAP_MIN on magnitude, but is not a stall. Yaw mode (flag off) throughout --
  this gate is measurement-source independent."""

  def _drive(self, desired, measured, frames=20, v_ego=10.0):
    # measured curvature comes exactly from yawRate / v; at v > 9 the deviation clip
    # binds on the gap, so devLim provides the detector's charging path.
    ext, CP = _pinion_harness(flag=False)
    cs = _CS(vEgoRaw=v_ego, vEgo=v_ego, yawRate=-measured * v_ego, steeringAngleDeg=0.0)
    for _ in range(frames):
      ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=desired), CP)
    return ext

  def test_entry_transient_never_fires(self):
    # delivering 0.72x of a deep demand: gap 0.007 > _STALL_GAP_MIN, but not a stall
    ext = self._drive(desired=0.025, measured=0.018)
    self.assertEqual(ext.stall_blip_count, 0)
    self.assertEqual(ext.stall_blip_hold_s, 0.0)

  def test_true_stall_still_fires(self):
    # delivering 0.2x of the same demand: fractional failure -> pulse
    ext = self._drive(desired=0.025, measured=0.005)
    self.assertEqual(ext.stall_blip_count, 1)


if __name__ == '__main__':
  unittest.main()
