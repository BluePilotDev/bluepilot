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


class _SmModel:
  """Minimal modelV2 stand-in: constant curvature along the horizon."""
  class _OR:
    def __init__(self, z):
      self.z = z

  class _Meta:
    laneChangeState = 0
    laneChangeDirection = 0

  def __init__(self, kappa, v):
    self.orientationRate = self._OR([kappa * v] * 33)
    self.meta = self._Meta()


class _SmParams:
  """Typed-enough mock params for the smoothing toggle glue."""
  def __init__(self, values):
    self.values = values

  def get(self, key, return_default=False):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put(self, key, value):
    self.values[key] = value


class TestAngleSmoothing(unittest.TestCase):
  """Anti-weave smoothing (FordAngleSmoothing). The OFF path must behave exactly like the
  unsmoothed math; the ON path must remove dither injectors without softening curve entry."""

  V = 20.0

  def _ext(self, smoothing):
    cp = _explorer_cp()
    ext = _Harness(cp)
    ext.CP = cp  # update_angle_params reads self.CP (set by carcontroller in the real stack)
    ext.human_turn_detector = _ForcedDetector(False)
    ext.smoothing_enabled = smoothing
    # Effective scale (menu - 1.0): tests exercise the tuned package (menu 2.0).
    ext.smoothing_strength = 1.0 if smoothing else 0.0
    return ext

  def _cs(self, desired=0.0):
    # yaw tracks desired so the deviation clip never binds and measured == desired.
    return _CS(vEgoRaw=self.V, vEgo=self.V, yawRate=-desired * self.V)

  def _drive(self, ext, desired_seq, model_kappa=None):
    out = []
    for d in desired_seq:
      if model_kappa is not None:
        ext.model = _SmModel(model_kappa, self.V)
      out.append(ext.update_angle_strategy(_CC(), self._cs(d), _Actuators(curvature=d), _explorer_cp()).path_angle)
    return out

  def test_off_gain_schedule_uses_raw_kappa(self):
    from numpy import interp as np_interp
    ext = self._ext(False)
    for d in [0.0006, 0.0011, 0.0006, 0.0011] * 10:
      ext.update_angle_strategy(_CC(), self._cs(d), _Actuators(curvature=d), _explorer_cp())
      expected = float(np_interp(abs(ext.bp_kappa_cmd), [0.0007, 0.001],
                                 [ext.low_gain_calc, ext.high_gain_calc]))
      self.assertAlmostEqual(ext.curvature_factor, expected, places=12)
    # OFF path must leave the smoothing filters untouched at their reset values.
    self.assertEqual(ext.smoother._sched, 0.0)
    self.assertIsNone(ext.smoother._b_blend)

  def test_on_gain_schedule_filters_oscillation(self):
    ext = self._ext(True)
    factors = []
    for d in [0.0006, 0.0011] * 40:  # square wave straddling the interp band
      ext.update_angle_strategy(_CC(), self._cs(d), _Actuators(curvature=d), _explorer_cp())
      factors.append(ext.curvature_factor)
    tail = factors[-20:]
    # Raw input would swing the factor across the whole low<->high range every frame;
    # the filtered schedule input must pin it nearly constant once settled.
    self.assertLess(max(tail) - min(tail), 0.05)

  def test_gain_filter_asymmetry(self):
    from opendbc.sunnypilot.car.ford.angle_smoothing import GAIN_RC_UP as _SM_GAIN_RC_UP, GAIN_RC_DOWN as _SM_GAIN_RC_DOWN
    ext = self._ext(True)
    rise_frames = int(2.3 * _SM_GAIN_RC_UP / 0.05) + 2
    self._drive(ext, [0.002] * rise_frames, model_kappa=0.002)
    self.assertGreater(ext.smoother._sched, 0.9 * 0.002)
    fall_frames = int(_SM_GAIN_RC_DOWN / 0.05)
    self._drive(ext, [0.0] * fall_frames, model_kappa=0.0)
    self.assertGreater(ext.smoother._sched, 0.3 * 0.002)

  def test_wire_hold_stops_sub_lsb_dither(self):
    from opendbc.sunnypilot.car.ford.angle_smoothing import WIRE_HOLD as _SM_WIRE_HOLD
    ext = self._ext(True)
    self._drive(ext, [0.0015] * 60)  # settle onto a working point
    held = ext.path_angle_last
    eps = _SM_WIRE_HOLD / (self.V * 3.0)
    out = self._drive(ext, [0.0015 + (eps if i % 2 else -eps) for i in range(40)])
    self.assertTrue(all(abs(pa - held) < 1e-12 for pa in out[5:]))
    out = self._drive(ext, [0.0030] * 30)  # a genuine move releases the hold
    self.assertNotAlmostEqual(out[-1], held, places=6)

  def test_blend_slew_bounded(self):
    from opendbc.sunnypilot.car.ford.angle_smoothing import BLEND_SLEW as _SM_BLEND_SLEW
    ext = self._ext(True)
    ext.model = _SmModel(0.002, self.V)
    prev = None
    for d in [0.002] * 20 + [0.015, 0.002] * 20:  # >0.010 drops toggle _desired_falling
      ext.update_angle_strategy(_CC(), self._cs(d), _Actuators(curvature=d), _explorer_cp())
      if prev is not None and ext.smoother._b_blend is not None:
        self.assertLessEqual(abs(ext.smoother._b_blend - prev), _SM_BLEND_SLEW + 1e-9)
      prev = ext.smoother._b_blend

  def test_kappa_entering_hysteresis(self):
    ext = self._ext(True)
    flips = 0
    last = None
    for i in range(60):
      mk = 0.0005 + (0.0001 if i % 2 else -0.0001)  # dither inside the +-0.0003 band
      ext.model = _SmModel(mk, self.V)
      ext.update_angle_strategy(_CC(), self._cs(0.0005), _Actuators(curvature=0.0005), _explorer_cp())
      if last is not None and ext.smoother._entering != last:
        flips += 1
      last = ext.smoother._entering
    self.assertEqual(flips, 0)

  def test_curve_entry_not_softened(self):
    ramp = [min(0.003, 0.0002 * i) for i in range(60)]
    off = self._drive(self._ext(False), ramp)
    on = self._drive(self._ext(True), ramp)
    target = 0.9 * off[-1]
    t_off = next(i for i, x in enumerate(off) if x >= target)
    t_on = next(i for i, x in enumerate(on) if x >= target)
    self.assertLessEqual(t_on - t_off, 2)  # <=0.1 s later at 20 Hz
    self.assertAlmostEqual(on[-1], off[-1], delta=abs(off[-1]) * 0.02 + 1e-9)

  def test_roc_property_holds_with_smoothing(self):
    import random
    rng = random.Random(3)
    ext = self._ext(True)
    prev = ext.path_angle_last
    for _ in range(300):
      d = rng.uniform(-0.004, 0.004)
      ext.update_angle_strategy(_CC(), self._cs(d), _Actuators(curvature=d), _explorer_cp())
      self.assertLessEqual(abs(ext.path_angle_last - prev), 0.055 + 1e-9)  # loosest soft ROC
      prev = ext.path_angle_last

  def test_resets_on_override_paths(self):
    ext = self._ext(True)
    self._drive(ext, [0.002] * 40)
    self.assertGreater(ext.smoother._sched, 0.0)
    ext.human_turn_detector = _ForcedDetector(True)  # forces the override early-return
    ext.update_angle_strategy(_CC(), self._cs(0.002), _Actuators(curvature=0.002), _explorer_cp())
    self.assertEqual(ext.smoother._sched, 0.0)
    self.assertEqual(ext.smoother._wire, 0.0)
    self.assertIsNone(ext.smoother._b_blend)

  def test_menu_one_is_bit_identical_stock(self):
    # Menu 1.0 (effective 0) must equal the toggle-off path EXACTLY, frame by frame.
    import random
    rng = random.Random(7)
    seq = [rng.uniform(-0.003, 0.003) for _ in range(200)]
    off = self._drive(self._ext(False), seq)
    neutral = self._ext(True)
    neutral.smoothing_strength = 0.0  # menu 1.0
    on = self._drive(neutral, seq)
    self.assertEqual(off, on)

  def test_strength_max_entry_still_fast(self):
    ramp = [min(0.003, 0.0002 * i) for i in range(60)]
    off = self._drive(self._ext(False), ramp)
    strong = self._ext(True)
    strong.smoothing_strength = 1.5
    on = self._drive(strong, ramp)
    target = 0.9 * off[-1]
    t_off = next(i for i, x in enumerate(off) if x >= target)
    t_on = next(i for i, x in enumerate(on) if x >= target)
    self.assertLessEqual(t_on - t_off, 2)  # entry guarantee is strength-independent

  def test_param_glue_reads_strength(self):
    ext = self._ext(True)
    p = _SmParams({"FordAngleSmoothing": True, "FordAngleSmoothStrength": 1.5,
                   "FordAngleAutoCal": 0, "FordAngleAutoCalState": ""})
    for _ in range(101):
      ext.update_angle_params(p)
    self.assertAlmostEqual(ext.smoothing_strength, 0.5)  # menu 1.5 -> effective 0.5
    p.values["FordAngleSmoothStrength"] = 9.0  # clamped to the menu max (2.5)
    for _ in range(101):
      ext.update_angle_params(p)
    self.assertAlmostEqual(ext.smoothing_strength, 1.5)
    p.values["FordAngleSmoothStrength"] = 0.2  # below stock clamps to menu 1.0 = neutral
    for _ in range(101):
      ext.update_angle_params(p)
    self.assertAlmostEqual(ext.smoothing_strength, 0.0)

  def test_param_glue_reads_toggle(self):
    ext = self._ext(True)
    p = _SmParams({"FordAngleSmoothing": False, "FordAngleAutoCal": 0, "FordAngleAutoCalState": ""})
    for _ in range(101):
      ext.update_angle_params(p)
    self.assertFalse(ext.smoothing_enabled)
    p.values["FordAngleSmoothing"] = True
    for _ in range(101):
      ext.update_angle_params(p)
    self.assertTrue(ext.smoothing_enabled)


if __name__ == '__main__':
  unittest.main()
