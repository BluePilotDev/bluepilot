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

  def get_bool(self, key):
    return bool(self.values.get(key, False))


class _XY:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class _Position:
  def __init__(self, x, y):
    self.x = x
    self.y = y


class _Meta:
  laneChangeState = 0
  laneChangeDirection = 0


class _OrientationRate:
  def __init__(self, z):
    self.z = z


class _Model:
  """Minimal fake modelV2, just the fields lateral_angle_ext / lane_center_trim read."""

  def __init__(self, lane_center_y=0.0, model_y=0.0, width=3.7, lane_change_state=0):
    xs = list(range(0, 60, 2))
    half = width / 2.0
    self.laneLines = [
      _XY(xs, [lane_center_y - half - 3.7] * len(xs)),
      _XY(xs, [lane_center_y - half] * len(xs)),
      _XY(xs, [lane_center_y + half] * len(xs)),
      _XY(xs, [lane_center_y + half + 3.7] * len(xs)),
    ]
    self.laneLineProbs = [0.9, 0.9, 0.9, 0.9]
    self.laneLineStds = [0.1, 0.1, 0.1, 0.1]
    self.position = _Position(xs, [model_y] * len(xs))
    # len must match ModelConstants.T_IDXS (33) -- update_angle_strategy interps orientationRate.z
    # against T_IDXS for the variable-lookup-time predicted-curvature blend.
    self.orientationRate = _OrientationRate([0.0] * 33)
    self.meta = _Meta()
    self.meta.laneChangeState = lane_change_state


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


class TestLaneCenteringIntegration(unittest.TestCase):
  """Lane centering trim (advanced lane positioning) as wired into update_angle_strategy --
  see lane_center_trim.py for the isolated unit tests of the trim itself."""

  V_EGO = 15.0

  def setUp(self):
    self.CP = _explorer_cp()
    self.ext = _Harness(self.CP)
    self.ext.human_turn_detector = _ForcedDetector(False)
    self.ext.model = _Model()
    self.cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=0.0)

  def _update(self, lat_active=True):
    return self.ext.update_angle_strategy(_CC(latActive=lat_active), self.cs, _Actuators(curvature=0.0), self.CP)

  def test_disabled_by_default(self):
    for _ in range(50):
      self._update()
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_enabling_with_offset_produces_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(500):
      self._update()
    self.assertNotEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_strength_param_scales_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(500):
      self._update()
    full_gain_correction = self.ext.lane_center_trim.correction

    self.ext.lane_center_trim.reset()
    self.ext.lane_centering_strength_ang = 0.5
    for _ in range(500):
      self._update()
    half_gain_correction = self.ext.lane_center_trim.correction

    self.assertAlmostEqual(half_gain_correction, full_gain_correction * 0.5, places=3)

  def test_lane_change_resets_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(200):
      self._update()
    self.assertNotEqual(self.ext.lane_center_trim.correction, 0.0)

    self.ext.model.meta.laneChangeState = 1  # laneChangeStarting
    self._update()
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_human_turn_resets_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(200):
      self._update()
    self.assertNotEqual(self.ext.lane_center_trim.correction, 0.0)

    self.ext.human_turn_detector = _ForcedDetector(True)
    self._update()
    self.assertTrue(self.ext.angle_human_turn_active)
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_inactive_resets_correction(self):
    self.ext.enable_lane_positioning_ang = True
    self.ext.custom_path_offset_ang = 5.0
    self.ext.lane_centering_strength_ang = 1.0
    for _ in range(200):
      self._update()
    self.assertNotEqual(self.ext.lane_center_trim.correction, 0.0)

    self._update(lat_active=False)
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)


class TestHandoffBlipDebounce(unittest.TestCase):
  """The hand-off blip's release debounce: a steeringPressed dip must persist >= 0.3 s to
  count as a release; press time survives the dip, so a re-press resumes the same grab."""

  TICK_S = 0.05  # 20 Hz lateral tick

  def setUp(self):
    self.CP = _explorer_cp()
    self.ext = _Harness(self.CP)
    self.ext.human_turn_detector = _ForcedDetector(False)
    self.cs = _CS(vEgoRaw=15.0, vEgo=15.0, yawRate=0.0)

  def _update(self, pressed=False):
    self.cs.out.steeringPressed = pressed
    return self.ext.update_angle_strategy(_CC(latActive=True), self.cs, _Actuators(curvature=0.0), self.CP)

  def test_short_dip_inside_hold_does_not_fire(self):
    # 10 ticks pressed (0.5 s), then a 1-tick (50 ms) dip: below the 0.3 s debounce.
    for _ in range(10):
      self._update(pressed=True)
    self._update(pressed=False)
    self.assertEqual(self.ext.stall_blip_frames_left, 0)
    self.assertAlmostEqual(self.ext.press_timer_s, 10 * self.TICK_S)
    self.assertAlmostEqual(self.ext.release_timer_s, self.TICK_S)

  def test_release_below_debounce_window_does_not_fire(self):
    for _ in range(10):
      self._update(pressed=True)
    for _ in range(5):  # 0.25 s released -- just short of _PRESS_RELEASE_S
      self._update(pressed=False)
    self.assertEqual(self.ext.stall_blip_frames_left, 0)
    # and the press time still stands for the next release
    self.assertAlmostEqual(self.ext.press_timer_s, 10 * self.TICK_S)

  def _release_until_fired(self, max_ticks=12):
    for _ in range(max_ticks):
      self._update(pressed=False)
      if self.ext.stall_blip_frames_left > 0:
        return
    self.fail("hand-off blip never fired")

  def test_re_press_resumes_press_timer(self):
    # 6 ticks (0.3 s) + dip (2 ticks) + 6 ticks (0.3 s): the dip is inside the debounce
    # window, so the press continues at 0.6 s total; a release that persists >= 0.3 s earns
    # the pulse.
    for _ in range(6):
      self._update(pressed=True)
    for _ in range(2):
      self._update(pressed=False)
    self.assertEqual(self.ext.stall_blip_frames_left, 0)
    for _ in range(6):
      self._update(pressed=True)
    self.assertAlmostEqual(self.ext.press_timer_s, 12 * self.TICK_S)
    self._release_until_fired()
    # the firing frame consumes the first pulse tick (the pulse block runs later in the same frame)
    self.assertEqual(self.ext.stall_blip_frames_left, 5)
    self.assertEqual(self.ext.angle_stall_blip_source, 1)
    self.assertEqual(self.ext.stall_blip_count, 0)  # hand-off blips don't consume episode count

  def test_pulse_ends_and_cooldown_rearms(self):
    for _ in range(12):
      self._update(pressed=True)
    self._release_until_fired()
    self.assertEqual(self.ext.stall_blip_frames_left, 5)
    # let the 6-frame pulse run out (5 remaining ticks)
    for _ in range(5):
      self._update(pressed=False)
    self.assertEqual(self.ext.stall_blip_frames_left, 0)
    self.assertEqual(self.ext.angle_stall_blip_source, 0)
    self.assertGreater(self.ext.stall_blip_cooldown_s, 0.0)
    # cooldown blocks a second pulse from the same press episode (timer already reset)
    for _ in range(12):
      self._update(pressed=True)
    for _ in range(8):
      self._update(pressed=False)
    self.assertEqual(self.ext.stall_blip_frames_left, 0)


class TestReactiveStallRealCurveGate(unittest.TestCase):
  """The reactive stall detector's real-curve floor: straight-line entry (small measured
  curvature) satisfies the gap test by construction and must not fire; a tight curve with
  the deviation clip binding for >= 0.5 s fires."""

  V_EGO = 10.0  # above the 9 m/s gate, low enough that path_angle stays under the 0.10 rad cap

  def setUp(self):
    self.CP = _explorer_cp()
    self.ext = _Harness(self.CP)
    self.ext.human_turn_detector = _ForcedDetector(False)
    # No model in the harness: drop the predicted-curvature blend so requested == desired and
    # the scenario reads directly off the planner kappa (default b=0.5 would dilute it).
    self.ext.path_angle_blend_ratio = 0.0
    # measured curvature = -yawRate / v (OP convention)
    self.cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=-0.05)  # -> +0.005 measured

  def _update(self, desired):
    self.cs.out.steeringPressed = False
    return self.ext.update_angle_strategy(_CC(latActive=True), self.cs,
                                          _Actuators(curvature=desired), self.CP)

  def test_curve_entry_from_straight_does_not_fire(self):
    # measured +0.0015 (below the 2x-tolerance floor), desired +0.007: gap and clip-binding
    # tests pass, but the real-curve floor must keep the detector dark for a full second.
    self.cs.out.yawRate = -0.015
    for _ in range(20):
      self._update(0.007)
    self.assertTrue(self.ext.bp_curvature_deviation_limited)  # setup sanity: the clip did bind
    self.assertEqual(self.ext.stall_blip_frames_left, 0)
    self.assertEqual(self.ext.stall_blip_hold_s, 0.0)
    self.assertEqual(self.ext.angle_stall_blip_source, 0)

  def test_tight_curve_stall_fires(self):
    # measured +0.005 (above the floor), desired +0.010: gap 0.005 > 2x tolerance, clip
    # binding every frame -> the episode holds to 0.5 s and the pulse fires.
    for _ in range(12):
      self._update(0.010)
    self.assertEqual(self.ext.stall_blip_count, 1)
    self.assertGreater(self.ext.stall_blip_frames_left, 0)
    self.assertEqual(self.ext.angle_stall_blip_source, 2)
    self.assertTrue(self.ext.angle_stall_blip_active)


if __name__ == '__main__':
  unittest.main()
