"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Tests for the wheel-nudge temporary in-lane offset (lane_offset_nudge.py).
#
# Three layers: the detector's own gates and accumulation; what a nudge does to the command once
# it is wired into angle mode; and the physical size of that effect -- the curvature bias, the
# lateral acceleration it implies, and how quickly it arrives. The last layer is what says whether
# a nudge feels like a bias or like a swerve, so the numbers are pinned here rather than left to
# on-road impressions.

import unittest

from opendbc.sunnypilot.car.ford.lane_center_trim import _MAX_RAW_CORRECTION
from opendbc.sunnypilot.car.ford.lane_offset_nudge import (
  LaneOffsetNudge, _STEER_DT, _MAX_PCT, _RATE_PCT_PER_S, _MIN_SPEED_MS, _NUDGE_DEBOUNCE_S,
  _TOTAL_OFFSET_MAX_M,
)
from opendbc.sunnypilot.car.ford.tests.test_lateral_angle_ext import (
  _Harness, _CS, _CC, _Actuators, _Model, _explorer_cp, _ForcedDetector,
)

NUDGE_DEG = 6.0  # a light push, inside the 2-12 deg nudge band
LANE_WIDTH = 3.7


class _NudgeModel:
  """Just the laneLines/probs fields the nudge reads for lane width."""

  def __init__(self, width=LANE_WIDTH, probs=(0.9, 0.9, 0.9, 0.9)):
    half = width / 2.0
    self.laneLines = [_Line(-half - 3.7), _Line(-half), _Line(half), _Line(half + 3.7)]
    self.laneLineProbs = list(probs)


class _Line:
  def __init__(self, y):
    self.y = [y] * 33


class TestNudgeDetection(unittest.TestCase):
  """The gates: what does and does not count as a nudge."""

  def setUp(self):
    self.nudge = LaneOffsetNudge()
    self.model = _NudgeModel()

  def _run(self, seconds, angle=NUDGE_DEG, pressed=True, v_ego=25.0, kappa=0.0,
           human_turn=False, enabled=True, lat_active=True, max_pct=_MAX_PCT):
    for _ in range(int(seconds / _STEER_DT)):
      self.nudge.update(enabled, lat_active, v_ego, pressed, angle, kappa, self.model,
                        human_turn, max_pct)
    return self.nudge.percent

  def test_hands_off_does_nothing(self):
    self.assertEqual(self._run(2.0, pressed=False), 0.0)

  def test_nudge_left_is_negative_percent(self):
    # steeringAngleDeg is positive LEFT; percent is positive RIGHT (model frame).
    self.assertLess(self._run(1.0), 0.0)

  def test_nudge_right_is_positive_percent(self):
    self.assertGreater(self._run(1.0, angle=-NUDGE_DEG), 0.0)

  def test_accumulates_at_the_documented_rate(self):
    # Measured after the debounce is served, so the rate is the only thing under test.
    self._run(_NUDGE_DEBOUNCE_S + 0.5)
    start = abs(self.nudge.percent)
    self._run(1.0)
    self.assertAlmostEqual(abs(self.nudge.percent) - start, _RATE_PCT_PER_S, places=6)

  def test_debounce_rejects_a_brief_touch(self):
    self.assertEqual(self._run(0.15), 0.0)

  def test_clips_to_max_percent(self):
    self.assertAlmostEqual(abs(self._run(10.0, max_pct=8.0)), 8.0, places=6)

  def test_below_min_speed_does_nothing(self):
    self.assertEqual(self._run(2.0, v_ego=_MIN_SPEED_MS - 1.0), 0.0)

  def test_in_a_curve_does_nothing(self):
    self.assertEqual(self._run(2.0, kappa=0.003), 0.0)

  def test_steering_angle_beyond_the_band_does_nothing(self):
    self.assertEqual(self._run(2.0, angle=25.0), 0.0)

  def test_holds_after_release(self):
    held = self._run(1.0)
    self.assertAlmostEqual(self._run(3.0, pressed=False), held, places=6)

  def test_holds_through_a_curve(self):
    held = self._run(1.0)
    self.assertAlmostEqual(self._run(3.0, pressed=False, kappa=0.003), held, places=6)

  def test_human_turn_clears_it(self):
    self._run(1.0)
    self.assertEqual(self._run(0.1, human_turn=True), 0.0)

  def test_sustained_curvature_clears_it(self):
    # A navigational turn lateral control drove itself -- no human turn is detected there.
    self._run(1.0)
    self.assertEqual(self._run(2.0, pressed=False, kappa=0.02), 0.0)

  def test_brief_curve_does_not_clear_it(self):
    held = self._run(1.0)
    self.assertAlmostEqual(self._run(1.0, pressed=False, kappa=0.02), held, places=6)

  def test_disable_clears_it(self):
    self._run(1.0)
    self.assertEqual(self._run(0.1, enabled=False), 0.0)

  def test_disengage_clears_it(self):
    self._run(1.0)
    self.assertEqual(self._run(0.1, lat_active=False), 0.0)


class TestNudgeToMeters(unittest.TestCase):
  """Percent of lane width -> meters, and the bound on the total offset."""

  def setUp(self):
    self.nudge = LaneOffsetNudge()

  def _hold(self, seconds, model, max_pct=_MAX_PCT):
    for _ in range(int(seconds / _STEER_DT)):
      self.nudge.update(True, True, 25.0, True, -NUDGE_DEG, 0.0, model, False, max_pct)
    return self.nudge.offset_m

  def test_percent_is_of_the_detected_lane_width(self):
    narrow = self._hold(10.0, _NudgeModel(width=3.0), max_pct=8.0)
    self.nudge.reset()
    wide = self._hold(10.0, _NudgeModel(width=4.0), max_pct=8.0)
    self.assertAlmostEqual(narrow, 0.08 * 3.0, places=3)
    self.assertAlmostEqual(wide, 0.08 * 4.0, places=3)

  def test_unusable_lanelines_fall_back_to_a_nominal_lane(self):
    offset = self._hold(10.0, _NudgeModel(probs=(0.0, 0.0, 0.0, 0.0)), max_pct=8.0)
    self.assertAlmostEqual(offset, 0.08 * 3.6, places=3)

  def test_total_offset_is_clipped_to_the_menu_range(self):
    # A right nudge on top of a right-set menu offset cannot push past the slider's own range.
    self._hold(10.0, _NudgeModel(), max_pct=_MAX_PCT)
    self.assertGreater(self.nudge.offset_m, 0.0)
    self.assertAlmostEqual(self.nudge.total_offset(0.5), _TOTAL_OFFSET_MAX_M, places=6)
    # ...and it subtracts from an opposing menu offset rather than being clipped away.
    self.assertAlmostEqual(self.nudge.total_offset(-0.5), -0.5 + self.nudge.offset_m, places=6)


class TestNudgeEffectInAngleMode(unittest.TestCase):
  """What a nudge does to the command, end to end through update_angle_strategy."""

  V_EGO = 25.0

  def setUp(self):
    self.CP = _explorer_cp()
    self.ext = _Harness(self.CP)
    self.ext.human_turn_detector = _ForcedDetector(False)
    self.ext.model = _Model()
    self.ext.enable_lane_positioning_ang = True
    self.ext.lane_centering_strength_ang = 1.0
    self.ext.enable_nudge_lane_offset = True
    self.ext.nudge_lane_offset_max_pct = 8.0
    self.cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=0.0)

  def _drive(self, seconds, angle=0.0, pressed=False):
    self.cs.out.steeringAngleDeg = angle
    self.cs.out.steeringPressed = pressed
    result = None
    for _ in range(int(seconds / _STEER_DT)):
      result = self.ext.update_angle_strategy(_CC(latActive=True), self.cs,
                                              _Actuators(curvature=0.0), self.CP)
    return result

  def test_no_nudge_no_correction(self):
    self._drive(3.0)
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)
    self.assertEqual(self.ext.bp_nudge_offset_m, 0.0)

  def test_nudge_left_steers_left(self):
    self._drive(3.0, angle=NUDGE_DEG, pressed=True)
    self._drive(2.0)  # hands back off; the offset stands
    # Model frame is positive right, and so is curvature -- a left nudge is negative in both.
    self.assertLess(self.ext.bp_nudge_offset_m, 0.0)
    self.assertLess(self.ext.lane_center_trim.correction, 0.0)

  def test_nudge_right_steers_right(self):
    self._drive(3.0, angle=-NUDGE_DEG, pressed=True)
    self._drive(2.0)
    self.assertGreater(self.ext.bp_nudge_offset_m, 0.0)
    self.assertGreater(self.ext.lane_center_trim.correction, 0.0)

  def test_correction_tracks_the_offset_it_came_from(self):
    self._drive(3.0, angle=-NUDGE_DEG, pressed=True)
    self._drive(5.0)  # hands off, long enough for the trim's smoothing to settle
    # kappa = 2 * offset / lookahead^2 at full gain and full speed authority; the lookahead is
    # v_ego clipped to [8, 35] m.
    lookahead = min(self.V_EGO, 35.0)
    expected = 2.0 * self.ext.bp_nudge_offset_m / (lookahead ** 2)
    self.assertAlmostEqual(self.ext.lane_center_trim.correction, expected,
                           delta=abs(expected) * 0.02)

  def test_disabled_nudge_leaves_the_command_alone(self):
    self.ext.enable_nudge_lane_offset = False
    self._drive(3.0, angle=-NUDGE_DEG, pressed=True)
    self.assertEqual(self.ext.bp_nudge_offset_m, 0.0)
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_human_turn_clears_the_nudge_and_the_correction(self):
    self._drive(3.0, angle=-NUDGE_DEG, pressed=True)
    self.assertNotEqual(self.ext.bp_nudge_offset_m, 0.0)
    self.ext.human_turn_detector = _ForcedDetector(True)
    self._drive(0.1)
    self.assertEqual(self.ext.bp_nudge_offset_m, 0.0)
    self.assertEqual(self.ext.lane_center_trim.correction, 0.0)

  def test_hand_off_stall_blip_does_not_clear_the_nudge(self):
    # A nudge is a sustained press, so releasing it arms the hand-off mode-0 pulse. That pulse is
    # a 300 ms PSCM reset, not a navigational turn -- it must not wipe the offset just set.
    self._drive(3.0, angle=-NUDGE_DEG, pressed=True)
    set_offset = self.ext.bp_nudge_offset_m
    self.assertGreater(set_offset, 0.0)
    self._drive(1.0)  # release: blip arms, fires, and finishes inside this window
    self.assertGreater(self.ext.stall_blip_cooldown_s, 0.0)  # the pulse really did fire
    self.assertAlmostEqual(self.ext.bp_nudge_offset_m, set_offset, places=6)

  def test_disengage_clears_the_nudge(self):
    self._drive(3.0, angle=-NUDGE_DEG, pressed=True)
    self.ext.update_angle_strategy(_CC(latActive=False), self.cs, _Actuators(curvature=0.0), self.CP)
    self.assertEqual(self.ext.bp_nudge_offset_m, 0.0)


class TestNudgeEffectSize(unittest.TestCase):
  """How big the effect is, in units a driver feels: lateral acceleration and the time to build
  it. These bound the nudge as a comfort feature -- a bias, not a lane change."""

  # In the 8-35 m/s band the trim's lookahead equals v_ego, so lateral acceleration works out to
  # 2 x offset x gain, independent of speed: 0.59 m/s^2 for the widest nudge (8% of a 3.7 m lane)
  # at full centering strength. Well under a lane change (~1.5 m/s^2), which is the point.
  COMFORT_LAT_ACCEL = 0.7   # m/s^2
  SETTLE_S = 2.0            # a held nudge should reach its full effect this fast

  def _trim_correction(self, v_ego, seconds=8.0, max_pct=8.0):
    CP = _explorer_cp()
    ext = _Harness(CP)
    ext.human_turn_detector = _ForcedDetector(False)
    ext.model = _Model()
    ext.enable_lane_positioning_ang = True
    ext.lane_centering_strength_ang = 1.0
    ext.enable_nudge_lane_offset = True
    ext.nudge_lane_offset_max_pct = max_pct
    cs = _CS(vEgoRaw=v_ego, vEgo=v_ego, yawRate=0.0)
    cs.out.steeringAngleDeg = -NUDGE_DEG
    cs.out.steeringPressed = True
    for _ in range(int(seconds / _STEER_DT)):
      ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.0), CP)
    return ext.lane_center_trim.correction

  def test_lateral_acceleration_stays_in_comfort_range(self):
    for v_ego in (10.0, 15.0, 20.0, 25.0, 30.0, 35.0):
      with self.subTest(v_ego=v_ego):
        lat_accel = abs(self._trim_correction(v_ego)) * v_ego ** 2
        self.assertLess(lat_accel, self.COMFORT_LAT_ACCEL)

  def test_lateral_acceleration_is_twice_the_offset_across_the_speed_band(self):
    # The closed form above, checked against the real command: the same nudge feels the same at
    # 15 m/s as at 35 m/s, which is what makes a percent-of-lane offset a sane unit.
    for v_ego in (15.0, 25.0, 35.0):
      with self.subTest(v_ego=v_ego):
        correction = self._trim_correction(v_ego)
        lat_accel = abs(correction) * v_ego ** 2
        expected = 2.0 * 0.08 * LANE_WIDTH  # 2 x offset, gain 1.0
        self.assertAlmostEqual(lat_accel, expected, places=2)

  def test_correction_never_exceeds_the_trim_ceiling(self):
    for v_ego in (10.0, 25.0, 35.0):
      with self.subTest(v_ego=v_ego):
        self.assertLessEqual(abs(self._trim_correction(v_ego, max_pct=_MAX_PCT)),
                             _MAX_RAW_CORRECTION)

  def test_a_held_nudge_reaches_full_effect_within_settle_time(self):
    settled = self._trim_correction(25.0, seconds=8.0)
    early = self._trim_correction(25.0, seconds=self.SETTLE_S)
    self.assertGreater(abs(early), abs(settled) * 0.9)

  def test_effect_is_monotonic_in_max_percent(self):
    small = abs(self._trim_correction(25.0, max_pct=4.0))
    large = abs(self._trim_correction(25.0, max_pct=8.0))
    self.assertGreater(large, small * 1.5)


if __name__ == "__main__":
  unittest.main()
