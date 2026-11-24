#!/usr/bin/env python3
"""
BluePilot Ford lateral, angle mode: the shadow-curvature plausibility check.

In angle mode the LatCtl curvature signal is held at 0 and the command rides on path_angle.
Lane_Assist_Data1 carries a side channel (angle mode engaged, and the curvature the controller
would have commanded, "shadow curvature"; layout in opendbc/bluepilot_lateral/can/LANE_ASSIST_DATA1.md)
that the panda checks against the yaw-measured curvature with the same error window as curvature mode
(ford_shadow_curvature_error_check, ford.h). Ported from BluePilot's earlier vendored tests.
"""
import unittest

from .ford_bluepilot_common import (
  BPFordTestCase, SMALL_ANGLE, FORMER_LATCH_WINDOW_FRAMES, CURVATURE_TO_CAN, MAX_CURVATURE_ERROR_CAN, CURVATURE_ERROR_MIN_SPEED,
)


class TestFordBPAngleModeCANFD(BPFordTestCase):
  CANFD = True

  def _angle_frame(self) -> bool:
    return self.tx(self.lat(True, 0, SMALL_ANGLE, 0, 0))

  def test_shadow_curvature_error_window(self):
    """Above the speed gate the shadow curvature must be within max_curvature_error of measured;
    below it the check is off. Covers ford_shadow_curvature_error_check and its call site."""
    measured_can = 250  # 0.005 rad/m
    for speed in (CURVATURE_ERROR_MIN_SPEED - 1, CURVATURE_ERROR_MIN_SPEED + 1, 20.0):
      enforced = speed > CURVATURE_ERROR_MIN_SPEED
      self.set_meas(measured_can / CURVATURE_TO_CAN, speed)
      m_min, m_max = self.meas()
      hi_edge = m_max + MAX_CURVATURE_ERROR_CAN + 1  # highest_allowed in the C
      lo_edge = m_min - MAX_CURVATURE_ERROR_CAN - 1
      # shadow arrives as int16 at 1e-6 rad/m; the C scales by 0.05 to CAN units and truncates,
      # so pick shadow values that land exactly on the CAN-unit boundaries
      for shadow_can, expect in ((m_max, True), (hi_edge, True), (hi_edge + 1, not enforced),
                                 (lo_edge, True), (lo_edge - 1, not enforced), (0, not enforced)):
        self.tx(self.lka_bp_status_msg(True, shadow_can / CURVATURE_TO_CAN))
        with self.subTest(speed=speed, shadow_can=shadow_can):
          self.assertEqual(expect, self._angle_frame())

  def test_shadow_curvature_has_no_rate_limit(self):
    """Only the path_angle ROC applies in angle mode: a large frame-to-frame jump in the shadow
    curvature that stays inside the window of a correspondingly moved measurement must pass."""
    speed = CURVATURE_ERROR_MIN_SPEED + 1
    for curvature in (0.015, -0.015, 0.018, -0.018, 0.001, -0.019):
      self.set_meas(curvature, speed)
      self.tx(self.lka_bp_status_msg(True, curvature))
      with self.subTest(curvature=curvature):
        self.assertTrue(self._angle_frame())

  def test_flag_clear_skips_shadow_check(self):
    """Without the angle-mode bit a curvature-0 frame is not checked against the shadow value."""
    speed = CURVATURE_ERROR_MIN_SPEED + 5
    self.set_meas(0.005, speed)
    self.tx(self.lka_bp_status_msg(False, 0.02))  # would fail the window if it were applied
    self.assertTrue(self._angle_frame())

  def test_curvature_mode_unaffected_by_angle_mode_flag(self):
    """A real (non-zero) curvature command is curvature mode by its own content; the flag and the
    shadow value must not gate it either way."""
    speed = CURVATURE_ERROR_MIN_SPEED + 5
    curvature_can = 500
    self.set_meas(curvature_can / CURVATURE_TO_CAN, speed)
    for flag in (True, False):
      self.tx(self.lka_bp_status_msg(flag, 0.02))
      self.set_prev_curvature_can(curvature_can)
      with self.subTest(flag=flag):
        self.assertTrue(self.tx(self.lat(True, 0, 0, curvature_can / CURVATURE_TO_CAN, 0)))

  def test_shadow_out_of_window_blocks_while_enabled(self):
    """An out-of-window shadow value blocks the enabled frame; the disabled all-zero frame (mode 0)
    is not checked against it."""
    speed = CURVATURE_ERROR_MIN_SPEED + 5
    self.set_meas(0.005, speed)
    self.tx(self.lka_bp_status_msg(True, 0.02))
    self.assertFalse(self._angle_frame())
    self.assertTrue(self.tx(self.lat(False, 0, 0, 0, 0)))

  def test_mode0_frame_leaves_shadow_check_active(self):
    """Angle mode's human-turn override sends mode 0 with every signal zero. The frames after it
    are checked: an out-of-window shadow curvature blocks on every frame of the 60-frame window
    the removed bypass latch used to cover (#9)."""
    speed = CURVATURE_ERROR_MIN_SPEED + 5
    self.set_meas(0.005, speed)
    self.tx(self.lka_bp_status_msg(True, 0.005))
    self.assertTrue(self.tx(self.lat(False, 0, 0, 0, 0)))
    self.tx(self.lka_bp_status_msg(True, 0.02))
    for frame in range(FORMER_LATCH_WINDOW_FRAMES):
      with self.subTest(frame=frame):
        self.assertFalse(self._angle_frame())


class TestFordBPAngleModeCAN(TestFordBPAngleModeCANFD):
  CANFD = False


if __name__ == "__main__":
  unittest.main()
