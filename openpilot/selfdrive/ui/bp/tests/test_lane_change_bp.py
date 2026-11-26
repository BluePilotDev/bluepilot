import unittest

import numpy as np

from openpilot.cereal import log
from openpilot.selfdrive.ui.bp.onroad.lane_change_bp import (
  LANE_CHANGE_BLOCKED, LANE_CHANGE_BLUE, CHEVRON_COUNT, LaneChangeOverlay,
)

LCS, LCD = log.LaneChangeState, log.LaneChangeDirection
X = np.linspace(0.0, 100.0, 33, dtype=np.float32)
SURE = [0.9, 0.95, 0.95, 0.9]


def lines_at(offsets) -> list[np.ndarray]:
  """Four straight lane lines at these lateral offsets (m, right positive)."""
  return [np.stack([X, np.full_like(X, y), np.zeros_like(X)], axis=-1) for y in offsets]


THREE_LANES = [-5.4, -1.8, 1.8, 5.4]


def run(ov: LaneChangeOverlay, state, direction, offsets=THREE_LANES, probs=SURE, frames: int = 1, blind=(False, False)):
  for _ in range(frames):
    ov.update(state, direction, blind[0], blind[1], lines_at(offsets), probs, 0.05)


def fill(ov: LaneChangeOverlay):
  shapes = ov.shapes()
  assert shapes, "nothing drawn"
  return shapes[0]


class TestLaneChangeOverlay(unittest.TestCase):
  def test_nothing_without_a_lane_change(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.off, LCD.none, frames=10)
    self.assertFalse(ov.visible)
    self.assertEqual(ov.shapes(), [])

  def test_the_lane_to_the_right_while_changing_right(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.laneChangeStarting, LCD.right, frames=10)
    f = fill(ov)
    self.assertAlmostEqual(float(f.left[0, 1]), 1.8, delta=0.2)
    self.assertAlmostEqual(float(f.right[0, 1]), 5.4, delta=0.2)
    self.assertEqual(f.color[:3], LANE_CHANGE_BLUE)
    self.assertEqual(len(ov.shapes()), 1 + 2 * CHEVRON_COUNT)  # the lane, then two arms per chevron

  def test_the_lane_to_the_left_while_changing_left(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.laneChangeStarting, LCD.left, frames=10)
    f = fill(ov)
    self.assertAlmostEqual(float(f.left[0, 1]), -5.4, delta=0.2)
    self.assertAlmostEqual(float(f.right[0, 1]), -1.8, delta=0.2)

  def test_red_while_the_blind_spot_on_that_side_is_occupied(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.preLaneChange, LCD.left, frames=10, blind=(True, False))
    self.assertEqual(fill(ov).color[:3], LANE_CHANGE_BLOCKED)
    ov = LaneChangeOverlay()
    run(ov, LCS.preLaneChange, LCD.left, frames=10, blind=(False, True))  # the other side does not count
    self.assertEqual(fill(ov).color[:3], LANE_CHANGE_BLUE)

  def test_it_pulses_while_waiting_to_start(self):
    ov = LaneChangeOverlay()
    alphas = []
    for _ in range(40):  # 2 s
      run(ov, LCS.preLaneChange, LCD.right)
      alphas.append(fill(ov).color[3])
    self.assertGreater(max(alphas[10:]) - min(alphas[10:]), 30)

  def test_the_highlight_stays_on_the_lane_when_the_model_renumbers_its_lines(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.laneChangeStarting, LCD.right, frames=10)
    # you drift right, the lines slide left...
    for i in range(1, 21):
      run(ov, LCS.laneChangeStarting, LCD.right, [y - 0.08 * i for y in THREE_LANES])
    self.assertFalse(ov.crossed)
    # ...and over the line the model makes the lane you moved into yours: lines 1 and 2
    run(ov, LCS.laneChangeStarting, LCD.right, [-3.5, 0.1, 3.7, 7.3])  # was -7.0, -3.4, 0.2, 3.8
    self.assertTrue(ov.crossed)
    run(ov, LCS.laneChangeStarting, LCD.right, [-3.5, 0.1, 3.7, 7.3], frames=5)
    f = fill(ov)
    self.assertLess(float(f.left[0, 1]), 0.5)  # the lane now holds you: its left side is to your left
    self.assertGreater(float(f.right[0, 1]), 1.0)

  def test_crossing_is_also_seen_from_your_lane_lines_coming_back(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.laneChangeStarting, LCD.left, frames=5)
    run(ov, LCS.laneChangeStarting, LCD.left, probs=[0.0, 0.05, 0.05, 0.0], frames=10)  # straddling the line
    self.assertFalse(ov.crossed)
    run(ov, LCS.laneChangeStarting, LCD.left, probs=[0.3, 0.6, 0.6, 0.3])
    self.assertTrue(ov.crossed)

  def test_after_crossing_it_fades_out_with_no_chevrons(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.laneChangeStarting, LCD.left, frames=5)
    run(ov, LCS.laneChangeStarting, LCD.left, probs=[0.0, 0.05, 0.05, 0.0], frames=5)
    run(ov, LCS.laneChangeStarting, LCD.left, frames=2)
    self.assertTrue(ov.crossed)
    self.assertEqual(len(ov.shapes()), 1)
    run(ov, LCS.laneChangeStarting, LCD.left, frames=40)  # 2 s
    self.assertEqual(ov.shapes(), [])

  def test_a_cancelled_lane_change_fades_out(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.preLaneChange, LCD.right, frames=10)
    run(ov, LCS.off, LCD.none, frames=20)
    self.assertFalse(ov.visible)
    self.assertEqual(ov.direction, 0)

  def test_frames_without_new_model_data_keep_it_going(self):
    ov = LaneChangeOverlay()
    run(ov, LCS.laneChangeStarting, LCD.right, frames=3)
    for _ in range(5):
      ov.update(LCS.laneChangeStarting, LCD.right, False, False, None, None, 1 / 60)
    self.assertTrue(ov.visible)
    self.assertTrue(ov.shapes())


if __name__ == "__main__":
  unittest.main()
