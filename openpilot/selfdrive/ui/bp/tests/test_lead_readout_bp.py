import unittest

import numpy as np

from openpilot.selfdrive.ui.bp.onroad.lead_readout_bp import FT_PER_M, LeadReadout, distance_step

DT = 0.05
V_EGO = 27.0  # m/s, about 60 mph


def feed(ro: LeadReadout, d, v_rel=0.0, present=True):
  for x in np.atleast_1d(d):
    ro.update(present, float(x), v_rel, DT)


class TestLeadReadout(unittest.TestCase):
  def test_noise_does_not_change_the_numbers(self):
    ro = LeadReadout()
    rng = np.random.default_rng(0)
    feed(ro, 30.0 + rng.normal(0, 0.3, 20), v_rel=0.0)
    shown = set()
    for x in 30.0 + rng.normal(0, 0.3, 200):  # 10 s of a vision lead's frame-to-frame noise
      ro.update(True, float(x), float(rng.normal(0, 0.3)), DT)
      shown.add(tuple(ro.lines(V_EGO, False)))
    self.assertLessEqual(len(shown), 2)

  def test_a_real_change_shows(self):
    ro = LeadReadout()
    feed(ro, [30.0] * 20)
    before = ro.distance(True)
    feed(ro, np.linspace(30.0, 25.0, 40))  # closing 5 m over 2 s
    feed(ro, [25.0] * 20)
    self.assertEqual(before, 30.0)
    self.assertEqual(ro.distance(True), 25.0)

  def test_another_lead_shows_at_once(self):
    ro = LeadReadout()
    feed(ro, [60.0] * 20)
    self.assertEqual(ro.distance(True), 60.0)
    feed(ro, [20.0])  # a car cuts in
    self.assertEqual(ro.distance(True), 20.0)

  def test_a_new_lead_starts_from_its_own_numbers(self):
    ro = LeadReadout()
    feed(ro, [60.0] * 20)
    feed(ro, [0.0], present=False)
    feed(ro, [55.0])
    self.assertEqual(ro.distance(True), 55.0)

  def test_far_distances_are_shown_in_coarser_steps(self):
    self.assertEqual(distance_step(40.0, False), 1.0)
    self.assertEqual(distance_step(163.0, False), 5.0)
    self.assertEqual(distance_step(300.0, False), 10.0)
    ro = LeadReadout()
    feed(ro, [50.0] * 20)  # 164 ft
    self.assertEqual(ro.distance(False) % 5.0, 0.0)
    self.assertAlmostEqual(ro.distance(False), 50.0 * FT_PER_M, delta=5.0)

  def test_no_time_gap_while_stopped(self):
    ro = LeadReadout()
    feed(ro, [5.0] * 5)
    self.assertIsNone(ro.gap(0.0))
    self.assertEqual(ro.lines(0.0, False)[2], "---")

  def test_the_setting_picks_the_lines(self):
    ro = LeadReadout()
    feed(ro, [30.0] * 5, v_rel=-2.0)
    self.assertEqual(ro.lines(V_EGO, True), ["30 m", f"{(V_EGO - 2.0) * 3.6:.0f} km/h", f"{30.0 / V_EGO:.1f} s"])
    self.assertEqual(ro.lines(V_EGO, True, speed=False, gap=False), ["30 m"])


if __name__ == "__main__":
  unittest.main()
