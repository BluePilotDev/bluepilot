import unittest
from types import SimpleNamespace

from ...car.ford.values import CAR
from ..core.lateral_angle_ext import LateralAngleExt
from ..core.values_ext import (
  ANGLE_GAIN_CANFD_BOF_CARS, ANGLE_GAIN_CANFD_SUV_CARS, ANGLE_GAIN_EXPEDITION, ANGLE_GAIN_EXPEDITION_CARS,
  FORD_HOST_ONLY_PLATFORMS, FORD_NO_PINION_PLATFORMS,
  FORD_PINION_GEOMETRY_INDEX, FORD_PINION_WHEELBASE,
)

TABLES = {
  "FORD_PINION_GEOMETRY_INDEX": FORD_PINION_GEOMETRY_INDEX,
  "FORD_PINION_WHEELBASE": FORD_PINION_WHEELBASE,
  "FORD_NO_PINION_PLATFORMS": FORD_NO_PINION_PLATFORMS,
  "ANGLE_GAIN_CANFD_BOF_CARS": ANGLE_GAIN_CANFD_BOF_CARS,
  "ANGLE_GAIN_CANFD_SUV_CARS": ANGLE_GAIN_CANFD_SUV_CARS,
  "ANGLE_GAIN_EXPEDITION_CARS": ANGLE_GAIN_EXPEDITION_CARS,
}


class TestPlatformTables(unittest.TestCase):
  def test_names_are_known_platforms(self):
    """Every per-platform row is keyed by a plain platform name, so a host without that platform
    imports fine, and a name this host's CAR doesn't have must be a declared host-only one: a typo
    would otherwise silently drop the platform's row. (A host that has a host-only platform, e.g.
    BP7 with Mondeo, sees it in CAR, so this holds there too.)"""
    here = {str(c) for c in CAR}
    for table, names in TABLES.items():
      for name in names:
        with self.subTest(table=table, name=name):
          self.assertIs(type(name), str)
          self.assertTrue(name in here or name in FORD_HOST_ONLY_PLATFORMS, name)

  def test_gain_groups_do_not_overlap(self):
    """update_angle_params takes the first group a platform is in (Expedition, then body-on-frame,
    then SUV), so a platform in two would silently get the earlier group's gains."""
    groups = (ANGLE_GAIN_EXPEDITION_CARS, ANGLE_GAIN_CANFD_BOF_CARS, ANGLE_GAIN_CANFD_SUV_CARS)
    for i, a in enumerate(groups):
      for b in groups[i + 1:]:
        self.assertFalse(a & b)

  def test_expedition_keeps_bluepilot_7s_base_gain(self):
    """BluePilotDev/bluepilot#220: 1.5x the body-on-frame gains, so BP7 settings carry over. The
    literal is pinned, and the Expedition is alone in its group."""
    self.assertEqual(ANGLE_GAIN_EXPEDITION_CARS, frozenset({"FORD_EXPEDITION_MK4", "FORD_EXPEDITION_MK5_5"}))  # TRON test branch
    self.assertAlmostEqual(ANGLE_GAIN_EXPEDITION[0], 1.425)
    self.assertAlmostEqual(ANGLE_GAIN_EXPEDITION[1], 1.425)

  def test_update_angle_params_selects_the_expedition_gain(self):
    """Ported from BP7's test for #220: the Expedition gets its own base gain from
    update_angle_params, and the other body-on-frame trucks keep theirs."""
    for fingerprint, gains in (("FORD_EXPEDITION_MK4", (1.425, 1.425)), ("FORD_F_150_LIGHTNING_MK1", (0.95, 0.95)),
                               ("FORD_F_150_MK14", (0.95, 0.95))):
      with self.subTest(fingerprint=fingerprint):
        ext = SimpleNamespace(CP=SimpleNamespace(carFingerprint=fingerprint))
        LateralAngleExt.update_angle_params(ext, None)
        self.assertAlmostEqual(ext.path_angle_gain_lowC_highV, gains[0])
        self.assertAlmostEqual(ext.path_angle_gain_highC_highV, gains[1])

  def test_no_pinion_platforms_have_no_row(self):
    self.assertFalse(FORD_NO_PINION_PLATFORMS & set(FORD_PINION_GEOMETRY_INDEX))


if __name__ == "__main__":
  unittest.main()
