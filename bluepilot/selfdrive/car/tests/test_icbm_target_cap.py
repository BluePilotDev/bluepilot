"""ICBM's target cap: an unset or zero target holds the cluster set speed instead of pressing toward it."""
import unittest

from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement


def icbm_target(target_kph: float, cluster: float, is_metric: bool) -> int:
  """ICBM's target in display units for a planner target in kph and a cluster set speed in display units."""
  unit = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
  icbm = IntelligentCruiseButtonManagement(car.CarParams(), custom.CarParamsSP(pcmCruiseSpeed=False))
  CS = car.CarState(cruiseState={"available": True, "enabled": True, "speedCluster": cluster * unit})
  icbm.run(CS, car.CarControl(enabled=True), custom.LongitudinalPlanSP(vTarget=target_kph * CV.KPH_TO_MS), is_metric)
  return icbm.v_target


class TestIcbmTargetCap(unittest.TestCase):
  def test_unset_target_holds_cluster(self):
    # the 255 kph unset value and the 145 kph maximum, imperial and metric
    for target_kph in (255., 145.):
      self.assertEqual(icbm_target(target_kph, 35, is_metric=False), 35)
      self.assertEqual(icbm_target(target_kph, 60, is_metric=True), 60)

  def test_zero_target_holds_cluster(self):
    self.assertEqual(icbm_target(0., 35, is_metric=False), 35)
    self.assertEqual(icbm_target(0., 60, is_metric=True), 60)

  def test_valid_target_passes_through(self):
    self.assertEqual(icbm_target(50 * CV.MPH_TO_KPH, 35, is_metric=False), 50)
    self.assertEqual(icbm_target(89 * CV.MPH_TO_KPH, 35, is_metric=False), 89)
    self.assertEqual(icbm_target(80., 60, is_metric=True), 80)
    self.assertEqual(icbm_target(144., 60, is_metric=True), 144)

  def test_no_cluster_speed_leaves_target(self):
    self.assertEqual(icbm_target(255., 0, is_metric=False), round(255 * CV.KPH_TO_MPH))


if __name__ == "__main__":
  unittest.main()
