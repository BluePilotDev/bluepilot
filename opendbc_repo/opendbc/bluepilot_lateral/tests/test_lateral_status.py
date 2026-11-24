import unittest
from dataclasses import fields
from types import SimpleNamespace

from ...can import CANPacker
from ...car.ford import fordcan
from ...car.ford.interface import CarInterface
from ...car.ford.values import CAR
from ..core.carcontroller_ext import BluePilotLateral
from ..core.lateral_curv_ext import LateralResult, PrimaryLateralControl
from ..core.values_ext import FordSafetyFlagsSP
from ..host_types import LateralStatus

# the names BP7's controllerStateBP publisher reads off its car controller (bp_card_publisher.py):
# a rename or a dropped value breaks BP7's UI
STATUS_NAMES = ["lateralUncertainty", "angleRateLimited", "curvatureRateLimited", "curvatureDeviationLimited",
                "humanTurnLateralPaused", "stallBlipActive", "disable_BP_lat_UI", "primary_lateral_control"]


def _bp():
  CP = CarInterface.get_non_essential_params(CAR.FORD_F_150_MK14)
  return BluePilotLateral(CP, FordSafetyFlagsSP.BP_LATERAL, CANPacker("ford_lincoln_base_pt"), fordcan.CanBus(CP))


def _result(uncertainty):
  return LateralResult(apply_curvature=0.0, curvature_rate=0.0, path_offset=0.0, path_angle=0.0, ramp_type=0,
                       precision_type=1, lateralUncertainty=uncertainty)


class TestLateralStatus(unittest.TestCase):
  def setUp(self):
    self.bp = _bp()
    self.bp.angle.bp_angle_rate_limited = True
    self.bp.angle.bp_curvature_rate_limited = True
    self.bp.angle.angle_human_turn_active = True
    self.bp.angle.angle_stall_blip_active = True
    self.bp.shared.bp_curvature_deviation_limited = True
    self.bp.disable_BP_lat_UI = True

  def test_names(self):
    self.assertEqual([f.name for f in fields(LateralStatus)], STATUS_NAMES)

  def test_angle_values_read_false_outside_angle_mode(self):
    # as BP7 sets them: only angle mode runs the path_angle clip, the rate sim, the human-turn pause and the blip
    self.bp.curv.primary_lateral_control = PrimaryLateralControl.curvature
    s = self.bp.status()
    self.assertEqual((s.angleRateLimited, s.curvatureRateLimited, s.humanTurnLateralPaused, s.stallBlipActive),
                     (False, False, False, False))
    self.assertTrue(s.curvatureDeviationLimited)  # either mode's strategy sets it
    self.assertTrue(s.disable_BP_lat_UI)
    self.assertEqual(s.primary_lateral_control, 0)

  def test_angle_mode_values(self):
    self.bp.curv.primary_lateral_control = PrimaryLateralControl.angle
    s = self.bp.status()
    self.assertEqual((s.angleRateLimited, s.curvatureRateLimited, s.humanTurnLateralPaused, s.stallBlipActive),
                     (True, True, True, True))
    self.assertEqual(s.primary_lateral_control, 1)
    self.assertIs(type(s.primary_lateral_control), int)

  def test_each_value_from_its_own_source(self):
    # one source at a time, so two swapped sources can't pass
    sources = {"angleRateLimited": (self.bp.angle, "bp_angle_rate_limited"),
               "curvatureRateLimited": (self.bp.angle, "bp_curvature_rate_limited"),
               "humanTurnLateralPaused": (self.bp.angle, "angle_human_turn_active"),
               "stallBlipActive": (self.bp.angle, "angle_stall_blip_active"),
               "curvatureDeviationLimited": (self.bp.shared, "bp_curvature_deviation_limited"),
               "disable_BP_lat_UI": (self.bp, "disable_BP_lat_UI")}
    self.bp.curv.primary_lateral_control = PrimaryLateralControl.angle
    for field, (obj, attr) in sources.items():
      for o, a in sources.values():
        setattr(o, a, False)
      setattr(obj, attr, True)
      s = self.bp.status()
      self.assertEqual([f for f in sources if getattr(s, f)], [field], field)

  def test_uncertainty_from_whichever_strategy_ran(self):
    self.bp.angle.angle_human_turn_active = self.bp.angle.angle_stall_blip_active = False
    self.bp.angle.update_angle_strategy = lambda *a: _result(0.25)
    self.bp.curv.update = lambda *a: _result(0.5)
    cc = SimpleNamespace(latActive=True)
    for mode, expected in ((PrimaryLateralControl.angle, 0.25), (PrimaryLateralControl.curvature, 0.5)):
      self.bp.curv.primary_lateral_control = mode
      self.bp.step_lateral(cc, None, None, 0.0)
      self.assertEqual(self.bp.status().lateralUncertainty, expected, mode)


if __name__ == "__main__":
  unittest.main()
