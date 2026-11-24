import unittest

from ..params.param_store import ParamStore
from . import host
from ..core.lateral_curv_ext import LateralCurvExt, PrimaryLateralControl
from ..params.manifest import value
from ..core.values_ext import BP_LATERAL_PARAMS


def _cc(**kv):
  cc = host.CarControlX()
  for k, v in kv.items():
    p = host.CarControlX.Param()
    p.key, p.value = k, v
    cc.params.append(p)
  return cc


class TestParamStore(unittest.TestCase):
  def test_host_adapter_reads_cc_params(self):
    ps = host.params_from_cc(_cc(disable_BP_lat_UI=b"1", LC_PID_gain_UI_curv=b"3.5"))
    self.assertEqual((ps.get("disable_BP_lat_UI"), ps.get("LC_PID_gain_UI_curv"), ps.get("custom_profile_curv")), ("1", "3.5", None))

  def test_reads_bool_int_float_like_params(self):
    ps = ParamStore(dict(disable_BP_lat_UI=b"1", custom_profile_curv=b"2", LC_PID_gain_UI_curv=b"3.5").items())
    self.assertTrue(value(ps, "disable_BP_lat_UI"))
    self.assertEqual(value(ps, "custom_profile_curv"), 2)
    self.assertEqual(value(ps, "LC_PID_gain_UI_curv"), 3.5)

  def test_unset_keys_behave_like_params(self):
    ps = ParamStore(dict().items())
    self.assertIsNone(ps.get("FordLowSpeedFactor_ang"))
    # the manifest default applies to any key the fork did not publish, bools included
    self.assertTrue(value(ps, "enable_human_turn_detection_curv"))
    self.assertEqual(value(ps, "LC_PID_gain_UI_curv"), 1.0)

  def test_empty_params_give_registry_defaults(self):
    """#6: with nothing published, the lateral runs the defaults the settings UI shows
    (params_keys.h in the fork): human-turn detection on, low-curvature PID gain 1.0."""
    ext = type("Ext", (), {})()
    LateralCurvExt.update_lateral_params(ext, ParamStore(dict().items()))
    self.assertTrue(ext.enable_human_turn_detection_curv)
    self.assertEqual(ext.LC_PID_gain_UI_curv, 1.0)
    # a published value still wins
    LateralCurvExt.update_lateral_params(ext, ParamStore(dict(enable_human_turn_detection_curv=b"0").items()))
    self.assertFalse(ext.enable_human_turn_detection_curv)

  def test_lateral_mode_outside_the_enum_falls_back_to_curvature(self):
    """#66: a mode other than 0 or 1 is curvature mode, the default, instead of raising every frame.
    Text that isn't a number already was; 0 and 1 are unchanged."""
    ext = type("Ext", (), {})()
    for published, expected in ((b"0", PrimaryLateralControl.curvature), (b"1", PrimaryLateralControl.angle),
                                (b"2", PrimaryLateralControl.curvature), (b"-1", PrimaryLateralControl.curvature),
                                (b"angle", PrimaryLateralControl.curvature)):
      LateralCurvExt.update_lateral_params(ext, ParamStore(dict(FordPrefLateralControl=published).items()))
      self.assertIs(ext.primary_lateral_control, expected, published)

  def test_bool_wire_format(self):
    # the fork publishes bools as "1"/"0"
    ps = ParamStore(dict(enable_lane_positioning_curv=b"1", enable_human_turn_detection_curv=b"0").items())
    self.assertIs(value(ps, "enable_lane_positioning_curv"), True)
    self.assertIs(value(ps, "enable_human_turn_detection_curv"), False)

  def test_angle_side_values(self):
    ps = ParamStore(dict(FordLowSpeedFactor_ang=b"1.2").items())
    self.assertEqual(value(ps, "FordLowSpeedFactor_ang"), 1.2)

  def test_key_list_is_consistent(self):
    # the count is pinned against the manifest in test_settings_manifest.py
    self.assertEqual(len(BP_LATERAL_PARAMS), len(set(BP_LATERAL_PARAMS)))


if __name__ == "__main__":
  unittest.main()
