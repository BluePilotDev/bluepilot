import unittest

from ..params import manifest
from ..params.manifest import SETTINGS, PER_FRAME_KEYS, startup_value, value
from ..params.param_store import ParamStore
from ..core.values_ext import BP_LATERAL_PARAMS


def _store(**kv):
  return ParamStore(kv.items())


PARAMS_KEYS_H = [
  '{"disable_BP_lat_UI", {PERSISTENT | BACKUP, BOOL, "0"}},',
  '{"enable_human_turn_detection_curv", {PERSISTENT | BACKUP, BOOL, "1"}},',
  '{"enable_lane_positioning_curv", {PERSISTENT | BACKUP, BOOL, "0"}},',
  '{"enable_lane_full_mode_curv", {PERSISTENT | BACKUP, BOOL, "0"}},',
  '{"enable_lane_positioning_ang", {PERSISTENT | BACKUP, BOOL, "0"}},',
  '{"FordPrefLateralControl", {PERSISTENT | BACKUP, INT, "0"}},',
  '{"custom_profile_curv", {PERSISTENT | BACKUP, INT, "0"}},',
  '{"lane_change_factor_high_curv", {PERSISTENT | BACKUP, FLOAT, "0.85"}},',
  '{"pc_blend_ratio_high_C_UI_curv", {PERSISTENT | BACKUP, FLOAT, "0.4"}},',
  '{"pc_blend_ratio_low_C_UI_curv", {PERSISTENT | BACKUP, FLOAT, "0.4"}},',
  '{"custom_path_offset_curv", {PERSISTENT | BACKUP, FLOAT, "0.0"}},',
  '{"LC_PID_gain_UI_curv", {PERSISTENT | BACKUP, FLOAT, "1.0"}},',
  '{"FordLowSpeedFactor_ang", {PERSISTENT | BACKUP, FLOAT, "1.0"}},',
  '{"FordHighSpeedFactor_ang", {PERSISTENT | BACKUP, FLOAT, "1.0"}},',
  '{"FordHighSpeedDampening_ang", {PERSISTENT | BACKUP, FLOAT, "1.0"}},',
  '{"lane_change_factor_high_ang", {PERSISTENT | BACKUP, FLOAT, "1.0"}},',
  '{"custom_path_offset_ang", {PERSISTENT | BACKUP, FLOAT, "0.0"}},',
  '{"lane_centering_strength_ang", {PERSISTENT | BACKUP, FLOAT, "0.25"}},',
  '{"FordPrefSteerAngleCurvature", {PERSISTENT | BACKUP, BOOL, "0"}},',
]


class TestSettingsManifest(unittest.TestCase):
  def test_entries_are_consistent(self):
    keys = [s.key for s in SETTINGS]
    self.assertEqual(len(keys), len(set(keys)))
    for s in SETTINGS:
      with self.subTest(key=s.key):
        self.assertTrue(s.type in (bool, int, float))
        self.assertIs(type(s.default), s.type)
        self.assertTrue(s.read in (manifest.STARTUP, manifest.PER_FRAME, manifest.BOTH))
        self.assertTrue(s.label and s.description)
        if s.range is not None:
          self.assertLessEqual(s.range[0], s.default)
          self.assertLessEqual(s.default, s.range[1])
        self.assertTrue(not s.clamp or s.range is not None)

  def test_per_frame_keys_are_what_hosts_publish(self):
    # hosts (bp-sunnypilot's controlsd) publish BP_LATERAL_PARAMS in CarControlSP.params
    self.assertEqual(BP_LATERAL_PARAMS, PER_FRAME_KEYS)
    self.assertEqual((len(SETTINGS), len(PER_FRAME_KEYS)), (19, 18))

  def test_one_read_rule(self):
    """A published, parseable value is used; missing, empty or unparsable gives the default."""
    self.assertEqual(value(_store(lane_change_factor_high_curv=b"0.7"), "lane_change_factor_high_curv"), 0.7)
    self.assertEqual(value(_store(), "lane_change_factor_high_curv"), 0.85)
    self.assertEqual(value(_store(lane_change_factor_high_curv=b""), "lane_change_factor_high_curv"), 0.85)
    self.assertEqual(value(_store(lane_change_factor_high_curv=b"x"), "lane_change_factor_high_curv"), 0.85)
    self.assertEqual(value(_store(custom_profile_curv=b"1"), "custom_profile_curv"), 1)
    self.assertIs(value(_store(enable_human_turn_detection_curv=b"0"), "enable_human_turn_detection_curv"), False)
    self.assertIs(value(_store(), "enable_human_turn_detection_curv"), True)
    self.assertIs(value(_store(enable_human_turn_detection_curv=b""), "enable_human_turn_detection_curv"), True)

  def test_only_the_angle_factors_are_clamped(self):
    self.assertEqual(value(_store(FordLowSpeedFactor_ang=b"9"), "FordLowSpeedFactor_ang"), 1.5)
    self.assertEqual(value(_store(lane_centering_strength_ang=b"-1"), "lane_centering_strength_ang"), 0.0)
    self.assertEqual(value(_store(lane_change_factor_high_curv=b"9"), "lane_change_factor_high_curv"), 9.0)

  def test_startup_values(self):
    # get_car's params dict holds whatever Params returned; a restored backup can hold text
    for on in (True, 1, "1", "true", " True "):
      self.assertIs(startup_value({"disable_BP_lat_UI": on}, "disable_BP_lat_UI"), True, on)
    for off in (False, 0, None, "0", "false", "", "junk"):
      self.assertIs(startup_value({"disable_BP_lat_UI": off}, "disable_BP_lat_UI"), False, off)
    self.assertIs(startup_value({}, "FordPrefSteerAngleCurvature"), False)

  def test_params_keys_lines(self):
    """Pins every key, type and default: these are bp-sunnypilot's params_keys.h lines (#6), so a typo
    in a default can't change what the car runs unnoticed."""
    self.assertEqual(manifest.params_keys_lines(), PARAMS_KEYS_H)

  def test_clamped_ranges(self):
    """The bounds the library clamps to (the angle-mode factors, as BluePilot's own fork does)."""
    self.assertEqual({s.key: s.range for s in SETTINGS if s.clamp}, {
      "FordLowSpeedFactor_ang": (0.5, 1.5), "FordHighSpeedFactor_ang": (0.5, 1.5), "FordHighSpeedDampening_ang": (0.25, 1.25),
      "lane_change_factor_high_ang": (0.85, 1.5), "custom_path_offset_ang": (-0.5, 0.5), "lane_centering_strength_ang": (0.0, 1.0),
    })

  def test_read_kind_is_enforced(self):
    with self.assertRaises(AssertionError):
      value(_store(), "FordPrefSteerAngleCurvature")  # startup only
    with self.assertRaises(AssertionError):
      startup_value({}, "FordLowSpeedFactor_ang")  # per frame only


if __name__ == "__main__":
  unittest.main()
