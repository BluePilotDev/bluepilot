"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import unittest
from pathlib import Path

from opendbc.bluepilot_lateral.params.manifest import SETTINGS

SETTINGS_UI = Path(__file__).resolve().parents[1] / "settings_ui.json"
OFFROAD_ONLY = ("FordPrefLateralControl", "FordPrefSteerAngleCurvature")


class TestFordSettingsMatchManifest(unittest.TestCase):
  """BluePilot: sunnylink's Ford section holds every setting the lateral library reads (bp-sunnypilot #12),
  with the manifest's limits. The disable toggle stays out: BluePilot is always on in bp-sunnypilot."""

  @classmethod
  def setUpClass(cls):
    cls.items = {i["key"]: i for i in json.loads(SETTINGS_UI.read_text())["vehicle_settings"]["ford"]["items"]}
    cls.manifest = {s.key: s for s in SETTINGS if s.key != "disable_BP_lat_UI"}

  def test_same_keys(self):
    self.assertEqual(set(self.items), set(self.manifest))

  def test_limits_match_the_manifest(self):
    for key, item in self.items.items():
      rng = self.manifest[key].range
      with self.subTest(key=key):
        if item["widget"] == "option":
          self.assertEqual((item["min"], item["max"]), rng)
        elif item["widget"] == "multiple_button":
          values = [o["value"] for o in item["options"]]
          self.assertEqual((min(values), max(values)), rng)

  def test_mode_and_pinion_are_offroad_only(self):
    for key in OFFROAD_ONLY:
      with self.subTest(key=key):
        assert {"type": "offroad_only"} in self.items[key].get("enablement", [])

  def test_mode_settings_show_only_for_their_mode(self):
    for key, item in self.items.items():
      mode = 0 if key.endswith("_curv") else 1 if key.endswith("_ang") else None
      if mode is None:
        continue
      with self.subTest(key=key):
        assert {"type": "param", "key": "FordPrefLateralControl", "equals": mode} in item.get("visibility", [])


if __name__ == "__main__":
  unittest.main()
