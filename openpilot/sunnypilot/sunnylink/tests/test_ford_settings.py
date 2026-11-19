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


class TestFordSettingsMatchManifest(unittest.TestCase):
  """BluePilot: sunnylink's Ford section holds every setting the lateral library reads, with the
  manifest's label and text. Each is read once at car start, so each is marked as needing a restart."""

  @classmethod
  def setUpClass(cls):
    cls.items = {i["key"]: i for i in json.loads(SETTINGS_UI.read_text())["vehicle_settings"]["ford"]["items"]}

  def test_every_manifest_setting_is_listed(self):
    for setting in SETTINGS:
      with self.subTest(key=setting.key):
        item = self.items[setting.key]
        self.assertEqual(item["widget"], "toggle")
        self.assertTrue(item["title"].endswith(setting.label))
        self.assertEqual(item["description"], setting.description)
        self.assertTrue(item["needs_onroad_cycle"])


if __name__ == "__main__":
  unittest.main()
