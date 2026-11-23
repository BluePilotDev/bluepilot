"""BluePilot's name and images where the sunnypilot build shows sunnypilot's (BluePilotDev/bluepilot#255)."""
import os
import subprocess
import unittest
from unittest import mock

import pyray as rl

from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.ui.bp_spinner import BPSpinner
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.spinner import Spinner
from openpilot.system.ui.sunnypilot.widgets.screen_saver import ScreenSaverSP

ASSETS_DIR = os.path.join(BASEDIR, "openpilot", "selfdrive", "assets")  # gui_app.texture() resolves paths against it
BP_SPINNER_IMAGE = os.path.join(BASEDIR, "bluepilot", "assets", "images", "spinner_bluepilot.png")


class TestScreenSaver(OpenpilotTestCase):
  def test_text_is_bluepilot(self):
    self.assertEqual(ScreenSaverSP().text, "BluePilot")


class TestSpinnerImage(unittest.TestCase):
  def _logo(self, spinner_cls) -> str:
    with mock.patch.object(gui_app, "texture", return_value=rl.Texture()) as texture:
      spinner_cls()
    path = os.path.join(ASSETS_DIR, texture.call_args_list[0].args[0])  # first texture = the logo
    self.assertTrue(os.path.isfile(path), path)
    return os.path.realpath(path)

  def test_stock_spinner_loads_bluepilot_logo(self):
    self.assertEqual(self._logo(Spinner), os.path.realpath(BP_SPINNER_IMAGE))

  def test_build_spinner_loads_bluepilot_logo(self):
    self.assertEqual(self._logo(BPSpinner), os.path.realpath(BP_SPINNER_IMAGE))

  def test_bluepilot_code_leaves_sunnypilot_spinner_alone(self):
    paths = ["bluepilot", "openpilot/selfdrive/ui/bp", "openpilot/system/ui/bp_spinner.py", "scripts", "launch_chffrplus.sh"]
    result = subprocess.run(["git", "grep", "-l", "spinner_sunnypilot", "--", *paths, ":!*/tests/*"],
                            cwd=BASEDIR, capture_output=True, text=True)
    self.assertEqual(result.returncode, 1, result.stdout + result.stderr)  # 1: no match; 0: matches; else git failed


if __name__ == "__main__":
  unittest.main()
