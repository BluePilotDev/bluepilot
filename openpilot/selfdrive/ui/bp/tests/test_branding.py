"""BluePilot's name and images where the sunnypilot build shows sunnypilot's (BluePilotDev/bluepilot#255)."""
import unittest

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.ui.sunnypilot.widgets.screen_saver import ScreenSaverSP


class TestScreenSaver(OpenpilotTestCase):
  def test_text_is_bluepilot(self):
    self.assertEqual(ScreenSaverSP().text, "BluePilot")


if __name__ == "__main__":
  unittest.main()
