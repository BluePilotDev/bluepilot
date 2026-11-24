import unittest

from openpilot.selfdrive.ui.bp.lib.hev_text import hev_engine_on_reason_text, hev_power_flow_text


class TestHevText(unittest.TestCase):
  # expected strings are BP7's (get_hev_power_flow_text / get_hev_engine_on_reason_text)
  def test_power_flow_text(self):
    self.assertEqual(hev_power_flow_text(1), "Hybrid Drive")
    self.assertEqual(hev_power_flow_text(11.0), "Regen Braking")
    self.assertEqual(hev_power_flow_text(0), "")
    self.assertEqual(hev_power_flow_text(16), "Unknown")

  def test_engine_on_reason_text(self):
    self.assertEqual(hev_engine_on_reason_text(5), "Engine Cold")
    self.assertEqual(hev_engine_on_reason_text(13), "Drive Mode")
    self.assertEqual(hev_engine_on_reason_text(0), "")
    self.assertEqual(hev_engine_on_reason_text(14), "Unknown")


if __name__ == "__main__":
  unittest.main()
