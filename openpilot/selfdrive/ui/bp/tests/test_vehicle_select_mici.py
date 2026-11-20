"""BluePilot's comma four vehicle selector: the confirmation says when the choice applies."""
import unittest
from unittest import mock

import pyray as rl

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.ui.lib.application import gui_app


class TestVehicleSelectMici(OpenpilotTestCase):
  def _confirm_title(self, started: bool) -> str:
    from openpilot.selfdrive.ui.ui_state import ui_state  # inside the test's params prefix: building it writes params
    from openpilot.selfdrive.ui.bp.mici.widgets import vehicle_select_mici as mod
    titles = []
    with mock.patch.object(gui_app, "font", return_value=rl.Font()), \
         mock.patch.object(gui_app, "texture", return_value=rl.Texture()), \
         mock.patch.object(gui_app, "push_widget"), \
         mock.patch.object(mod, "BigConfirmationDialog", lambda title, *a, **k: titles.append(title)), \
         mock.patch.object(ui_state, "started", started):
      mod.VehicleModelSelectMici({}, [])._ask_confirm("FORD_F_150_MK14")
    return titles[0]

  def test_offroad_applies_now(self):
    self.assertEqual(self._confirm_title(started=False), "slide to\napply now")

  def test_onroad_applies_when_offroad(self):
    self.assertEqual(self._confirm_title(started=True), "slide to\napply when offroad")


if __name__ == "__main__":
  unittest.main()
