"""The 3X debug panel's data cards against the current message definitions (no window: fonts stubbed).

The panel swallows AttributeError per tab, so a renamed or removed field leaves that card and every card after
it blank instead of crashing. Every card must get rows from full messages.
"""
import unittest
from unittest import mock

import pyray as rl

from openpilot.cereal import messaging
from openpilot.common.test import OpenpilotTestCase
from openpilot.system.ui.lib.application import gui_app

SERVICES = ("carState", "carOutput", "radarState", "carParams", "deviceState")


class _SM(dict):
  """Every service valid, each a default message; a lead is present and the car uses torque tuning."""
  def __init__(self):
    super().__init__({s: getattr(messaging.new_message(s), s) for s in SERVICES})
    self["radarState"].leadOne.present = True
    self["carParams"].lateralTuning.init("torque")
    self.valid = dict.fromkeys(SERVICES, True)


def make_panel():
  from openpilot.bluepilot.ui.widgets.debug.other_debug_panel import OtherDebugPanel
  return OtherDebugPanel()


class TestOtherDebugPanel(OpenpilotTestCase):
  def setUp(self):
    from openpilot.selfdrive.ui.ui_state import ui_state  # inside the test's params prefix: building it writes params
    for patch in (mock.patch.object(gui_app, "font", return_value=rl.Font()), mock.patch.object(ui_state, "sm", _SM())):
      patch.start()
      self.addCleanup(patch.stop)

  def test_every_card_gets_rows(self):
    panel = make_panel()
    for tab, cards in panel._tab_cards.items():
      panel._update_tab_data(tab)
      for card in cards:
        with self.subTest(tab=tab, card=card._title):
          self.assertGreater(card.get_height(), card.HEADER_HEIGHT + card.PADDING * 2, "no rows")
          self.assertNotIn(("Status", "N/A"), card._rows)  # a card's own fallback when one of its fields is gone

  def test_lead_status_reads_present(self):
    panel = make_panel()
    panel._update_tab_data(1)
    self.assertEqual(dict(panel._tab_cards[1][0]._rows)["Status"], "Tracking")


if __name__ == "__main__":
  unittest.main()
