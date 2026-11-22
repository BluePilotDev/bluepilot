"""
The home screen's Driving Models card (selfdrive/ui/bp/lib/driving_models.py): which models it shows, which drives,
and what it says about JetLink's link. Pure view logic, so no JetLink or UI is needed; the StatusReader's USB speed is
read through a patched jetlink_adapter.
"""
import types
import unittest
from unittest import mock

from openpilot.bluepilot import jetlink_adapter
from openpilot.selfdrive.ui.bp.lib import driving_models as dm, jetlink_status


def status(**fields):
  base = {'phase': 'off', 'progress': None, 'mode': 'usb', 'port_state': None, 'transport': 'USB', 'reason': None,
          'enabled': True, 'active_model': None, 'model': None, 'holds_port': False, 'shown': True, 'default_model': None}
  return types.SimpleNamespace(**{**base, **fields})


def texts(tile):
  return [n.text for n in tile.notes]


class TestView(unittest.TestCase):
  def test_no_big_model(self):
    v = dm.view(None)
    self.assertIsNone(v.big)
    self.assertEqual((v.small.name, v.small.tag), (dm.DEFAULT_SMALL, ""))
    # JetLink checked out but the link off and nothing to say: still no big model
    self.assertIsNone(dm.view("Mine", status(shown=False)).big)

  def test_ios_ready_on_usb2(self):
    s = status(phase='prepared', mode='ios', port_state='connected', transport='iOS over USB (192.168.60.12)',
               model='Light Speed 2', active_model='Light Speed 2')
    v = dm.view("Small One", s, 'high-speed')
    self.assertEqual(v.big.role, dm.BIG)
    self.assertEqual((v.big.name, v.big.tag, v.big.tone), ("Light Speed 2", "Ready", dm.READY))
    self.assertEqual(v.big.facts, (dm.Fact("Source", "JetLink iOS"), dm.Fact("Link", "iOS over USB")))
    self.assertEqual((v.big.badge, v.big.badge_tone), ("USB2", dm.WORK))
    self.assertEqual(v.big.notes, ())
    self.assertFalse(v.big.driving)
    self.assertEqual((v.small.name, v.small.tag, v.small.standby), ("Small One", "Fallback", False))

  def test_driving_on_usb3(self):
    v = dm.view(None, status(phase='active', active_model='Big', port_state='connected'), 'super-speed')
    self.assertTrue(v.big.driving)
    self.assertEqual((v.big.badge, v.big.badge_tone), ("USB3", dm.DRIVE))
    self.assertEqual((v.small.tag, v.small.standby), ("Standby", True))

  def test_no_speed_no_badge(self):
    self.assertEqual(dm.view(None, status(phase='prepared', model='Big')).big.badge, "")

  def test_preparing_shows_progress(self):
    s = status(phase='preparing', model='Big', progress={'stage': 'compile', 'frac': 0.42, 'msg': 'about 2 min left'})
    v = dm.view(None, s)
    self.assertEqual(v.big.tag, "Preparing 42%")
    self.assertAlmostEqual(v.big.progress, 0.42)
    self.assertEqual(texts(v.big), ["about 2 min left"])
    self.assertEqual(v.big.notes[0].tone, dm.WORK)

  def test_not_built_yet_and_port(self):
    v = dm.view(None, status(phase='prepared', model='Next', port_state='empty'))
    self.assertEqual(texts(v.big), ["Nothing on the USB port", dm.NOT_BUILT])
    self.assertIsNone(v.big.progress)
    self.assertEqual([f.label for f in v.big.facts], ["Source"])

  def test_failed_and_unavailable(self):
    v = dm.view(None, status(phase='failed', model='Big'))
    self.assertEqual((v.big.tag, v.big.tone), ("Failed", dm.ERROR))
    v = dm.view(None, status(phase='unavailable', shown=False, reason="no warp built for this camera"))
    self.assertEqual(v.big.notes[0], dm.Note("no warp built for this camera", dm.ERROR))

  def test_unknown_phase_and_default_name(self):
    v = dm.view(None, status(phase='a phase from a newer jetlink', default_model='Default Big'))
    self.assertEqual((v.big.tag, v.big.name), ("Working", "Default Big"))
    self.assertEqual(dm.view(None, status(phase='prepared')).big.name, dm.NO_BIG_PICK)

  def test_chestnut(self):
    v = dm.view(None, chestnut_state='ready', chestnut_name='Nutty')
    self.assertEqual((v.big.name, v.big.tag, v.big.facts), ("Nutty", "Ready", (dm.Fact("Source", "chestnut"),)))
    self.assertTrue(dm.view(None, chestnut_state='active').big.driving)
    self.assertIsNone(dm.view(None, chestnut_state='disconnected').big)

  def test_jetlink_wins_over_a_chestnut_guess(self):
    # a JetLink host on the port can look like a chestnut to ui_state; JetLink's tile is the one shown
    v = dm.view(None, status(phase='prepared', model='Big'), chestnut_state='ready', chestnut_name='Nutty')
    self.assertEqual(v.big.facts[0].value, "JetLink")

  def test_every_phase_has_a_tag(self):
    for phase in jetlink_status.PHASE_LABELS:
      self.assertIn(phase, dm.JETLINK_TAGS)


class TestSpeed(unittest.TestCase):
  def test_labels(self):
    self.assertEqual(jetlink_status.speed_label('super-speed-plus'), "USB 3")
    self.assertEqual(jetlink_status.speed_label('super-speed'), "USB 3")
    self.assertEqual(jetlink_status.speed_label('high-speed'), "USB 2")
    self.assertIsNone(jetlink_status.speed_label(None))
    self.assertIsNone(jetlink_status.speed_label('UNKNOWN'))

  def test_reader_reads_speed_only_when_asked_and_with_status(self):
    calls = []
    with mock.patch.object(jetlink_adapter, 'status', lambda: status()), \
         mock.patch.object(jetlink_adapter, 'usb_speed', lambda: calls.append(1) or 'high-speed'):
      self.assertIsNone(jetlink_status.StatusReader().speed())
      self.assertEqual(calls, [])
      self.assertEqual(jetlink_status.StatusReader(speed=True).speed(), 'high-speed')
    with mock.patch.object(jetlink_adapter, 'status', lambda: None), \
         mock.patch.object(jetlink_adapter, 'usb_speed', lambda: 'high-speed'):
      self.assertIsNone(jetlink_status.StatusReader(speed=True).speed())

  def test_usb_speed_off_without_link(self):
    with mock.patch.object(jetlink_adapter, 'link_on', lambda: False):
      self.assertIsNone(jetlink_adapter.usb_speed())


if __name__ == '__main__':
  unittest.main()
