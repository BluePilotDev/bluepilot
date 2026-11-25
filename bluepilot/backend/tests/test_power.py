import unittest
from unittest import mock

from bluepilot.backend.utils import power


class TestRestorePowerSave(unittest.TestCase):
  def setUp(self):
    """Record calls to restore_power_save instead of writing to sysfs."""
    self.restored = []
    patcher = mock.patch.object(power, "restore_power_save", lambda: self.restored.append(True))
    patcher.start()
    self.addCleanup(patcher.stop)

  def idle_for(self, seconds):
    patcher = mock.patch.object(power, "last_activity_time", power.time.time() - seconds)
    patcher.start()
    self.addCleanup(patcher.stop)

  def test_restores_power_save_when_idle_and_offroad(self):
    self.idle_for(power.IDLE_TIMEOUT_SECONDS + 1)
    power.check_and_restore_power_save(lambda: False)
    self.assertEqual(self.restored, [True])

  def test_never_restores_power_save_while_onroad(self):
    self.idle_for(power.IDLE_TIMEOUT_SECONDS + 1)
    power.check_and_restore_power_save(lambda: True)
    self.assertEqual(self.restored, [])

  def test_keeps_performance_mode_until_idle_timeout(self):
    self.idle_for(power.IDLE_TIMEOUT_SECONDS - 60)
    power.check_and_restore_power_save(lambda: False)
    self.assertEqual(self.restored, [])
