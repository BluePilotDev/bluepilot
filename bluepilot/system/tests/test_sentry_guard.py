"""The guard upstream files call crash reporting through: it can never stop the process."""
import copy
import sys
import unittest
from unittest import mock

from openpilot.common import bluepilot
from openpilot.common.bluepilot import CrashReporting

MODULE = "openpilot.bluepilot.system.sentry"


class TestSentryGuard(unittest.TestCase):
  def setUp(self):
    self.module = mock.MagicMock(name="sentry")
    modules = mock.patch.dict(sys.modules, {MODULE: self.module})
    modules.start()
    self.addCleanup(modules.stop)
    is_bluepilot = mock.patch.object(bluepilot, "is_bluepilot", return_value=True)
    self.is_bluepilot = is_bluepilot.start()
    self.addCleanup(is_bluepilot.stop)

  def test_calls_reach_the_module_on_bluepilot(self):
    guard = CrashReporting()
    self.assertTrue(guard.init("SELFDRIVE"))
    self.module.init.assert_called_once_with(self.module.SentryProject["SELFDRIVE"])
    self.assertTrue(guard.set_tag("daemon", "card"))
    self.module.set_tag.assert_called_once_with("daemon", "card")

  def test_does_nothing_when_not_bluepilot(self):
    self.is_bluepilot.return_value = False
    guard = CrashReporting()
    self.assertFalse(guard.init("SELFDRIVE"))
    self.assertFalse(guard.capture_exception())
    self.module.init.assert_not_called()
    self.module.capture_exception.assert_not_called()

  def test_does_nothing_when_the_module_fails_to_import(self):
    with mock.patch.dict(sys.modules, {MODULE: None}):
      guard = CrashReporting()
      self.assertFalse(guard.init("SELFDRIVE"))
      self.assertFalse(guard.set_tag("daemon", "card"))
      self.assertFalse(guard.capture_exception())

  def test_does_nothing_when_the_sdk_is_missing(self):
    del sys.modules[MODULE]
    with mock.patch.dict(sys.modules, {"sentry_sdk": None}):
      guard = CrashReporting()
      self.assertFalse(guard.init("SELFDRIVE"))
      self.assertFalse(guard.report_tombstone("fn", "message", "contents"))

  def test_does_nothing_without_an_active_client(self):
    self.module.is_active.return_value = False  # loaded, but this process never started reporting
    guard = CrashReporting()
    self.assertFalse(guard.capture_exception())
    self.assertFalse(guard.set_tag("daemon", "card"))
    self.module.capture_exception.assert_not_called()
    self.module.set_tag.assert_not_called()
    self.assertTrue(guard.init("SELFDRIVE"))

  def test_private_names_are_not_calls(self):
    guard = CrashReporting()
    self.assertFalse(hasattr(guard, "_x"))
    with mock.patch.object(bluepilot, "_log_failure") as log_failure:
      copy.copy(guard)
      copy.deepcopy(guard)
    log_failure.assert_not_called()

  def test_does_nothing_after_init_fails(self):
    self.module.init.side_effect = RuntimeError("bad DSN")
    guard = CrashReporting()
    self.assertFalse(guard.init("SELFDRIVE"))
    self.assertFalse(guard.capture_exception())
    self.module.capture_exception.assert_not_called()

  def test_a_failing_call_does_not_raise(self):
    self.module.report_tombstone.side_effect = OSError("disk full")
    guard = CrashReporting()
    self.assertFalse(guard.report_tombstone("fn", "message", "contents"))
    self.assertTrue(guard.set_tag("daemon", "card"))


if __name__ == "__main__":
  unittest.main()
