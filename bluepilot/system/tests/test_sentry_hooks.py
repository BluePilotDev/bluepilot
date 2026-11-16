"""The crash-reporting hooks in upstream files fall back to upstream's code when reporting doesn't run."""
import os
import tempfile
import unittest
from unittest import mock

from openpilot.system import tombstoned
from openpilot.system.manager import manager


class StopLoop(Exception):
  pass


class TestManagerHook(unittest.TestCase):
  def crash(self, reported: bool) -> mock.Mock:
    sentry = mock.Mock()
    sentry.capture_exception.return_value = reported
    with mock.patch.multiple(manager, sentry=sentry, cloudlog=mock.DEFAULT, manager_init=mock.DEFAULT, manager_cleanup=mock.DEFAULT,
                             manager_thread=mock.Mock(side_effect=RuntimeError("manager crash")), Params=mock.DEFAULT) as patched, \
         mock.patch.object(manager.signal, "signal"), mock.patch.object(manager.traceback, "print_exc"), \
         mock.patch.dict(os.environ, {}, clear=False):
      os.environ.pop("PREPAREONLY", None)
      patched["Params"].return_value.get_bool.return_value = False
      manager.main()
    sentry.capture_exception.assert_called_once_with()
    return patched["cloudlog"]

  def test_reported_crash_is_not_logged_again(self):
    self.crash(reported=True).exception.assert_not_called()

  def test_unreported_crash_gets_upstreams_log_line(self):
    self.crash(reported=False).exception.assert_called_once_with("crash")


class TestTombstonedHook(unittest.TestCase):
  def setUp(self):
    self.tmp = self.enterContext(tempfile.TemporaryDirectory())
    self.apport = os.path.join(self.tmp, "_usr_bin_ui.crash")
    with open(self.apport, "w") as f:
      f.write("ExecutablePath: /data/openpilot/selfdrive/ui/ui\nSignal: 11\n")

  def run_main(self, reporting: bool) -> mock.Mock:
    """One pass of tombstoned's loop on a build upstream wouldn't report from; returns report_tombstone_apport."""
    sentry = mock.Mock()
    sentry.init.return_value = reporting
    with mock.patch.multiple(tombstoned, sentry=sentry, is_registered_device=mock.Mock(return_value=False), clear_apport_folder=mock.DEFAULT,
                             get_tombstones=mock.Mock(side_effect=[[], [(self.apport, 0)]]), report_tombstone_apport=mock.DEFAULT,
                             cloudlog=mock.DEFAULT) as patched, \
         mock.patch.object(tombstoned.time, "sleep", side_effect=StopLoop):
      self.assertRaises(StopLoop, tombstoned.main)
    sentry.init.assert_called_once_with("SELFDRIVE_NATIVE")
    return patched["report_tombstone_apport"]

  def test_bluepilot_reports_tombstones_on_every_build(self):
    self.run_main(reporting=True).assert_called_once_with(self.apport)
    self.assertTrue(os.path.isfile(self.apport))

  def test_without_reporting_upstreams_rule_removes_them(self):
    self.run_main(reporting=False).assert_not_called()
    self.assertFalse(os.path.isfile(self.apport))

  def tombstone_log_lines(self, reported: bool) -> list:
    sentry = mock.Mock()
    sentry.report_tombstone_file.return_value = reported
    stacktrace = mock.Mock(return_value="#0\n#1 crash_here\n#2")
    with mock.patch.multiple(tombstoned, sentry=sentry, cloudlog=mock.DEFAULT, get_apport_stacktrace=stacktrace) as patched, \
         mock.patch.object(tombstoned.Paths, "log_root", lambda: self.tmp):
      tombstoned.report_tombstone_apport(self.apport)
    sentry.report_tombstone_file.assert_called_once()
    return [c for c in patched["cloudlog"].error.call_args_list if "tombstone" in c.args[0]]

  def test_reported_tombstone_is_not_logged_again(self):
    self.assertEqual(self.tombstone_log_lines(reported=True), [])  # report_tombstone logged it

  def test_unreported_tombstone_gets_upstreams_log_line(self):
    self.assertEqual(len(self.tombstone_log_lines(reported=False)), 1)


if __name__ == "__main__":
  unittest.main()
