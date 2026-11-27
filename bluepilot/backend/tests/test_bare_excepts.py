"""The portal's guarded reads keep their defaults on the failures they expect, and no longer swallow Ctrl-C (#205)."""
import os
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from openpilot.common.params import UnknownKeyName

from bluepilot.backend.network import utils
from bluepilot.backend.routes import parsing, scanner, segments  # routes before storage: the two packages import each other
from bluepilot.backend.storage import preservation


def failing_params(side_effect):
  return mock.patch.object(utils, "params", mock.Mock(get_bool=mock.Mock(side_effect=side_effect)))


def run_should_server_run(side_effect):
  with failing_params(side_effect):
    return utils.should_server_run()


def run_get_wifi_ip(side_effect):
  # Without netifaces the portal falls back to `ip addr`
  with mock.patch.dict(sys.modules, {"netifaces": None}), mock.patch("subprocess.run", side_effect=side_effect):
    return utils.get_wifi_ip()


def run_get_connection_type(side_effect):
  with mock.patch("subprocess.run", side_effect=side_effect):
    return utils.get_connection_type()


def run_scan_dongle_route(side_effect):
  # A route named by dongle ID has no date in its name, so the scanner falls back to the folder's mtime
  with tempfile.TemporaryDirectory() as routes:
    os.mkdir(os.path.join(routes, "0123456789abcdef--0"))
    with mock.patch.object(scanner, "ROUTES_DIR", routes), \
         mock.patch.object(scanner, "get_cached_deletion_data", return_value={}), \
         mock.patch.object(scanner, "get_disk_space_info", return_value={}), \
         mock.patch.object(scanner, "datetime", mock.Mock(fromtimestamp=mock.Mock(side_effect=side_effect))):
      return scanner.scan_routes()


def run_get_file_size(side_effect):
  # The file is listed, then gone (or unreadable) when its size is read
  with tempfile.TemporaryDirectory() as segment:
    open(os.path.join(segment, "rlog.zst"), "w").close()
    with mock.patch.object(segments.os.path, "getsize", side_effect=side_effect):
      return segments.get_file_size(segment)


def run_has_preserve_xattr_segment(side_effect):
  with mock.patch.object(preservation, "getxattr", side_effect=side_effect):
    return preservation.has_preserve_xattr_segment("/no/such/segment")


def run_has_lock_file(side_effect):
  with mock.patch.object(preservation.os, "listdir", side_effect=side_effect):
    return preservation.has_lock_file("0123456789abcdef--0")


def short_inet_line(*args, **kwargs):
  return subprocess.CompletedProcess(args[0], 0, stdout="    inet\n", stderr="")


TIMEOUT = subprocess.TimeoutExpired(["ip"], 2)

# (site, call it with the guarded call failing, failures it expects, default it gives)
SITES = [
  ("should_server_run", run_should_server_run, [UnknownKeyName("EnableWebRoutesServer"), RuntimeError("params unavailable")], True),
  ("get_wifi_ip", run_get_wifi_ip, [FileNotFoundError("ip"), TIMEOUT, short_inet_line], None),
  ("get_connection_type", run_get_connection_type, [FileNotFoundError("ip"), TIMEOUT], "unknown"),
  ("scan_routes mtime fallback", run_scan_dongle_route, [OSError(), ValueError(), OverflowError()], []),
  ("get_file_size", run_get_file_size, [FileNotFoundError(), PermissionError()], 0),
  ("has_preserve_xattr_segment", run_has_preserve_xattr_segment, [OSError()], False),
  ("has_lock_file", run_has_lock_file, [FileNotFoundError(), NotADirectoryError(), PermissionError()], False),
]


class TestGuardedReads(unittest.TestCase):
  def test_expected_failure_gives_the_default(self):
    for site, run, failures, default in SITES:
      for failure in failures:
        with self.subTest(site=site, failure=failure):
          self.assertEqual(run(failure), default)

  def test_keyboard_interrupt_propagates(self):
    for site, run, _, _ in SITES:
      with self.subTest(site=site), self.assertRaises(KeyboardInterrupt):
        run(KeyboardInterrupt())

  def test_unreadable_server_setting_is_logged(self):
    with failing_params(UnknownKeyName("EnableWebRoutesServer")), self.assertLogs(utils.logger, "WARNING"):
      self.assertTrue(utils.should_server_run())


class TestSegmentNumber(unittest.TestCase):
  def test_segment_name_gives_its_number(self):
    self.assertEqual(parsing.get_segment_number("2024-09-18--14-30-00--5"), 5)
    self.assertEqual(parsing.get_segment_number("0123456789abcdef--00000012--27"), 27)

  def test_name_without_a_segment_gives_zero(self):
    self.assertEqual(parsing.get_segment_number("2024-09-18--14-30-00"), 0)


if __name__ == "__main__":
  unittest.main()
