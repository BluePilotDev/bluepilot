import importlib.machinery
import sys
import types
import unittest
from unittest import mock

from bluepilot.backend.core import lifecycle


def fake_websockets():
  module = types.ModuleType("websockets")
  module.__spec__ = importlib.machinery.ModuleSpec("websockets", None)
  return module


class WebsocketsProbeTestCase(unittest.TestCase):
  """Controls whether websockets is importable through sys.modules (None = import fails), and records any install."""
  def setUp(self):
    self.install = mock.Mock(return_value=mock.Mock(returncode=0, stderr=""))
    for patcher in (mock.patch.object(lifecycle.subprocess, "run", self.install),
                    mock.patch.object(lifecycle.shutil, "which", return_value="/usr/bin/uv")):
      patcher.start()
      self.addCleanup(patcher.stop)

  def websockets(self, module):
    patcher = mock.patch.dict(sys.modules, {"websockets": module})
    patcher.start()
    self.addCleanup(patcher.stop)


class TestEnsureDependencies(WebsocketsProbeTestCase):
  def test_available_needs_no_restart_and_installs_nothing(self):
    self.websockets(fake_websockets())
    self.assertFalse(lifecycle.ensure_dependencies())
    self.install.assert_not_called()

  def test_missing_installs_websockets_and_asks_for_restart(self):
    self.websockets(None)
    self.assertTrue(lifecycle.ensure_dependencies())
    self.assertEqual(self.install.call_args.args[0], ["uv", "pip", "install", "websockets", "websocket-client"])


class TestInstallDependenciesBackground(WebsocketsProbeTestCase):
  def run_install(self):
    callback = mock.Mock()
    lifecycle.install_dependencies_background(callback).join(timeout=5)
    return callback

  def test_available_reports_no_restart_and_installs_nothing(self):
    self.websockets(fake_websockets())
    self.run_install().assert_called_once_with(False)
    self.install.assert_not_called()

  def test_missing_installs_websockets_and_reports_restart(self):
    self.websockets(None)
    self.addCleanup(setattr, lifecycle, "_restart_pending", lifecycle._restart_pending)
    self.run_install().assert_called_once_with(True)
    self.assertEqual(self.install.call_args.args[0], ["uv", "pip", "install", "websockets", "websocket-client"])


if __name__ == "__main__":
  unittest.main()
