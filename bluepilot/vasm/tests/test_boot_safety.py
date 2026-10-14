import builtins
import importlib
import sys

from openpilot.common.params import UnknownKeyName


def test_daemon_import_does_not_load_optional_native_dependencies(monkeypatch):
  """Manager prepare must never require OpenCV just to boot."""
  real_import = builtins.__import__

  def guarded_import(name, global_vars=None, local_vars=None, fromlist=(), level=0):
    if name.split(".", 1)[0] == "cv2":
      raise ModuleNotFoundError(name)
    return real_import(name, global_vars, local_vars, fromlist, level)

  sys.modules.pop("openpilot.bluepilot.vasm.daemon", None)
  monkeypatch.setattr(builtins, "__import__", guarded_import)
  daemon = importlib.import_module("openpilot.bluepilot.vasm.daemon")
  disabled_reasons = []
  monkeypatch.setattr(daemon, "_run_disabled", disabled_reasons.append)
  daemon.main()
  assert disabled_reasons and "runtime dependency unavailable" in disabled_reasons[0]


def test_vasm_process_skips_manager_preimport(monkeypatch):
  from openpilot.system.manager.process_config import managed_processes

  process = managed_processes["adj_spot_monitor_vision"]
  assert not process.preimport
  monkeypatch.setattr(importlib, "import_module", lambda _: (_ for _ in ()).throw(AssertionError("unexpected pre-import")))
  process.prepare()


def test_vasm_is_non_critical_to_engagement():
  from openpilot.selfdrive.selfdrived.selfdrived import NON_CRITICAL_PROCESSES

  assert "adj_spot_monitor_vision" in NON_CRITICAL_PROCESSES


def test_vasm_process_stays_disabled_with_stale_param_binary():
  from openpilot.system.manager.process_config import managed_processes

  class StaleParams:
    def get_bool(self, key):
      raise UnknownKeyName(key)

  process = managed_processes["adj_spot_monitor_vision"]
  assert not process.should_run(True, StaleParams(), None)
