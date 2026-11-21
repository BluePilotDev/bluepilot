"""Startup must not depend on the removed optional vision blindspot monitor."""
import builtins
import importlib

import pytest

from openpilot.common.params import Params, UnknownKeyName


def test_startup_imports_without_optional_vision_monitor(monkeypatch):
  original = builtins.__import__

  def guarded(name, *args, **kwargs):
    if 'bluepilot.vasm' in name or name == 'cv2':
      raise ModuleNotFoundError(name)
    return original(name, *args, **kwargs)

  monkeypatch.setattr(builtins, '__import__', guarded)
  config = importlib.import_module('openpilot.system.manager.process_config')
  driving = importlib.import_module('openpilot.selfdrive.selfdrived.selfdrived')
  importlib.import_module('openpilot.selfdrive.controls.lib.desire_helper')
  assert 'adj_spot_monitor_vision' not in config.managed_processes
  assert 'adj_spot_monitor_vision' not in driving.NON_CRITICAL_PROCESSES
  assert 'bp_portal' in config.managed_processes


def test_retired_settings_are_not_registered():
  params = Params()
  for key in ('VASMEnabled', 'VASMConfidenceThreshold', 'VASMSmoothSeconds', 'VASMAnnotationConfig',
              'VASMLeftActive', 'VASMRightActive', 'VASMHeartbeat'):
    with pytest.raises(UnknownKeyName):
      params.check_key(key)
  assert params.check_key('ShowBlindspotOverlay')
