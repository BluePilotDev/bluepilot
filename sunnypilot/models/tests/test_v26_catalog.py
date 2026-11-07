import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest
import requests

from openpilot.common.basedir import BASEDIR
from openpilot.sunnypilot.modeld_v2.catalog_runtime import V26_RUNTIME_PATH, is_plain_model
from openpilot.sunnypilot.models.helpers import is_bundle_version_compatible
from openpilot.sunnypilot.models.fetcher import ModelFetcher


def test_supported_catalog_versions():
  assert is_bundle_version_compatible({'minimumSelectorVersion': 19})
  assert is_bundle_version_compatible({'minimumSelectorVersion': 21})
  assert not is_bundle_version_compatible({'minimumSelectorVersion': 22})
  assert not is_bundle_version_compatible({})


def test_plain_model_detection_preserves_legacy_formats(tmp_path):
  path = tmp_path / 'model.pkl'
  path.write_bytes(b'SPTGPLN1' + bytes(32))
  assert is_plain_model(str(path))
  path.write_bytes(b'\x80\x05legacy pickle')
  assert not is_plain_model(str(path))


def test_v26_native_runtime_in_isolated_process(tmp_path):
  script = Path(BASEDIR) / 'sunnypilot/modeld_v2/tests/v26_runtime_smoke.py'
  env = dict(os.environ, DEV='CPU', CACHEDB=str(tmp_path / 'tinygrad-cache.db'),
             PYTHONPATH=os.pathsep.join((str(V26_RUNTIME_PATH), BASEDIR, str(Path(BASEDIR) / 'opendbc_repo'))))
  result = subprocess.run([sys.executable, str(script)], cwd=tmp_path, env=env, capture_output=True, text=True, timeout=120)
  assert result.returncode == 0, result.stdout + result.stderr
  assert 'PASS:' in result.stdout


@pytest.mark.parametrize('offline', [False, True])
def test_fresh_legacy_cache_migrates_or_stays_available_offline(monkeypatch, offline):
  bundle = {'short_name': 'TEST', 'display_name': 'Test Model', 'ref': 'test', 'index': 0, 'generation': '12',
            'minimum_selector_version': '19', 'environment': 'development', 'runner': 'tinygrad', 'models': []}
  cached = {'bundles': [bundle]}
  current = {'bundles': [dict(bundle, minimum_selector_version='21')]}

  class ParamsStub:
    values = {'ModelManager_ModelsCache': cached, 'ModelManager_LastSyncTime': time.monotonic_ns()}

    def get(self, key):
      return self.values.get(key)

    def put(self, key, value, **kwargs):
      self.values[key] = value

  calls = []

  def get(url, **kwargs):
    calls.append(url)
    if offline:
      raise requests.ConnectionError('Offline test')
    return SimpleNamespace(status_code=200, raise_for_status=lambda: None, json=lambda: current)

  monkeypatch.setattr(requests, 'get', get)
  available = ModelFetcher(ParamsStub()).get_available_bundles()
  assert len(calls) == 1
  assert available[0].minimumSelectorVersion == (19 if offline else 21)
