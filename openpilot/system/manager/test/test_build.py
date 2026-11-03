import hashlib

import openpilot.system.manager.build as build

DIGEST_V1 = hashlib.sha256(b"lock-v1").hexdigest()


class TestSyncPythonEnv:
  """
  Regression tests for the venv/lockfile reconciliation in build.py.

  The venv is provisioned once at install and persists across OTA updates, but nothing
  re-syncs it afterwards. When an update adds a Python dependency (e.g. acados), the stale
  venv is missing it and scons dies importing it. sync_python_env() re-runs `uv sync`
  whenever the checked-out uv.lock differs from what the venv was last synced against.
  """

  def _run(self, mocker, tmp_path, lock_bytes=b"lock-v1", marker_text=None, uv_found=True):
    lock = tmp_path / "uv.lock"
    if lock_bytes is not None:
      lock.write_bytes(lock_bytes)
    marker = tmp_path / ".venv" / ".op_synced_lock"
    if marker_text is not None:
      marker.parent.mkdir(parents=True, exist_ok=True)
      marker.write_text(marker_text)

    calls: list[list[str]] = []
    mocker.patch.multiple(build, UV_LOCK=str(lock), SYNC_MARKER=str(marker))
    mocker.patch.object(build.shutil, "which", return_value="/usr/bin/uv" if uv_found else None)
    mocker.patch.object(build.os.path, "exists", return_value=uv_found)
    mocker.patch.object(build.subprocess, "run", side_effect=lambda cmd, **kw: calls.append(cmd))

    build.sync_python_env()
    return marker, calls

  def test_first_run_syncs_and_records_marker(self, mocker, tmp_path):
    marker, calls = self._run(mocker, tmp_path)
    assert len(calls) == 1
    assert "sync" in calls[0] and "--frozen" in calls[0]
    assert marker.read_text().strip() == DIGEST_V1

  def test_unchanged_lock_is_noop(self, mocker, tmp_path):
    _, calls = self._run(mocker, tmp_path, marker_text=DIGEST_V1)
    assert calls == []

  def test_changed_lock_triggers_resync(self, mocker, tmp_path):
    marker, calls = self._run(mocker, tmp_path, marker_text=hashlib.sha256(b"OLD").hexdigest())
    assert len(calls) == 1
    assert marker.read_text().strip() == DIGEST_V1

  def test_missing_lockfile_is_noop(self, mocker, tmp_path):
    _, calls = self._run(mocker, tmp_path, lock_bytes=None)
    assert calls == []

  def test_missing_uv_binary_is_noop(self, mocker, tmp_path):
    _, calls = self._run(mocker, tmp_path, uv_found=False)
    assert calls == []
