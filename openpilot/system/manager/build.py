#!/usr/bin/env python3
import hashlib
import os
import shutil
import subprocess

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.spinner import Spinner
from openpilot.common.text_window import TextWindow
from openpilot.common.hardware import HARDWARE, AGNOS

# The venv is provisioned once (at install) and persists across OTA updates, but nothing
# reconciles it with the updated lockfile afterwards: updated.py only does a git fetch/reset.
# When an update adds a Python dependency (e.g. acados), the stale venv is missing it and
# scons dies importing it. Re-sync the venv whenever the checked-out uv.lock changes.
UV_LOCK = os.path.join(BASEDIR, "uv.lock")
SYNC_MARKER = os.path.join(BASEDIR, ".venv", ".op_synced_lock")


def _uv_lock_digest() -> str | None:
  try:
    with open(UV_LOCK, "rb") as f:
      return hashlib.sha256(f.read()).hexdigest()
  except FileNotFoundError:
    return None


def sync_python_env() -> None:
  # No-op unless uv.lock changed since the last successful sync. The marker lives inside
  # the venv, so a wiped/recreated venv also re-syncs (its marker disappears with it).
  digest = _uv_lock_digest()
  if digest is None:
    return

  try:
    with open(SYNC_MARKER) as f:
      if f.read().strip() == digest:
        return
  except FileNotFoundError:
    pass

  uv = shutil.which("uv") or os.path.expanduser("~/.local/bin/uv")
  if not os.path.exists(uv):
    print("uv not found; skipping dependency sync")
    return

  # --frozen: install exactly what uv.lock pins, no re-resolution.
  # --inexact: only add missing packages, never remove extras (won't clobber a dev's env).
  subprocess.run([uv, "sync", "--frozen", "--inexact"], cwd=BASEDIR, check=True)

  os.makedirs(os.path.dirname(SYNC_MARKER), exist_ok=True)
  with open(SYNC_MARKER, "w") as f:
    f.write(digest)


def build() -> None:
  spinner = Spinner()
  spinner.update_progress(0, 100)

  HARDWARE.set_power_save(False)
  if AGNOS:
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  # reconcile the venv with the checked-out lockfile before building
  try:
    sync_python_env()
  except subprocess.CalledProcessError:
    spinner.close()
    if not os.getenv("CI"):
      msg = "openpilot failed to update dependencies\n \nEnsure the device has an internet connection, then reboot."
      with TextWindow(msg) as t:
        t.wait_for_exit()
    exit(1)

  # building with all cores can result in using too much memory, so retry serially
  compile_output: list[bytes] = []
  for parallelism in ([], ["-j4"], ["-j1"]):
    compile_output.clear()
    with subprocess.Popen(["scons", *parallelism], cwd=BASEDIR, env={**os.environ, "PWD": BASEDIR}, stderr=subprocess.PIPE) as scons:
      assert scons.stderr is not None

      # Read progress from stderr and update spinner
      while scons.poll() is None:
        try:
          line = scons.stderr.readline()
          if line is None:
            continue
          line = line.rstrip()

          prefix = b'progress: '
          if line.startswith(prefix):
            progress = float(line[len(prefix):])
            spinner.update_progress(100 * min(1., progress / 100.), 100.)
          elif len(line):
            compile_output.append(line)
            print(line.decode('utf8', 'replace'))
        except Exception:
          pass

      # Drain and close the pipe before retrying or returning.
      for line in scons.stderr.read().split(b'\n'):
        line = line.rstrip()
        if len(line):
          compile_output.append(line)

    if scons.returncode == 0:
      break

  os.sync()

  if scons.returncode != 0:
    # Build failed log errors
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')

    # Show TextWindow
    spinner.close()
    if not os.getenv("CI"):
      with TextWindow("openpilot failed to build\n \n" + error_s) as t:
        t.wait_for_exit()
    exit(1)

if __name__ == "__main__":
  build()
