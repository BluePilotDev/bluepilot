#!/usr/bin/env python3
# BluePilot: verbose build. Identical build logic to build.py (kept in sync with it), but drives
# the BP spinner (common/bp_spinner.py -> system/ui/bp_spinner.py) so an on-device build shows a
# progress bar + the live scons line, and a scrollable build log on failure instead of a black
# screen. Wired in via launch_chffrplus.sh (./bp_build.py in place of ./build.py).
import hashlib
import os
import shutil
import subprocess

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.bp_spinner import BPSpinner
from openpilot.system.hardware import HARDWARE, AGNOS

# See build.py for the rationale — re-sync the venv whenever the checked-out uv.lock changes.
UV_LOCK = os.path.join(BASEDIR, "uv.lock")
SYNC_MARKER = os.path.join(BASEDIR, ".venv", ".op_synced_lock")


def _uv_lock_digest() -> str | None:
  try:
    with open(UV_LOCK, "rb") as f:
      return hashlib.sha256(f.read()).hexdigest()
  except FileNotFoundError:
    return None


def sync_python_env() -> None:
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

  subprocess.run([uv, "sync", "--frozen", "--inexact"], cwd=BASEDIR, check=True)

  os.makedirs(os.path.dirname(SYNC_MARKER), exist_ok=True)
  with open(SYNC_MARKER, "w") as f:
    f.write(digest)


def _fail(spinner: BPSpinner, message: str) -> None:
  for line in message.split("\n"):
    if line:
      spinner.update(f"0|{line}")
  spinner.build_failed()
  if not os.getenv("CI"):
    spinner.wait_for_exit()  # stays on the error screen until the user reboots
  spinner.close()
  exit(1)


def build() -> None:
  spinner = BPSpinner()
  spinner.update_progress_with_text(0, 100, "Starting build...")

  HARDWARE.set_power_save(False)
  if AGNOS:
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  # reconcile the venv with the checked-out lockfile before building
  try:
    sync_python_env()
  except subprocess.CalledProcessError:
    _fail(spinner, "openpilot failed to update dependencies\n \nEnsure the device has an internet connection, then reboot.")

  last_status = "Building..."
  compile_output: list[bytes] = []
  for attempt, parallelism in enumerate(([], ["-j4"], ["-j1"])):
    compile_output.clear()
    if attempt > 0:
      spinner.build_retry()
      last_status = f"Retrying build ({' '.join(parallelism) or 'all cores'})..."
      spinner.update_progress_with_text(0, 100, last_status)

    with subprocess.Popen(["scons", *parallelism], cwd=BASEDIR, env={**os.environ, "PWD": BASEDIR}, stderr=subprocess.PIPE) as scons:
      assert scons.stderr is not None

      while scons.poll() is None:
        try:
          line = scons.stderr.readline()
          if not line:
            continue
          line = line.rstrip()

          prefix = b'progress: '
          if line.startswith(prefix):
            progress = min(100.0, float(line[len(prefix):]))
            spinner.update_progress_with_text(progress, 100, last_status)
          elif len(line):
            compile_output.append(line)
            last_status = line.decode('utf8', 'replace')
            print(last_status)
            spinner.update(f"0|{last_status}")  # stream the line into the spinner's log buffer
        except Exception:
          pass

      # drain remaining stderr before retrying or returning
      for line in scons.stderr.read().split(b'\n'):
        line = line.rstrip()
        if len(line):
          compile_output.append(line)
          try:
            spinner.update(f"0|{line.decode('utf8', 'replace')}")
          except Exception:
            pass

    if scons.returncode == 0:
      break

  if scons.returncode != 0:
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')
    print(error_s)
    _fail(spinner, "")  # build output already streamed into the spinner's log buffer

  spinner.update_progress_with_text(100, 100, "Build complete")
  spinner.close()


if __name__ == "__main__":
  build()
