#!/usr/bin/env python3
"""BluePilot: run ford_reads.py from the settings button, then reboot.

ford_reads.py (Jacob's read-only module reader, unchanged next to this file) stops openpilot to take the panda,
so it can't run inside the UI. The button starts this runner in its own session, which survives the tmux
session being killed. The runner:
  - reads every module once, saving /data/ford-reads/<name>.json as ford_reads.py does;
  - keeps everything the reader printed in /data/ford-reads/<name>.log;
  - writes the outcome to /data/ford-reads/last.json, which the settings row shows after the reboot;
  - reboots the device, whether the read worked or not, so openpilot is back afterwards.

  python3 -m openpilot.bluepilot.system.ford_reads.runner            what the button runs
  python3 -m openpilot.bluepilot.system.ford_reads.runner --no-reboot   the same, and stay up (over SSH)
"""
import contextlib
import json
import os
import signal
import subprocess
import sys
import time

from openpilot.bluepilot.system.ford_reads import ford_reads

OUT_DIR = ford_reads.OUT_DIR
LAST = os.path.join(OUT_DIR, "last.json")


def last_read() -> dict | None:
  """The outcome of the last button read, or None."""
  try:
    with open(LAST) as f:
      return json.load(f)
  except (OSError, ValueError):
    return None


def _write_last(state: dict) -> None:
  os.makedirs(OUT_DIR, exist_ok=True)
  tmp = LAST + ".tmp"
  with open(tmp, "w") as f:
    json.dump(state, f, indent=1)
  os.replace(tmp, LAST)


def launch() -> None:
  """Start the runner detached from the UI: its own session, so killing openpilot's tmux session doesn't end it."""
  subprocess.Popen([sys.executable, "-m", "openpilot.bluepilot.system.ford_reads.runner"], start_new_session=True,
                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, close_fds=True)


def run(reboot: bool = True) -> int:
  signal.signal(signal.SIGHUP, signal.SIG_IGN)
  name = time.strftime("read-%Y%m%dT%H%M%S")
  os.makedirs(OUT_DIR, exist_ok=True)
  state = {"name": name, "started": time.strftime("%Y-%m-%dT%H:%M:%S"), "ok": False, "message": "running"}
  _write_last(state)
  log = os.path.join(OUT_DIR, f"{name}.log")
  try:
    with open(log, "w") as f, contextlib.redirect_stdout(f), contextlib.redirect_stderr(f):
      sys.argv = ["ford_reads.py", "--save-as", name]
      try:
        rc = ford_reads.main()
      except SystemExit as e:  # ford_reads ends with sys.exit("<why>") when it can't read
        rc = e.code if isinstance(e.code, int) else 1
        if not isinstance(e.code, int):
          print(e.code)
          state["message"] = str(e.code)
      except Exception as e:
        rc = 1
        print(repr(e))
        state["message"] = repr(e)
    saved = os.path.join(OUT_DIR, f"{name}.json")
    state["ok"] = rc == 0 and os.path.exists(saved)
    if state["ok"]:
      state["message"] = f"saved {saved}"
    elif state["message"] == "running":
      state["message"] = f"failed: see {log}"
  finally:
    state["finished"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    _write_last(state)
    if reboot:
      time.sleep(2)
      subprocess.run(["sudo", "reboot"])
  return 0 if state["ok"] else 1


if __name__ == "__main__":
  sys.exit(run(reboot="--no-reboot" not in sys.argv))
