"""
A PythonProcess that manager starts again after it dies, for JetLink's gadget owner (jetlinkd).

Adapted from zoompilot's openpilot/sunnypilot/system/manager/processes.py (https://github.com/zoompilot/zoompilot,
MIT, Copyright (c) 2026- Zeph Leggett). See bluepilot/jetlink_adapter/THIRD_PARTY_NOTICES.md.
"""
import time

from openpilot.common.swaglog import cloudlog
from openpilot.system.manager.process import PythonProcess


class RestartingPythonProcess(PythonProcess):
  """Upstream's PythonProcess.start() leaves a process that has exited in place for good; this one starts it again.
  For jetlinkd, which holds the USB gadget for as long as the link is on; JetLink's owner adopts what a dead one
  left and holds a crash loop back itself.

  One that dies within QUICK_DEATH of its start never got that far (an import error, a raise before the owner's
  loop, a second owner stepping aside for a live one), so the next start waits BACKOFF, doubling to BACKOFF_MAX,
  rather than forking manager twice a second for a whole drive. One that ran longer is started again on the next
  loop."""
  QUICK_DEATH = 10.0
  BACKOFF = 10.0
  BACKOFF_MAX = 300.0

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.started_at = 0.0
    self.backoff = 0.0
    self.next_start = 0.0

  def now(self) -> float:
    return time.monotonic()

  def start(self) -> None:
    now = self.now()
    if self.proc is not None and self.proc.exitcode is not None:
      if now - self.started_at < self.QUICK_DEATH:
        self.backoff = min(self.BACKOFF_MAX, 2 * self.backoff or self.BACKOFF)
        self.next_start = now + self.backoff
        cloudlog.warning(f"{self.name} died {now - self.started_at:.1f} s after it started, starting it again in {self.backoff:.0f} s")
      else:
        self.backoff = 0.0
      self.stop()  # reaps it, logs the exit code and clears proc
    if self.proc is None:
      if now < self.next_start:
        return
      self.started_at = now
    super().start()
