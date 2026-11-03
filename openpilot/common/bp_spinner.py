# BluePilot: client for the verbose build spinner (system/ui/bp_spinner.py).
# Mirrors common/spinner.py but speaks the extended BP protocol (progress+text, retry, failed).
import os
import subprocess

from openpilot.common.basedir import BASEDIR


class BPSpinner:
  def __init__(self):
    try:
      self.spinner_proc = subprocess.Popen(["./bp_spinner.py"],
                                           stdin=subprocess.PIPE,
                                           cwd=os.path.join(BASEDIR, "openpilot", "system", "ui"),
                                           close_fds=True)
    except OSError:
      self.spinner_proc = None

  def __enter__(self):
    return self

  def update(self, spinner_text: str):
    if self.spinner_proc is not None and self.spinner_proc.stdin is not None:
      self.spinner_proc.stdin.write(spinner_text.encode('utf8') + b"\n")
      try:
        self.spinner_proc.stdin.flush()
      except BrokenPipeError:
        pass

  def update_progress(self, cur: float, total: float):
    self.update(str(round(100 * cur / total)))

  def update_progress_with_text(self, cur: float, total: float, text: str):
    """Update progress bar and the live status line in one message."""
    self.update(f"{round(100 * cur / total)}|{text}")

  def build_retry(self):
    """Clear the error screen and resume building (retry attempt)."""
    self.update("BUILD_RETRY")

  def build_failed(self):
    """Show the full-screen scrollable build log + Reboot button."""
    self.update("BUILD_FAILED")

  def wait_for_exit(self):
    """Block until the user dismisses the error screen (or the process ends)."""
    if self.spinner_proc is not None:
      try:
        self.spinner_proc.wait()
      except KeyboardInterrupt:
        pass

  def close(self):
    if self.spinner_proc is not None:
      self.spinner_proc.kill()
      try:
        self.spinner_proc.communicate(timeout=2.)
      except subprocess.TimeoutExpired:
        print("WARNING: failed to kill bp_spinner")
      self.spinner_proc = None

  def __del__(self):
    self.close()

  def __exit__(self, exc_type, exc_value, traceback):
    self.close()
