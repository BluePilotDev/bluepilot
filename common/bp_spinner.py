import os
import subprocess
from openpilot.common.basedir import BASEDIR


class BPSpinner:
  def __init__(self):
    try:
      cwd_path = os.path.join(BASEDIR, "system", "ui")
      print(f"BPSpinner: Working directory: {cwd_path}")
      print(f"BPSpinner: Looking for: {os.path.join(cwd_path, 'bp_spinner.py')}")
      print(f"BPSpinner: File exists: {os.path.exists(os.path.join(cwd_path, 'bp_spinner.py'))}")

      self.spinner_proc = subprocess.Popen(["./bp_spinner.py"],
                                          stdin=subprocess.PIPE,
                                          cwd=cwd_path,
                                          close_fds=True)
      print(f"BPSpinner: Subprocess PID: {self.spinner_proc.pid}")
    except OSError as e:
      print(f"BPSpinner: OSError: {e}")
      self.spinner_proc = None

  def __enter__(self):
    return self

  def update(self, spinner_text: str):
    if self.spinner_proc is not None:
      self.spinner_proc.stdin.write(spinner_text.encode('utf8') + b"\n")
      try:
        self.spinner_proc.stdin.flush()
      except BrokenPipeError:
        pass

  def update_progress(self, cur: float, total: float):
    self.update(str(round(100 * cur / total)))

  def update_progress_with_text(self, cur: float, total: float, text: str):
    """Update both progress percentage and status text in a single call"""
    percentage = round(100 * cur / total)
    self.update(f"{percentage}|{text}")

  def build_retry(self):
    """Signal a build retry to trigger retry modal display"""
    self.update("BUILD_RETRY")

  def build_failed(self):
    """Signal a build failure to trigger error modal display"""
    self.update("BUILD_FAILED")

  def wait_for_exit(self):
    """Wait for user interaction with error screen"""
    if self.spinner_proc is not None:
      try:
        self.spinner_proc.wait()
      except KeyboardInterrupt:
        print("Build interrupted by user")

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


if __name__ == "__main__":
  import time
  with BPSpinner() as s:
    s.update("BP Spinner text")
    time.sleep(2.0)
    s.update_progress_with_text(50, 100, "Compiling file.cc")
    time.sleep(3.0)
  print("gone")
  time.sleep(2.0)
