"""launch_chffrplus.sh's AGNOS update: when the bundled updater can't start, the device flashes without its screen.

On a device whose AGNOS isn't the build's, agnos_init runs the bundled updater (a zipapp built from older source) in a
loop. On a newer AGNOS that updater dies at once (its UI code needs pyray's raygui, which that AGNOS's pyray lacks), so
the device looped for ever. The updater is only a screen around `agnos.py --swap`, which runs on its own: the loop now
runs that when the updater fails, and reboots once it has swapped. `sudo reboot` only queues the reboot and returns, so
after it the script must start nothing else: a second `agnos.py --swap` while the system shuts down would mark the slot
it just switched to unbootable and start flashing again. agnos_init runs here against stub programs (sudo, the updater,
agnos.py), the device's own commands never; the stub reboot returns, as the real one does."""
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

from openpilot.common.basedir import BASEDIR

SCRIPT = Path(BASEDIR) / "launch_chffrplus.sh"

UPDATER = """#!/bin/sh
echo updater >> "$LOG"
exit "$UPDATER_EXIT"
"""
AGNOS_PY = """#!/bin/sh
echo "agnos $1" >> "$LOG"
if [ "$1" = "--verify" ]; then exit 1; fi
n=$(grep -c "agnos --swap" "$LOG")
[ "$n" -gt "$SWAP_FAILS" ]
"""
# agnos_init alone, with sudo and rm as shell functions (sudo reboot is logged and returns, as the real one does)
RUN = """
sudo() { echo "sudo $*" >> "$LOG"; }
rm() { :; }
source <(sed -n '/^function agnos_init/,/^}/p' "$SCRIPT")
agnos_init
"""


class TestAgnosInit(unittest.TestCase):
  def _run(self, updater_exit: int, swap_fails: int = 0, timeout: float = 3.) -> tuple[list[str], bool]:
    with tempfile.TemporaryDirectory() as d:
      comma = Path(d, "openpilot/common/hardware/comma")
      comma.mkdir(parents=True)
      for name, text in (("updater", UPDATER), ("agnos.py", AGNOS_PY)):
        (comma / name).write_text(text)
        (comma / name).chmod(0o755)
      log = Path(d, "log")
      log.touch()
      env = dict(os.environ, DIR=d, LOG=str(log), SCRIPT=str(SCRIPT), AGNOS_VERSION="not-the-installed-one",
                 UPDATER_EXIT=str(updater_exit), SWAP_FAILS=str(swap_fails))
      try:
        subprocess.run(["bash", "-c", RUN], env=env, timeout=timeout, capture_output=True)
        ended = True
      except subprocess.TimeoutExpired:
        ended = False
      return [line for line in log.read_text().splitlines() if not line.startswith(("sudo rm", "sudo abctl", "sudo chgrp", "sudo chmod"))], ended

  def test_an_updater_that_cant_start_flashes_without_its_screen(self):
    # after the reboot request the script waits for the shutdown: nothing else starts
    log, ended = self._run(updater_exit=1)
    self.assertFalse(ended)
    self.assertEqual(log, ["agnos --verify", "updater", "agnos --swap", "sudo reboot"],
                     "something started after the reboot request")

  def test_a_failed_swap_goes_round_again(self):
    log, ended = self._run(updater_exit=1, swap_fails=1)
    self.assertFalse(ended)
    self.assertEqual(log, ["agnos --verify", "updater", "agnos --swap", "updater", "agnos --swap", "sudo reboot"],
                     "something started after the reboot request")

  def test_a_working_updater_runs_as_before(self):
    # its screen does the swap and reboots the device itself; the fallback never runs
    log, ended = self._run(updater_exit=0, timeout=2.)
    self.assertFalse(ended)
    self.assertNotIn("agnos --swap", log)
    self.assertEqual(log[:2], ["agnos --verify", "updater"])


if __name__ == "__main__":
  unittest.main()
