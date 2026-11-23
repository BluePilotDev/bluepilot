"""scripts/boot_logo.sh puts BluePilot's boot logo where AGNOS 19.9's display service reads it (BluePilotDev/bluepilot#255).

Runs the script against a fake /usr/comma/bg.png; a fake sudo on PATH logs each call and skips the remounts."""
import os
import shutil
import subprocess
import tempfile
import unittest

from openpilot.common.basedir import BASEDIR

SCRIPT = os.path.join(BASEDIR, "scripts", "boot_logo.sh")
FAKE_SUDO = '#!/bin/sh\necho "$*" >> "$SUDO_LOG"\n[ "$1" = mount ] && exit 0\nexec "$@"\n'
COMMA, BLUEPILOT = b"comma's logo", b"BluePilot's logo"


class TestBootLogo(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.mkdtemp()
    self.addCleanup(shutil.rmtree, self.tmp)
    os.mkdir(os.path.join(self.tmp, "bin"))
    sudo = os.path.join(self.tmp, "bin", "sudo")
    with open(sudo, "w") as f:
      f.write(FAKE_SUDO)
    os.chmod(sudo, 0o755)
    self.target, self.source, self.log = (os.path.join(self.tmp, n) for n in ("bg.png", "source.png", "sudo.log"))
    self._write(self.target, COMMA)
    self._write(self.source, BLUEPILOT)

  @staticmethod
  def _write(path, data):
    with open(path, "wb") as f:
      f.write(data)

  @staticmethod
  def _read(path):
    with open(path, "rb") as f:
      return f.read()

  def _run(self, *args) -> int:
    env = {**os.environ, "PATH": os.path.join(self.tmp, "bin") + os.pathsep + os.environ["PATH"], "SUDO_LOG": self.log,
           "BOOT_LOGO_TARGET": self.target, "BOOT_LOGO_SOURCE": self.source}
    return subprocess.run(["bash", SCRIPT, "--headless", "--force", "--quiet", *args], env=env, capture_output=True).returncode

  def _sudo_calls(self) -> list[str]:
    return self._read(self.log).decode().splitlines() if os.path.exists(self.log) else []

  def test_update_installs_logo_and_backs_up_comma_logo(self):
    self.assertEqual(self._run("--update"), 0)
    self.assertEqual(self._read(self.target), BLUEPILOT)
    self.assertEqual(self._read(self.target + ".backup"), COMMA)
    self.assertEqual([c for c in self._sudo_calls() if c.startswith("mount")], ["mount -o remount,rw /", "mount -o remount,ro /"])
    self.assertEqual(self._run("--status"), 0)

  def test_update_with_logo_in_place_touches_nothing(self):
    self._run("--update")
    os.remove(self.log)
    self.assertEqual(self._run("--update"), 0)
    self.assertEqual(self._sudo_calls(), [])  # no remount, no copy
    self.assertEqual(self._read(self.target + ".backup"), COMMA)

  def test_new_bluepilot_logo_keeps_comma_backup(self):
    self._run("--update")
    self._write(self.source, BLUEPILOT + b" v2")
    self.assertEqual(self._run("--update"), 0)
    self.assertEqual(self._read(self.target), BLUEPILOT + b" v2")
    self.assertEqual(self._read(self.target + ".backup"), COMMA)

  def test_restore_puts_comma_logo_back(self):
    self._run("--update")
    self.assertEqual(self._run("--restore"), 0)
    self.assertEqual(self._read(self.target), COMMA)
    self.assertFalse(os.path.exists(self.target + ".backup"))
    self.assertEqual(self._run("--status"), 1)


if __name__ == "__main__":
  unittest.main()
