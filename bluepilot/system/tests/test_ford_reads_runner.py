"""The settings button's runner around ford_reads.py: what it records, that it always reboots, and that the reader's
guard lets only read-only requests through."""
import json
import os
import tempfile
import unittest
from unittest import mock

from openpilot.bluepilot.system.ford_reads import ford_reads, runner


class TestFordReadsRunner(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.mkdtemp()
    for target, value in ((runner, "OUT_DIR"), (runner, "LAST"), (ford_reads, "OUT_DIR")):
      p = mock.patch.object(target, value, os.path.join(self.dir, "last.json") if value == "LAST" else self.dir)
      p.start()
      self.addCleanup(p.stop)
    self.calls = []
    p = mock.patch.object(runner.subprocess, "run", side_effect=lambda cmd, **kw: self.calls.append(cmd))
    p.start()
    self.addCleanup(p.stop)
    p = mock.patch.object(runner.time, "sleep")
    p.start()
    self.addCleanup(p.stop)

  def _run(self, main):
    with mock.patch.object(ford_reads, "main", side_effect=main):
      rc = runner.run(reboot=True)
    return rc, runner.last_read()

  def test_a_saved_read_is_recorded_and_the_device_reboots(self):
    def main():
      name = ford_reads.sys.argv[ford_reads.sys.argv.index("--save-as") + 1]
      with open(os.path.join(self.dir, f"{name}.json"), "w") as f:
        json.dump({"modules": {}}, f)
      print("PSCM 0x730")
      return 0
    rc, last = self._run(main)
    self.assertEqual(rc, 0)
    self.assertTrue(last["ok"])
    self.assertIn(last["name"] + ".json", last["message"])
    with open(os.path.join(self.dir, last["name"] + ".log")) as f:
      self.assertIn("PSCM 0x730", f.read())
    self.assertEqual(self.calls, [["sudo", "reboot"]])

  def test_the_readers_own_refusal_is_shown_and_the_device_still_reboots(self):
    def main():
      raise SystemExit("no module answered and nothing was saved: is the ignition on? Run the same command again.")
    rc, last = self._run(main)
    self.assertEqual(rc, 1)
    self.assertFalse(last["ok"])
    self.assertIn("is the ignition on", last["message"])
    self.assertEqual(self.calls, [["sudo", "reboot"]])

  def test_a_crash_is_recorded_and_the_device_still_reboots(self):
    def main():
      raise OSError("panda not found")
    rc, last = self._run(main)
    self.assertEqual(rc, 1)
    self.assertFalse(last["ok"])
    self.assertIn("panda not found", last["message"])
    self.assertEqual(self.calls, [["sudo", "reboot"]])


class TestBlocker(unittest.TestCase):
  @staticmethod
  def _cs(standstill=True, gear="park"):
    return mock.Mock(standstill=standstill, gearShifter=gear)

  def test_a_read_starts_only_parked_with_the_ignition_on_and_disengaged(self):
    self.assertIsNone(runner.blocker(True, False, self._cs()))
    self.assertIn("ignition", runner.blocker(False, False, self._cs()))
    self.assertIn("Disengage", runner.blocker(True, True, self._cs()))
    self.assertIn("Park", runner.blocker(True, False, self._cs(gear="drive")))
    self.assertIn("Park", runner.blocker(True, False, self._cs(standstill=False)))


class TestGuard(unittest.TestCase):
  def test_only_read_only_services_reach_the_bus(self):
    sent = []

    class Client:
      def _uds_request(self, service_type, subfunction=None, data=None):
        sent.append((service_type, subfunction))
        return b""

    c = ford_reads.guard(Client())
    c._uds_request(0x10, 0x03)
    c._uds_request(0x22, None, b"\xf1\x88")
    c._uds_request(0x19, 0x02, b"\xff")
    for service, sub in ((0x10, 0x02), (0x2E, None), (0x27, 0x01), (0x31, 0x01), (0x11, 0x01), (0x14, None), (0x23, None)):
      with self.assertRaises(RuntimeError):
        c._uds_request(service, sub, b"\x00")
    self.assertEqual(sent, [(0x10, 0x03), (0x22, None), (0x19, 0x02)])


if __name__ == "__main__":
  unittest.main()
