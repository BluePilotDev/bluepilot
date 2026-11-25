"""The settings button's runner around ford_reads.py: what it records, that it always reboots, and that the reader's
guard lets only read-only requests through."""
import fcntl
import json
import os
import shutil
import tempfile
import unittest
from unittest import mock

from openpilot.bluepilot.system.ford_reads import ford_reads, runner


class TestFordReadsRunner(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.mkdtemp()
    self.addCleanup(shutil.rmtree, self.dir, ignore_errors=True)
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

  def test_a_second_start_while_a_read_runs_does_nothing(self):
    # The first runner holds the lock (here: this test); a second one must not read, record or reboot.
    with open(os.path.join(self.dir, ".lock"), "w") as held:
      fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
      main = mock.Mock(return_value=0)
      rc, last = self._run(main)
    self.assertEqual(rc, 1)
    main.assert_not_called()
    self.assertIsNone(last)
    self.assertEqual(self.calls, [])


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


class FakeUdsClient:
  """Answers every request, except the identifiers in `odd`, whose answer is a late one for another identifier."""
  odd: dict = {}

  def __init__(self, panda, addr, bus=0, timeout=0.3):
    self.addr = addr

  def _uds_request(self, service_type, subfunction=None, data=None):
    return b""

  def diagnostic_session_control(self, session_type):
    return b""

  def read_data_by_identifier(self, did):
    if did in self.odd.get(self.addr, ()):
      raise ValueError(f"invalid response data identifier: {hex(did - 1)} expected: {hex(did)}")
    return did.to_bytes(2, "big")

  def read_dtc_information(self, report_type, mask):
    return b"\xff"


class TestReadModules(unittest.TestCase):
  def test_an_unexpected_answer_skips_that_identifier_and_the_read_goes_on(self):
    FakeUdsClient.odd = {0x730: {0xF111}}
    with mock.patch("opendbc.car.uds.UdsClient", FakeUdsClient):
      try:
        out = ford_reads.read_modules(panda=None, bus=0)
      except ValueError as e:
        self.fail(f"one unexpected answer ended the whole read, and nothing would be saved: {e}")
    pscm = out["modules"]["PSCM"]
    self.assertIn("F111", pscm["errors"])
    self.assertIn("invalid response data identifier", pscm["errors"]["F111"])
    self.assertNotIn("F111", pscm["dids"])
    every = len(ford_reads.ID_DIDS) + len(ford_reads.ASBUILT_DIDS)
    self.assertEqual(len(pscm["dids"]), every - 1)
    self.assertEqual(pscm["dtc_raw"], "ff")
    for name in ford_reads.MODULES.values():
      self.assertEqual(len(out["modules"][name]["dids"]), every - (name == "PSCM"), name)


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
