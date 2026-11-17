"""BluePilot crash reporting: the filters, the frame-drop rate limit and an outgoing report."""
import os
import tempfile
import unittest
import warnings
from unittest import mock

from opendbc.car import structs
from openpilot.bluepilot.system import sentry
from openpilot.common.bluepilot import CrashReporting


def log_event(message: str, daemon: str = "card") -> dict:
  return {"logentry": {"formatted": message}, "tags": {"daemon": daemon}}


class TestBeforeSend(unittest.TestCase):
  def test_real_error_passes_through(self):
    event = log_event("controlsd crashed: division by zero")
    self.assertIs(sentry._before_send(event, {}), event)

  def test_event_without_log_entry_passes_through(self):
    event = {"exception": {"values": [{"type": "ValueError"}]}}
    self.assertIs(sentry._before_send(event, {}), event)

  def test_noisy_log_lines_are_dropped(self):
    for message in ("iso-tp query bad response: 0x7e0 - 0x7f",
                    "car doesn't match any Neural Network model: F-150",
                    "{'event': 'fingerprinted', 'fingerprint': 'FORD_F_150_MK14'}",
                    '{"event": "selfdrived.initialized", "error": true}',
                    '{"event": "commIssue", "invalid": ["modelV2"]}',
                    "Failed to fetch prime status: timed out"):
      with self.subTest(message=message):
        self.assertIsNone(sentry._before_send(log_event(message), {}))

  def test_upstream_only_daemons_are_dropped(self):
    for daemon in ("deleter", "uploader", "hardwared"):
      with self.subTest(daemon=daemon):
        self.assertIsNone(sentry._before_send(log_event("anything at all", daemon), {}))


class TestFrameDropRateLimit(unittest.TestCase):
  def setUp(self):
    sentry._last_model_eval_drop_report_t = 0.0
    self.now = 10_000.0
    patcher = mock.patch.object(sentry.time, "monotonic", lambda: self.now)
    patcher.start()
    self.addCleanup(patcher.stop)

  def sent(self, frames: int) -> bool:
    return sentry._before_send(log_event(f"skipping model eval. Dropped {frames} frames"), {}) is not None

  def test_drops_up_to_75_frames_are_never_sent(self):
    self.assertFalse(self.sent(1))
    self.assertFalse(self.sent(75))

  def test_large_drop_is_sent_once_per_ten_minutes(self):
    self.assertTrue(self.sent(76))
    self.now += 599
    self.assertFalse(self.sent(200))
    self.now += 1
    self.assertTrue(self.sent(86))
    self.assertFalse(self.sent(86))


class RecordingTransport(sentry.sentry_sdk.transport.Transport):
  """Keeps every outgoing envelope instead of sending it: nothing leaves the machine."""
  def __init__(self, options=None):
    super().__init__(options)
    self.envelopes = []

  def capture_envelope(self, envelope):
    self.envelopes.append(envelope)


class TestOutgoingReport(unittest.TestCase):
  def setUp(self):
    self.transport = RecordingTransport()
    real_init = sentry.sentry_sdk.init
    for patcher in (mock.patch.object(sentry.sentry_sdk, "init", lambda *a, **kw: real_init(*a, transport=self.transport, **kw)),
                    mock.patch.object(sentry, "CRASHES_DIR", self.enterContext(tempfile.TemporaryDirectory())),
                    mock.patch.object(sentry, "PC", False)):  # reports as on a device; test_pc_does_not_report covers a PC
      patcher.start()
      self.addCleanup(patcher.stop)
    self.addCleanup(lambda: sentry.sentry_sdk.get_isolation_scope().clear())  # tags like daemon stay on the scope
    self.addCleanup(lambda: sentry.sentry_sdk.get_global_scope().set_client(None))
    self.addCleanup(lambda: sentry.sentry_sdk.get_client().close())

  def sent_events(self) -> list:
    return [event for envelope in self.transport.envelopes if (event := envelope.get_event()) is not None]

  def test_captured_exception_is_sent_with_the_device_tags(self):
    self.assertTrue(sentry.init(sentry.SentryProject.SELFDRIVE))
    sentry.set_tag("daemon", "selfdrived")
    try:
      raise ZeroDivisionError("bluepilot test crash")
    except ZeroDivisionError:
      sentry.capture_exception()

    events = self.sent_events()
    self.assertEqual(len(events), 1)
    event = events[0]
    self.assertEqual(event["exception"]["values"][-1]["type"], "ZeroDivisionError")
    self.assertEqual(event["exception"]["values"][-1]["value"], "bluepilot test crash")
    self.assertEqual(event["tags"]["daemon"], "selfdrived")
    self.assertLessEqual({"dongle_id", "sunnylink_dongle_id", "branch", "commit", "origin", "dirty", "device"}, event["tags"].keys())
    self.assertEqual(event["user"]["id"], event["tags"]["dongle_id"])
    self.assertTrue(os.path.isfile(os.path.join(sentry.CRASHES_DIR, "error.log")))  # the local crash log too

  def test_native_crash_is_sent_without_the_core_dump_and_memory_maps(self):
    self.assertTrue(sentry.init(sentry.SentryProject.SELFDRIVE_NATIVE))
    apport = os.path.join(sentry.CRASHES_DIR, "_usr_bin_ui.crash")
    with open(apport, "w") as f:
      f.write("ExecutablePath: /data/openpilot/selfdrive/ui/ui\nSignal: 11\nProcMaps:\n 7f00-7f01 r-xp libc.so\n" +
              "ProcStatus:\n Name: ui\nCoreDump: base64\n AAAA\n")
    with warnings.catch_warnings():
      warnings.simplefilter("ignore", DeprecationWarning)  # BP7's report_tombstone uses sentry_sdk.configure_scope
      sentry.report_tombstone_file(apport, "selfdrive/ui/ui - Signal: 11", "#0 crash_here")

    # as in BP7, report_tombstone's own cloudlog.error line is sent too, as a second event
    events = [event for event in self.sent_events() if "tombstone" in event.get("extra", {})]
    self.assertEqual(len(events), 1)
    self.assertEqual(events[0]["message"], "selfdrive/ui/ui - Signal: 11")
    self.assertEqual(events[0]["extra"]["tombstone_fn"], apport)
    self.assertEqual(events[0]["extra"]["tombstone"],
                     "#0 crash_here\n\nExecutablePath: /data/openpilot/selfdrive/ui/ui\nSignal: 11\nProcStatus:\n Name: ui\n")

  def test_tombstone_is_logged_once_when_the_sdk_fails(self):
    self.assertTrue(sentry.init(sentry.SentryProject.SELFDRIVE_NATIVE))
    apport = os.path.join(sentry.CRASHES_DIR, "_usr_bin_ui.crash")
    with open(apport, "w") as f:
      f.write("ExecutablePath: /data/openpilot/selfdrive/ui/ui\n")
    with mock.patch.object(sentry, "cloudlog") as cloudlog, mock.patch.object(sentry.sentry_sdk, "capture_message", side_effect=RuntimeError):
      # the guard reports that it ran, so tombstoned doesn't log the line a second time
      self.assertTrue(CrashReporting().report_tombstone_file(apport, "ui - Signal: 11", "#0 crash_here"))
    self.assertEqual(cloudlog.error.call_args_list, [mock.call({'tombstone': "ui - Signal: 11"})])

  def test_guard_reports_only_with_an_active_client(self):
    guard = CrashReporting()
    try:
      raise ZeroDivisionError("before init")
    except ZeroDivisionError:
      self.assertFalse(guard.capture_exception())  # no client: the hook logs upstream's line instead
    self.assertTrue(guard.init("SELFDRIVE"))
    try:
      raise ZeroDivisionError("after init")
    except ZeroDivisionError:
      self.assertTrue(guard.capture_exception())
    self.assertEqual([event["exception"]["values"][-1]["value"] for event in self.sent_events()], ["after init"])

  def test_pc_does_not_report(self):
    with mock.patch.object(sentry, "PC", True):
      self.assertFalse(sentry.init(sentry.SentryProject.SELFDRIVE))
      self.assertFalse(sentry.sentry_sdk.get_client().is_active())
      guard = CrashReporting()
      self.assertFalse(guard.init("SELFDRIVE"))
      try:
        raise ZeroDivisionError("crash on a PC")
      except ZeroDivisionError:
        self.assertFalse(guard.capture_exception())  # the hook logs upstream's line instead
    self.assertEqual(self.sent_events(), [])
    self.assertTrue(guard.init("SELFDRIVE"))  # the guard stays usable
    self.assertTrue(guard.set_tag("daemon", "card"))

  def test_half_started_init_leaves_no_live_client(self):
    with mock.patch.object(sentry.HARDWARE, "get_device_type", side_effect=RuntimeError("no hardware")):
      self.assertRaises(RuntimeError, sentry.init, sentry.SentryProject.SELFDRIVE)
    self.assertFalse(sentry.sentry_sdk.get_client().is_active())

  def test_unrecognized_car_is_sent_with_its_vin(self):
    self.assertTrue(sentry.init(sentry.SentryProject.SELFDRIVE))
    sentry.log_fingerprint(structs.CarParams(carFingerprint="MOCK", carVin="1FTFW1E50MFA00000", brand="mock"))
    events = self.sent_events()
    self.assertEqual([event["message"] for event in events], ["car doesn't match any fingerprints"])
    self.assertEqual(events[0]["level"], "error")
    self.assertEqual(events[0]["tags"]["carVin"], "1FTFW1E50MFA00000")

  def test_recognized_car_is_only_tagged(self):
    self.assertTrue(sentry.init(sentry.SentryProject.SELFDRIVE))
    sentry.log_fingerprint(structs.CarParams(carFingerprint="FORD_F_150_MK14", brand="ford"))
    self.assertEqual(self.sent_events(), [])
    try:
      raise ZeroDivisionError("later crash")
    except ZeroDivisionError:
      sentry.capture_exception()
    tags = self.sent_events()[0]["tags"]
    self.assertEqual((tags["carFingerprint"], tags["carName"]), ("FORD_F_150_MK14", "ford"))
    self.assertNotIn("carVin", tags)

  def test_filtered_daemon_sends_nothing(self):
    self.assertTrue(sentry.init(sentry.SentryProject.SELFDRIVE))
    sentry.set_tag("daemon", "uploader")
    try:
      raise ZeroDivisionError("upstream daemon crash")
    except ZeroDivisionError:
      sentry.capture_exception()
    self.assertEqual(self.sent_events(), [])


if __name__ == "__main__":
  unittest.main()
