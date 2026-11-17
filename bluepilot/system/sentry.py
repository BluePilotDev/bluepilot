"""Install exception handler for process crash.

BluePilot's crash reporting to glitchtip.bluepilot.dev, ported from BP7's system/sentry.py (upstream
removed Sentry). Upstream files call it through the guard `sentry` in openpilot/common/bluepilot.py,
which does nothing when this module or the SDK can't load or start.
"""
import logging  # BluePilot: LoggingIntegration below
import os
import re  # BluePilot: model-eval frame-drop threshold filter
import sys
import time  # BluePilot: model-eval frame-drop rate limiting
import traceback
from datetime import datetime
from enum import Enum

# the SDK is vendored (bluepilot/vendor/VERSION) and imports itself as the top-level package `sentry_sdk`
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vendor"))

import sentry_sdk
from sentry_sdk.integrations.threading import ThreadingIntegration
from sentry_sdk.integrations.logging import LoggingIntegration  # BluePilot: log capture, tuned for memory (see init())

from openpilot.common.params import Params
from openpilot.system.athena.registration import UNREGISTERED_DONGLE_ID
from openpilot.common.hardware import HARDWARE, PC
from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.common.version import get_build_metadata, get_version

from openpilot.sunnypilot.sunnylink.api import UNREGISTERED_SUNNYLINK_DONGLE_ID

CRASHES_DIR = Paths.crash_log_root()


class SentryProject(Enum):
  # BluePilot: report to our OWN self-hosted GlitchTip (Sentry-API compatible), which we fully control
  # — replaces the coworker-owned Sentry we could neither see nor configure.
  # Public HTTPS via Cloudflare Tunnel (glitchtip.bluepilot.dev) — reachable by the whole fleet,
  # not just devices on the IoT VLAN. GlitchTip VM's LAN/IoT addresses (10.88.1.236 / 10.88.20.236)
  # remain valid for on-bench testing if the tunnel is ever down.
  # Prior DSNs for reference:  sunnypilot upstream  = o1138119.ingest.us.sentry.io/4508660076052480
  #                            BluePilot Sentry(6.0) = o4510926103379968.ingest.us.sentry.io/4510926105739264
  # python project
  SELFDRIVE = "https://06436d643ff64a6eadc42b3691de2198@glitchtip.bluepilot.dev/1"
  # native project
  SELFDRIVE_NATIVE = SELFDRIVE


# BluePilot: known-noisy log lines that log at ERROR severity for routine/expected conditions, not
# bugs — e.g. isotp_parallel_query logs a bad ECU response at .error() on every fingerprint scan where
# an ECU doesn't answer a diagnostic query cleanly (normal on real vehicles), and NNLC logs "no model
# for this car" at .error() for the majority of supported vehicles that simply lack a custom model.
# Filtered here (client-side, so they never leave the device) instead of at each call site, so this
# stays one central, reviewable/extendable list rather than scattered severity fixes.
# A filtered log line is still recorded as a breadcrumb (LoggingIntegration adds those independently
# of before_send), so it still shows up as trailing context on a real crash from the same process —
# only suppressed from becoming its own standalone issue.
_NOISY_LOG_SUBSTRINGS = (
  "iso-tp query bad response",
  "iso-tp query response pending",
  "car doesn't match any Neural Network model",
  # opendbc/car/car_helpers.py — routine, every boot; capture_fingerprint() above already tags
  # carFingerprint/carName on this process's scope, which is the reliable way to get fingerprint
  # context on a real crash (a tag doesn't decay out of the breadcrumb ring buffer like this would).
  # carlog.error({...}) with a PLAIN dict -> Python's default repr -> single-quoted.
  "'event': 'fingerprinted'",
  # opendbc/car/fw_versions.py — fuzzy-match fingerprint variant of the same routine event.
  "using fuzzy match",
  # selfdrive/selfdrived/selfdrived.py — stock/sunnypilot code logs init diagnostics via
  # cloudlog.event(..., error=True) UNCONDITIONALLY on every successful init, healthy or not (not
  # gated on anything actually being wrong) — fires fleet-wide on every boot forever if not filtered.
  # NOTE: cloudlog.event(...) wraps the payload in NiceOrderedDict (common/logging_extra.py), whose
  # __str__ is json_robust_dumps() -> real JSON, DOUBLE-quoted. This is NOT the same as a plain dict
  # literal passed straight to .error({...}) (like the fingerprinted entry above), which renders via
  # Python's default repr (single-quoted). Get this wrong and the entry silently never matches.
  '"event": "selfdrived.initialized"',
  # selfdrive/ui/lib/prime_state.py — polls comma's own api.commadotai.com every 5s on a background
  # thread, broadly caught (no crash, retries automatically) — a transient network hiccup, not a bug.
  # Matches the stable prefix only; the exception text after it varies by failure type (timeout, DNS,
  # connection refused, ...). Plain f-string, no NiceOrderedDict/JSON quoting concern here.
  "Failed to fetch prime status:",
  # selfdrive/selfdrived/selfdrived.py — commIssue ("Communication Issue Between Processes") IS a
  # real driver-facing alert, and we tried a threshold approach first (filter only transient ones in
  # the first 20s after process start, via a 'dt' field, keep anything after that as presumed
  # persistent). In practice a single flapping/unstable device blows past that: 151 commIssue alerts
  # from one dongle overnight, all with dt well past the startup window, so "persistent" != "rare".
  # Blanket-filtered now — cloudlog.event() still writes it locally on-device every time (this only
  # stops it from becoming a GlitchTip issue), so it's still in a user's RLOG if we need to pull one
  # for someone reporting real persistent comm issues. Matches on the JSON event key only (see the
  # selfdrived.initialized entry above for why: NiceOrderedDict -> double-quoted JSON, not repr).
  '"event": "commIssue"',
)

# BluePilot: daemons whose code we never touch (verified: no "# BluePilot:" markers in either file
# at the time this was added). Errors there are upstream comma/openpilot issues, not ours to fix —
# comma's own fleet-scale telemetry is the right place for that signal, not our GlitchTip. Filters
# the WHOLE daemon regardless of message content, unlike the precise per-line _NOISY_LOG_SUBSTRINGS
# above. Re-verify with `grep -rn BluePilot <file>` before adding another daemon here — this is a
# blind spot by design: a genuinely new/severe bug in a listed daemon would go unreported too.
_UPSTREAM_ONLY_DAEMONS = (
  "deleter",
  "uploader",
  "hardwared",
)

# BluePilot: selfdrive/modeld/modeld.py and sunnypilot/modeld_v2/modeld.py (stock/sunnypilot,
# unmodified by us — verified no "# BluePilot:" markers) log "skipping model eval. Dropped N
# frames" via cloudlog.error() UNCONDITIONALLY on any vipc_dropped_frames > 0 — no rate-limit,
# no dedup, one event per occurrence. In practice this floods GlitchTip with a wide spread of
# values (dozens in the 20s-30s, up to 86 seen), not just a handful of small outliers — so a
# magnitude cutoff alone isn't enough; even a genuinely large stall can repeat many times in one
# rough drive. Two layers: drops <=75 frames (~3.75s at MODEL_RUN_FREQ=20Hz) are suppressed
# outright as unremarkable for this fleet; anything larger is rate-limited to at most one report
# per _MODEL_EVAL_DROP_REPORT_INTERVAL_S so a single bad drive can't spam dozens of alerts for
# the same underlying condition. Rate-limit state is per-process (module-level), so it naturally
# resets on each daemon restart (new boot/drive gets a fresh look).
_MODEL_EVAL_DROP_PATTERN = re.compile(r"skipping model eval\. Dropped (\d+) frames")
_MODEL_EVAL_DROP_THRESHOLD_FRAMES = 75  # ~3.75s at MODEL_RUN_FREQ=20Hz
_MODEL_EVAL_DROP_REPORT_INTERVAL_S = 600  # at most one report per 10 min, even for large stalls
_last_model_eval_drop_report_t = 0.0


def _should_suppress_frame_drop(formatted: str) -> bool:
  m = _MODEL_EVAL_DROP_PATTERN.search(formatted)
  if m is None:
    return False
  if int(m.group(1)) <= _MODEL_EVAL_DROP_THRESHOLD_FRAMES:
    return True
  global _last_model_eval_drop_report_t
  now = time.monotonic()
  if now - _last_model_eval_drop_report_t < _MODEL_EVAL_DROP_REPORT_INTERVAL_S:
    return True
  _last_model_eval_drop_report_t = now
  return False


def _before_send(event: dict, hint: dict) -> dict | None:
  if event.get("tags", {}).get("daemon") in _UPSTREAM_ONLY_DAEMONS:
    return None
  formatted = event.get("logentry", {}).get("formatted", "")
  if _should_suppress_frame_drop(formatted):
    return None
  if any(s in formatted for s in _NOISY_LOG_SUBSTRINGS):
    return None
  return event
# End BluePilot


def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})

  with sentry_sdk.configure_scope() as scope:
    set_user()
    scope.set_extra("tombstone_fn", fn)
    scope.set_extra("tombstone", contents)
    sentry_sdk.capture_message(message=message)
    sentry_sdk.flush()


# BluePilot: tombstoned's half of the tombstone report, which upstream removed from
# report_tombstone_apport() along with Sentry: the apport file's text without the core dump and the
# memory maps, after the stack trace
def report_tombstone_file(fn: str, message: str, stacktrace: str) -> None:
  contents = ""  # Full file contents without coredump
  proc_maps = False

  with open(fn) as f:
    for line in f:
      if "CoreDump" in line:
        break

      if "ExecutablePath" in line or "Signal" in line:
        pass  # tombstoned reads these for the message
      elif "ProcMaps" in line:
        proc_maps = True
      elif "ProcStatus" in line:
        proc_maps = False

      if not proc_maps:
        contents += line

  try:
    report_tombstone(fn, message, stacktrace + "\n\n" + contents)
  except Exception:
    # report_tombstone logs the tombstone line before it reports, so this still counts as done:
    # tombstoned mustn't log the line a second time
    cloudlog.exception("sentry tombstone exception")
# End BluePilot


def capture_exception(*args, **kwargs) -> None:
  # BluePilot: was cloudlog.error(...) -- LoggingIntegration(event_level=ERROR) turned this into its
  # own GlitchTip event duplicating the properly stack-trace-grouped sentry_sdk.capture_exception()
  # call a few lines down, for the exact same exception. Doesn't inflate *issue* count the way
  # save_exception()'s did below (exc_info fingerprints this into the same issue), just adds a
  # redundant event under it. info() keeps it as a breadcrumb.
  cloudlog.info("crash", exc_info=kwargs.get('exc_info', 1))
  # End BluePilot

  try:
    save_exception(traceback.format_exc())

    set_user()
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def save_exception(content: str) -> None:
  try:
    if not os.path.exists(CRASHES_DIR):
      os.makedirs(CRASHES_DIR)

    files = [
      os.path.join(CRASHES_DIR, datetime.now().strftime("%Y-%m-%d--%H-%M-%S.log")),
      os.path.join(CRASHES_DIR, "error.log")
    ]

    for fn in files:
      with open(fn, 'w') as f:
        if fn == "error.log":
          lines = content.splitlines()[-3:]
          f.write("\n".join(lines))
        else:
          f.write(content)

    # BluePilot: was cloudlog.error(...). Two compounding problems: (1) same redundancy as
    # capture_exception() above -- the real report is sentry_sdk.capture_exception(), called right
    # after this returns. (2) worse: the message embeds a per-call timestamped filename, so GlitchTip
    # can never group repeats -- every single occurrence is a distinct "new issue". Confirmed via direct
    # DB query: one device stuck in a UI crash loop produced 11,767 distinct "logged crash to [...]"
    # issues in 20 hours, each firing its own alert (this is what was flooding Discord). info() keeps
    # it as a breadcrumb without creating an issue at all.
    cloudlog.info(f"logged crash to {files}")
    # End BluePilot
  except Exception:
    cloudlog.exception("error when attempting to save exception")


def capture_fingerprint_mock(vin: str | None = None) -> None:
  try:
    set_user()
    # BluePilot: tag the VIN so this is actionable — without it we can't tell a VIN-query failure
    # (vin == VIN_UNKNOWN, "0"*17, see opendbc/car/vin.py) from a case where the VIN was read fine
    # but nothing (VIN-based or CAN/FW) matched it. Same data opendbc_repo/car_helpers.py now logs
    # locally for the "car doesn't match any fingerprints" carlog.error() path — this is the other
    # reporting path for the same failure (see sunnypilot/selfdrive/car/interfaces.py:log_fingerprint).
    if vin:
      sentry_sdk.set_tag("carVin", vin)
    # End BluePilot
    message = "car doesn't match any fingerprints"
    sentry_sdk.capture_message(message=message, level="error")
    sentry_sdk.flush()
  except Exception as e:
    cloudlog.exception(f"sentry fingerprint MOCK exception: {e}")


def capture_fingerprint(candidate: str, car_name: str) -> None:
  # BluePilot: tag-only, no capture_message() — a successful fingerprint is not itself a crash and
  # shouldn't create its own issue. The tags persist on this process's scope, so they're attached
  # automatically to any real crash this process reports later.
  try:
    set_user()
    sentry_sdk.set_tag("carFingerprint", candidate)
    sentry_sdk.set_tag("carName", car_name)
  except Exception as e:
    cloudlog.exception(f"sentry fingerprint exception: {e}")
  # End BluePilot


# BluePilot: was log_fingerprint() in sunnypilot/selfdrive/car/interfaces.py, which upstream removed with Sentry
def log_fingerprint(CP) -> None:  # CP: CarParams; no opendbc import here, manager loads this module
  if CP.carFingerprint == "MOCK":
    capture_fingerprint_mock(CP.carVin)  # pass vin for diagnosability
  else:
    capture_fingerprint(CP.carFingerprint, CP.brand)


def is_active() -> bool:
  """Whether this process has a running client: init() ran here, or in the parent before the fork."""
  return sentry_sdk.get_client().is_active()
# End BluePilot


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def set_user() -> None:
  dongle_id, git_username, _ = get_properties()
  sentry_sdk.set_user({"id": dongle_id, "name": git_username})


def get_properties() -> tuple[str, str, str]:
  params = Params()
  hardware_serial: str = params.get("HardwareSerial") or ""
  git_username: str = params.get("GithubUsername") or ""
  dongle_id: str = params.get("DongleId") or f"{UNREGISTERED_DONGLE_ID}-{hardware_serial}"
  sunnylink_dongle_id: str = params.get("SunnylinkDongleId") or UNREGISTERED_SUNNYLINK_DONGLE_ID

  return dongle_id, git_username, sunnylink_dongle_id


# BluePilot: a failed start leaves no live client behind, so a half-configured one never reports
def init(project: SentryProject) -> bool:
  # BluePilot: a PC or CI run is not a device, so it never starts a client (BP7 reported from PCs)
  if PC:
    return False
  try:
    return _init(project)
  except Exception:
    sentry_sdk.get_client().close()
    sentry_sdk.get_global_scope().set_client(None)
    raise
# End BluePilot


def _init(project: SentryProject) -> bool:
  build_metadata = get_build_metadata()

  env = build_metadata.channel_type
  dongle_id, git_username, sunnylink_dongle_id = get_properties()

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))

  # BluePilot: cloudlog is a SwagLogger(logging.Logger) with propagate=True, so a LoggingIntegration
  # on the root logger captures it: ERROR+ become Sentry events (catches daemon errors that are
  # caught-and-logged rather than crashing the process), WARNING+ become breadcrumbs for trailing
  # context on a real crash.
  #
  # This used to be level=logging.INFO with max_value_length=8192 (8x sentry_sdk's own 1024 default)
  # and no max_breadcrumbs cap (sentry_sdk default: 100) and traces_sample_rate=1.0 (100% APM/span
  # sampling) and enable_logs=True (a SECOND structured-logs capture pipeline duplicating the same
  # log stream) -- "maximum coverage", applied fleet-wide since sentry.init() runs once in
  # manager.py before the pre-fork daemon-spawn loop, so every one of ~30-40 daemon processes per
  # device independently accumulates its own breadcrumb/trace/log buffers for its whole run.
  # Multiple low-memory-reboot reports on bp-7.0 traced back to this -- INFO-level cloudlog calls are
  # extremely frequent throughout this codebase, and 8x-larger breadcrumb values times up to 100 of
  # them times dozens of long-running processes adds up on a 3.7GB device. Tuned down, not reverted:
  # WARNING-level breadcrumbs still give useful trailing context without routine info-level chatter;
  # traces and the redundant second logs pipeline are dropped since nothing in our crash-triage
  # workflow has used either (everything so far has come from Issues/breadcrumbs/tags).
  integrations.append(LoggingIntegration(level=logging.WARNING, event_level=logging.ERROR))

  init_kw = {
    "default_integrations": False,
    "release": get_version(),
    "integrations": integrations,
    "traces_sample_rate": 0.0,
    "max_value_length": 1024,  # back to sentry_sdk's own default
    "max_breadcrumbs": 50,  # explicit cap, half of sentry_sdk's 100 default
    "environment": env,
    "before_send": _before_send,
  }

  sentry_sdk.init(project.value, **init_kw)
  # End BluePilot

  sentry_sdk.set_user({"id": dongle_id, "name": git_username})
  sentry_sdk.set_tag("dirty", build_metadata.openpilot.is_dirty)
  sentry_sdk.set_tag("origin", build_metadata.openpilot.git_origin)
  sentry_sdk.set_tag("branch", build_metadata.channel)
  sentry_sdk.set_tag("commit", build_metadata.openpilot.git_commit)
  sentry_sdk.set_tag("device", HARDWARE.get_device_type())
  # BluePilot: comma dongle ID as its own tag (not just sentry_sdk.set_user above) — Discord/webhook
  # notifications only surface issue *tags* (apps.alerts.webhooks.gather_issue_tags), never the
  # Sentry "user" object, so without this the dongle ID never appears outside the GlitchTip UI.
  sentry_sdk.set_tag("dongle_id", dongle_id)
  sentry_sdk.set_tag("sunnylink_dongle_id", sunnylink_dongle_id)
  # End BluePilot

  return True
