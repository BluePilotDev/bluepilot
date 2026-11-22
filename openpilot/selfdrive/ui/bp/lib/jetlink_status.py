"""
JetLink's status as both BluePilot settings menus show it (TICI and MICI; R8.2): the phase chip, the progress line,
the USB-C port's state and the ADB note, from one status() snapshot. status() is file and param reads, safe at 5 Hz
(JetLink's jetlink/openpilot/status.py); StatusReader holds the UI to that.

Without a usable JetLink checkout status() is None, and there is nothing to show.

Adapted from zoompilot's openpilot/selfdrive/ui/sunnypilot/accelerator_link.py (https://github.com/zoompilot/zoompilot,
MIT, Copyright (c) 2026- Zeph Leggett). See bluepilot/jetlink_adapter/THIRD_PARTY_NOTICES.md.
"""
import time

from openpilot.bluepilot import jetlink_adapter

# status() is safe at 5 Hz
STATUS_PERIOD_S = 0.2

# the chip's words by Status.phase. PHASES only grows: one this build does not know reads as UNKNOWN_PHASE
PHASE_LABELS = {
  'unavailable': "Unavailable",
  'off': "Off",
  'active': "Big model driving",
  'waiting': "Ready: re-engage to switch",
  'reconnecting': "Reconnecting",
  'failed': "Failed",
  'preparing': "Preparing",
  'connecting': "Connecting",
  'prepared': "Ready",
}
UNKNOWN_PHASE = "Working"
# no usable JetLink checkout: status() is None
NO_JETLINK = "Not available"

PORT_LINES = {'empty': "Nothing on the USB port.", 'device': "A device is on the USB port."}
ADB_NOTE = "ADB over USB is unavailable while JetLink holds the port; the BluePilot portal over WiFi still works."


# the USB speed the host negotiated (jetlink_adapter.usb_speed), as the UI names it
SPEED_LABELS = {'super-speed-plus': "USB 3", 'super-speed': "USB 3", 'high-speed': "USB 2", 'full-speed': "USB 1",
                'low-speed': "USB 1"}


class StatusReader:
  """JetLink's status, and with speed=True the link's USB speed, at most every STATUS_PERIOD_S, from any number of
  widgets in one frame."""

  def __init__(self, clock=time.monotonic, speed: bool = False):
    self._clock = clock
    self._at = -float('inf')
    self._status = None
    self._read_speed = speed
    self._speed = None

  def _refresh(self):
    now = self._clock()
    if now - self._at >= STATUS_PERIOD_S:
      self._at = now
      try:
        self._status = jetlink_adapter.status()
      except Exception:
        self._status = None
      if self._read_speed:
        try:
          self._speed = jetlink_adapter.usb_speed() if self._status is not None else None
        except Exception:
          self._speed = None

  def get(self):
    self._refresh()
    return self._status

  def speed(self) -> str | None:
    """The link's USB speed as Linux names it, or None (speed=False, no JetLink, no host)."""
    self._refresh()
    return self._speed


def shown(status) -> bool:
  """Anything to show: JetLink is checked out and usable here, and the link is on, a host is there or something is
  in progress (Status.shown), or the link cannot run (its reason)."""
  return status is not None and (bool(status.shown) or status.reason is not None)


def phase_label(status) -> str:
  if status is None:
    return NO_JETLINK
  return PHASE_LABELS.get(str(status.phase), UNKNOWN_PHASE)


def progress(status) -> tuple[str, float, str] | None:
  """(stage, 0..1, the line) while JetLink is working, else None. The line names the cable once JetLink has counted
  enough link drops to blame it."""
  record = getattr(status, 'progress', None)
  if not isinstance(record, dict):
    return None
  stage = str(record.get('stage') or '')
  if stage in ('', 'ready'):
    return None
  try:
    frac = min(max(float(record.get('frac') or 0.0), 0.0), 1.0)
  except (TypeError, ValueError):
    frac = 0.0
  line = str(record.get('msg') or stage)
  if drops := record.get('drops'):
    hint = "check cable or app" if getattr(status, 'mode', '') == 'ios' else "check cable"
    line = f"{line}, {hint} ({drops} drops)"
  return stage, frac, line


def port_line(status) -> str:
  """What is on the comma's USB-C port now: JetLink's host, or what the CC pin says. Empty where the kernel does not
  say, rather than claiming an empty port."""
  port = getattr(status, 'port_state', None)
  if port == 'connected':
    return f"Connected: {status.transport}."
  return PORT_LINES.get(port, "")


def speed_label(speed: str | None) -> str | None:
  """"USB 3", "USB 2" or "USB 1" for the link's speed, None where it is not known."""
  return SPEED_LABELS.get(speed) if speed else None


def chip(status) -> str:
  """The chip, with the progress's percentage while there is one to show."""
  label = phase_label(status)
  if (p := progress(status)) is not None and 0.0 < p[1] < 1.0:
    return f"{label} {p[1]:.0%}"
  return label


def detail_lines(status) -> list[str]:
  """Everything under the chip: why it cannot run, the progress line, the port, the models, the ADB note."""
  if status is None:
    return []
  lines = []
  if status.reason:
    lines.append(f"Unavailable: {status.reason}")
  if (p := progress(status)) is not None:
    lines.append(p[2])
  if port := port_line(status):
    lines.append(port)
  if status.enabled:
    if status.active_model:
      lines.append(f"Big model: {status.active_model}")
    elif status.model:
      lines.append(f"Big model: {status.model} (not built yet)")
  if status.holds_port:
    lines.append(ADB_NOTE)
  return lines


def headline(status) -> str:
  """The one line a small screen has room for: why it cannot run, else the progress, else the big model, else the port."""
  lines = [line for line in detail_lines(status) if line != ADB_NOTE]
  if not lines:
    return ""
  if status.reason or progress(status) is not None:
    return lines[0]
  return next((line for line in lines if line.startswith("Big model: ")), lines[0])


def default_big_model(status) -> str | None:
  """JetLink's default big model, named as the chestnut's are: what runs with no pick. None without JetLink."""
  name = getattr(status, 'default_model', None)
  return name if isinstance(name, str) and name else None


# the pickers' reader: model_info asks every frame a model screen draws
_PICKER_READER = StatusReader()
# the big-model catalog's cache, where JetLink's list stamps itself (catalog.JETLINK_KEY)
_BIG_CATALOG_CACHE = "ModelManager_ModelsCache_Chestnut"


def picker_default_big_model(params) -> str | None:
  """The big-model pickers' default (R8.3): JetLink's default big model while the big-model list is JetLink's, else
  None and the picker keeps sunnypilot's. Never raises."""
  try:
    cached = params.get(_BIG_CATALOG_CACHE)
    if not (isinstance(cached, dict) and cached.get('jetlink') is True):
      return None
    return default_big_model(_PICKER_READER.get())
  except Exception:
    return None
