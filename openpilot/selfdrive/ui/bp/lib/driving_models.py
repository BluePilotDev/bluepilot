"""
BluePilot: what the home screen's Driving Models card shows. The on-device model, and the big model when one is in
play (JetLink's on a device on the USB-C port, or a chestnut's), with which of them drives or would, from one
snapshot of their state. Pure: the card (bluepilot/ui/widgets/model_info.py) reads the inputs and draws the view.

The on-device model is sunnypilot's active bundle (modelManagerSP.activeBundle). The big model is JetLink's while the
link is on (its status() names the model it builds), else a fitted chestnut's (ui_state.chestnut_state and the
model manager's chestnut pick). JetLink comes first: a JetLink host on the port is not a chestnut.
"""
from dataclasses import dataclass

from openpilot.selfdrive.ui.bp.lib import jetlink_status

# tones: what a tag or the pill says about the model, and the card's colour for it
DRIVE, READY, WORK, ERROR, IDLE = 'drive', 'ready', 'work', 'error', 'idle'

# JetLink's phase -> the big model's tag and tone
JETLINK_TAGS = {
  'active': ("Driving", DRIVE),
  'waiting': ("Ready: re-engage", READY),
  'prepared': ("Ready", READY),
  'preparing': ("Preparing", WORK),
  'connecting': ("Connecting", WORK),
  'reconnecting': ("Reconnecting", WORK),
  'failed': ("Failed", ERROR),
  'unavailable': ("Unavailable", ERROR),
  'off': ("Off", IDLE),
}
UNKNOWN_TAG = ("Working", WORK)

# sunnypilot's ChestnutState values -> the big model's tag and tone
CHESTNUT_TAGS = {
  'active': ("Driving", DRIVE),
  'ready': ("Ready", READY),
  'loading': ("Loading", WORK),
  'uncompiled': ("Not compiled", WORK),
  'failed': ("Failed", ERROR),
}

DEFAULT_SMALL = "Default Model"
NO_BIG_PICK = "No big model picked"
USB3_HINT = "USB 3 recommended"


@dataclass(frozen=True)
class Tile:
  role: str                         # what this model is: "On-device model", "Big model (JetLink, iOS)"
  name: str                         # the model's name
  tag: str = ""                     # its state, "" for none
  tone: str = IDLE
  details: tuple[str, ...] = ()     # the lines under it: connection, speed, progress, why it cannot run
  progress: float | None = None     # 0..1 while it is being prepared
  driving: bool = False             # it drives now


@dataclass(frozen=True)
class View:
  small: Tile
  big: Tile | None                  # None: no big model in play (link off, no chestnut)
  pill: str = ""                    # the card's headline state, "" for none
  pill_tone: str = IDLE


def _jetlink_tile(status, speed: str | None) -> Tile:
  phase = str(getattr(status, 'phase', ''))
  tag, tone = JETLINK_TAGS.get(phase, UNKNOWN_TAG)
  name = status.active_model or status.model or jetlink_status.default_big_model(status) or NO_BIG_PICK

  details = []
  frac = None
  if status.reason:
    details.append(str(status.reason))
  if (p := jetlink_status.progress(status)) is not None:
    _, frac, line = p
    if phase == 'preparing' and 0.0 < frac < 1.0:
      tag = f"{tag} {frac:.0%}"
    details.append(line)
  if getattr(status, 'enabled', False) and not status.active_model and status.model and phase != 'preparing':
    details.append("Not built yet")
  port = jetlink_status.port_line(status)
  if port:
    details.append(port.removesuffix('.'))
  label = jetlink_status.speed_label(speed)
  if label is not None:
    details.append(label if label == "USB 3" else f"{label}: {USB3_HINT}")

  mode = str(getattr(status, 'mode', '') or '')
  role = "Big model (JetLink, iOS)" if mode == 'ios' else "Big model (JetLink)"
  return Tile(role, name, tag, tone, tuple(details), frac if frac is not None and phase == 'preparing' else None,
              driving=phase == 'active')


def _chestnut_tile(chestnut_state: str, name: str | None) -> Tile | None:
  if chestnut_state not in CHESTNUT_TAGS:
    return None
  tag, tone = CHESTNUT_TAGS[chestnut_state]
  return Tile("Big model (chestnut)", name or NO_BIG_PICK, tag, tone, driving=chestnut_state == 'active')


def view(small_name: str | None, status=None, speed: str | None = None, chestnut_state: str = 'disconnected',
         chestnut_name: str | None = None) -> View:
  """The card's view.

  small_name: the on-device model's name (modelManagerSP.activeBundle), None or "" for the default.
  status, speed: JetLink's status() and the link's USB speed (jetlink_status.StatusReader).
  chestnut_state, chestnut_name: ui_state.chestnut_state's value and the model manager's chestnut pick.
  """
  big = _jetlink_tile(status, speed) if jetlink_status.shown(status) else _chestnut_tile(chestnut_state, chestnut_name)

  if big is None:
    small = Tile("On-device model", small_name or DEFAULT_SMALL)
    return View(small, None)

  if big.driving:
    small = Tile("On-device model", small_name or DEFAULT_SMALL, "Standby", IDLE)
    return View(small, big, "Big model driving", DRIVE)

  # the big model is not driving: the on-device model drives until it is ready, and takes back over if it drops
  small = Tile("On-device model", small_name or DEFAULT_SMALL, "Fallback", IDLE)
  pills = {DRIVE: "Big model driving", READY: "Big model ready", WORK: "Preparing big model",
           ERROR: "Big model unavailable", IDLE: "Big model off"}
  pill = pills[big.tone]
  if big.tone == WORK and big.tag.startswith(("Connecting", "Reconnecting")):
    pill = "Connecting"
  if big.tone == ERROR and big.tag == "Failed":
    pill = "Big model failed"
  return View(small, big, pill, big.tone)
