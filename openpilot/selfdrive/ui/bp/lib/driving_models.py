"""
BluePilot: what the home screen's Driving Models card shows. The on-device model, and the big model when one is in
play (JetLink's on a device on the USB-C port, or a chestnut's), with which of them drives or would, from one
snapshot of their state. Pure: the card (bluepilot/ui/widgets/model_info.py) reads the inputs and draws the view.

The on-device model is sunnypilot's active bundle (modelManagerSP.activeBundle). The big model is JetLink's while the
link is on (its status() names the model it builds), else a fitted chestnut's (ui_state.chestnut_state and the
model manager's chestnut pick). JetLink comes first: a JetLink host on the port is not a chestnut.
"""
import re
from dataclasses import dataclass

from openpilot.selfdrive.ui.bp.lib import jetlink_status

# tones: what a tag, a fact or a note says, and the card's colour for it. PLAIN is a value with nothing to flag.
DRIVE, READY, WORK, ERROR, IDLE, PLAIN = 'drive', 'ready', 'work', 'error', 'idle', 'plain'

# JetLink's phase -> the big model's tag and tone
JETLINK_TAGS = {
  'active': ("Driving", DRIVE),
  'waiting': ("Re-engage", READY),
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

BIG, SMALL = "Big model", "On-device model"
DEFAULT_SMALL = "Default Model"
NO_BIG_PICK = "No big model picked"
RECOMMENDED_SPEED = "USB 3"  # jetlink_status.speed_label's name for the speed JetLink wants
NOT_BUILT = "Not built yet"

# JetLink's transport names the iOS peer: 'iOS over USB (192.168.60.12)'. The card has no room for the address.
_PEER = re.compile(r'\s*\(.*\)\s*$')


@dataclass(frozen=True)
class Fact:
  label: str                        # "Source", "Link"
  value: str
  tone: str = PLAIN


@dataclass(frozen=True)
class Note:
  text: str                         # why it cannot run, the build's progress, what is on the port, a hint
  tone: str = IDLE


@dataclass(frozen=True)
class Tile:
  role: str                         # BIG or SMALL
  name: str                         # the model's name
  tag: str = ""                     # its state, "" for none
  tone: str = IDLE
  facts: tuple[Fact, ...] = ()      # where it runs and how it is linked
  notes: tuple[Note, ...] = ()
  progress: float | None = None     # 0..1 while it is being prepared
  driving: bool = False             # it drives now
  standby: bool = False             # another model drives: drawn dimmed
  badge: str = ""                   # the link's USB speed ("USB3", "USB2"), at the right of the last fact's row
  badge_tone: str = IDLE


@dataclass(frozen=True)
class View:
  small: Tile
  big: Tile | None                  # None: no big model in play (link off, no chestnut)


def _jetlink_tile(status, speed: str | None) -> Tile:
  phase = str(getattr(status, 'phase', ''))
  tag, tone = JETLINK_TAGS.get(phase, UNKNOWN_TAG)
  name = status.active_model or status.model or jetlink_status.default_big_model(status) or NO_BIG_PICK
  ios = str(getattr(status, 'mode', '') or '') == 'ios'

  facts = [Fact("Source", "JetLink iOS" if ios else "JetLink")]
  notes = []
  if getattr(status, 'port_state', None) == 'connected':
    facts.append(Fact("Link", _PEER.sub('', str(status.transport)) or "USB"))
  elif port := jetlink_status.port_line(status):
    notes.append(Note(port.removesuffix('.')))
  # the link's USB speed as a badge on the link's row: green at USB 3, what JetLink wants; yellow below it
  badge = badge_tone = ""
  if (label := jetlink_status.speed_label(speed)) is not None:
    badge, badge_tone = label.replace(" ", ""), (DRIVE if label == RECOMMENDED_SPEED else WORK)

  frac = None
  if status.reason:
    notes.insert(0, Note(str(status.reason), ERROR))
  if (p := jetlink_status.progress(status)) is not None:
    _, frac, line = p
    if phase == 'preparing' and 0.0 < frac < 1.0:
      tag = f"{tag} {frac:.0%}"
    notes.insert(0, Note(line, WORK))
  if getattr(status, 'enabled', False) and not status.active_model and status.model and phase != 'preparing':
    notes.append(Note(NOT_BUILT))

  return Tile(BIG, name, tag, tone, tuple(facts), tuple(notes),
              frac if frac is not None and phase == 'preparing' else None, driving=phase == 'active',
              badge=badge, badge_tone=badge_tone or IDLE)


def _chestnut_tile(chestnut_state: str, name: str | None) -> Tile | None:
  if chestnut_state not in CHESTNUT_TAGS:
    return None
  tag, tone = CHESTNUT_TAGS[chestnut_state]
  return Tile(BIG, name or NO_BIG_PICK, tag, tone, (Fact("Source", "chestnut"),), driving=chestnut_state == 'active')


def view(small_name: str | None, status=None, speed: str | None = None, chestnut_state: str = 'disconnected',
         chestnut_name: str | None = None) -> View:
  """The card's view.

  small_name: the on-device model's name (modelManagerSP.activeBundle), None or "" for the default.
  status, speed: JetLink's status() and the link's USB speed (jetlink_status.StatusReader).
  chestnut_state, chestnut_name: ui_state.chestnut_state's value and the model manager's chestnut pick.
  """
  big = _jetlink_tile(status, speed) if jetlink_status.shown(status) else _chestnut_tile(chestnut_state, chestnut_name)

  if big is None:
    return View(Tile(SMALL, small_name or DEFAULT_SMALL), None)

  # the on-device model drives until the big model is ready, and takes back over if it drops
  if big.driving:
    return View(Tile(SMALL, small_name or DEFAULT_SMALL, "Standby", IDLE, standby=True), big)
  return View(Tile(SMALL, small_name or DEFAULT_SMALL, "Fallback", IDLE), big)
