"""
The JetLink settings both BluePilot settings menus show (TICI and MICI): the link selector and the charge-phone
toggle. Changeable offroad only: the link moves the USB-C port between the comma and a host, and modeld decides at
its start whether the big model may join.

It offers JetLink's three modes: Off, USB (a Jetson, Mac or PC, or an Android phone through a USB 3 hub: JetLink's
Android app uses the USB mode) and iOS (an iPhone or iPad). The phones are experimental upstream. A value the selector
does not offer shows no selection rather than a wrong one, and is left alone until the user picks another.
"""
from openpilot.bluepilot import jetlink_adapter

LINK_PARAM = jetlink_adapter.KEYS.link
CHARGE_PHONE_PARAM = jetlink_adapter.KEYS.charge_phone
# the selector's buttons, by JetlinkLink value (jetlink_adapter.SELECTABLE_MODES)
LINK_LABELS = {0: "Off", 1: "USB", 2: "iOS"}

TITLE = "Big Model Link (JetLink)"
DESCRIPTION = " ".join((
  "Run the big driving model on a device running JetLink on the comma's USB-C port.",
  "USB: a Jetson, Mac or PC, or an Android phone through a USB 3 hub. iOS: an iPhone or iPad. Phones are experimental.",
  "The comma must be powered from the car harness, not that port.",
  "While the link holds the port, ADB over USB is unavailable; the BluePilot portal over WiFi still works.",
  "Change offroad only, then reboot.",
))
CHARGE_PHONE_TITLE = "JetLink: Charge Phone"
CHARGE_PHONE_DESCRIPTION = "Let a phone running JetLink charge from the comma's USB-C port. Change offroad only."


def read_link(params) -> int:
  """JetlinkLink, or 0 when it is unset or unreadable."""
  try:
    return int(params.get(LINK_PARAM, return_default=True) or 0)
  except Exception:
    return 0


def selected_index(params) -> int:
  """The selector's button for the stored value; -1 (none) for a value it does not offer."""
  mode = read_link(params)
  return jetlink_adapter.SELECTABLE_MODES.index(mode) if mode in jetlink_adapter.SELECTABLE_MODES else -1


def write_link(params, index: int, offroad: bool) -> bool:
  """Store the selector's button `index`, offroad only. True when written."""
  if not offroad or not 0 <= index < len(jetlink_adapter.SELECTABLE_MODES):
    return False
  params.put(LINK_PARAM, jetlink_adapter.SELECTABLE_MODES[index])
  return True
