"""
BluePilot's own onroad events (cereal OnroadEventBP, in the customReserved10 slot), which selfdrived turns into alerts
beside openpilot's and sunnypilot's and publishes as onroadEventsBP. JetLink's two first (jetlink_events.py).

A separate enum rather than new names in sunnypilot's OnroadEventSP, whose ordinals a sunnypilot merge would clash
with (AGENTS.md: no upstream schema edits).
"""
from openpilot.cereal import log, custom
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EventsBase, Priority, ET, Alert, AlertCallbackType

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventNameBP = custom.OnroadEventBP.EventName

EVENT_NAME_BP = {v: k for k, v in EventNameBP.schema.enumerants.items()}


EVENTS_BP: dict[int, dict[str, Alert | AlertCallbackType]] = {
  # JetLink's big model is ready while openpilot or MADS is in control: it swaps in only while nothing is, so the
  # driver disengages and engages again to use it (jetlink/openpilot/events.py, `offer`)
  EventNameBP.bigModelAvailable: {
    ET.PERMANENT: Alert(
      "Big Model Ready",
      "Re-engage to switch",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .2),
  },

  # JetLink's link fell, or lagged, while in control (`link_lost`): the small model drives on from a reset history
  # and nothing disengages, for HANDBACK_S (5 s). The driver is told to take control while it settles; a disengage
  # ends it
  EventNameBP.bigModelLinkLost: {
    ET.WARNING: Alert(
      "TAKE CONTROL",
      "Big model lost: small model driving",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.warningSoft, .2),
  },
}


class EventsBP(EventsBase):
  def __init__(self):
    super().__init__()
    self.event_counters = dict.fromkeys(EVENTS_BP.keys(), 0)

  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    return EVENTS_BP

  def get_event_name(self, event: int):
    return EVENT_NAME_BP[event]

  def get_event_msg_type(self):
    return custom.OnroadEventBP.Event
