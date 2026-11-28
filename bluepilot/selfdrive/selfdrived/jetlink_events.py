"""
selfdrived's JetLink swap events: JetLink's tracker (jetlink/openpilot/events.py, behind the adapter's hooks), its
flags raised as BluePilot's, sunnypilot's and openpilot's events.

JetLink's big model joins mid-drive and can leave again. It swaps in only while nothing is in control (offer), holds
engagement off for a second after a swap (no_entry, then the chime), warns when the link falls while in control
(link_lost) and forgives the frame a switch drops (settling). With no JetLink on the device, or the link off, there
are none.

Adapted from zoompilot's openpilot/sunnypilot/selfdrive/selfdrived/accelerator_events.py
(https://github.com/zoompilot/zoompilot, MIT, Copyright (c) 2026- Zeph Leggett). BluePilot publishes no state of its
own (D2 a): the tracker reads modeld's runtime record.
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom
from openpilot.common.realtime import DT_CTRL
from openpilot.bluepilot import jetlink_adapter
from openpilot.bluepilot.selfdrive.selfdrived.events_bp import EventsBP, EventNameBP

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName


class JetlinkEvents:
  # JetLink is optional: its owner exiting costs the big model, never engagement, so it is never processNotRunning
  OPTIONAL_PROCESSES = frozenset({jetlink_adapter.OWNER})

  def __init__(self):
    # JetLink's tracker behind the hooks' net; None without JetLink
    self.swap = jetlink_adapter.swap_events(DT_CTRL)
    # the second after a swap, this tick: MADS pauses rather than refusing (mads/state.py)
    self.no_entry = False

  @property
  def settling(self) -> bool:
    """Within a second of the big model swapping in or handing back: hold the comm and localizer errors its dropped
    frame raises, as for a native big model's load."""
    return self.swap is not None and self.swap.settling

  def update(self, sm: messaging.SubMaster, in_control: bool, events, events_sp, events_bp: EventsBP) -> None:
    """`in_control`: openpilot or MADS is engaged, MADS even while its lateral is paused, the rule the adapter's swap
    waits on (Adapter.engaged)."""
    self.no_entry = False
    if self.swap is None:
      return
    model_ok = sm.alive['modelV2'] and sm.valid['modelV2']
    flags = self.swap.update(model_ok, model_ok and sm['modelV2'].big, in_control)
    if flags.offer:
      events_bp.add(EventNameBP.bigModelAvailable)
    if flags.link_lost:
      events_bp.add(EventNameBP.bigModelLinkLost)
    if flags.no_entry or flags.chime:
      # a native chime at the start of the no-entry second would say engage now; it waits for the end of it
      events_sp.remove(EventNameSP.bigModelReady)
    if flags.no_entry:
      # openpilot's no-entry for a big model not ready to drive; native, since the state machine reads only those
      self.no_entry = True
      events.add(EventName.bigModelLoading)
    if flags.chime:
      events_sp.add(EventNameSP.bigModelReady)
