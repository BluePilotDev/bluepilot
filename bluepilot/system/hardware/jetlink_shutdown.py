"""
hardwared's JetLink hooks: a host on the USB-C port (a Jetson that is always on) powers off with the comma, and the
offroad alert for a link the user turned on that cannot run. JetLink's docs/porting.md, "hardwared".
"""
import time

from openpilot.bluepilot import jetlink_adapter

# JetLink's SHUTDOWN_TIMEOUT (25 s) plus 5 s, written out so it holds without JetLink: the longest a wanted power-off
# waits for the host to go down first. BluePilot's own guarantee on top of shutdown_step(), which says yes on any
# failure itself
JETLINK_SHUTDOWN_CAP_S = 30.

OFFROAD_ALERT = 'Offroad_JetlinkUnavailable'


class JetlinkShutdown:
  """Asked once per hardwared loop. Only the DoShutdown path calls step(), never a reboot: a host powered off at a
  comma reboot does not come back."""

  def __init__(self, clock=time.monotonic):
    self.clock = clock
    # when power_monitor first wanted the power-off; None until then
    self.since: float | None = None

  @property
  def waiting(self) -> bool:
    """A power-off is wanted and waits for the host: no drive starts meanwhile (startup_conditions)."""
    return self.since is not None

  def step(self, should_shutdown: bool, reason: str = '') -> bool:
    """May the device power off now? should_shutdown is power_monitor's; JetLink holds it until the host has taken
    the request, at most JETLINK_SHUTDOWN_CAP_S."""
    now = self.clock()
    if should_shutdown and self.since is None:
      self.since = now
    capped = self.since is not None and now - self.since > JETLINK_SHUTDOWN_CAP_S
    return bool(jetlink_adapter.shutdown_step(should_shutdown, reason) or capped)

  @staticmethod
  def update_alert(set_offroad_alert_if_changed) -> None:
    """Why the link the user turned on cannot run, as the offroad alert; None, and no alert, with it off."""
    reason = jetlink_adapter.reason()
    set_offroad_alert_if_changed(OFFROAD_ALERT, reason is not None, extra_text=reason)
