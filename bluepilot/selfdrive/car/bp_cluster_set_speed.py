"""BluePilot: with ICBM on a Ford, openpilot's set-speed target follows the truck's cluster set speed at engage and
after a driver Set+/Set- press, so the two can't disagree and ICBM never presses toward a speed the truck isn't
showing (BluePilotDev/bluepilot#232). The rest of the time the target is openpilot's: Speed Limit Assist writes it
and ICBM presses until the cluster matches. A press that confirms a speed limit isn't followed: the truck moves one
step on it, then Speed Limit Assist writes the limit as the target and ICBM takes the truck there."""
from opendbc.car.structs import car
from openpilot.common.realtime import DT_CTRL

ButtonType = car.CarState.ButtonEvent.Type
SET_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise)  # ICBM's own presses never show up as these

# Following lasts this long after a release (or after the cluster first shows a speed at engage). card runs at
# 100 Hz; ICBM waits INACTIVE_TIMER (0.4 s) after a release before it compares its target with the cluster, and its
# target reaches it through the planner (20 Hz), so the final target must be in carState well before 0.4 s.
SETTLE_TIME = 0.2  # s
SETTLE_FRAMES = round(SETTLE_TIME / DT_CTRL)


def follows_cluster(CP, CP_SP) -> bool:
  """A Ford with ICBM on: stock ACC holds the set speed (pcmCruise) and ICBM sets it (not pcmCruiseSpeed)."""
  return CP.brand == "ford" and CP.pcmCruise and not CP_SP.pcmCruiseSpeed


class ClusterSetSpeed:
  def __init__(self):
    self.frames_left = 0
    self.enabled_prev = False
    self.held = False
    self.confirming = False

  def update(self, CS, enabled: bool, confirms_limit) -> bool:
    """Once per card frame. True when the target should be the cluster set speed this frame.
    confirms_limit(button): whether a press of that button confirms Speed Limit Assist's prompt."""
    shown = CS.cruiseState.speedCluster > 0
    for b in CS.buttonEvents:
      if b.type in SET_BUTTONS:
        self.held = b.pressed
        if b.pressed:
          self.confirming = confirms_limit(b.type.raw)
    if not enabled:
      self.frames_left = 0
      self.held = self.confirming = False
    elif self.held or not self.enabled_prev:
      self.frames_left = SETTLE_FRAMES
    elif shown:
      self.frames_left = max(self.frames_left - 1, 0)
    self.enabled_prev = enabled
    return shown and self.frames_left > 0 and not self.confirming
