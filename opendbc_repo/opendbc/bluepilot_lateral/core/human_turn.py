"""
BluePilot: shared manual-steering-override ("human turn") detection for Ford lateral control.

Both Ford lateral strategies hand control back to the driver on a sustained manual turn, and the
detection is identical, so it lives here as one class instead of being copied into each mode. What
each mode DOES with the signal differs: curvature-primary (``lateral_curv_ext``) zeroes its command
(reset_steering + post-reset ramp); angle-primary (``lateral_angle_ext``) forces lateral inactive
(mode 0 on the wire) and ramps path_angle back in from zero through its soft ROC on release.
"""
from ...car import DT_CTRL
from ...car.ford.values import CarControllerParams

# Require sustained hands-on AND a large angle (avoids resetting on small wheel nudges in a curve).
HUMAN_TURN_ANGLE_DEG = 45.0
HUMAN_TURN_HOLD_S = 1.5
# When the wheel was ALREADY past HUMAN_TURN_ANGLE_DEG at first contact -- lateral control had it
# turned mid-curve -- the angle condition is pre-satisfied, so a brief corrective nudge would latch
# after only HUMAN_TURN_HOLD_S of light contact and kill steering mid-curve. Require a longer hold
# there before reading it as an intentional takeover. Route 000000bd (2026-07-14) showed the
# discriminator holds on-road: all 7 deliberate turns began with the wheel below the threshold
# (driver wound it up through 45 deg); only mid-maneuver grabs/nudges began beyond it.
HUMAN_TURN_HOLD_PRETURNED_S = 3.0
_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick


class HumanTurnDetector:
  """Latches ``active`` once the driver holds real steering pressure AND ``|wheel angle|`` >
  ``HUMAN_TURN_ANGLE_DEG`` continuously for ``HUMAN_TURN_HOLD_S`` — long enough to tell an
  intentional turn from a brief nudge. ``just_released`` pulses True on the first frame after the
  override clears, so a mode can re-seed its command as it re-engages.

  Call ``update`` once per lateral tick while control is active. Modes that want the timer to zero
  on disengage call ``reset`` on their inactive path; modes that want it to persist simply stop
  calling ``update`` (the timer holds its value).
  """

  def __init__(self):
    self.hold_timer_s = 0.0
    self.active = False
    self._active_last = False
    self._pressed_last = False
    self._press_started_preturned = False

  def update(self, enabled: bool, steering_pressed: bool, steering_angle_deg: float) -> bool:
    self._active_last = self.active
    # Was the wheel already past the angle threshold when this press began? If so the driver is
    # touching a wheel that lateral control turned (mid-curve nudge), not driving a turn -- hold
    # the longer HUMAN_TURN_HOLD_PRETURNED_S before latching.
    if steering_pressed and not self._pressed_last:
      self._press_started_preturned = abs(steering_angle_deg) > HUMAN_TURN_ANGLE_DEG
    self._pressed_last = steering_pressed
    if not enabled:
      self.hold_timer_s = 0.0
    elif steering_pressed and abs(steering_angle_deg) > HUMAN_TURN_ANGLE_DEG:
      self.hold_timer_s += _STEER_DT
    else:
      self.hold_timer_s = 0.0
    hold_req = HUMAN_TURN_HOLD_PRETURNED_S if self._press_started_preturned else HUMAN_TURN_HOLD_S
    self.active = self.hold_timer_s >= hold_req
    return self.active

  @property
  def just_released(self) -> bool:
    return self._active_last and not self.active

  def reset(self) -> None:
    self.hold_timer_s = 0.0
    self.active = False
    self._active_last = False
    self._pressed_last = False
    self._press_started_preturned = False
