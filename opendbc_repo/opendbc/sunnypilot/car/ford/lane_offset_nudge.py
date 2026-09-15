"""
BluePilot: temporary in-lane offset set by nudging the wheel on a straight road.

The in-lane offset in the BluePilot menu (``custom_path_offset_ang`` / ``custom_path_offset_curv``)
is a persistent, menu-only setting. This adds a *temporary* offset on top of it, set the way a
driver actually notices they want one: a light push on the wheel while the car is tracking a
straight, which biases the car that way and holds until the next turn.

The offset accumulates as a **percent of the detected lane width** rather than in meters, so the
same nudge means the same thing in a 3.0 m lane and a 3.9 m one -- "a tenth of a lane to the right"
travels between roads in a way "0.35 m" does not. It is converted to meters against the live
laneline width at the moment it is applied (see ``offset_m``).

Where it is applied: the meters value is added to the menu offset and handed to the same offset
argument the lane centering code already takes (``lane_center_trim.update`` in angle mode,
``path_offset`` in curvature mode). It therefore inherits every limiter those paths already apply
-- deviation clip, PSCM-saturation clamp, DBC clip, soft ROC in angle mode; the path_offset clip
and PID in curvature mode -- and adds no new route to the actuator. That is deliberate: this is a
comfort setting, and it should not be able to do anything the menu offset could not already do,
which is why the total is clipped to ``_TOTAL_OFFSET_MAX_M`` (the menu slider's own range).

**What counts as a nudge.** Only a small, deliberate, sustained push while lateral is active and
the road is straight:

- speed at or above ``_MIN_SPEED_MS`` -- the same 9 m/s where the lane centering speed ramp starts
  earning authority, below which the trim does nothing with the offset anyway;
- commanded curvature under ``_STRAIGHT_KAPPA`` (~660 m radius) -- on a curve the wheel angle is
  mostly the car's own steering, not the driver's intent;
- wheel angle between ``_NUDGE_MIN_DEG`` and ``_NUDGE_MAX_DEG`` with ``steeringPressed``, held for
  ``_NUDGE_DEBOUNCE_S``. The upper bound is what separates "bias me over" from "I am steering":
  past it nothing accumulates, and a real turn is caught by the reset below.

**What clears it.** A navigational turn, by either driver:

- ``HumanTurnDetector`` latching (the driver turned the car -- the same signal angle mode already
  uses to hand back control), or
- sustained curvature past ``_TURN_KAPPA`` for ``_TURN_HOLD_S`` (~100 m radius held 1.5 s), which
  catches a turn that lateral control drove itself, since no human turn is detected there.

Disengaging clears it too (the callers' inactive paths call ``reset``). It deliberately survives a
lane change: wanting to sit left of center is a preference about the road, and it should not need
re-entering after every pass.
"""
from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CarControllerParams

_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick (matches human_turn.py)

# Gates on when a nudge can accumulate at all.
_MIN_SPEED_MS = 9.0
_STRAIGHT_KAPPA = 0.0015      # 1/m, ~660 m radius
_NUDGE_MIN_DEG = 2.0          # below this is noise//torque ripple, not intent
_NUDGE_MAX_DEG = 12.0         # above this the driver is steering, not biasing
_NUDGE_DEBOUNCE_S = 0.2

# How fast the offset builds while a nudge is held, and how far it can go. The cap is a percent of
# lane width; _MAX_OFFSET_M is the belt-and-braces meters bound for an implausibly wide detection.
_RATE_PCT_PER_S = 6.0
_MAX_PCT = 12.0               # ~0.44 m in a 3.7 m lane, in line with the menu slider's +-0.5 m
_MAX_OFFSET_M = 0.45

# Reset on a navigational turn taken by lateral control itself (a human turn is caught separately).
_TURN_KAPPA = 0.01            # 1/m, ~100 m radius
_TURN_HOLD_S = 1.5

# Lane width used to convert percent -> meters when the model's lanelines are unusable.
_DEFAULT_LANE_WIDTH_M = 3.6
_LANE_WIDTH_MIN_M = 2.5
_LANE_WIDTH_MAX_M = 4.5

# Total (menu + nudge) offset bound -- the menu slider's own range, so the nudge can never ask for
# a position the menu could not already have asked for.
_TOTAL_OFFSET_MAX_M = 0.5


def _clip(v: float, lo: float, hi: float) -> float:
  return lo if v < lo else (hi if v > hi else v)



class LaneOffsetNudge:
  def __init__(self):
    self.percent = 0.0        # signed, positive = right (model frame, matches custom_path_offset)
    self.offset_m = 0.0       # percent converted against the current lane width
    self.nudging = False      # a nudge is being accumulated this frame (telemetry)
    self._hold_timer_s = 0.0
    self._turn_timer_s = 0.0
    self._lane_width_m = _DEFAULT_LANE_WIDTH_M

  def reset(self) -> None:
    self.percent = 0.0
    self.offset_m = 0.0
    self.nudging = False
    self._hold_timer_s = 0.0
    self._turn_timer_s = 0.0

  def update(self, enabled: bool, lat_active: bool, v_ego: float, steering_pressed: bool,
             steering_angle_deg: float, kappa_cmd: float, model, human_turn: bool,
             max_percent: float) -> float:
    """Accumulates/clears the temporary offset and returns it in meters (positive = right).

    Call once per lateral tick. ``kappa_cmd`` is the commanded curvature before the offset is
    applied -- it is what says whether the car is on a straight. ``human_turn`` is the
    ``HumanTurnDetector`` state the caller already computes.
    """
    if not enabled or not lat_active:
      self.reset()
      return 0.0

    # A navigational turn clears it, by whichever route the turn happened -- see module docstring.
    if human_turn:
      self.reset()
      return 0.0
    if abs(kappa_cmd) > _TURN_KAPPA:
      self._turn_timer_s += _STEER_DT
      if self._turn_timer_s >= _TURN_HOLD_S:
        self.reset()
        return 0.0
    else:
      self._turn_timer_s = 0.0

    self._lane_width_m = self._lane_width(model)

    straight = abs(kappa_cmd) <= _STRAIGHT_KAPPA and v_ego >= _MIN_SPEED_MS
    nudge_angle = _NUDGE_MIN_DEG <= abs(steering_angle_deg) <= _NUDGE_MAX_DEG
    if straight and steering_pressed and nudge_angle:
      self._hold_timer_s += _STEER_DT
    else:
      self._hold_timer_s = 0.0

    self.nudging = self._hold_timer_s >= _NUDGE_DEBOUNCE_S
    if self.nudging:
      # steeringAngleDeg is positive LEFT; percent is positive RIGHT (model frame), so the sign
      # flips here. See the Ford sign convention notes in lateral_curv_ext.
      direction = -1.0 if steering_angle_deg > 0.0 else 1.0
      cap = _clip(max_percent, 0.0, _MAX_PCT)
      self.percent = _clip(self.percent + direction * _RATE_PCT_PER_S * _STEER_DT, -cap, cap)

    self.offset_m = _clip(self.percent / 100.0 * self._lane_width_m, -_MAX_OFFSET_M, _MAX_OFFSET_M)
    return self.offset_m

  def total_offset(self, menu_offset_m: float) -> float:
    """Menu offset + nudge, clipped to the menu slider's own range."""
    return _clip(menu_offset_m + self.offset_m, -_TOTAL_OFFSET_MAX_M, _TOTAL_OFFSET_MAX_M)

  def _lane_width(self, model) -> float:
    """Live laneline width, falling back to the last good value / a nominal lane."""
    try:
      left = float(model.laneLines[1].y[0])
      right = float(model.laneLines[2].y[0])
      probs = model.laneLineProbs
      width = right - left
      if min(float(probs[1]), float(probs[2])) < 0.5:
        return self._lane_width_m
      if not (_LANE_WIDTH_MIN_M <= width <= _LANE_WIDTH_MAX_M):
        return self._lane_width_m
      return width
    except (AttributeError, IndexError, TypeError, ValueError):
      return self._lane_width_m
