"""BluePilot: anti-weave smoothing for the Ford angle-mode command path.

Pure math, no I/O — the same pattern as angle_autocal.py, so every element is
unit-testable without cereal. FordLateralAngleExt owns one AngleSmoother, feeds it
the toggle/strength from params, and calls one method per smoothing element at the
exact point in the command path where that element lives:

  entering()        hysteresis on the curve-entering decision (VLT direction gate)
  prediction()      low-pass on the model predicted curvature (pre-blend)
  blend()           slew on the exit-blend ratio (no 4x steps from boolean chatter)
  kappa_schedule()  asymmetric filter on |kappa_cmd| feeding the gain interp — the
                    PRIMARY fix: the 0.0007-0.001 interp band sits in straight-road
                    noise, so unfiltered |kappa| lets the weave modulate its own loop
                    gain every cycle (0.23 Hz limit cycle measured on route 00000006)
  wire()            1-LSB hold on the outgoing path_angle (kills LSB dither)

Semantics: menu 1.0 = stock (strength 0.0 internally) — every method is an exact
passthrough, bit-identical to the toggle being off. Strength scales the release
time constant, the prediction RC, and the wire-hold band; curve ENTRY behavior is
strength-independent by design (fast-rise RC is fixed).

Never add output-side lag here: the closed-loop rig measured a curvature-scheduled
output low-pass degrading station-keeping 2.5x (in-loop lag). Input-side shaping +
the wire hold is the whole design. Validate control changes in the closed-loop rig
(bp-tools/sim/closed_loop_weave.py), not just open-loop replay.
"""

_STEER_DT = 0.05          # 20 Hz lateral cadence (mirrors lateral_angle_ext._STEER_DT)

GAIN_RC_UP = 0.10         # s — gain-schedule filter, rising |kappa| (preserves curve entry)
GAIN_RC_DOWN = 0.60       # s — falling side (kills the 0.2-0.27 Hz gain modulation)
PRED_RC = 0.12            # s — model predicted-curvature low-pass (inside VLT slack)
ENTER_HYST = 0.0003       # 1/m — hysteresis on the curve-entering decision (above model noise)
BLEND_SLEW = 0.0375       # blend-ratio step per 20 Hz call = 0.75/s
WIRE_HOLD = 0.0005        # rad = 1 LSB of LatCtlPath_An_Actl at strength 1.0

MENU_MIN = 1.0            # menu 1.0 = stock, no smoothing (bit-identical to toggle off)
MENU_MAX = 2.5            # strongest damping; internal strength = menu - 1.0 (0..1.5)


def _one_pole(state: float, target: float, rc: float, dt: float) -> float:
  """Discrete one-pole low-pass step toward target with time constant rc (rc<=0 -> passthrough)."""
  a = dt / (rc + dt)
  return state + a * (target - state)


class AngleSmoother:
  """State container + per-element filters. Every method returns its input unchanged
  (and keeps its internal state seeded for a clean future enable) whenever smoothing
  is inactive — so toggling or stepping strength mid-drive can never produce a
  transient from stale state."""

  def __init__(self, dt: float = _STEER_DT):
    self.dt = dt
    self.enabled = True       # master toggle (param re-read ~1 Hz by the owner)
    self.strength = 0.0       # EFFECTIVE scale (menu - 1.0); 0 = stock passthrough
    self.reset()

  def configure(self, enabled: bool, menu_value: float):
    """From the params poll: menu value is clamped to [MENU_MIN, MENU_MAX]."""
    self.enabled = bool(enabled)
    menu = min(MENU_MAX, max(MENU_MIN, float(menu_value)))
    self.strength = menu - 1.0

  @property
  def active(self) -> bool:
    return self.enabled and self.strength > 1e-6

  def reset(self):
    """Command-path discontinuity (disengage / human turn / stall blip): every filter
    re-seeds on its next active frame instead of averaging across the gap."""
    self._sched = 0.0
    self._sched_init = False
    self._pred = 0.0
    self._pred_init = False
    self._b_blend = None
    self._entering = False
    self._wire = 0.0

  # -- elements, in command-path order ------------------------------------------------------
  def entering(self, d_enter: float, raw_entering: bool) -> bool:
    """Hysteresis on the curve-entering boolean so noise straddling the boundary can't
    flip it (and the exit-blend gate with it) frame to frame near zero curvature."""
    if not self.active:
      self._entering = raw_entering
      return raw_entering
    if d_enter > ENTER_HYST:
      self._entering = True
    elif d_enter < -ENTER_HYST:
      self._entering = False
    # inside the band: hold the previous decision
    return self._entering

  def prediction(self, predicted_curvature: float) -> float:
    """Low-pass the model prediction (~50% of the straight-road command) to strip
    frame-to-frame model jitter. Equivalent to a ~PRED_RC earlier lookahead — inside
    the VLT's own slack, so no curve-entry cost."""
    if not self.active:
      self._pred_init = False       # a future enable re-seeds from the live value
      return predicted_curvature
    if not self._pred_init:
      self._pred = predicted_curvature
      self._pred_init = True
    else:
      self._pred = _one_pole(self._pred, predicted_curvature, PRED_RC * self.strength, self.dt)
    return float(self._pred)

  def blend(self, b_target: float) -> float:
    """Slew the exit-blend ratio instead of stepping it — boolean chatter then produces
    bounded 0.75/s ramps in the command mix rather than 4x discontinuities."""
    if not self.active:
      self._b_blend = None
      return b_target
    if self._b_blend is None:
      self._b_blend = float(b_target)
    else:
      step = b_target - self._b_blend
      step = max(-BLEND_SLEW, min(BLEND_SLEW, step))
      self._b_blend = float(self._b_blend + step)
    return self._b_blend

  def kappa_schedule(self, k_abs: float) -> float:
    """Asymmetric filter on |kappa_cmd| feeding the curvature-gain interp — the PRIMARY
    anti-weave fix (see module docstring). Fast rise keeps curve-entry gain arrival
    within ~0.2 s at any strength; the slow strength-scaled fall removes the gain
    modulation and ratchets toward the stable higher gain under oscillation.

    Seeds at the CURRENT |kappa| on its first active frame — enabling mid-curve must
    not start the schedule from zero and momentarily read 'straight road'."""
    if not self.active:
      self._sched_init = False      # a future enable re-seeds from the live value
      return k_abs
    if not self._sched_init:
      self._sched = k_abs
      self._sched_init = True
      return k_abs
    rc_down = max(GAIN_RC_UP, GAIN_RC_DOWN * self.strength)
    rc = GAIN_RC_UP if k_abs > self._sched else rc_down
    self._sched = _one_pole(self._sched, k_abs, rc, self.dt)
    return float(self._sched)

  def wire(self, path_angle: float) -> float:
    """Hold the outgoing wire value inside a 1-LSB band (scaled by strength).
    LatCtlPath_An_Actl's LSB is 0.0005 rad; near-zero commands crossed a code boundary
    962/min on the baseline route and the PSCM integrates that square wave into the
    felt dither. Worst steady-state bias is under the yaw-measurement quantum; a held
    frame is a zero-ROC frame and the release step is far inside the tightest soft
    ROC, so panda's path_angle checks cannot be tripped by this."""
    if not self.active:
      self._wire = path_angle
      return path_angle
    if abs(path_angle - self._wire) < WIRE_HOLD * self.strength:
      return self._wire
    self._wire = path_angle
    return path_angle
