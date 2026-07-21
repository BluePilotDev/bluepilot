"""BluePilot: one-time auto-calibration of the angle-mode speed adjustment factors.

The angle strategy (lateral_angle_ext.py) computes, for the high-curvature branch:

  factor(v) = interp(v, [V_LOW, V_HIGH], [1.30 * LOW_FACTOR, platform_gain * HIGH_FACTOR])
  path_angle = kappa_cmd * v * factor

LOW_FACTOR / HIGH_FACTOR (FordLowSpeedFactor_ang / FordHighSpeedFactor_ang) are per-car
constants the driver is asked to hand-tune with +/- buttons by comparing requested turn
to actual turn. This module automates exactly that comparison: in steady engaged curves
the ratio r = kappa_measured / kappa_commanded is the plant gain error, so the factor
that would have made actual == requested is factor_applied / r. Samples are collected
across speeds and solved as a weighted least-squares fit of the same two-anchor interp
model, yielding the corrected anchors directly.

The estimator is pure math with no I/O so the exact same code runs in two places:
  - offline, in tools/bp/angle_autocal_analyze.py, replaying logged drives so the
    result (and every accepted sample) can be inspected before anything touches the car
  - onboard, fed from lateral_angle_ext during normal driving until convergence, after
    which the factors are written once and the calibration locks (per-car, not per-drive)
"""
import math

from opendbc.car.ford.values import CAR

# Hard-coded per-platform gain defaults (moved here from lateral_angle_ext so this module
# and the offline analyzer share one source; lateral_angle_ext imports them back).
# CAN vehicles (Escape MK4, Bronco Sport, Explorer, Maverick, Edge)
GAIN_CAN = (1.00, 1.15)
# CAN-FD body-on-frame trucks (F-150, Lightning, Expedition, Ranger)
GAIN_CANFD_BOF = (0.95, 0.95)
# CAN-FD unibody SUVs (Mustang Mach-E, Escape MK4.5)
GAIN_CANFD_SUV = (1.00, 1.05)

CANFD_BOF_CARS = frozenset({
  CAR.FORD_F_150_MK14,
  CAR.FORD_F_150_LIGHTNING_MK1,
  CAR.FORD_EXPEDITION_MK4,
  CAR.FORD_RANGER_MK2,
})
CANFD_SUV_CARS = frozenset({
  CAR.FORD_MUSTANG_MACH_E_MK1,
  CAR.FORD_ESCAPE_MK4_5,
})


def platform_gains(fingerprint: str) -> tuple[float, float]:
  """(lowC_highV, highC_highV) platform gain pair for a car fingerprint."""
  if fingerprint in CANFD_BOF_CARS:
    return GAIN_CANFD_BOF
  if fingerprint in CANFD_SUV_CARS:
    return GAIN_CANFD_SUV
  return GAIN_CAN


# Speed anchors of the gain interpolation in lateral_angle_ext (m/s: ~30 mph and ~60 mph).
V_LOW = 13.5
V_HIGH = 26.82
# Fixed multiplier on the low-speed anchor in lateral_angle_ext.
LOW_ANCHOR_BASE = 1.30

# Sample admission gates (mirrored by both the offline analyzer and the onboard hook).
MIN_SPEED = 9.5             # m/s; below this the deviation clip is off and measurement is noisy
MIN_KAPPA = 0.001           # 1/m; fully inside the high-curvature branch the factors scale
MAX_KAPPA_RATE = 0.0015     # 1/m/s; quasi-steady curvature only
STEADY_TIME_S = 0.6         # command must be steady this long before samples count (PSCM lag)
MIN_RATIO, MAX_RATIO = 0.4, 2.5  # discard absurd ratios (measurement glitches)
MAX_LAT_ACCEL = 2.5         # m/s^2; kappa*v^2 above this is tire/comfort-limit territory, not gain error

# Driver-contamination guards. Ford flips steeringPressed at STEER_DRIVER_ALLOWANCE (1.0 Nm)
# sustained — a light grip below that threshold still steers the car, and the PSCM under-delivers
# for seconds after any touch (post-override attenuation observed on the Mach-E). So:
TORQUE_GUARD_NM = 0.5       # treat half the pressed threshold as hands-on for calibration purposes
PRESS_HOLDBACK_S = 1.0      # samples are staged this long; any grip during staging cancels them
PRESS_COOLDOWN_S = 3.0      # after any grip ends, delivery is suspect this long — no samples

# Convergence: effective weight is accumulated seconds of valid steady cornering.
CONVERGE_MIN_WEIGHT = 30.0   # per anchor (~30 s of steady curves near each anchor)
# Real-road residual scatter (crown, wind, surface) floors around 2-2.5% even with hours of
# clean samples (measured: 2.7 h Mach-E drive -> stderr 0.022/0.026), so 2% was unreachably
# strict. 3% matches VERIFY_TOL: a fit good to ~3% is exactly what a verification round can
# confirm or refute, and rounds are the real lock guard.
CONVERGE_MAX_STDERR = 0.03   # fit standard error per anchor

FACTOR_MIN, FACTOR_MAX = 0.5, 1.5  # same clamp as the settings +/- buttons

# Multi-round verification: after a converged fit the factors are applied and collection
# restarts against them; the calibration only locks when a subsequent round's recommendation
# is a no-change within VERIFY_TOL (proof the correction actually landed on the real car —
# the ratio model is first-order, so a large correction deserves a confirmation pass).
VERIFY_TOL = 0.03    # per-factor |new - applied| considered "no further change"
MAX_ROUNDS = 4       # safety bound; lock after this many applications regardless


def speed_alpha(v_ego: float) -> float:
  """Blend position of v between the two anchors: 0 = pure low anchor, 1 = pure high."""
  if v_ego <= V_LOW:
    return 0.0
  if v_ego >= V_HIGH:
    return 1.0
  return (v_ego - V_LOW) / (V_HIGH - V_LOW)


class AngleFactorEstimator:
  """Weighted least-squares fit of the two gain anchors from steady-curve samples.

  Model: the ideal gain at sample i is y_i = F_applied(v_i) / r_i where r_i is the
  measured/commanded curvature ratio. F_ideal(v) = (1-a)*A + a*B with a = speed_alpha(v),
  A = ideal low anchor (1.30 * LOW_FACTOR), B = ideal high anchor (gain * HIGH_FACTOR).
  Linear in (A, B) -> closed-form normal equations, accumulated incrementally so the
  onboard hook carries O(1) state.
  """

  def __init__(self, platform_gain_high: float, low_factor_applied: float, high_factor_applied: float):
    self.platform_gain_high = float(platform_gain_high)
    self.low_factor_applied = float(low_factor_applied)
    self.high_factor_applied = float(high_factor_applied)
    # Normal-equation accumulators for min sum w*((1-a)A + aB - y)^2
    self.s_ll = 0.0  # sum w*(1-a)^2
    self.s_lh = 0.0  # sum w*(1-a)*a
    self.s_hh = 0.0  # sum w*a^2
    self.s_ly = 0.0  # sum w*(1-a)*y
    self.s_hy = 0.0  # sum w*a*y
    self.s_w = 0.0   # sum w
    self.s_wy2 = 0.0  # sum w*y^2 (for residual/stderr)
    self.n = 0

  def applied_gain(self, v_ego: float) -> float:
    a = speed_alpha(v_ego)
    low = LOW_ANCHOR_BASE * self.low_factor_applied
    high = self.platform_gain_high * self.high_factor_applied
    return (1.0 - a) * low + a * high

  def add_sample(self, v_ego: float, kappa_cmd: float, kappa_meas: float, weight: float = 1.0) -> bool:
    """Add one steady-curve observation. Returns True if accepted.

    kappa_cmd is the curvature the strategy converted to path_angle (post any clips);
    kappa_meas is the pinion-derived measured curvature. Both in OP sign convention —
    only same-sign, above-threshold pairs are accepted.
    """
    if abs(kappa_cmd) < MIN_KAPPA or v_ego < MIN_SPEED:
      return False
    if abs(kappa_cmd) * v_ego * v_ego > MAX_LAT_ACCEL:
      return False  # the car may physically be unable to make this turn — not gain information
    if kappa_cmd * kappa_meas <= 0.0:
      return False
    r = kappa_meas / kappa_cmd
    if not (MIN_RATIO <= r <= MAX_RATIO):
      return False
    y = self.applied_gain(v_ego) / r
    a = speed_alpha(v_ego)
    w = float(weight)
    la = 1.0 - a
    self.s_ll += w * la * la
    self.s_lh += w * la * a
    self.s_hh += w * a * a
    self.s_ly += w * la * y
    self.s_hy += w * a * y
    self.s_w += w
    self.s_wy2 += w * y * y
    self.n += 1
    return True

  @property
  def weight_low(self) -> float:
    """Effective sample weight attributed to the low anchor."""
    return self.s_ll + self.s_lh

  @property
  def weight_high(self) -> float:
    return self.s_hh + self.s_lh

  def solve(self):
    """Solve for the ideal anchors. Returns (low_factor, high_factor, stats) or None.

    low_factor / high_factor are the values to store in FordLowSpeedFactor_ang /
    FordHighSpeedFactor_ang (already divided by the fixed anchor bases and clamped to
    the same range the +/- buttons allow).
    """
    det = self.s_ll * self.s_hh - self.s_lh * self.s_lh
    if self.n < 10 or det < 1e-9:
      return None
    anchor_low = (self.s_ly * self.s_hh - self.s_hy * self.s_lh) / det
    anchor_high = (self.s_hy * self.s_ll - self.s_ly * self.s_lh) / det

    # Residual variance -> per-anchor standard errors from the normal-equation inverse.
    sse = max(0.0, self.s_wy2
              - 2.0 * (anchor_low * self.s_ly + anchor_high * self.s_hy)
              + anchor_low * anchor_low * self.s_ll
              + 2.0 * anchor_low * anchor_high * self.s_lh
              + anchor_high * anchor_high * self.s_hh)
    dof = max(1.0, self.s_w - 2.0)
    var = sse / dof
    stderr_low = math.sqrt(max(0.0, var * self.s_hh / det))
    stderr_high = math.sqrt(max(0.0, var * self.s_ll / det))

    low_factor = min(FACTOR_MAX, max(FACTOR_MIN, anchor_low / LOW_ANCHOR_BASE))
    high_factor = min(FACTOR_MAX, max(FACTOR_MIN, anchor_high / self.platform_gain_high))
    stats = {
      "n": self.n,
      "weight_low": self.weight_low,
      "weight_high": self.weight_high,
      "stderr_low": stderr_low,
      "stderr_high": stderr_high,
      "anchor_low": anchor_low,
      "anchor_high": anchor_high,
    }
    return low_factor, high_factor, stats

  def converged(self) -> bool:
    result = self.solve()
    if result is None:
      return False
    _, _, stats = result
    return (stats["weight_low"] >= CONVERGE_MIN_WEIGHT
            and stats["weight_high"] >= CONVERGE_MIN_WEIGHT
            and stats["stderr_low"] <= CONVERGE_MAX_STDERR
            and stats["stderr_high"] <= CONVERGE_MAX_STDERR)


class SteadyStateGate:
  """Admits samples only after the command has been steady for STEADY_TIME_S.

  Both consumers drive this at the 20 Hz lateral rate with the same flags the
  strategy itself computes, so offline and onboard gating are identical.
  """

  def __init__(self, dt: float = 0.05):
    self.dt = dt
    self.steady_s = 0.0
    self.kappa_last = None
    self.grip_cooldown_s = 0.0

  def update(self, lat_active: bool, kappa_cmd: float, steering_pressed: bool,
             angle_rate_limited: bool, deviation_limited: bool,
             human_turn: bool, stall_blip: bool,
             saturated: bool = False, driver_torque: float = 0.0) -> bool:
    # Any grip — including light torque below the steeringPressed threshold — starts a
    # cooldown: the driver was steering, and the PSCM's delivery stays suspect for a while
    # after release (post-touch attenuation).
    grip = steering_pressed or human_turn or abs(driver_torque) > TORQUE_GUARD_NM
    if grip:
      self.grip_cooldown_s = PRESS_COOLDOWN_S
    else:
      self.grip_cooldown_s = max(0.0, self.grip_cooldown_s - self.dt)

    ok = (lat_active and not grip and self.grip_cooldown_s <= 0.0
          and not angle_rate_limited and not deviation_limited
          and not stall_blip and not saturated
          and abs(kappa_cmd) >= MIN_KAPPA)
    if ok and self.kappa_last is not None:
      ok = abs(kappa_cmd - self.kappa_last) / self.dt <= MAX_KAPPA_RATE
    self.kappa_last = kappa_cmd if lat_active else None
    self.steady_s = self.steady_s + self.dt if ok else 0.0
    return self.steady_s >= STEADY_TIME_S


class AutoCalPipeline:
  """Gate + holdback staging + estimator, driven with one call per 20 Hz lateral frame.

  Samples sit in a staging queue for PRESS_HOLDBACK_S before they reach the estimator;
  if the driver grips the wheel while they wait, they are cancelled — the grip was
  likely already influencing the car before detection tripped. Used identically by the
  onboard hook and the offline analyzer.
  """

  def __init__(self, platform_gain_high: float, low_factor_applied: float,
               high_factor_applied: float, dt: float = 0.05):
    self.est = AngleFactorEstimator(platform_gain_high, low_factor_applied, high_factor_applied)
    self.gate = SteadyStateGate(dt=dt)
    self.dt = dt
    self._staged: list[list] = []  # [age_s, v, kappa_cmd, kappa_meas]
    self._meas_last = None

  def idle(self):
    """Call on frames where lateral is inactive (disengaged / human-turn override)."""
    self.gate.update(False, 0.0, False, False, False, False, False)
    self._staged.clear()
    self._meas_last = None

  def update(self, v_ego: float, kappa_cmd: float, kappa_meas: float,
             steering_pressed: bool, angle_rate_limited: bool, deviation_limited: bool,
             human_turn: bool, stall_blip: bool,
             saturated: bool = False, driver_torque: float = 0.0) -> list:
    """Advance one frame. Returns the samples committed to the estimator this frame
    as (v, kappa_cmd, kappa_meas) tuples — the offline analyzer plots them; the
    onboard hook ignores the return value."""
    grip = steering_pressed or human_turn or abs(driver_torque) > TORQUE_GUARD_NM
    if grip:
      self._staged.clear()

    eligible = self.gate.update(True, kappa_cmd, steering_pressed,
                                angle_rate_limited, deviation_limited,
                                human_turn, stall_blip,
                                saturated=saturated, driver_torque=driver_torque)

    # The CAR must be settled too, not just the command: during closed-loop compensation
    # swings (understeer -> harder request -> convergence tail) the command can sit steady
    # while the measurement is still moving toward it — those ratios are transient, not
    # gain. Measured curvature is noisier than the command, so the bound is 3x looser.
    if eligible and self._meas_last is not None:
      eligible = abs(kappa_meas - self._meas_last) / self.dt <= 3.0 * MAX_KAPPA_RATE
    self._meas_last = kappa_meas

    # Age the staging queue; entries that survived the holdback graduate to the estimator.
    committed = []
    still_staged = []
    for entry in self._staged:
      entry[0] += self.dt
      if entry[0] >= PRESS_HOLDBACK_S:
        if self.est.add_sample(entry[1], entry[2], entry[3], weight=self.dt):
          committed.append((entry[1], entry[2], entry[3]))
      else:
        still_staged.append(entry)
    self._staged = still_staged

    if eligible:
      self._staged.append([0.0, v_ego, kappa_cmd, kappa_meas])
    return committed
