"""BluePilot: continuous auto-calibration of the angle-mode speed adjustment factors.

The angle strategy (lateral_angle_ext.py) computes, for the high-curvature branch:

  factor(v) = interp(v, [V_LOW, V_HIGH], [1.30 * LOW_FACTOR, platform_gain * HIGH_FACTOR])
  path_angle = kappa_cmd * v * factor

LOW_FACTOR / HIGH_FACTOR (FordLowSpeedFactor_ang / FordHighSpeedFactor_ang) are per-car
constants the driver is asked to hand-tune with the +/- buttons by comparing requested turn
to actual turn — watching the tops and bottoms of the two curvature traces and bumping the
factor until the peaks line up. This module automates exactly that loop, continuously:

  - evidence comes from curve apexes (matched command/measured curvature extrema, the
    "tops and bottoms of the graphs") and from steady-state curve segments;
  - a quality layer admits only clean linear-regime cornering — road bumps, rough
    surface, tire-limit turns, longitudinal load transfer, driver grip and bank-biased
    one-sided evidence are excised (with per-cause counters, so the offline analyzer can
    show exactly what was rejected and why);
  - as evidence accrues the applied factors are nudged in small bounded steps toward the
    estimate — visible live in the lateral tuning menu — backing off automatically when a
    step overshoots (each sample records the gain in force when it was taken, so the
    estimate of the IDEAL gain is invariant to the nudge trajectory);
  - the estimator's sufficient statistics serialize to/from the FordAngleAutoCalState
    param, so evidence spans drives and ignition cycles;
  - when there is nothing left to adjust for LOCK_STABLE_S of driving, the calibration
    locks (per-car, not per-drive). Toggling the setting off clears everything.

The estimator is pure math with no I/O so the exact same code runs in two places:
  - offline, in bp-tools/bp/angle_autocal_analyze.py, replaying logged drives so the
    behavior (and every accepted/rejected sample) can be inspected before touching the car
  - onboard, fed from lateral_angle_ext during normal driving
"""
import math
from dataclasses import dataclass
from typing import NamedTuple

# The strategy owns the gain model; this module (and the offline analyzer) consume it.
from opendbc.sunnypilot.car.ford.values_ext import V_LOW, V_HIGH, LOW_ANCHOR_BASE


@dataclass(frozen=True)
class Frame:
  """One 20 Hz lateral frame of evidence inputs, shared verbatim by the onboard
  controller and the offline analyzer. Every field is required — defaults on physical
  signals (a_ego, saturated, ...) were a silent-wrong-answer risk whenever the two
  consumers drifted on parameter order."""
  v_ego: float
  kappa_cmd: float
  kappa_meas: float
  steering_pressed: bool
  angle_rate_limited: bool
  deviation_limited: bool
  saturated: bool
  driver_torque: float
  a_ego: float
  ws_spread: float | None
  low_factor: float
  high_factor: float
  lateral_delay: float  # liveDelay.lateralDelay (s) — evidence is aligned against cmd(t - delay)

# Sample admission gates (mirrored by both the offline analyzer and the onboard hook).
MIN_SPEED = 9.5             # m/s; below this the deviation clip is off and measurement is noisy
MIN_KAPPA = 0.001           # 1/m; fully inside the high-curvature branch the factors scale
# --- Lag-aligned steadiness --------------------------------------------------------------
# The measurement lags the command by the actuation delay (liveDelay: ~0.15-0.30s typical,
# up to ~0.42s observed). Evidence ratios are therefore taken against the command from
# lateral_delay seconds AGO (a short ring buffer in the pipeline), which removes the
# first-order lag bias outright instead of demanding a nearly frozen command. What remains
# is the DELAY ESTIMATE's own error (~±0.1s), so the bounds below only need to keep
# kappa's fractional change over that residual small: bias ≈ tau_err * |dk/dt| / |k| —
# at the 0.5/s relative bound and 0.1s residual, <= 5%, sign-symmetric over entries and
# exits, inside the stderr machinery. Validated on-road 2026-07-22 (routes 0b/12/13):
# vs the old frozen-command gate this recovers 3-5x the evidence on winding roads with
# the pooled fit unchanged (high 1.127 before and after, stderr 0.033 -> 0.012).
LAG_MIN_S, LAG_MAX_S = 0.10, 0.42  # trust clamp for the liveDelay estimate
# --- Loop-quietness admission ------------------------------------------------------------
# Steady evidence is taken only while the tracking loop is CALM: the smoothed
# |meas - aligned cmd| error trend must be flat. Samples taken mid-excursion or
# mid-correction carry the LOOP's dynamics, not the plant's gain — audited on-road
# 2026-07-23: 39% of admissions were non-quiet and read up to 3% low, biasing the fit
# ~+0.01 high ("moving the needle and calling it done"). Convergence may take as many
# passes as calm data requires; a slower right answer beats a faster wrong one.
QUIET_TAU_S = 0.4           # smoothing of |tracking error| before its trend is judged
QUIET_ERR_RATE = 0.0002     # 1/m/s: |d|err|/dt| above this = loop dynamics, no evidence
MAX_KAPPA_RATE = 0.0015     # 1/m/s absolute floor of the admission rate bound
REL_KAPPA_RATE = 0.5        # 1/s: |dk/dt| may reach this fraction of |k| (winding roads)
STEADY_TIME_S = 0.3         # command steady this long before samples count (PSCM settle)
STEADY_DRIFT_FRAC = 0.25    # max |kappa - window start| as a fraction of |kappa|
MIN_RATIO, MAX_RATIO = 0.4, 2.5  # discard absurd ratios (measurement glitches)
MAX_LAT_ACCEL = 2.5         # m/s^2; kappa*v^2 above this is tire/comfort-limit territory, not gain error
# Near the limit cmd!=meas is physics, not gain error: evidence weight fades linearly to
# zero over the last LAT_ACCEL_SOFT_BAND m/s^2 below MAX_LAT_ACCEL.
LAT_ACCEL_SOFT_BAND = 1.0
# Longitudinal load transfer changes the effective lateral gain — no evidence while
# braking/accelerating hard mid-curve. 1.0 rejected 8587 frames of ordinary city
# braking-into-corners on the reference drive (route 00000006); 2.0 keeps those and
# still cuts genuine hard stops (541 frames).
MAX_LONG_ACCEL = 2.0        # m/s^2 |aEgo|

# Driver-contamination guards. Ford flips steeringPressed at STEER_DRIVER_ALLOWANCE (1.0 Nm)
# sustained — a light grip below that threshold still steers the car, and the PSCM under-delivers
# for seconds after any touch (post-override attenuation observed on the Mach-E). So:
TORQUE_GUARD_NM = 0.5       # treat half the pressed threshold as hands-on for calibration purposes
PRESS_HOLDBACK_S = 1.0      # samples are staged this long; any grip during staging cancels them
PRESS_COOLDOWN_S = 3.0      # after any grip ends, delivery is suspect this long — no samples

# --- Disturbance / road-quality rejection ------------------------------------------------
# All four thresholds below were tuned on the 2.7 h Mach-E reference drive (route
# 00000006--319e078ab5): tight enough to catch real disturbances, loose enough that the
# pinion-derived measurement's normal noise floor doesn't starve the estimator.
# A bump, pothole, crosswind gust or passing truck flicks the car without the command
# moving: measured curvature jumps while the command is steady. Not gain information.
SPIKE_MEAS_RATE = 0.02      # 1/m/s measured-curvature rate with a quiet command = disturbance
DISTURBANCE_BLANK_S = 1.0   # no evidence while a disturbance settles
DISTURBANCE_POISON_S = 0.3  # peak-buffer frames this far BACK from detection are suspect too
# A bump shakes the wheels before it shows in the pinion: a jump in the wheel-speed spread
# (max-min of the four wheels) corroborates and triggers/extends the blanking.
WS_SPREAD_JUMP = 0.6        # m/s frame-to-frame change of the spread
# Washboard / broken pavement: sustained high-frequency content in measured curvature.
# The residual is high-passed (curve content lives below ~1 Hz and stays in the low-pass),
# so ordinary cornering — sweepers, S-curves, apexes — can never trip this.
ROUGH_LP_TAU_S = 0.3        # low-pass defining the "curve content" of the measurement
ROUGH_RMS_TAU_S = 2.0       # window of the residual RMS
ROUGH_RMS_MAX = 0.0015      # 1/m residual RMS above this = rough stretch, no evidence

# --- Peak (apex) evidence ----------------------------------------------------------------
# The manual method compares the tops/bottoms of the requested vs actual curvature traces.
# A command apex is a local extremum that dominates a +-PEAK_HALF_WINDOW_S neighborhood
# with real prominence; its measured twin is the same-sign extremum within the actuator
# lag horizon. The amplitude ratio at the pair is direct gain evidence — available on
# winding roads where the steady-state gate never gets its STEADY_TIME_S.
PEAK_BUF_S = 2.5            # ring buffer length
PEAK_HALF_WINDOW_S = 1.0    # apex must dominate this neighborhood on both sides
PEAK_LAG_MAX_S = 0.7        # measured extremum searched this far after the command apex
PEAK_MIN_KAPPA = 0.0012     # 1/m minimum apex amplitude
PEAK_PROMINENCE = 0.0004    # 1/m above the window minimum — rejects ripple
PEAK_REFRACTORY_S = 1.0     # one apex per this interval
# Apex evidence carries real transient content (the plant attenuates fast transients a bit
# more than steady curves), so it supplements the steady evidence rather than dominating:
# on the reference drive w=3.0 pulled the low anchor ~0.02 above the steady-only fit,
# w=1.5 keeps the combined fit within ±0.015 of it while still covering winding roads
# where the steady gate never fires.
PEAK_WEIGHT_S = 1.5         # one clean apex counts like this many seconds of steady evidence
PEAK_MEDIAN_N = 3           # apexes commit as the median of this many — kills single outliers

# --- Estimator robustness ----------------------------------------------------------------
# Forgetting: slow enough that a normal drive's evidence equilibrium (commit rate x TAU)
# clears the lock threshold — the reference drive commits ~0.012-0.023 s/s per anchor,
# giving equilibria of ~90-165 s — while old drives still fade within a couple of hours
# of active collection. (An outlier gate against the running fit was tried here and
# removed: it is path-dependent — early evidence anchors the fit, then contradicting
# evidence gets rejected — and it visibly distorted the reference-drive fit. Robustness
# comes from the frame quality layer, the ratio sanity bounds, staging cancellation and
# the apex median-of-3 instead.)
TAU_EVIDENCE_S = 7200.0     # evidence forgetting time constant (seconds of active collection)
# Banked/crowned roads bias one turn direction. If the left- and right-turn estimates of an
# anchor diverge beyond LR_TOL the divergence excess inflates that anchor's effective stderr,
# blocking nudges and lock until balanced evidence arrives.
LR_TOL = 0.06
LR_MIN_WEIGHT = 5.0         # per-direction weight before the divergence check means anything

# --- Live nudging (the closed loop) ------------------------------------------------------
NUDGE_PERIOD_S = 20.0       # at most one nudge per this much active collection
NUDGE_MIN_WEIGHT = 10.0     # anchor evidence before it may move its factor
NUDGE_MAX_STDERR = 0.06     # effective stderr must be at least this good
# Damped-proportional step: move a fraction of the remaining error toward the fit, quantized
# to the 0.01 menu step and capped. Big steps when far (fast convergence), single 0.01 steps
# near the target, and — because the target is invariant to the applied factor — it glides in
# without overshooting instead of crawling a fixed 0.02 or jumping past and reversing.
FACTOR_STEP = 0.01          # menu granularity (factors are 2-decimal)
NUDGE_GAIN = 0.7            # fraction of the remaining error per nudge (< 1 damps overshoot)
NUDGE_MAX_STEP = 0.05       # max factor change per nudge
_NUDGE_MAX_UNITS = round(NUDGE_MAX_STEP / FACTOR_STEP)  # = 5

# --- Adjust-then-verify ------------------------------------------------------------------
# 2026-07-22 design decision: the per-drive movement caps (0.10 high / 0.04 low) are GONE —
# a genuinely miscalibrated car must be allowed to walk all the way to its fit in one
# drive. What replaces them is an explicit check instead of a leash: every step is judged
# against FRESH post-step evidence before its anchor may step again. The response ratio
# (measured/commanded curvature) scales with the applied gain, so a step from g_old to
# g_new predicts the direction the ratio must move; a fast-forgetting per-anchor ratio
# tracker measures whether it actually did. Confirmed -> keep walking. Failed -> the data
# contradicted the model: hold that anchor until twice the normal evidence has spoken.
# This is the "poll a couple turns, adjust, poll some more" loop, made enforceable.
VERIFY_MIN_WEIGHT = 6.0        # fresh post-step evidence (s) before the step is judged
VERIFY_FAIL_HOLD_WEIGHT = 12.0 # a failed check demands this much evidence before stepping again
VERIFY_OK_BAND = 0.04          # |1 - ratio| inside this after a step = success outright
TAU_RECENT_S = 90.0            # recent-response forgetting (seconds of active collection)

# --- Lock --------------------------------------------------------------------------------
# LOCK_MIN_WEIGHT sits below the reference drive's decay equilibrium (~90 s on the weaker
# anchor) so a normally-driven car can actually reach it; the 5-minute stability window is
# the real proof that there is nothing left to adjust.
LOCK_MIN_WEIGHT = 60.0      # per-anchor evidence before locking is possible
LOCK_DEADBAND = 0.03        # applied factors this close to the estimate count as "nothing to adjust"
LOCK_STABLE_S = 300.0       # this much active collection with nothing to adjust => locked

FACTOR_MIN, FACTOR_MAX = 0.5, 1.5  # same clamp as the settings +/- buttons

# Rejection cause codes (order fixed: serialized as a dict of these keys).
REJ_CAUSES = ("flick", "rough", "limit", "accel", "grip")


def speed_alpha(v_ego: float) -> float:
  """Blend position of v between the two anchors: 0 = pure low anchor, 1 = pure high."""
  if v_ego <= V_LOW:
    return 0.0
  if v_ego >= V_HIGH:
    return 1.0
  return (v_ego - V_LOW) / (V_HIGH - V_LOW)


def lat_accel_margin(kappa: float, v_ego: float) -> float:
  """Evidence weight vs lateral accel: 1.0 well below MAX_LAT_ACCEL, fading linearly to 0
  over the last LAT_ACCEL_SOFT_BAND — near the limit cmd != meas is physics, not gain."""
  return min(1.0, max(0.0, (MAX_LAT_ACCEL - abs(kappa) * v_ego * v_ego) / LAT_ACCEL_SOFT_BAND))


def fit_trustworthy(weight: float, stderr_eff: float, min_weight: float) -> bool:
  """An anchor's fit is trustworthy enough to act on: enough evidence and a tight enough
  effective stderr. Lock, nudge and the live status share this — they differ only in the
  weight bar and in what they do with |target - applied| afterwards. A new eligibility
  condition (e.g. a sensor-health gate) is added here once."""
  return weight >= min_weight and stderr_eff <= NUDGE_MAX_STDERR


def nudge_units(err: float) -> int:
  """Damped, menu-quantized nudge size in whole FACTOR_STEP units, capped. Returns 0 inside
  the (implicit) deadband — |NUDGE_GAIN * err| below half a step. Shared by the stepper and
  the live status so 'would nudge' is defined in one place."""
  n = round(NUDGE_GAIN * err / FACTOR_STEP)
  return max(-_NUDGE_MAX_UNITS, min(_NUDGE_MAX_UNITS, n))


class AngleFactorEstimator:
  """Weighted least-squares fit of the two gain anchors from curve samples.

  Model: the ideal gain at sample i is y_i = g_i / r_i where r_i is the measured/commanded
  curvature ratio and g_i the gain that was IN FORCE when the sample was taken (recorded
  per sample — the applied factors move while the nudger works, and old samples stay valid).
  F_ideal(v) = (1-a)*A + a*B with a = speed_alpha(v), A = ideal low anchor
  (1.30 * LOW_FACTOR), B = ideal high anchor (gain * HIGH_FACTOR). Linear in (A, B) ->
  closed-form normal equations, accumulated incrementally so the onboard hook carries
  O(1) state — and that state serializes to a dict for cross-drive persistence.
  """

  def __init__(self, platform_gain_high: float):
    self.platform_gain_high = float(platform_gain_high)
    # Normal-equation accumulators for min sum w*((1-a)A + aB - y)^2
    self.s_ll = 0.0  # sum w*(1-a)^2
    self.s_lh = 0.0  # sum w*(1-a)*a
    self.s_hh = 0.0  # sum w*a^2
    self.s_ly = 0.0  # sum w*(1-a)*y
    self.s_hy = 0.0  # sum w*a*y
    self.s_w = 0.0   # sum w
    self.s_wy2 = 0.0  # sum w*y^2 (for residual/stderr)
    self.n = 0
    # Left/right split per anchor half for bank-bias detection: {(half, dir): [w, wy]}
    # half: 0 = alpha < 0.5 (low anchor side), 1 = high side; dir: 0 = left, 1 = right.
    self.lr = {(h, d): [0.0, 0.0] for h in (0, 1) for d in (0, 1)}
    # Fast-forgetting per-anchor response ratio r = meas/cmd (TAU_RECENT_S): "what is the
    # car doing RIGHT NOW under the current factors" — the adjust-then-verify check and
    # the live dashboard read this, the long-memory fit above never does.
    self.recent = {0: [0.0, 0.0], 1: [0.0, 0.0]}  # half -> [w, sum w*r]

  def add_sample(self, v_ego: float, kappa_cmd: float, kappa_meas: float,
                 applied_gain: float, weight: float = 1.0) -> bool:
    """Add one curve observation taken while applied_gain was in force.

    kappa_cmd is the curvature the strategy converted to path_angle (post any clips);
    kappa_meas is the pinion-derived measured curvature. Both in OP sign convention —
    only same-sign, above-threshold pairs are accepted. Returns True if accepted.
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
    y = float(applied_gain) / r
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
    half = 0 if a < 0.5 else 1
    acc = self.lr[(half, 0 if kappa_cmd > 0 else 1)]
    acc[0] += w
    acc[1] += w * y
    rec = self.recent[half]
    rec[0] += w
    rec[1] += w * r
    return True

  def decay(self, seconds: float):
    """Exponential evidence forgetting: old drives fade so adaptation stays possible,
    while the ~TAU saturation keeps lock thresholds reachable and stable. The recent
    tracker forgets much faster (TAU_RECENT_S) — it must answer for the car as it is
    now, not as it averaged over hours."""
    f = math.exp(-float(seconds) / TAU_EVIDENCE_S)
    self.scale(f)
    fr = math.exp(-float(seconds) / TAU_RECENT_S)
    for rec in self.recent.values():
      rec[0] *= fr
      rec[1] *= fr

  def scale(self, f: float):
    self.s_ll *= f
    self.s_lh *= f
    self.s_hh *= f
    self.s_ly *= f
    self.s_hy *= f
    self.s_w *= f
    self.s_wy2 *= f
    for acc in self.lr.values():
      acc[0] *= f
      acc[1] *= f
    for rec in self.recent.values():
      rec[0] *= f
      rec[1] *= f

  def recent_response(self, half: int):
    """(weight, mean ratio) of the fast tracker for one anchor half; ratio is None until
    any evidence exists. Ratio 0.93 reads as 'turns 93% of requested'."""
    w, s = self.recent[half]
    if w <= 1e-6:
      return 0.0, None
    return w, s / w

  @property
  def weight_low(self) -> float:
    """Effective sample weight attributed to the low anchor."""
    return self.s_ll + self.s_lh

  @property
  def weight_high(self) -> float:
    return self.s_hh + self.s_lh

  def lr_divergence(self, half: int) -> float:
    """|mean_left - mean_right| of the implied ideal gain for one anchor half.
    0.0 until both directions carry LR_MIN_WEIGHT — one-sided evidence is not yet
    proof of bias, it just hasn't been contradicted."""
    wl, yl = self.lr[(half, 0)]
    wr, yr = self.lr[(half, 1)]
    if wl < LR_MIN_WEIGHT or wr < LR_MIN_WEIGHT:
      return 0.0
    return abs(yl / wl - yr / wr)

  def solve(self):
    """Solve for the ideal anchors. Returns (low_factor, high_factor, stats) or None.

    low_factor / high_factor are the values to store in FordLowSpeedFactor_ang /
    FordHighSpeedFactor_ang (already divided by the fixed anchor bases and clamped to
    the same range the +/- buttons allow). stats stderr_low/high are the plain fit
    errors; stderr_eff_low/high add the left/right divergence excess — the values the
    nudge/lock eligibility checks use.
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
    div_low = self.lr_divergence(0)
    div_high = self.lr_divergence(1)

    low_factor = min(FACTOR_MAX, max(FACTOR_MIN, anchor_low / LOW_ANCHOR_BASE))
    high_factor = min(FACTOR_MAX, max(FACTOR_MIN, anchor_high / self.platform_gain_high))
    stats = {
      "n": self.n,
      "weight_low": self.weight_low,
      "weight_high": self.weight_high,
      "stderr_low": stderr_low,
      "stderr_high": stderr_high,
      "stderr_eff_low": stderr_low + max(0.0, div_low - LR_TOL),
      "stderr_eff_high": stderr_high + max(0.0, div_high - LR_TOL),
      "lr_div_low": div_low,
      "lr_div_high": div_high,
      "anchor_low": anchor_low,
      "anchor_high": anchor_high,
    }
    return low_factor, high_factor, stats

  def to_dict(self) -> dict:
    return {
      "s_ll": self.s_ll, "s_lh": self.s_lh, "s_hh": self.s_hh,
      "s_ly": self.s_ly, "s_hy": self.s_hy, "s_w": self.s_w, "s_wy2": self.s_wy2,
      "n": self.n,
      "lr": [self.lr[(h, d)][:] for h in (0, 1) for d in (0, 1)],
      "recent": [self.recent[0][:], self.recent[1][:]],
    }

  def from_dict(self, d: dict):
    self.s_ll = float(d["s_ll"])
    self.s_lh = float(d["s_lh"])
    self.s_hh = float(d["s_hh"])
    self.s_ly = float(d["s_ly"])
    self.s_hy = float(d["s_hy"])
    self.s_w = float(d["s_w"])
    self.s_wy2 = float(d["s_wy2"])
    self.n = int(d["n"])
    flat = d.get("lr")
    if isinstance(flat, list) and len(flat) == 4:
      for i, (h, dd) in enumerate(((0, 0), (0, 1), (1, 0), (1, 1))):
        self.lr[(h, dd)] = [float(flat[i][0]), float(flat[i][1])]
    rec = d.get("recent")  # absent in pre-verify saves: tracker rebuilds in ~a minute
    if isinstance(rec, list) and len(rec) == 2:
      for h in (0, 1):
        self.recent[h] = [float(rec[h][0]), float(rec[h][1])]


class QualityMonitor:
  """Frame-level evidence quality: excises anomalous moments WITHIN curves without ever
  rejecting the cornering itself. Detectors are curve-safe by construction — thresholds
  sit above what clean cornering produces, blanking windows are short, and the rough-road
  residual is high-passed so slow curve content cannot trip it."""

  def __init__(self, dt: float = 0.05):
    self.dt = dt
    self.blank_s = 0.0
    self.flick_fired = False       # True only on the frame a disturbance was detected
    self._meas_last = None
    self._cmd_last = None
    self._ws_spread_last = None
    self._lp = None                # low-passed measurement ("curve content")
    self._rms2 = 0.0               # EMA of squared high-passed residual
    self.counters = {c: 0 for c in REJ_CAUSES}

  def update(self, kappa_cmd: float, kappa_meas: float,
             a_ego: float = 0.0, ws_spread: float | None = None) -> bool:
    """Advance one 20 Hz frame; returns True when the frame may carry evidence."""
    dt = self.dt
    self.flick_fired = False

    # Transient flick: measurement jumps while the command is quiet.
    if self._meas_last is not None and self._cmd_last is not None:
      meas_rate = abs(kappa_meas - self._meas_last) / dt
      cmd_rate = abs(kappa_cmd - self._cmd_last) / dt
      if meas_rate > SPIKE_MEAS_RATE and cmd_rate <= MAX_KAPPA_RATE:
        self.blank_s = DISTURBANCE_BLANK_S
        self.flick_fired = True
    # Corroborating bump signal: the wheels get shaken before the pinion shows it.
    if ws_spread is not None and self._ws_spread_last is not None:
      if abs(ws_spread - self._ws_spread_last) > WS_SPREAD_JUMP:
        self.blank_s = max(self.blank_s, DISTURBANCE_BLANK_S)
        self.flick_fired = True
    self._meas_last = kappa_meas
    self._cmd_last = kappa_cmd
    self._ws_spread_last = ws_spread

    # Rough road: RMS of the high-passed measurement residual. The low-pass tracks curve
    # content; what remains is surface noise.
    if self._lp is None:
      self._lp = kappa_meas
    alpha_lp = dt / (ROUGH_LP_TAU_S + dt)
    self._lp += alpha_lp * (kappa_meas - self._lp)
    resid = kappa_meas - self._lp
    alpha_rms = dt / (ROUGH_RMS_TAU_S + dt)
    self._rms2 += alpha_rms * (resid * resid - self._rms2)
    rough = math.sqrt(self._rms2) > ROUGH_RMS_MAX

    ok = True
    if self.blank_s > 0.0:
      self.blank_s = max(0.0, self.blank_s - dt)
      self.counters["flick"] += 1
      ok = False
    elif rough:
      self.counters["rough"] += 1
      ok = False
    if abs(a_ego) > MAX_LONG_ACCEL:
      if ok:
        self.counters["accel"] += 1
      ok = False
    return ok

  def idle(self):
    """Lateral inactive: rate baselines are meaningless across the gap."""
    self._meas_last = None
    self._cmd_last = None
    self._ws_spread_last = None


class _PeakFrame(NamedTuple):
  """One ring-buffer slot of the apex matcher — named so nothing indexes it by magic number."""
  kappa_cmd: float
  kappa_meas: float
  v_ego: float
  gain: float
  clean: bool


class PeakMatcher:
  """Apex evidence — the video method. A ring buffer of recent frames; when a command
  apex (dominant, prominent local extremum with an all-clean neighborhood) scrolls to
  the decision point, its measured twin is the same-sign extremum within the actuator
  lag horizon, and the amplitude ratio is committed as gain evidence (median-of-N so a
  single weird apex dies before reaching the estimator)."""

  def __init__(self, dt: float = 0.05):
    self.dt = dt
    self.n_buf = int(round(PEAK_BUF_S / dt))            # 50
    self.half_w = int(round(PEAK_HALF_WINDOW_S / dt))   # 20
    self.lag_max = int(round(PEAK_LAG_MAX_S / dt))      # 14
    self.c = self.n_buf - 1 - max(self.half_w, self.lag_max)  # decision index
    self.buf: list[_PeakFrame] = []
    self._refractory = 0
    self._pending: dict[int, list] = {0: [], 1: []}     # anchor half -> [(r, sample), ...]
    self.apexes_seen = 0
    self.apexes_committed = 0

  def clear(self):
    self.buf.clear()
    self._pending = {0: [], 1: []}
    self._refractory = 0

  def poison_recent(self, seconds: float):
    """A disturbance was just detected: frames shortly BEFORE detection are suspect
    (the bump was already moving the car). Mark them not-ok retroactively."""
    n = int(round(seconds / self.dt))
    for i in range(max(0, len(self.buf) - n), len(self.buf)):
      self.buf[i] = self.buf[i]._replace(clean=False)

  def push(self, kappa_cmd: float, kappa_meas: float, v_ego: float,
           applied_gain: float, ok: bool) -> list[tuple]:
    """Advance one frame. Returns samples to commit: (v, kappa_cmd, kappa_meas,
    applied_gain) tuples (already median-filtered)."""
    self.buf.append(_PeakFrame(kappa_cmd, kappa_meas, v_ego, applied_gain, ok))
    if len(self.buf) > self.n_buf:
      self.buf.pop(0)
    if self._refractory > 0:
      self._refractory -= 1
    if len(self.buf) < self.n_buf or self._refractory > 0:
      return []

    c = self.c
    fc = self.buf[c]
    k_c, v_c, g_c = fc.kappa_cmd, fc.v_ego, fc.gain
    if abs(k_c) < PEAK_MIN_KAPPA or v_c < MIN_SPEED:
      return []
    if abs(k_c) * v_c * v_c > MAX_LAT_ACCEL:
      return []
    window = self.buf[c - self.half_w:c + self.half_w + 1]
    # Every frame around the apex must be clean — the retroactive-cancel analog for peaks.
    if not all(f.clean for f in window):
      return []
    mags = [abs(f.kappa_cmd) for f in window]
    k_mag = abs(k_c)
    # Dominant: >= everything before, strictly > everything after (fires once per plateau).
    before = mags[:self.half_w + 1]
    after = mags[self.half_w + 1:]
    if any(m > k_mag for m in before) or any(m >= k_mag for m in after):
      return []
    if k_mag - min(mags) < PEAK_PROMINENCE:
      return []

    self.apexes_seen += 1
    self._refractory = int(round(PEAK_REFRACTORY_S / self.dt))

    # Measured twin: same-sign extremum within the lag horizon (clean frames only).
    sign = 1.0 if k_c > 0 else -1.0
    m_pk = 0.0
    for f in self.buf[c:c + self.lag_max + 1]:
      if not f.clean:
        return []  # disturbance inside the match window: the pair is unusable
      if f.kappa_meas * sign > m_pk:
        m_pk = f.kappa_meas * sign
    if m_pk <= 0.0:
      return []
    r = (m_pk * sign) / k_c
    if not (MIN_RATIO <= r <= MAX_RATIO):
      return []

    half = 0 if speed_alpha(v_c) < 0.5 else 1
    self._pending[half].append((r, (v_c, k_c, m_pk * sign, g_c)))
    if len(self._pending[half]) < PEAK_MEDIAN_N:
      return []
    # Median by ratio — the middle apex is committed, the outliers die here.
    self._pending[half].sort(key=lambda t: t[0])
    _, sample = self._pending[half][PEAK_MEDIAN_N // 2]
    self._pending[half] = []
    self.apexes_committed += 1
    return [sample]


class SteadyStateGate:
  """Admits samples only after the command has been steady for STEADY_TIME_S.

  Both consumers drive this at the 20 Hz lateral rate with the same flags the
  strategy itself computes, so offline and onboard gating are identical.
  """

  def __init__(self, dt: float = 0.05):
    self.dt = dt
    self.steady_s = 0.0
    self.kappa_last = None
    self.kappa_window_start = None  # command value when the current steady window opened
    self.grip_cooldown_s = 0.0
    self.grip_this_frame = False    # grip seen on the frame last passed to update()
    self.frame_clear = False        # per-frame admission (no grip/cooldown/limiter flags),
                                    # computed ONCE here and shared with the apex path

  def reset(self):
    """Inactive frame (disengaged / human turn / stall blip): steadiness restarts and the
    last-command baseline is dropped; the grip cooldown keeps decaying in real time."""
    self.grip_cooldown_s = max(0.0, self.grip_cooldown_s - self.dt)
    self.steady_s = 0.0
    self.kappa_last = None
    self.kappa_window_start = None
    self.grip_this_frame = False
    self.frame_clear = False

  def update(self, lat_active: bool, kappa_cmd: float, steering_pressed: bool,
             angle_rate_limited: bool, deviation_limited: bool,
             saturated: bool = False, driver_torque: float = 0.0) -> bool:
    # Human-turn and stall-blip frames never reach this call — the strategy early-returns
    # and idles the pipeline instead — so those flags are not parameters here.
    # Any grip — including light torque below the steeringPressed threshold — starts a
    # cooldown: the driver was steering, and the PSCM's delivery stays suspect for a while
    # after release (post-touch attenuation).
    grip = steering_pressed or abs(driver_torque) > TORQUE_GUARD_NM
    self.grip_this_frame = grip
    if grip:
      self.grip_cooldown_s = PRESS_COOLDOWN_S
    else:
      self.grip_cooldown_s = max(0.0, self.grip_cooldown_s - self.dt)

    # The per-frame admission predicate, computed exactly once: the pipeline's apex path
    # reads it back instead of maintaining a hand-synced copy.
    self.frame_clear = (not grip and self.grip_cooldown_s <= 0.0
                        and not angle_rate_limited and not deviation_limited
                        and not saturated)
    ok = lat_active and self.frame_clear and abs(kappa_cmd) >= MIN_KAPPA
    if ok and self.kappa_last is not None:
      # Relative rate bound with an absolute floor: lag alignment absorbs the transport
      # delay, so kappa is allowed to actually MOVE (winding roads) — the bound only has
      # to cap the residual delay-estimate error's effect (see the constants block).
      ok = (abs(kappa_cmd - self.kappa_last) / self.dt
            <= max(MAX_KAPPA_RATE, REL_KAPPA_RATE * abs(kappa_cmd)))
    # Actuation-lag protection: per-frame rate alone admits slow ramps whose same-frame
    # ratio is lag-biased; the window-total drift bound caps that (see STEADY_DRIFT_FRAC).
    if ok and self.kappa_window_start is not None:
      ok = abs(kappa_cmd - self.kappa_window_start) <= STEADY_DRIFT_FRAC * abs(kappa_cmd)
    self.kappa_last = kappa_cmd if lat_active else None
    if ok:
      if self.kappa_window_start is None:
        self.kappa_window_start = kappa_cmd
      self.steady_s += self.dt
    else:
      self.steady_s = 0.0
      self.kappa_window_start = None
    return self.steady_s >= STEADY_TIME_S


@dataclass
class _StagedSample:
  """One holdback-staged evidence sample. Named fields (not a list-of-lists) so a
  reordered field can't silently shift every value — they are all floats and nothing
  would raise. age mutates as the sample waits, so this is mutable, not a NamedTuple."""
  age: float
  v_ego: float
  kappa_cmd: float
  kappa_meas: float
  gain: float
  weight: float


class AutoCalPipeline:
  """Quality layer + steady gate + apex matcher + estimator + nudger + lock, driven with
  one call per 20 Hz lateral frame. Samples sit in a staging queue for PRESS_HOLDBACK_S
  before they reach the estimator; a grip or disturbance while they wait cancels them.
  Used identically by the onboard hook and the offline analyzer.

  The pipeline itself never writes params: recommend() returns proposed factor values and
  the glue (or the analyzer's virtual car) applies them and passes the applied values back
  in on subsequent update() calls — that closes the loop.
  """

  def __init__(self, platform_gain_high: float, dt: float = 0.05):
    self.platform_gain_high = float(platform_gain_high)
    self.est = AngleFactorEstimator(platform_gain_high)
    self.gate = SteadyStateGate(dt=dt)
    self.quality = QualityMonitor(dt=dt)
    self.peaks = PeakMatcher(dt=dt)
    self.dt = dt
    self._staged: list[_StagedSample] = []
    self._meas_last = None
    self._err_lp = None            # smoothed |tracking error| for the quietness gate
    self._decay_accum = 0.0
    # Lag alignment: ring of recent (kappa_cmd, applied_gain) so this frame's measurement
    # can be ratioed against the command (and the gain in force) when it was ISSUED.
    self._hist: list[tuple[float, float]] = []
    self._hist_max = int(round(LAG_MAX_S / dt)) + 2
    # Nudge / lock bookkeeping (persisted).
    self.since_nudge_s = NUDGE_PERIOD_S  # first nudge allowed as soon as evidence permits
    self.stable_s = 0.0
    self.nudges = 0
    self.locked = False
    self.lock_enabled = True   # FordAngleAutoCalLock (set by the controller, not persisted):
                               # False = never freeze, keep adapting for the life of the toggle
    # Adjust-then-verify state (persisted): each step opens a window that must be judged
    # against fresh evidence before its anchor may step again. half 0 = low, 1 = high.
    self.verify = {0: None, 1: None}     # {"frm": factor, "to": factor, "pre_r": ratio|None}
    self.verify_result = {0: "", 1: ""}  # last judgment: "confirmed" / "failed" / ""
    self.verify_hold = {0: 0.0, 1: 0.0}  # extra fresh evidence demanded after a failure

  # -- gain model -------------------------------------------------------------------------
  def applied_gain(self, v_ego: float, low_factor: float, high_factor: float) -> float:
    a = speed_alpha(v_ego)
    return (1.0 - a) * (LOW_ANCHOR_BASE * low_factor) + a * (self.platform_gain_high * high_factor)

  def idle(self):
    """Call on frames where lateral is inactive (disengaged / human turn / stall blip)."""
    self.gate.reset()
    self.quality.idle()
    self.peaks.clear()
    self._staged.clear()
    self._meas_last = None
    self._err_lp = None
    self._hist.clear()  # commands across a discontinuity must never be an alignment target

  def update(self, frame: Frame) -> list:
    """Advance one frame. frame.low_factor/high_factor are the values currently steering
    the car — each committed sample records the gain that produced it. Returns the samples
    committed to the estimator this frame as (v, kappa_cmd, kappa_meas) tuples — the
    offline analyzer plots them; the onboard hook ignores the return value."""
    if self.locked:
      return []
    v_ego, kappa_cmd, kappa_meas = frame.v_ego, frame.kappa_cmd, frame.kappa_meas
    gain_now = self.applied_gain(v_ego, frame.low_factor, frame.high_factor)

    # Lag alignment: this frame's MEASUREMENT answers the command from lateral_delay ago.
    # All steadiness gating and every steady-state ratio below use that reference pair
    # (cmd + the gain in force when it was issued); the apex path keeps its own explicit
    # lag matching and stays on the current command.
    self._hist.append((kappa_cmd, gain_now))
    if len(self._hist) > self._hist_max:
      self._hist.pop(0)
    lag_f = int(round(min(max(frame.lateral_delay, LAG_MIN_S), LAG_MAX_S) / self.dt))
    aligned = len(self._hist) > lag_f
    kappa_ref, gain_ref = self._hist[-1 - lag_f] if aligned else (0.0, gain_now)

    if aligned:
      # The gate computes grip + the per-frame admission predicate once; everything
      # below reads them back instead of keeping a hand-synced copy.
      eligible = self.gate.update(True, kappa_ref, frame.steering_pressed,
                                  frame.angle_rate_limited, frame.deviation_limited,
                                  saturated=frame.saturated, driver_torque=frame.driver_torque)
    else:
      # Not enough history yet (first ~lateral_delay after engaging): no steady evidence,
      # and the gate restarts so its baselines never straddle the warmup.
      self.gate.reset()
      eligible = False
    if self.gate.grip_this_frame:
      self._staged.clear()
      self.quality.counters["grip"] += 1

    q_ok = self.quality.update(kappa_cmd, kappa_meas, a_ego=frame.a_ego, ws_spread=frame.ws_spread)
    if self.quality.flick_fired:
      # Retroactive: the bump was already moving the car before detection tripped.
      self._staged.clear()
      self.peaks.poison_recent(DISTURBANCE_POISON_S)

    eligible = eligible and q_ok

    # The CAR must be settled too, not just the command: during closed-loop compensation
    # swings (understeer -> harder request -> convergence tail) the command can sit steady
    # while the measurement is still moving toward it — those ratios are transient, not
    # gain. The measurement legitimately moves as fast as the (lag-aligned) command does,
    # and it is noisier — so the bound is 3x the command's own admission bound.
    if eligible and self._meas_last is not None:
      eligible = (abs(kappa_meas - self._meas_last) / self.dt
                  <= 3.0 * max(MAX_KAPPA_RATE, REL_KAPPA_RATE * abs(kappa_ref)))
    self._meas_last = kappa_meas

    # Loop-quietness: only calm-tracking frames are gain evidence (see QUIET_* constants).
    # A constant plant deficit keeps a FLAT error trend and passes; an excursion or the
    # correction that follows it moves the trend and is refused — the loop's hunting can
    # never masquerade as gain information, however long that makes a step take.
    if aligned:
      err_now = abs(kappa_meas - kappa_ref)
      if self._err_lp is None:
        self._err_lp = err_now
        eligible = False  # no trend established yet
      else:
        prev = self._err_lp
        self._err_lp += (self.dt / (QUIET_TAU_S + self.dt)) * (err_now - self._err_lp)
        if abs(self._err_lp - prev) / self.dt > QUIET_ERR_RATE:
          eligible = False

    # Evidence near the physical limit fades to nothing: there, cmd != meas is physics.
    margin_w = lat_accel_margin(kappa_cmd, v_ego)
    if eligible and margin_w <= 0.0:
      self.quality.counters["limit"] += 1
      eligible = False

    # Age the staging queue; entries that survived the holdback graduate to the estimator.
    committed = []
    still_staged = []
    for s in self._staged:
      s.age += self.dt
      if s.age >= PRESS_HOLDBACK_S:
        if self.est.add_sample(s.v_ego, s.kappa_cmd, s.kappa_meas, s.gain, weight=s.weight):
          committed.append((s.v_ego, s.kappa_cmd, s.kappa_meas))
      else:
        still_staged.append(s)
    self._staged = still_staged

    if eligible:
      self._staged.append(_StagedSample(0.0, v_ego, kappa_ref, kappa_meas, gain_ref, self.dt * margin_w))

    # Apex evidence: gated by everything EXCEPT the steadiness timer (an apex is by
    # definition not steady). Quality, grip, limit flags all poison the window —
    # the grip/flag part is the gate's own frame_clear, computed once above.
    frame_ok = q_ok and self.gate.frame_clear and margin_w > 0.0
    for (pv, pk, pm, pg) in self.peaks.push(kappa_cmd, kappa_meas, v_ego, gain_now, frame_ok):
      p_margin = lat_accel_margin(pk, pv)
      if self.est.add_sample(pv, pk, pm, pg, weight=PEAK_WEIGHT_S * p_margin):
        committed.append((pv, pk, pm))

    # Housekeeping clocks: forgetting, nudge cadence, lock stability.
    self._decay_accum += self.dt
    if self._decay_accum >= 1.0:
      self.est.decay(self._decay_accum)
      self._decay_accum = 0.0
    self.since_nudge_s += self.dt
    self._judge_verifies()

    sol = self.est.solve()
    if sol is not None:
      low_t, high_t, st = sol
      ready = (fit_trustworthy(st["weight_low"], st["stderr_eff_low"], LOCK_MIN_WEIGHT)
               and fit_trustworthy(st["weight_high"], st["stderr_eff_high"], LOCK_MIN_WEIGHT)
               and abs(low_t - frame.low_factor) <= LOCK_DEADBAND
               and abs(high_t - frame.high_factor) <= LOCK_DEADBAND)
      if ready:
        self.stable_s += self.dt
        if self.stable_s >= LOCK_STABLE_S and self.lock_enabled:
          self.locked = True
      else:
        self.stable_s = 0.0

    return committed

  def _judge_verifies(self):
    """Judge any pending step once enough FRESH post-step evidence exists (the fast
    tracker was reset to zero when the step was taken, so it holds post-step data only).

    The criterion is CLOSENESS, not direction: the response ratio tracks the applied
    gain mechanically (drop the factor 2%, delivery drops ~2%, right or wrong), so
    'did it move the predicted way' would confirm every step the actuator executed.
    A step is only right if it brought the car closer to doing exactly what's asked:
    |1 - ratio| strictly smaller than before the step, or inside VERIFY_OK_BAND of
    1.0 outright — the step landed where calibration aims."""
    for half in (0, 1):
      pend = self.verify[half]
      if pend is None:
        continue
      w, r = self.est.recent_response(half)
      if w < VERIFY_MIN_WEIGHT or r is None:
        continue  # keep polling — the window stays open until the data has spoken
      ok = abs(1.0 - r) <= VERIFY_OK_BAND
      if not ok and pend["pre_r"] is not None:
        ok = abs(1.0 - r) < abs(1.0 - pend["pre_r"])
      elif not ok:
        ok = True  # no pre-step baseline to compare against (shouldn't happen in practice)
      self.verify[half] = None
      if ok:
        self.verify_result[half] = "confirmed"
        self.verify_hold[half] = 0.0
      else:
        self.verify_result[half] = "failed"
        self.verify_hold[half] = VERIFY_FAIL_HOLD_WEIGHT

  def recommend(self, low_factor: float, high_factor: float):
    """The closed-loop step: propose nudged factor values, or None.

    Call once per frame with the currently applied factors; at most one nudge per
    NUDGE_PERIOD_S of active collection, one bounded step at a time — but no cumulative
    cap: total movement is unbounded (within FACTOR_MIN/MAX) as long as every step keeps
    verifying against fresh evidence. The caller applies the returned values (params +
    in-memory) — they flow back in through update()'s low_factor/high_factor and the
    loop closes: an overshoot pulls the ratios past 1, the target backs off, the next
    nudge reverses.
    """
    if self.locked or self.since_nudge_s < NUDGE_PERIOD_S:
      return None
    sol = self.est.solve()
    if sol is None:
      return None
    low_t, high_t, st = sol

    def step(half, target, applied, weight, stderr_eff):
      if self.verify[half] is not None:
        return None  # the last step hasn't been judged against fresh evidence yet
      if not fit_trustworthy(weight, stderr_eff, NUDGE_MIN_WEIGHT):
        return None
      if self.verify_hold[half] > 0.0:
        w_rec, _ = self.est.recent_response(half)
        if w_rec < self.verify_hold[half]:
          return None  # a failed check demands extra evidence before moving again
        self.verify_hold[half] = 0.0
      units = nudge_units(target - applied)
      if units == 0:
        return None
      new = round(max(FACTOR_MIN, min(FACTOR_MAX, applied + units * FACTOR_STEP)), 2)
      return new if new != round(applied, 2) else None

    new_low = step(0, low_t, low_factor, st["weight_low"], st["stderr_eff_low"])
    new_high = step(1, high_t, high_factor, st["weight_high"], st["stderr_eff_high"])
    if new_low is None and new_high is None:
      return None
    # Open a verify window per moved anchor: capture the pre-step response as the
    # baseline, then zero the fast tracker so the judgment sees only post-step data.
    for half, applied, new in ((0, low_factor, new_low), (1, high_factor, new_high)):
      if new is not None:
        _, pre_r = self.est.recent_response(half)
        self.verify[half] = {"frm": round(float(applied), 4), "to": new, "pre_r": pre_r}
        self.verify_result[half] = ""
        self.est.recent[half] = [0.0, 0.0]
    out_low = new_low if new_low is not None else round(low_factor, 2)
    out_high = new_high if new_high is not None else round(high_factor, 2)
    self.since_nudge_s = 0.0
    self.stable_s = 0.0
    self.nudges += 1
    return out_low, out_high

  def user_edit(self):
    """The driver moved a factor by hand mid-collection: their judgment is information —
    adopt the value (it arrives via update()'s low/high_factor), soft-reset confidence so
    the estimator re-earns it, and restart lock progress. Evidence is NOT wiped: every
    sample recorded its own applied gain, so history stays valid."""
    self.est.scale(0.5)
    self.stable_s = 0.0
    self.since_nudge_s = 0.0

  # -- live UI state ----------------------------------------------------------------------
  def ui_state(self, low_factor: float, high_factor: float) -> dict:
    """Per-anchor state for live dashboards (the phone /lateral page). Everything here is
    ground truth from the running pipeline — never a param re-read — and rounded hard
    because it travels as a ~1 Hz telemetry string.

    Per anchor: f applied factor, w/need evidence progress, r recent response ratio
    ("turns 93% of requested" = 0.93), ph phase (collect / propose / verify / good),
    t proposed factor (propose), to+vw+vneed step being checked (verify), vr last
    judgment (confirmed / failed)."""
    sol = self.est.solve()
    st = sol[2] if sol is not None else None
    targets = (sol[0], sol[1]) if sol is not None else (None, None)
    out = {"nudges": self.nudges, "stable_s": round(self.stable_s)}
    for half, name, applied in ((0, "low", low_factor), (1, "high", high_factor)):
      weight = (st["weight_low"] if half == 0 else st["weight_high"]) if st is not None \
        else (self.est.weight_low if half == 0 else self.est.weight_high)
      stderr = (st["stderr_eff_low"] if half == 0 else st["stderr_eff_high"]) if st is not None else None
      w_rec, r_rec = self.est.recent_response(half)
      target = targets[half]
      d = {"f": round(float(applied), 2), "w": round(weight, 1), "need": NUDGE_MIN_WEIGHT}
      if r_rec is not None and w_rec >= 2.0:
        d["r"] = round(r_rec, 3)
      if self.verify[half] is not None:
        d["ph"] = "verify"
        d["to"] = self.verify[half]["to"]
        d["vw"] = round(w_rec, 1)
        d["vneed"] = VERIFY_MIN_WEIGHT
      elif (target is not None and stderr is not None
            and fit_trustworthy(weight, stderr, NUDGE_MIN_WEIGHT)):
        if nudge_units(target - applied) != 0:
          d["ph"] = "propose"
          d["t"] = round(target, 2)
        else:
          d["ph"] = "good"
      else:
        d["ph"] = "collect"
      if self.verify_result[half]:
        d["vr"] = self.verify_result[half]
      out[name] = d
    return out

  # -- persistence ------------------------------------------------------------------------
  def to_dict(self) -> dict:
    return {
      "est": self.est.to_dict(),
      "stable_s": round(self.stable_s, 2),
      "since_nudge_s": round(min(self.since_nudge_s, NUDGE_PERIOD_S), 2),
      "nudges": self.nudges,
      "locked": self.locked,
      "apexes": self.peaks.apexes_committed,
      "rej": dict(self.quality.counters),
      "verify": [self.verify[0], self.verify[1]],
      "verify_result": [self.verify_result[0], self.verify_result[1]],
      "verify_hold": [self.verify_hold[0], self.verify_hold[1]],
    }

  def from_dict(self, d: dict):
    self.est.from_dict(d["est"])
    self.stable_s = float(d.get("stable_s", 0.0))
    self.since_nudge_s = float(d.get("since_nudge_s", NUDGE_PERIOD_S))
    self.nudges = int(d.get("nudges", 0))
    self.locked = bool(d.get("locked", False))
    self.peaks.apexes_committed = int(d.get("apexes", 0))
    rej = d.get("rej", {})
    for c in REJ_CAUSES:
      self.quality.counters[c] = int(rej.get(c, 0))
    ver = d.get("verify")  # absent in pre-verify saves: everything simply starts clear
    if isinstance(ver, list) and len(ver) == 2:
      for h in (0, 1):
        self.verify[h] = dict(ver[h]) if isinstance(ver[h], dict) else None
    vr = d.get("verify_result")
    if isinstance(vr, list) and len(vr) == 2:
      for h in (0, 1):
        self.verify_result[h] = str(vr[h] or "")
    vh = d.get("verify_hold")
    if isinstance(vh, list) and len(vh) == 2:
      for h in (0, 1):
        self.verify_hold[h] = float(vh[h])
