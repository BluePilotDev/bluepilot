"""Tests for the continuous angle-mode factor auto-calibration (angle_autocal.py)."""
import json
import math
import random

import pytest

from opendbc.sunnypilot.car.ford.angle_autocal import (
  Frame,
  AngleFactorEstimator, AutoCalPipeline, PeakMatcher, QualityMonitor, SteadyStateGate,
  speed_alpha, V_LOW, V_HIGH, LOW_ANCHOR_BASE, STEADY_TIME_S, MIN_KAPPA, REL_KAPPA_RATE,
  PRESS_HOLDBACK_S, PRESS_COOLDOWN_S, MAX_LAT_ACCEL, MAX_LONG_ACCEL,
  PEAK_MIN_KAPPA, PEAK_PROMINENCE, PEAK_MEDIAN_N, PEAK_WEIGHT_S,
  SPIKE_MEAS_RATE, DISTURBANCE_BLANK_S, ROUGH_RMS_MAX, WS_SPREAD_JUMP,
  TAU_EVIDENCE_S, LR_MIN_WEIGHT, LR_TOL,
  NUDGE_PERIOD_S, NUDGE_MIN_WEIGHT, NUDGE_MAX_STEP, FACTOR_STEP, nudge_units,
  VERIFY_MIN_WEIGHT, VERIFY_FAIL_HOLD_WEIGHT,
  LOCK_MIN_WEIGHT, LOCK_DEADBAND, LOCK_STABLE_S,
)

PLATFORM_GAIN_HIGH = 1.05  # Mach-E
DT = 0.05


def applied_gain(v, low_factor, high_factor):
  a = speed_alpha(v)
  return (1.0 - a) * (LOW_ANCHOR_BASE * low_factor) + a * (PLATFORM_GAIN_HIGH * high_factor)


def ideal_gain(v, true_low, true_high):
  return applied_gain(v, true_low, true_high)


def feed_plant(est, true_low, true_high, speeds, applied_low=1.0, applied_high=1.0,
               kappa=0.002, n_per_speed=200, noise=0.0, seed=42):
  """Feed samples from a plant whose true gain corresponds to the given ideal factors.

  The plant's response ratio r = applied_gain / ideal_gain: if the applied factors already
  matched the true ones, r would be 1 everywhere.
  """
  rng = random.Random(seed)
  for v in speeds:
    g = applied_gain(v, applied_low, applied_high)
    r0 = g / ideal_gain(v, true_low, true_high)
    for _ in range(n_per_speed):
      r = r0 * (1.0 + (rng.uniform(-noise, noise) if noise else 0.0))
      est.add_sample(v, kappa, kappa * r, g, weight=DT)


class TestAngleFactorEstimator:
  def test_recovers_true_factors(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    feed_plant(est, 0.92, 1.21, speeds=[10, 12, 15, 18, 21, 24, 27, 29], n_per_speed=200, noise=0.03)
    low, high, _ = est.solve()
    assert abs(low - 0.92) < 0.02, low
    assert abs(high - 1.21) < 0.02, high

  def test_invariant_to_applied_factor_trajectory(self):
    # Half the drive on one applied pair, half on another: same truth must come out.
    # This is the property that keeps the nudge loop stable.
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    feed_plant(est, 0.92, 1.21, speeds=[10, 15, 20, 25, 29], applied_low=1.10, applied_high=0.90,
               n_per_speed=150, noise=0.03, seed=1)
    feed_plant(est, 0.92, 1.21, speeds=[10, 15, 20, 25, 29], applied_low=0.95, applied_high=1.20,
               n_per_speed=150, noise=0.03, seed=2)
    low, high, _ = est.solve()
    assert abs(low - 0.92) < 0.02, low
    assert abs(high - 1.21) < 0.02, high

  def test_rejects_bad_samples(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    g = applied_gain(20.0, 1.0, 1.0)
    assert not est.add_sample(20.0, 0.0005, 0.0005, g)   # below curvature threshold
    assert not est.add_sample(5.0, 0.002, 0.002, g)      # below speed threshold
    assert not est.add_sample(20.0, 0.002, -0.002, g)    # sign mismatch
    assert not est.add_sample(20.0, 0.002, 0.02, g)      # absurd ratio
    assert not est.add_sample(29.0, 0.004, 0.004, g)     # 3.4 m/s^2 lat accel
    assert est.n == 0
    assert est.add_sample(29.0, 0.0025, 0.0025, g)       # 2.1 m/s^2 — within tire limits

  def test_factor_clamp(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    feed_plant(est, 2.5, 0.2, speeds=[10, 20, 29], n_per_speed=100)
    low, high, _ = est.solve()
    assert low == 1.5 and high == 0.5  # clamped to the +/- button range

  def test_decay_halves_weight_at_tau_ln2(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    feed_plant(est, 1.0, 1.0, speeds=[10, 29], n_per_speed=100)
    w0 = est.s_w
    est.decay(TAU_EVIDENCE_S * math.log(2.0))
    assert abs(est.s_w - 0.5 * w0) < 1e-9

  def test_lr_divergence_flags_bank_bias(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    n = int((LR_MIN_WEIGHT + 2) / DT)
    g_hi = applied_gain(28.0, 1.0, 1.0)
    for _ in range(n):  # balanced clean evidence at the high anchor keeps the fit solvable
      est.add_sample(28.0, 0.0015, 0.0015, g_hi, weight=DT)
      est.add_sample(28.0, -0.0015, -0.0015, g_hi, weight=DT)
    g = applied_gain(10.0, 1.0, 1.0)
    for _ in range(n):  # left turns read 10% strong, right turns 10% weak — crowned road
      est.add_sample(10.0, 0.002, 0.002 * 1.10, g, weight=DT)
      est.add_sample(10.0, -0.002, -0.002 * 0.90, g, weight=DT)
    assert est.lr_divergence(0) > LR_TOL
    _, _, st = est.solve()
    assert st["stderr_eff_low"] > st["stderr_low"]  # divergence inflates the effective error

  def test_serialization_round_trip(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    feed_plant(est, 0.95, 1.12, speeds=[10, 15, 20, 25, 29], n_per_speed=120, noise=0.02)
    d = json.loads(json.dumps(est.to_dict()))  # through real JSON, like the param
    est2 = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    est2.from_dict(d)
    assert est.solve() == est2.solve()
    assert est2.n == est.n


class TestSteadyStateGate:
  def test_requires_sustained_steady(self):
    gate = SteadyStateGate(dt=DT)
    needed = int(STEADY_TIME_S / DT)
    results = [gate.update(True, MIN_KAPPA * 2, False, False, False)
               for _ in range(needed + 2)]
    assert not any(results[:needed - 1])
    assert results[-1]

  def test_resets_on_any_flag(self):
    gate = SteadyStateGate(dt=DT)
    for _ in range(int(STEADY_TIME_S / DT) + 1):
      gate.update(True, MIN_KAPPA * 2, False, False, False)
    assert gate.update(True, MIN_KAPPA * 2, False, False, False)
    gate.update(True, MIN_KAPPA * 2, True, False, False)  # pressed
    assert gate.steady_s == 0.0

  def test_ramp_within_relative_bound_admitted(self):
    # Lag alignment absorbs the transport delay, so a genuinely winding road — kappa
    # moving at up to REL_KAPPA_RATE of itself — IS evidence now. This ramp (25%/s
    # relative) was rejected by the old frozen-command gate; that starvation discarded
    # 89-100% of clean curve time on real winding-road drives (2026-07-22 analysis).
    gate = SteadyStateGate(dt=DT)
    k = MIN_KAPPA * 3
    admitted = False
    for _ in range(int(STEADY_TIME_S / DT) * 6):
      admitted |= gate.update(True, k, False, False, False)
      k += 0.25 * k * DT
    assert admitted

  def test_ramp_beyond_relative_bound_rejected(self):
    # Twice the relative bound: the residual delay-estimate error would bias these
    # ratios beyond what the stderr machinery is sized for — still rejected.
    gate = SteadyStateGate(dt=DT)
    k = MIN_KAPPA * 3
    admitted = False
    for _ in range(int(STEADY_TIME_S / DT) * 6):
      admitted |= gate.update(True, k, False, False, False)
      k += 2.0 * REL_KAPPA_RATE * k * DT
    assert not admitted

  def test_saturation_blocks(self):
    gate = SteadyStateGate(dt=DT)
    for _ in range(int(STEADY_TIME_S / DT) + 2):
      assert not gate.update(True, 0.002, False, False, False, saturated=True)

  def test_light_torque_starts_cooldown(self):
    gate = SteadyStateGate(dt=DT)
    gate.update(True, 0.002, False, False, False, driver_torque=0.7)
    assert gate.grip_cooldown_s > 0.0
    blocked = int(PRESS_COOLDOWN_S / DT) - 1
    for _ in range(blocked):
      assert not gate.update(True, 0.002, False, False, False)


class TestQualityMonitor:
  def test_clean_cornering_never_rejected(self):
    q = QualityMonitor(dt=DT)
    # A realistic apex sweep: command and measurement move together at plausible rates.
    t = 0.0
    ok_all = True
    for _ in range(400):
      k = 0.002 * math.sin(2 * math.pi * t / 10.0)
      ok_all &= q.update(k, k * 0.98, a_ego=0.2, ws_spread=0.05)
      t += DT
    assert ok_all
    assert all(v == 0 for v in q.counters.values())

  def test_flick_blanks_and_recovers(self):
    q = QualityMonitor(dt=DT)
    for _ in range(50):
      assert q.update(0.002, 0.002)
    # Bump: measurement jumps a full SPIKE step in one frame, command quiet. The return
    # to baseline next frame is itself a spike (the down-edge of the same bump) and
    # legitimately re-arms the blanking, so recovery takes blank + 1 frame.
    assert not q.update(0.002, 0.002 + SPIKE_MEAS_RATE * DT * 2)
    assert q.flick_fired
    blank_frames = int(DISTURBANCE_BLANK_S / DT)
    for i in range(blank_frames):
      assert not q.update(0.002, 0.002), i
    assert q.update(0.002, 0.002)
    assert q.counters["flick"] > 0

  def test_command_tracking_spike_is_not_flick(self):
    # The measurement racing after a moving command is control, not disturbance.
    q = QualityMonitor(dt=DT)
    q.update(0.002, 0.002)
    q.update(0.002 + 0.001, 0.002 + SPIKE_MEAS_RATE * DT * 2)  # command moved too
    assert not q.flick_fired

  def test_wheel_speed_jump_corroborates(self):
    q = QualityMonitor(dt=DT)
    q.update(0.002, 0.002, ws_spread=0.05)
    assert not q.update(0.002, 0.002, ws_spread=0.05 + WS_SPREAD_JUMP * 1.5)
    assert q.flick_fired

  def test_rough_road_blocks_until_settled(self):
    q = QualityMonitor(dt=DT)
    rng = random.Random(7)
    # Washboard: broadband measurement noise well above the RMS threshold.
    rejected = 0
    for _ in range(200):
      if not q.update(0.002, 0.002 + rng.uniform(-4, 4) * ROUGH_RMS_MAX):
        rejected += 1
    assert rejected > 100
    assert q.counters["rough"] + q.counters["flick"] == rejected

  def test_long_accel_rejects(self):
    q = QualityMonitor(dt=DT)
    assert q.update(0.002, 0.002, a_ego=MAX_LONG_ACCEL * 0.5)
    assert not q.update(0.002, 0.002, a_ego=MAX_LONG_ACCEL * 1.5)
    assert q.counters["accel"] == 1


def _sine_apex_drive(pm, v, amp, period_s, n_frames, gain_ratio=1.0, lag_frames=6,
                     ok=True, dt=DT):
  """Drive the peak matcher with a sinusoidal command and a lagged, scaled measurement.
  Returns all committed samples."""
  out = []
  hist = []
  t = 0.0
  for _ in range(n_frames):
    k = amp * math.sin(2 * math.pi * t / period_s)
    hist.append(k)
    k_lag = hist[-1 - lag_frames] if len(hist) > lag_frames else 0.0
    out += pm.push(k, k_lag * gain_ratio, v, applied_gain(v, 1.0, 1.0), ok)
    t += dt
  return out


class TestPeakMatcher:
  def test_recovers_gain_ratio_from_lagged_sine(self):
    pm = PeakMatcher(dt=DT)
    committed = _sine_apex_drive(pm, v=12.0, amp=0.003, period_s=8.0, n_frames=2400,
                                 gain_ratio=0.92)
    assert len(committed) >= 2
    for (_v, k_cmd, k_meas, _g) in committed:
      assert abs(k_meas / k_cmd - 0.92) < 0.02

  def test_ripple_below_prominence_never_fires(self):
    pm = PeakMatcher(dt=DT)
    committed = _sine_apex_drive(pm, v=12.0, amp=PEAK_PROMINENCE * 0.4 + PEAK_MIN_KAPPA,
                                 period_s=1.6, n_frames=1200)
    # Fast ripple: the +-1s dominance window contains multiple crests, so no apex is
    # dominant and nothing commits.
    assert committed == []

  def test_poisoned_window_discards_apex(self):
    pm = PeakMatcher(dt=DT)
    n = 0
    hist = []
    t = 0.0
    for i in range(2400):
      k = 0.003 * math.sin(2 * math.pi * t / 8.0)
      hist.append(k)
      k_lag = hist[-7] if len(hist) > 6 else 0.0
      if i == 1200:
        pm.poison_recent(0.3)  # a disturbance was detected mid-drive
      n += len(pm.push(k, k_lag, 12.0, applied_gain(12.0, 1.0, 1.0), ok=True))
      t += DT
    pm2 = PeakMatcher(dt=DT)
    n_clean = len(_sine_apex_drive(pm2, v=12.0, amp=0.003, period_s=8.0, n_frames=2400))
    assert n <= n_clean  # the poisoned apex (and only that region) was lost

  def test_median_of_three_kills_single_outlier(self):
    pm = PeakMatcher(dt=DT)
    committed = []
    hist = []
    t = 0.0
    # One apex in the middle of the drive measures wildly strong (loose gravel moment):
    # the median commit must not let its ratio through.
    for i in range(3600):
      k = 0.003 * math.sin(2 * math.pi * t / 8.0)
      hist.append(k)
      k_lag = hist[-7] if len(hist) > 6 else 0.0
      ratio = 2.2 if 1180 <= i <= 1260 else 1.0
      committed += pm.push(k, k_lag * ratio, 12.0, applied_gain(12.0, 1.0, 1.0), True)
      t += DT
    assert len(committed) >= 2
    for (_v, k_cmd, k_meas, _g) in committed:
      assert abs(k_meas / k_cmd) < 1.5  # the 2.2x apex never got committed


def run_pipeline(pipe, n, torque=0.0, pressed=False, saturated=False, kappa=0.002, v=20.0,
                 low=1.0, high=1.0):
  committed = []
  for _ in range(n):
    committed += pipe.update(_frame(v, kappa, kappa, pressed=pressed,
                                    saturated=saturated, torque=torque,
                                    low=low, high=high))
  return committed


class TestAutoCalPipeline:
  def test_commits_after_holdback(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int((STEADY_TIME_S + PRESS_HOLDBACK_S) / DT) + 3 + _LAG_F
    committed = run_pipeline(pipe, warm)
    assert pipe.est.n > 0
    assert len(committed) == pipe.est.n

  def test_grip_cancels_staged_samples(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int(STEADY_TIME_S / DT) + 1 + int(PRESS_HOLDBACK_S / DT) // 2 + _LAG_F
    run_pipeline(pipe, warm)
    assert len(pipe._staged) > 0 and pipe.est.n == 0
    pipe.update(_frame(20.0, 0.002, 0.002, pressed=True))  # grip
    assert len(pipe._staged) == 0
    assert pipe.est.n == 0  # nothing from before the grip ever reached the estimator

  def test_disturbance_cancels_staged_samples(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int(STEADY_TIME_S / DT) + 1 + int(PRESS_HOLDBACK_S / DT) // 2 + _LAG_F
    run_pipeline(pipe, warm)
    assert len(pipe._staged) > 0 and pipe.est.n == 0
    # Bump: measured curvature jumps while the command sits still.
    pipe.update(_frame(20.0, 0.002, 0.002 + SPIKE_MEAS_RATE * DT * 2))
    assert len(pipe._staged) == 0 and pipe.est.n == 0
    # And the blanking window keeps evidence off while the car settles.
    committed = run_pipeline(pipe, int(DISTURBANCE_BLANK_S / DT) - 2)
    assert committed == []

  def test_saturated_frames_never_commit(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    committed = run_pipeline(pipe, 100, saturated=True)
    assert committed == [] and pipe.est.n == 0

  def test_idle_clears_staging(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    run_pipeline(pipe, int(STEADY_TIME_S / DT) + 5 + _LAG_F)
    assert len(pipe._staged) > 0
    pipe.idle()
    assert len(pipe._staged) == 0 and pipe.gate.steady_s == 0.0
    assert pipe._hist == []  # alignment must never target commands across a discontinuity

  def test_unsettled_measurement_not_staged(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int((STEADY_TIME_S + PRESS_HOLDBACK_S) / DT) + 10
    kappa_meas = 0.0010
    staged_during_sweep = 0
    for _ in range(warm):
      pipe.update(_frame(20.0, 0.002, kappa_meas))
      if kappa_meas < 0.0019:
        kappa_meas += 0.0002  # 0.004/s sweep, far above the settle bound
        staged_during_sweep = len(pipe._staged) + pipe.est.n
    assert staged_during_sweep == 0
    assert pipe.est.n > 0

  def test_near_limit_evidence_downweighted_to_zero(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    # kappa*v^2 = 2.55 > MAX_LAT_ACCEL: hard-rejected as 'limit'.
    committed = run_pipeline(pipe, 60, kappa=0.0034, v=27.4)
    assert committed == []
    assert pipe.quality.counters["limit"] > 0

  def test_pipeline_serialization_round_trip(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    run_pipeline(pipe, 200)
    pipe.stable_s = 123.0
    pipe.nudges = 4
    pipe.verify_result[0] = "confirmed"
    pipe.verify_hold[1] = 12.0
    d = json.loads(json.dumps(pipe.to_dict()))
    pipe2 = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    pipe2.from_dict(d)
    assert pipe2.est.solve() == pipe.est.solve()
    assert pipe2.stable_s == 123.0 and pipe2.nudges == 4 and not pipe2.locked
    assert pipe2.verify_result[0] == "confirmed" and pipe2.verify_hold[1] == 12.0


def _evidenced_pipe(true_low=1.10, true_high=1.10, applied=(1.0, 1.0), weight_s=15.0):
  """Pipeline with clean steady evidence at both anchors against a known plant."""
  pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
  n = int(weight_s / DT)
  for v, kappa in ((10.0, 0.004), (28.0, 0.0015)):
    g = applied_gain(v, *applied)
    r = g / ideal_gain(v, true_low, true_high)
    pipe.gate.steady_s = 0.0
    pipe._meas_last = None
    pipe._hist.clear()  # speed-block boundary: never align against the other block's cmd
    for _ in range(n):
      pipe.update(_frame(v, kappa, kappa * r, low=applied[0], high=applied[1]))
  return pipe


def _feed_low(pipe, applied, seconds, true_low, true_high, ratio_scale=1.0,
              v=10.0, kappa=0.004):
  """Steady low-band frames from the plant under the given applied factors; ratio_scale
  != 1 makes the car respond off-model (the adjust-then-verify failure case)."""
  g = applied_gain(v, applied[0], applied[1])
  r = g / ideal_gain(v, true_low, true_high) * ratio_scale
  pipe.gate.steady_s = 0.0
  pipe._meas_last = None
  pipe._hist.clear()
  for _ in range(int(seconds / DT)):
    pipe.update(_frame(v, kappa, kappa * r, low=applied[0], high=applied[1]))


class TestFactorNudger:
  def _evidenced_pipe(self, **kw):
    return _evidenced_pipe(**kw)

  def test_nudges_toward_target_bounded(self):
    # err 0.10, damped by NUDGE_GAIN then capped: a big-but-bounded step, not the whole error.
    pipe = self._evidenced_pipe(true_low=1.10, true_high=1.10)
    rec = pipe.recommend(1.0, 1.0)
    assert rec is not None
    low, high = rec
    assert low == round(1.0 + NUDGE_MAX_STEP, 2)
    assert high == round(1.0 + NUDGE_MAX_STEP, 2)

  def test_single_step_when_close(self):
    # A target ~0.01 away moves by exactly one menu step (the point of damped stepping).
    pipe = self._evidenced_pipe(true_low=1.01, true_high=1.01)
    rec = pipe.recommend(1.0, 1.0)
    assert rec is not None
    assert rec[0] == round(1.0 + FACTOR_STEP, 2) and rec[1] == round(1.0 + FACTOR_STEP, 2)

  def test_deadband_no_nudge(self):
    # Inside the implicit deadband (|gain*err| < half a step): leave it alone.
    pipe = self._evidenced_pipe(true_low=1.004, true_high=1.004)
    assert pipe.recommend(1.0, 1.0) is None

  def test_nudge_units_damped_and_capped(self):
    cap = round(NUDGE_MAX_STEP / FACTOR_STEP)
    assert nudge_units(0.0) == 0
    assert nudge_units(0.004) == 0          # implicit deadband
    assert nudge_units(0.01) == 1           # single menu step when close
    assert nudge_units(0.10) == cap         # far: damped then capped
    assert nudge_units(-0.10) == -cap       # symmetric

  def test_rate_limited(self):
    pipe = self._evidenced_pipe()
    assert pipe.recommend(1.0, 1.0) is not None
    assert pipe.recommend(1.02, 1.02) is None  # inside NUDGE_PERIOD_S
    # advance active time
    for _ in range(int(NUDGE_PERIOD_S / DT) + 1):
      pipe.update(_frame(10.0, 0.004, 0.004, low=1.02, high=1.02))
    assert pipe.recommend(1.02, 1.02) is not None

  def test_insufficient_evidence_no_nudge(self):
    pipe = self._evidenced_pipe(weight_s=NUDGE_MIN_WEIGHT * 0.3)
    assert pipe.recommend(1.0, 1.0) is None

  def test_no_cumulative_cap_walks_to_the_fit(self):
    # 2026-07-22 design decision: no per-drive movement cap. A car that is genuinely
    # 40% off must be allowed to walk all the way in one drive, as long as every step
    # keeps verifying against fresh evidence (the plant here always agrees).
    pipe = self._evidenced_pipe(true_low=1.40, true_high=1.40)
    applied = [1.0, 1.0]
    for _ in range(30):
      for _f in range(int(NUDGE_PERIOD_S / DT) + 1):
        g = applied_gain(10.0, *applied)
        r = g / ideal_gain(10.0, 1.40, 1.40)
        pipe.update(_frame(10.0, 0.004, 0.004 * r, low=applied[0], high=applied[1]))
      rec = pipe.recommend(*applied)
      if rec is not None:
        applied = list(rec)
    assert applied[0] >= 1.35, applied      # far past the old 0.04/0.10 caps
    assert pipe.verify_result[0] == "confirmed"  # and every step was checked on the way

  def test_user_edit_soft_resets(self):
    pipe = self._evidenced_pipe()
    w0 = pipe.est.s_w
    pipe.stable_s = 100.0
    pipe.user_edit()
    assert abs(pipe.est.s_w - 0.5 * w0) < 1e-9
    assert pipe.stable_s == 0.0
    # Evidence NOT wiped: the fit is still there, just less confident.
    assert pipe.est.solve() is not None


class TestLagAlignment:
  """The 2026-07-22 evidence-starvation fix: ratios are taken against the command from
  lateral_delay ago, so winding roads (a moving command) become usable evidence without
  lag bias — the exact scenario the old frozen-command gate had to discard."""

  def _ramped_pipe(self, true_gain_ratio, lag_frames=4, n=1200, rel_rate=0.25):
    """Plant with a PURE transport delay: meas(t) = ratio * cmd(t - lag). The command
    ramps continuously at rel_rate (within the admission bound) — under the old gate
    this drive yields nothing; under alignment it must recover the ratio exactly."""
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    hist = []
    k = 0.002
    for _ in range(n):
      hist.append(k)
      k_meas = true_gain_ratio * (hist[-1 - lag_frames] if len(hist) > lag_frames else 0.0)
      pipe.update(_frame(10.0, k, k_meas, lat_delay=lag_frames * DT))
      k *= 1.0 + rel_rate * DT
      if k > 0.004:
        k = 0.002  # saw-tooth reset; the drop is a huge rate step the gate must absorb
    return pipe

  def test_recovers_ratio_from_delayed_moving_command(self):
    # Car delivers 90% of requested with a 0.2s transport delay, command always moving.
    pipe = self._ramped_pipe(0.90)
    assert pipe.est.n > 100  # the old gate got ~zero here
    _w, r = pipe.est.recent_response(0)  # all evidence at v=10 -> low half
    assert r is not None and abs(r - 0.90) < 0.005, r  # aligned ratio is exact, not lag-biased

  def test_same_frame_ratio_would_have_been_biased(self):
    # Sanity for the whole design: on this plant the same-frame ratio is NOT the gain —
    # the lag makes it read low on a rising ramp. Alignment is what removes that bias.
    k = 0.002
    hist = []
    biased = []
    for _ in range(200):
      hist.append(k)
      if len(hist) > 4:
        biased.append((0.90 * hist[-5]) / k)
      k *= 1.0 + 0.25 * DT
    assert max(biased) < 0.90 - 0.01  # every same-frame sample reads low

  def test_no_evidence_before_history_fills(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    for _ in range(_LAG_F - 2):
      pipe.update(_frame(10.0, 0.003, 0.003))
    assert pipe._staged == [] and pipe.est.n == 0

  def test_delay_clamped_to_trust_window(self):
    # An absurd liveDelay value must not demand an absurd history depth.
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int((STEADY_TIME_S + PRESS_HOLDBACK_S) / DT) + 3 + int(round(0.42 / DT)) + 1
    for _ in range(warm):
      pipe.update(_frame(10.0, 0.003, 0.003, lat_delay=5.0))
    assert pipe.est.n > 0  # clamped to LAG_MAX_S and evidence still flows


class TestQuietGate:
  """Loop hunting must never become gain evidence — the user's criterion, 2026-07-23:
  taking 1-4 passes per step is fine; moving the needle on mid-dynamics data is not."""

  def test_hunting_yields_almost_no_evidence(self):
    # Same duration, same command: a calm constant-deficit plant vs a hunting plant
    # whose error swings on a ~3s loop cycle (all swings INSIDE the rate bounds that
    # used to admit them). The hunting run must yield a small fraction of the weight.
    quiet = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    hunt = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    for i in range(1200):
      quiet.update(_frame(10.0, 0.003, 0.003 * 0.90))
      swing = 0.0006 * math.sin(2 * math.pi * i * DT / 3.0)
      hunt.update(_frame(10.0, 0.003, 0.003 * 0.90 + swing))
    assert quiet.est.s_w > 0
    assert hunt.est.s_w < 0.35 * quiet.est.s_w, (hunt.est.s_w, quiet.est.s_w)

  def test_constant_deficit_is_calm_and_admitted(self):
    # A steady plant deficit keeps a FLAT error trend: exactly the signal we want,
    # and the quiet gate must not confuse it with dynamics.
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    _feed_low(pipe, (1.0, 1.0), 10.0, 1.10, 1.10)
    assert pipe.est.s_w > 0
    _w, r = pipe.est.recent_response(0)
    assert r is not None and abs(r - 1.0 / 1.10) < 0.01


class TestAdjustVerify:
  """Every step is judged against FRESH post-step evidence before its anchor may step
  again — the no-cap regime's runaway protection ('poll a couple turns, adjust, poll
  some more', made enforceable)."""

  def test_step_opens_verify_window(self):
    pipe = _evidenced_pipe()
    rec = pipe.recommend(1.0, 1.0)
    assert rec is not None
    assert pipe.verify[0] is not None and pipe.verify[0]["to"] == rec[0]
    assert pipe.est.recent[0] == [0.0, 0.0]  # the judgment sees only post-step data

  def test_no_second_step_until_fresh_evidence(self):
    pipe = _evidenced_pipe(true_low=1.40, true_high=1.40)
    rec = pipe.recommend(1.0, 1.0)
    assert rec is not None
    # Advance the nudge clock with frames that carry NO evidence (below MIN_KAPPA):
    # plenty of time passes, but the step has not been answered by data.
    for _ in range(int(NUDGE_PERIOD_S / DT) + 1):
      pipe.update(_frame(10.0, 0.0005, 0.0005, low=rec[0], high=rec[1]))
    assert pipe.recommend(*rec) is None       # window still open
    # Fresh agreeing evidence arrives: the step is judged and the walk continues.
    _feed_low(pipe, rec, VERIFY_MIN_WEIGHT + 3.0, 1.40, 1.40)
    assert pipe.verify[0] is None
    assert pipe.verify_result[0] == "confirmed"
    assert pipe.recommend(*rec) is not None

  def test_failed_verify_holds_anchor(self):
    pipe = _evidenced_pipe(true_low=1.10, true_high=1.10)
    rec = pipe.recommend(1.0, 1.0)
    assert rec is not None and rec[0] > 1.0   # stepped UP toward the fit
    # Contrarian car: after the step up, the measured response DROPS — the data
    # contradicts the model, so the step must not be trusted.
    _feed_low(pipe, rec, VERIFY_MIN_WEIGHT + 3.0, 1.10, 1.10, ratio_scale=0.85)
    assert pipe.verify_result[0] == "failed"
    assert pipe.verify_hold[0] == VERIFY_FAIL_HOLD_WEIGHT
    # Kill more clock without evidence: still held (fresh weight < the fail demand).
    for _ in range(int(NUDGE_PERIOD_S / DT) + 1):
      pipe.update(_frame(10.0, 0.0005, 0.0005, low=rec[0], high=rec[1]))
    assert pipe.recommend(*rec) is None
    # Twice the evidence arrives and keeps asking for movement: the hold releases.
    _feed_low(pipe, rec, VERIFY_FAIL_HOLD_WEIGHT + 4.0, 1.10, 1.10, ratio_scale=0.85)
    assert pipe.recommend(*rec) is not None

  def test_lock_disabled_never_freezes(self):
    # FordAngleAutoCalLock off: stability may accumulate forever, the pipeline must not
    # lock — continuous adaptation for the life of the toggle. Lock-eligible evidence
    # (weights past LOCK_MIN_WEIGHT, target == applied) is earned for real so the
    # 'ready' predicate holds and only the lock_enabled check stands between
    # stable_s and the freeze.
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    feed_plant(pipe.est, 1.0, 1.0, speeds=[10, 28], n_per_speed=1400)
    pipe.lock_enabled = False
    pipe.stable_s = LOCK_STABLE_S - 0.1
    _feed_low(pipe, (1.0, 1.0), 3.0, 1.0, 1.0)
    assert not pipe.locked
    assert pipe.stable_s > LOCK_STABLE_S  # kept counting straight past the threshold

  def test_verify_state_survives_serialization(self):
    pipe = _evidenced_pipe()
    rec = pipe.recommend(1.0, 1.0)
    assert rec is not None
    d = json.loads(json.dumps(pipe.to_dict()))
    pipe2 = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    pipe2.from_dict(d)
    assert pipe2.verify[0] == pipe.verify[0]
    assert pipe2.est.recent == pipe.est.recent


class TestRecentResponse:
  def test_tracks_current_ratio(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH)
    g = applied_gain(10.0, 1.0, 1.0)
    for _ in range(100):
      est.add_sample(10.0, 0.002, 0.002 * 0.93, g, weight=DT)
    w, r = est.recent_response(0)
    assert abs(r - 0.93) < 1e-9 and w > 4.0   # "turns 93% of requested"
    assert est.recent_response(1)[1] is None  # no high-band evidence yet


class TestUiState:
  def test_propose_then_verify_phases(self):
    pipe = _evidenced_pipe(true_low=1.10, true_high=1.10)
    ui = pipe.ui_state(1.0, 1.0)
    assert ui["low"]["ph"] == "propose" and ui["low"]["t"] > 1.0
    assert abs(ui["low"]["r"] - 1.0 / 1.10) < 0.02
    json.dumps(ui)  # must survive the telemetry string
    rec = pipe.recommend(1.0, 1.0)
    ui = pipe.ui_state(*rec)
    assert ui["low"]["ph"] == "verify" and ui["low"]["to"] == rec[0]

  def test_collect_phase_before_evidence(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    ui = pipe.ui_state(1.0, 1.0)
    assert ui["low"]["ph"] == "collect" and ui["high"]["ph"] == "collect"
    json.dumps(ui)

  def test_good_phase_when_matched(self):
    pipe = _evidenced_pipe(true_low=1.0, true_high=1.0)
    ui = pipe.ui_state(1.0, 1.0)
    assert ui["low"]["ph"] == "good" and ui["high"]["ph"] == "good"


class TestClosedLoopConvergence:
  """The whole point: a synthetic car with true factors 1.02/1.15 driven from 1.00/1.00
  must be nudged into the lock deadband and eventually lock, across simulated drives,
  with bumps and grips injected along the way."""

  TRUE_LOW, TRUE_HIGH = 1.02, 1.15

  def _drive(self, pipe, applied, seconds, v, kappa_amp, rng):
    """Alternating-direction steady arcs with brief transitions; occasional bumps and
    grips. Plant: first-order lag toward gain-scaled command. Nudges applied live."""
    lag_tau = 0.35
    k_meas = 0.0
    frames = int(seconds / DT)
    seg_frames = int(20.0 / DT)
    nudge_log = []
    for i in range(frames):
      seg, pos = divmod(i, seg_frames)
      direction = 1.0 if seg % 2 == 0 else -1.0
      # 1.5s ramp between arcs (clearly non-steady), then constant curvature.
      ramp = min(1.0, pos / int(1.5 / DT))
      k_cmd = direction * kappa_amp * ramp
      g = applied_gain(v, *applied)
      k_target = k_cmd * g / ideal_gain(v, self.TRUE_LOW, self.TRUE_HIGH)
      k_meas += (k_target - k_meas) * DT / (lag_tau + DT)
      bump = rng.random() < 0.001  # ~one flick per 50 s
      meas = k_meas + (SPIKE_MEAS_RATE * DT * 3 if bump else 0.0)
      grip = 1.2 if rng.random() < 0.0005 else 0.0
      pipe.update(_frame(v, k_cmd, meas, torque=grip, a_ego=0.1,
                         low=applied[0], high=applied[1]))
      rec = pipe.recommend(*applied)
      if rec is not None:
        nudge_log.append(rec)
        applied[0], applied[1] = rec
      if pipe.locked:
        break
    return applied, nudge_log

  def test_converges_and_locks(self):
    rng = random.Random(11)
    applied = [1.00, 1.00]
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    all_nudges = []
    # Simulated multi-drive: each "drive" serializes and restores like an ignition cycle.
    for _drive_i in range(8):
      if pipe.locked:
        break
      # Half the drive at the low anchor, half at the high anchor.
      applied, n1 = self._drive(pipe, applied, 240.0, v=11.0, kappa_amp=0.004, rng=rng)
      applied, n2 = self._drive(pipe, applied, 240.0, v=28.0, kappa_amp=0.0015, rng=rng)
      all_nudges += n1 + n2
      d = json.loads(json.dumps(pipe.to_dict()))
      pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)  # new card process
      pipe.from_dict(d)
      pipe.idle()

    assert abs(applied[0] - self.TRUE_LOW) <= LOCK_DEADBAND, (applied, len(all_nudges))
    assert abs(applied[1] - self.TRUE_HIGH) <= LOCK_DEADBAND, (applied, len(all_nudges))
    assert pipe.locked, (applied, pipe.stable_s, pipe.est.weight_low, pipe.est.weight_high)
    # No oscillation: once inside the deadband the nudger must not bounce in and out.
    lows = [r[0] for r in all_nudges]
    assert all(l2 >= l1 - NUDGE_MAX_STEP - 1e-9 for l1, l2 in zip(lows, lows[1:])), lows


def _frame(v, kc, km, pressed=False, rate=False, dev=False, saturated=False,
           torque=0.0, a_ego=0.0, ws=None, low=1.0, high=1.0, lat_delay=0.2) -> Frame:
  """Test scaffolding: Frame with benign defaults (the production dataclass has none)."""
  return Frame(v_ego=v, kappa_cmd=kc, kappa_meas=km, steering_pressed=pressed,
               angle_rate_limited=rate, deviation_limited=dev, saturated=saturated,
               driver_torque=torque, a_ego=a_ego, ws_spread=ws,
               low_factor=low, high_factor=high, lateral_delay=lat_delay)


# Alignment warmup at the default test delay (0.2 s): the pipeline needs this many frames
# of command history before any steady evidence can exist.
_LAG_F = int(round(0.2 / DT)) + 1


class _MockParams:
  """Duck-typed openpilot Params: just enough for update_angle_params. put() lands
  immediately (readable on the next get), like a completed async write.

  TYPE-CHECKED like the real fork's Params (params_pyx python2cpp): writing the wrong
  python type raises TypeError. The real system silently ate a str-into-FLOAT nudge
  write on-device because the old mock accepted anything — never again."""

  _TYPES = {
    "FordLowSpeedFactor_ang": float,
    "FordHighSpeedFactor_ang": float,
    "FordAngleAutoCal": bool,
    "FordAngleAutoCalState": str,
    "FordAngleAutoCalError": str,
    "FordAngleAutoCalReset": bool,
    "FordAngleAutoCalLock": bool,
    "lane_change_factor_high_ang": float,
  }

  # Params whose declared default (params_keys.h) is true — the real get_bool returns
  # the default for unwritten keys, so the mock must too.
  _BOOL_DEFAULTS = {"FordAngleAutoCalLock": True}

  def __init__(self, values):
    self.values = values
    self.written = {}

  def get(self, key, return_default=False):
    return self.values.get(key)

  def get_bool(self, key):
    if key not in self.values:
      return self._BOOL_DEFAULTS.get(key, False)
    return bool(self.values.get(key))

  def put(self, key, value, block=False):
    # The real Params.put lands immediately when block=True; this mock always lands
    # immediately, so both paths behave the same here (readable on the next get).
    expected = self._TYPES.get(key)
    if expected is not None and not isinstance(value, expected):
      raise TypeError(f"Type mismatch while writing param {key}: got {type(value)}, expected {expected}")
    self.values[key] = value
    self.written[key] = value

  def put_bool(self, key, value):
    self.put(key, bool(value))


class TestOnboardGlue:
  """Exercise the REAL LateralAngleExt param/arming glue — the seam unit tests of the
  pipeline cannot see. This is the class of test that caught the on-device card
  crash-loop (stale attribute) that component tests missed."""

  def _ext(self):
    pytest.importorskip("cereal.messaging")  # linux-only
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt

    class _Harness(LateralAngleExt):
      def _ensure_lateral_curv_initialized(self, CP):
        pass

    ext = _Harness()
    class _CP:
      carFingerprint = "FORD_MUSTANG_MACH_E_MK1"
    ext.CP = _CP()
    return ext

  def _tick(self, ext, p, n=1):
    for _ in range(100 * n):
      ext.update_angle_params(p)

  def test_param_glue_runs_without_error(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 0, "FordAngleAutoCalState": ""})
    self._tick(ext, p, n=2)
    assert ext.autocal_ctl.pipeline is None and not ext.autocal_enabled

  def test_arming_builds_pipeline_with_baseline(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.10", "FordHighSpeedFactor_ang": "0.95"})
    ext.update_angle_params(p)
    assert ext.autocal_enabled and ext.autocal_ctl.pipeline is not None
    assert ext.autocal_ctl._last_written == (1.10, 0.95)

  def test_arming_restores_serialized_evidence(self):
    donor = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    feed_plant(donor.est, 1.05, 1.05, speeds=[10, 28], n_per_speed=200)
    state = json.dumps({"v": 1, "phase": "collecting", "pipe": donor.to_dict()})
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": state,
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    assert ext.autocal_ctl.pipeline is not None
    assert ext.autocal_ctl.pipeline.est.n == donor.est.n
    assert ext.autocal_ctl.pipeline.est.solve() == donor.est.solve()

  def test_locked_json_never_arms(self):
    ext = self._ext()
    state = json.dumps({"v": 1, "phase": "locked", "pipe": {}})
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": state})
    ext.update_angle_params(p)
    assert ext.autocal_ctl.pipeline is None and ext.autocal_ctl.done and not ext.autocal_enabled

  def test_legacy_done_state_never_arms(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "done low=1.02 high=1.15"})
    ext.update_angle_params(p)
    assert ext.autocal_ctl.pipeline is None and ext.autocal_ctl.done and not ext.autocal_enabled

  def test_garbage_state_starts_fresh(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "round 3 collecting; applied"})
    ext.update_angle_params(p)
    assert ext.autocal_ctl.pipeline is not None and ext.autocal_ctl.pipeline.est.n == 0

  def test_nudge_writes_params_and_state(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    assert ext.autocal_ctl._apply_nudge((1.02, 1.15))
    # The fork's params are typed FLOAT — a string write raises and the nudge dies.
    assert p.written["FordLowSpeedFactor_ang"] == 1.02 and isinstance(p.written["FordLowSpeedFactor_ang"], float)
    assert p.written["FordHighSpeedFactor_ang"] == 1.15 and isinstance(p.written["FordHighSpeedFactor_ang"], float)
    st = json.loads(p.written["FordAngleAutoCalState"])
    assert st["phase"] == "collecting" and st["applied"] == {"low": 1.02, "high": 1.15}
    # The blocking write landed, so it must NOT read back as a user edit. Point the strategy
    # at the written values (the single reader) and tick: no soft-reset, evidence untouched.
    p.values["FordLowSpeedFactor_ang"] = 1.02
    p.values["FordHighSpeedFactor_ang"] = 1.15
    n0 = ext.autocal_ctl.pipeline.est.n
    self._tick(ext, p, n=2)
    assert ext.autocal_ctl.pipeline is not None
    assert ext.autocal_ctl.pipeline.est.n == n0  # user_edit() not triggered

  def test_user_edit_adopted_single_tick(self):
    # Blocking nudge writes mean any param/last_written mismatch is a real driver edit —
    # detected and adopted on ONE tick, no async-lag debounce.
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    feed_plant(ext.autocal_ctl.pipeline.est, 1.05, 1.05, speeds=[10, 28], n_per_speed=200)
    w0 = ext.autocal_ctl.pipeline.est.s_w
    p.values["FordLowSpeedFactor_ang"] = "1.08"  # driver taps + in the menu
    self._tick(ext, p, n=1)
    assert abs(ext.autocal_ctl.pipeline.est.s_w - 0.5 * w0) < 1e-9  # soft reset, not a wipe
    assert ext.autocal_ctl._last_written == (1.08, 1.00)

  def test_save_restore_round_trip_through_param(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    feed_plant(ext.autocal_ctl.pipeline.est, 1.05, 1.05, speeds=[10, 28], n_per_speed=200)
    sol = ext.autocal_ctl.pipeline.est.solve()
    ext.autocal_ctl._save("collecting", (1.00, 1.00))
    # New process, same params: evidence must come back.
    ext2 = self._ext()
    ext2.update_angle_params(p)
    assert ext2.autocal_ctl.pipeline is not None
    assert ext2.autocal_ctl.pipeline.est.solve() == sol

  def test_lock_off_resumes_a_locked_calibration(self):
    # A finished (locked) calibration + FordAngleAutoCalLock=0: the lock is treated as
    # "resume from this evidence" — the controller arms, restores, and un-locks.
    donor = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    feed_plant(donor.est, 1.05, 1.05, speeds=[10, 28], n_per_speed=200)
    donor.locked = True
    state = json.dumps({"v": 1, "phase": "locked", "pipe": donor.to_dict()})
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": state,
                     "FordAngleAutoCalLock": 0,
                     "FordLowSpeedFactor_ang": "1.05", "FordHighSpeedFactor_ang": "1.05"})
    ext.update_angle_params(p)
    ctl = ext.autocal_ctl
    assert ctl.enabled and ctl.pipeline is not None and not ctl.done
    assert not ctl.pipeline.locked and not ctl.pipeline.lock_enabled
    assert ctl.pipeline.est.n == donor.est.n  # evidence carried over, nothing lost
    # Flipping the lock back ON mid-run re-enables freezing (but doesn't instantly lock).
    p.values["FordAngleAutoCalLock"] = 1
    self._tick(ext, p, n=1)
    assert ctl.pipeline is not None and ctl.pipeline.lock_enabled and not ctl.pipeline.locked

  def test_reset_param_erases_everything(self):
    # The "erase calibration memory" button: evidence, error log, the LOCK, and the
    # factors themselves all go back to neutral — a finished calibration can be retried.
    ext = self._ext()
    state = json.dumps({"v": 1, "phase": "locked", "pipe": {}})
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": state,
                     "FordAngleAutoCalReset": 1,
                     "FordLowSpeedFactor_ang": "1.12", "FordHighSpeedFactor_ang": "1.20"})
    ext.update_angle_params(p)  # first ~1 Hz tick consumes the reset
    assert p.values["FordAngleAutoCalReset"] is False
    assert p.values["FordAngleAutoCalState"] == "" and p.values["FordAngleAutoCalError"] == ""
    assert p.written["FordLowSpeedFactor_ang"] == 1.0 and p.written["FordHighSpeedFactor_ang"] == 1.0
    assert ext.autocal_ctl.status == "reset" and not ext.autocal_ctl.done
    # Next tick arms a FRESH collection despite the previously locked state.
    self._tick(ext, p, n=1)
    assert ext.autocal_ctl.pipeline is not None and ext.autocal_ctl.pipeline.est.n == 0
    assert ext.autocal_ctl._last_written == (1.0, 1.0)  # the wipe is not a "user edit"
    assert ext.low_speed_curv_factor == 1.0 and ext.high_speed_curv_factor == 1.0

  def test_status_is_json_when_armed(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    self._tick(ext, p, n=1)
    st = json.loads(ext.autocal_ctl.status)  # dashboards parse this
    assert st["low"]["ph"] == "collect" and st["high"]["f"] == 1.0
    assert st["low"]["need"] == NUDGE_MIN_WEIGHT

  def test_toggle_off_disarms(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    assert ext.autocal_ctl.pipeline is not None
    p.values["FordAngleAutoCal"] = 0
    self._tick(ext, p, n=1)
    assert ext.autocal_ctl.pipeline is None and not ext.autocal_enabled
