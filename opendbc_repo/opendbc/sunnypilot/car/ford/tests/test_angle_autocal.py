"""Tests for the continuous angle-mode factor auto-calibration (angle_autocal.py)."""
import json
import math
import random

import pytest

from opendbc.sunnypilot.car.ford.angle_autocal import (
  AngleFactorEstimator, AutoCalPipeline, PeakMatcher, QualityMonitor, SteadyStateGate,
  speed_alpha, V_LOW, V_HIGH, LOW_ANCHOR_BASE, STEADY_TIME_S, MIN_KAPPA,
  PRESS_HOLDBACK_S, PRESS_COOLDOWN_S, MAX_LAT_ACCEL, MAX_LONG_ACCEL,
  PEAK_MIN_KAPPA, PEAK_PROMINENCE, PEAK_MEDIAN_N, PEAK_WEIGHT_S,
  SPIKE_MEAS_RATE, DISTURBANCE_BLANK_S, ROUGH_RMS_MAX, WS_SPREAD_JUMP,
  TAU_EVIDENCE_S, LR_MIN_WEIGHT, LR_TOL,
  NUDGE_PERIOD_S, NUDGE_MIN_WEIGHT, NUDGE_DEADBAND, NUDGE_STEP, MAX_DRIVE_DELTA,
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

  def test_slow_ramp_bounded_by_window_drift(self):
    # A ramp inside the per-frame rate bound but drifting through the window must not
    # pass: the same-frame ratio would be actuation-lag-biased (liveDelay up to ~0.42s).
    gate = SteadyStateGate(dt=DT)
    k = MIN_KAPPA * 2
    admitted = False
    for _ in range(int(STEADY_TIME_S / DT) * 4):
      admitted |= gate.update(True, k, False, False, False)
      k += 0.5 * 0.0015 * DT   # half the per-frame rate limit, sustained
    assert not admitted
    # A truly flat command still passes — one extra frame for the drift reset that
    # closes the ramp's stale window, then a full fresh steady period.
    for _ in range(int(STEADY_TIME_S / DT) + 3):
      ok = gate.update(True, k, False, False, False)
    assert ok

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
    committed += pipe.update(v, kappa, kappa, pressed, False, False,
                             saturated=saturated, driver_torque=torque,
                             low_factor=low, high_factor=high)
  return committed


class TestAutoCalPipeline:
  def test_commits_after_holdback(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int((STEADY_TIME_S + PRESS_HOLDBACK_S) / DT) + 3
    committed = run_pipeline(pipe, warm)
    assert pipe.est.n > 0
    assert len(committed) == pipe.est.n

  def test_grip_cancels_staged_samples(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int(STEADY_TIME_S / DT) + 1 + int(PRESS_HOLDBACK_S / DT) // 2
    run_pipeline(pipe, warm)
    assert len(pipe._staged) > 0 and pipe.est.n == 0
    pipe.update(20.0, 0.002, 0.002, True, False, False)  # grip
    assert len(pipe._staged) == 0
    assert pipe.est.n == 0  # nothing from before the grip ever reached the estimator

  def test_disturbance_cancels_staged_samples(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int(STEADY_TIME_S / DT) + 1 + int(PRESS_HOLDBACK_S / DT) // 2
    run_pipeline(pipe, warm)
    assert len(pipe._staged) > 0 and pipe.est.n == 0
    # Bump: measured curvature jumps while the command sits still.
    pipe.update(20.0, 0.002, 0.002 + SPIKE_MEAS_RATE * DT * 2, False, False, False)
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
    run_pipeline(pipe, int(STEADY_TIME_S / DT) + 5)
    assert len(pipe._staged) > 0
    pipe.idle()
    assert len(pipe._staged) == 0 and pipe.gate.steady_s == 0.0

  def test_unsettled_measurement_not_staged(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    warm = int((STEADY_TIME_S + PRESS_HOLDBACK_S) / DT) + 10
    kappa_meas = 0.0010
    staged_during_sweep = 0
    for _ in range(warm):
      pipe.update(20.0, 0.002, kappa_meas, False, False, False)
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
    d = json.loads(json.dumps(pipe.to_dict()))
    pipe2 = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    pipe2.from_dict(d)
    assert pipe2.est.solve() == pipe.est.solve()
    assert pipe2.stable_s == 123.0 and pipe2.nudges == 4 and not pipe2.locked


class TestFactorNudger:
  def _evidenced_pipe(self, true_low=1.10, true_high=1.10, applied=(1.0, 1.0), weight_s=15.0):
    """Pipeline with clean steady evidence at both anchors against a known plant."""
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    n = int(weight_s / DT)
    for v, kappa in ((10.0, 0.004), (28.0, 0.0015)):
      g = applied_gain(v, *applied)
      r = g / ideal_gain(v, true_low, true_high)
      pipe.gate.steady_s = 0.0
      pipe._meas_last = None
      for _ in range(n):
        pipe.update(v, kappa, kappa * r, False, False, False,
                    low_factor=applied[0], high_factor=applied[1])
    return pipe

  def test_nudges_toward_target_bounded(self):
    pipe = self._evidenced_pipe(true_low=1.10, true_high=1.10)
    rec = pipe.recommend(1.0, 1.0)
    assert rec is not None
    low, high = rec
    assert low == round(1.0 + NUDGE_STEP, 2)   # full step, not the whole error
    assert high == round(1.0 + NUDGE_STEP, 2)

  def test_deadband_no_nudge(self):
    pipe = self._evidenced_pipe(true_low=1.01, true_high=1.01)
    assert pipe.recommend(1.0, 1.0) is None  # |err| ~ 0.01 < deadband

  def test_rate_limited(self):
    pipe = self._evidenced_pipe()
    assert pipe.recommend(1.0, 1.0) is not None
    assert pipe.recommend(1.02, 1.02) is None  # inside NUDGE_PERIOD_S
    # advance active time
    for _ in range(int(NUDGE_PERIOD_S / DT) + 1):
      pipe.update(10.0, 0.004, 0.004, False, False, False,
                  low_factor=1.02, high_factor=1.02)
    assert pipe.recommend(1.02, 1.02) is not None

  def test_insufficient_evidence_no_nudge(self):
    pipe = self._evidenced_pipe(weight_s=NUDGE_MIN_WEIGHT * 0.3)
    assert pipe.recommend(1.0, 1.0) is None

  def test_per_drive_cap(self):
    pipe = self._evidenced_pipe(true_low=1.40, true_high=1.40)
    applied = [1.0, 1.0]
    moved = 0.0
    for _ in range(30):  # far more opportunities than the cap allows
      for _f in range(int(NUDGE_PERIOD_S / DT) + 1):
        g = applied_gain(10.0, *applied)
        r = g / ideal_gain(10.0, 1.40, 1.40)
        pipe.update(10.0, 0.004, 0.004 * r, False, False, False,
                    low_factor=applied[0], high_factor=applied[1])
      rec = pipe.recommend(*applied)
      if rec is not None:
        moved += abs(rec[0] - applied[0])
        applied = list(rec)
    assert moved <= MAX_DRIVE_DELTA + 1e-9
    assert pipe.drive_delta_low <= MAX_DRIVE_DELTA + 1e-9

  def test_user_edit_soft_resets(self):
    pipe = self._evidenced_pipe()
    w0 = pipe.est.s_w
    pipe.stable_s = 100.0
    pipe.user_edit()
    assert abs(pipe.est.s_w - 0.5 * w0) < 1e-9
    assert pipe.stable_s == 0.0
    # Evidence NOT wiped: the fit is still there, just less confident.
    assert pipe.est.solve() is not None


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
      pipe.update(v, k_cmd, meas, False, False, False,
                  driver_torque=grip, a_ego=0.1,
                  low_factor=applied[0], high_factor=applied[1])
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
    assert all(l2 >= l1 - NUDGE_STEP - 1e-9 for l1, l2 in zip(lows, lows[1:])), lows


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
    "lane_change_factor_high_ang": float,
  }

  def __init__(self, values):
    self.values = values
    self.written = {}

  def get(self, key, return_default=False):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put(self, key, value):
    expected = self._TYPES.get(key)
    if expected is not None and not isinstance(value, expected):
      raise TypeError(f"Type mismatch while writing param {key}: got {type(value)}, expected {expected}")
    self.values[key] = value
    self.written[key] = value


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
    assert ext.autocal is None and not ext.autocal_enabled

  def test_arming_builds_pipeline_with_baseline(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.10", "FordHighSpeedFactor_ang": "0.95"})
    ext.update_angle_params(p)
    assert ext.autocal_enabled and ext.autocal is not None
    assert ext._autocal_last_written == ("1.10", "0.95")

  def test_arming_restores_serialized_evidence(self):
    donor = AutoCalPipeline(PLATFORM_GAIN_HIGH)
    feed_plant(donor.est, 1.05, 1.05, speeds=[10, 28], n_per_speed=200)
    state = json.dumps({"v": 1, "phase": "collecting", "pipe": donor.to_dict()})
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": state,
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    assert ext.autocal is not None
    assert ext.autocal.est.n == donor.est.n
    assert ext.autocal.est.solve() == donor.est.solve()

  def test_locked_json_never_arms(self):
    ext = self._ext()
    state = json.dumps({"v": 1, "phase": "locked", "pipe": {}})
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": state})
    ext.update_angle_params(p)
    assert ext.autocal is None and ext.autocal_done and not ext.autocal_enabled

  def test_legacy_done_state_never_arms(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "done low=1.02 high=1.15"})
    ext.update_angle_params(p)
    assert ext.autocal is None and ext.autocal_done and not ext.autocal_enabled

  def test_garbage_state_starts_fresh(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "round 3 collecting; applied"})
    ext.update_angle_params(p)
    assert ext.autocal is not None and ext.autocal.est.n == 0

  def test_nudge_writes_params_and_state(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    ext._autocal_apply_nudge((1.02, 1.15))
    # The fork's params are typed FLOAT — a string write raises and the nudge dies.
    assert p.written["FordLowSpeedFactor_ang"] == 1.02 and isinstance(p.written["FordLowSpeedFactor_ang"], float)
    assert p.written["FordHighSpeedFactor_ang"] == 1.15 and isinstance(p.written["FordHighSpeedFactor_ang"], float)
    st = json.loads(p.written["FordAngleAutoCalState"])
    assert st["phase"] == "collecting" and st["applied"] == {"low": 1.02, "high": 1.15}
    assert ext.low_speed_curv_factor == 1.02 and ext.high_speed_curv_factor == 1.15
    # Our own write must NOT read back as a user edit.
    self._tick(ext, p, n=2)
    assert ext.autocal is not None and not ext._autocal_edit_pending

  def test_user_edit_adopted_after_two_ticks(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    feed_plant(ext.autocal.est, 1.05, 1.05, speeds=[10, 28], n_per_speed=200)
    w0 = ext.autocal.est.s_w
    p.values["FordLowSpeedFactor_ang"] = "1.08"  # driver taps + in the menu
    self._tick(ext, p, n=1)   # first tick: pending
    assert ext._autocal_edit_pending and abs(ext.autocal.est.s_w - w0) < 1e-9
    self._tick(ext, p, n=1)   # second tick: confirmed
    assert not ext._autocal_edit_pending
    assert abs(ext.autocal.est.s_w - 0.5 * w0) < 1e-9  # soft reset, not a wipe
    assert ext._autocal_last_written == ("1.08", "1.00")

  def test_save_restore_round_trip_through_param(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    feed_plant(ext.autocal.est, 1.05, 1.05, speeds=[10, 28], n_per_speed=200)
    sol = ext.autocal.est.solve()
    ext._autocal_save("collecting")
    # New process, same params: evidence must come back.
    ext2 = self._ext()
    ext2.update_angle_params(p)
    assert ext2.autocal is not None
    assert ext2.autocal.est.solve() == sol

  def test_toggle_off_disarms(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.00", "FordHighSpeedFactor_ang": "1.00"})
    ext.update_angle_params(p)
    assert ext.autocal is not None
    p.values["FordAngleAutoCal"] = 0
    self._tick(ext, p, n=1)
    assert ext.autocal is None and not ext.autocal_enabled
