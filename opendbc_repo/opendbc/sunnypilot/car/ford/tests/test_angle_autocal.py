"""Tests for the one-time angle-mode factor auto-calibration (angle_autocal.py)."""
import random

import pytest

from opendbc.sunnypilot.car.ford.angle_autocal import (
  AngleFactorEstimator, AutoCalPipeline, SteadyStateGate, speed_alpha,
  V_LOW, V_HIGH, LOW_ANCHOR_BASE, STEADY_TIME_S, MIN_KAPPA,
  PRESS_HOLDBACK_S, PRESS_COOLDOWN_S, MAX_LAT_ACCEL,
)

PLATFORM_GAIN_HIGH = 1.05  # Mach-E


def simulate_plant(est, true_low_factor, true_high_factor, speeds, kappa=0.002, n_per_speed=200, noise=0.0):
  """Feed samples from a plant whose true gain corresponds to the given ideal factors.

  The plant's response ratio r = applied_gain / ideal_gain: if the applied factors already
  matched the true ones, r would be 1 everywhere.
  """
  rng = random.Random(42)
  for v in speeds:
    a = speed_alpha(v)
    ideal = (1.0 - a) * (LOW_ANCHOR_BASE * true_low_factor) + a * (PLATFORM_GAIN_HIGH * true_high_factor)
    for _ in range(n_per_speed):
      r = est.applied_gain(v) / ideal
      r *= 1.0 + (rng.uniform(-noise, noise) if noise else 0.0)
      est.add_sample(v, kappa, kappa * r, weight=0.05)


class TestAngleFactorEstimator:
  def test_recovers_true_factors(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    simulate_plant(est, true_low_factor=0.92, true_high_factor=1.21,
                   speeds=[10, 12, 15, 18, 21, 24, 27, 29], n_per_speed=200, noise=0.03)
    low, high, stats = est.solve()
    assert abs(low - 0.92) < 0.02, low
    assert abs(high - 1.21) < 0.02, high
    assert est.converged()

  def test_accounts_for_applied_factors(self):
    # A drive made with non-default factors must still recover the same truth.
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.10, 0.90)
    simulate_plant(est, true_low_factor=0.92, true_high_factor=1.21,
                   speeds=[10, 15, 20, 25, 29], n_per_speed=300, noise=0.03)
    low, high, _ = est.solve()
    assert abs(low - 0.92) < 0.02, low
    assert abs(high - 1.21) < 0.02, high

  def test_not_converged_with_one_sided_speeds(self):
    # Only low-speed driving: high anchor must not report convergence.
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    simulate_plant(est, 0.95, 1.10, speeds=[10, 11, 12], n_per_speed=400, noise=0.02)
    assert not est.converged()

  def test_rejects_bad_samples(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    assert not est.add_sample(20.0, 0.0005, 0.0005)   # below curvature threshold
    assert not est.add_sample(5.0, 0.002, 0.002)      # below speed threshold
    assert not est.add_sample(20.0, 0.002, -0.002)    # sign mismatch
    assert not est.add_sample(20.0, 0.002, 0.02)      # absurd ratio
    assert not est.add_sample(29.0, 0.004, 0.004)     # 3.4 m/s^2 lat accel — car may not make this turn
    assert est.n == 0
    assert est.add_sample(29.0, 0.0025, 0.0025)       # 2.1 m/s^2 — comfortably within tire limits

  def test_factor_clamp(self):
    est = AngleFactorEstimator(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    simulate_plant(est, 2.5, 0.2, speeds=[10, 20, 29], n_per_speed=100)
    low, high, _ = est.solve()
    assert low == 1.5 and high == 0.5  # clamped to the +/- button range


class TestSteadyStateGate:
  def test_requires_sustained_steady(self):
    gate = SteadyStateGate(dt=0.05)
    needed = int(STEADY_TIME_S / 0.05)
    results = [gate.update(True, MIN_KAPPA * 2, False, False, False, False, False)
               for _ in range(needed + 2)]
    assert not any(results[:needed - 1])
    assert results[-1]

  def test_resets_on_any_flag(self):
    gate = SteadyStateGate(dt=0.05)
    for _ in range(int(STEADY_TIME_S / 0.05) + 1):
      gate.update(True, MIN_KAPPA * 2, False, False, False, False, False)
    assert gate.update(True, MIN_KAPPA * 2, False, False, False, False, False)
    gate.update(True, MIN_KAPPA * 2, True, False, False, False, False)  # pressed
    assert gate.steady_s == 0.0

  def test_resets_on_curvature_jump(self):
    gate = SteadyStateGate(dt=0.05)
    for _ in range(int(STEADY_TIME_S / 0.05) + 1):
      gate.update(True, 0.002, False, False, False, False, False)
    assert gate.update(True, 0.002, False, False, False, False, False)
    assert not gate.update(True, 0.004, False, False, False, False, False)  # jump
    assert gate.steady_s == 0.0

  def test_saturation_blocks(self):
    gate = SteadyStateGate(dt=0.05)
    for _ in range(int(STEADY_TIME_S / 0.05) + 2):
      assert not gate.update(True, 0.002, False, False, False, False, False, saturated=True)

  def test_light_torque_starts_cooldown(self):
    gate = SteadyStateGate(dt=0.05)
    # Torque below the steeringPressed threshold but above the guard: hands are on.
    gate.update(True, 0.002, False, False, False, False, False, driver_torque=0.7)
    assert gate.grip_cooldown_s > 0.0
    # Cooldown blocks sampling for PRESS_COOLDOWN_S after the grip ends.
    blocked = int(PRESS_COOLDOWN_S / 0.05) - 1
    for _ in range(blocked):
      assert not gate.update(True, 0.002, False, False, False, False, False)


def run_pipeline(pipe, n, torque=0.0, pressed=False, saturated=False, kappa=0.002, v=20.0):
  committed = []
  for _ in range(n):
    committed += pipe.update(v, kappa, kappa, pressed, False, False, False, False,
                             saturated=saturated, driver_torque=torque)
  return committed


class TestAutoCalPipeline:
  def test_commits_after_holdback(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    warm = int((STEADY_TIME_S + PRESS_HOLDBACK_S) / 0.05) + 3
    committed = run_pipeline(pipe, warm)
    assert pipe.est.n > 0
    assert len(committed) == pipe.est.n

  def test_grip_cancels_staged_samples(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    # Reach eligibility, stage a few samples, but grip before the holdback elapses:
    warm = int(STEADY_TIME_S / 0.05) + 1 + int(PRESS_HOLDBACK_S / 0.05) // 2
    run_pipeline(pipe, warm)
    assert len(pipe._staged) > 0 and pipe.est.n == 0
    pipe.update(20.0, 0.002, 0.002, True, False, False, False, False)  # grip
    assert len(pipe._staged) == 0
    assert pipe.est.n == 0  # nothing from before the grip ever reached the estimator

  def test_light_torque_also_cancels(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    warm = int(STEADY_TIME_S / 0.05) + 5
    run_pipeline(pipe, warm)
    assert len(pipe._staged) > 0
    pipe.update(20.0, 0.002, 0.002, False, False, False, False, False, driver_torque=0.7)
    assert len(pipe._staged) == 0 and pipe.est.n == 0

  def test_saturated_frames_never_commit(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    committed = run_pipeline(pipe, 100, saturated=True)
    assert committed == [] and pipe.est.n == 0

  def test_idle_clears_staging(self):
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    run_pipeline(pipe, int(STEADY_TIME_S / 0.05) + 5)
    assert len(pipe._staged) > 0
    pipe.idle()
    assert len(pipe._staged) == 0 and pipe.gate.steady_s == 0.0

  def test_unsettled_measurement_not_staged(self):
    # Command steady but the car still converging toward it (closed-loop compensation
    # tail): those frames must not be sampled even though the command gate passes.
    pipe = AutoCalPipeline(PLATFORM_GAIN_HIGH, 1.0, 1.0)
    warm = int((STEADY_TIME_S + PRESS_HOLDBACK_S) / 0.05) + 10
    kappa_meas = 0.0010  # far from the 0.002 command, sweeping up fast
    staged_during_sweep = 0
    for _ in range(warm):
      pipe.update(20.0, 0.002, kappa_meas, False, False, False, False, False)
      if kappa_meas < 0.0019:
        kappa_meas += 0.0002  # 0.004/s sweep, far above the settle bound
        staged_during_sweep = len(pipe._staged) + pipe.est.n
    assert staged_during_sweep == 0  # nothing accepted while the car was still turning in
    assert pipe.est.n > 0            # but samples flow once the measurement settles


class _MockParams:
  """Duck-typed openpilot Params: just enough for update_angle_params."""
  def __init__(self, values):
    self.values = values
    self.written = {}

  def get(self, key, return_default=False):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put(self, key, value):
    self.written[key] = value


class TestOnboardGlue:
  """Exercise the REAL LateralAngleExt param/arming glue — the seam unit tests of the
  pipeline cannot see. This is the test that would have caught the autocal_gate
  AttributeError that crashed card on-device while every component test passed."""

  def _ext(self):
    messaging = pytest.importorskip("cereal.messaging")  # noqa: F841  (linux-only)
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt

    class _Harness(LateralAngleExt):
      # In the real CarController this comes from the LateralCurvExt mixin, where it is
      # a no-op compatibility shim (state is initialized eagerly). Same no-op here.
      def _ensure_lateral_curv_initialized(self, CP):
        pass

    ext = _Harness()
    class _CP:
      carFingerprint = "FORD_MUSTANG_MACH_E_MK1"
    ext.CP = _CP()
    return ext

  def test_param_glue_runs_without_error(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 0, "FordAngleAutoCalState": ""})
    for _ in range(205):  # spans two 100-call read cycles including the first-call read
      ext.update_angle_params(p)
    assert ext.autocal is None and not ext.autocal_enabled

  def test_arming_builds_pipeline(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "",
                     "FordLowSpeedFactor_ang": "1.10", "FordHighSpeedFactor_ang": "0.95"})
    ext.update_angle_params(p)
    assert ext.autocal_enabled and ext.autocal is not None
    assert abs(ext.autocal.est.low_factor_applied - 1.10) < 1e-6
    assert abs(ext.autocal.est.high_factor_applied - 0.95) < 1e-6
    assert ext._autocal_round == 1

  def test_done_state_never_arms(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1, "FordAngleAutoCalState": "done low=1.02 high=1.15"})
    ext.update_angle_params(p)
    assert ext.autocal is None and ext.autocal_done and not ext.autocal_enabled

  def test_round_number_survives_restart(self):
    ext = self._ext()
    p = _MockParams({"FordAngleAutoCal": 1,
                     "FordAngleAutoCalState": "round 3 collecting; applied low=1.05 high=1.10"})
    ext.update_angle_params(p)
    assert ext._autocal_round == 3 and ext.autocal_enabled
