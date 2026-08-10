"""
BluePilot: Ford CAN-FD path-angle–primary lateral control (developer).

Steering intent is c1 from κ → θ. Converts planner/model curvature into ``path_angle`` using
PSCM short lookahead d_ref and y ≈ ½κ x² ⇒ path_angle = ½ κ d_ref (see
``bluepilot/agent_info/20_FORD_PSCM_KNOWLEDGE_PACK.md``). Predicted curvature (modelV2) is
blended with ``actuators.curvature`` per ``FordPathAngleBlendRatio`` (0 = planner only,
1 = model only).

**c0 (path_offset) is always zero on the wire, unconditionally.** Angle mode has no centering
trim -- an earlier port attempt piped a small additive trim onto path_angle through the
curv-mode ``LC_PID_controller``, but it never actually tracked lane center correctly in this
mode and was removed; only the DBC-required zero c0 remains.

**Human-turn override**: while the driver manually turns (same sustained-press + angle criteria
as ``lateral_curv_ext``, via the shared ``HumanTurnDetector``), lateral is forced inactive (mode
0, all-zero signals) instead of winding path_angle into a stale command the PSCM has to reconcile
on release -- on the Mach-E's PSCM that reconciliation cost 2-3 s of dead time before control
resumed. Mode 0 is panda-clean by construction: every ford.h check has a legitimate
!steer_control_enabled branch, so no reset-bypass latch involvement. On release, path_angle ramps
back in from zero through the soft ROC below (no jump seed) -- generous at human-turn speeds, and
admitted by ford.h's path_angle ROC check (2% looser) without any bypass.
"""

import numpy as np
from numpy import clip, interp

from opendbc.car import DT_CTRL
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.angle_autocal import Frame
from opendbc.sunnypilot.car.ford.angle_autocal_controller import AutoCalController
from opendbc.sunnypilot.car.ford.angle_smoothing import AngleSmoother
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralResult
from opendbc.sunnypilot.car.ford.human_turn import HumanTurnDetector
from opendbc.sunnypilot.car.ford.values_ext import (BP_ANGLE_LIMITS, platform_gains,
                                                    V_LOW, V_HIGH, LOW_ANCHOR_BASE)
from selfdrive.modeld.constants import ModelConstants


# DBC ``LatCtlPath_An_Actl`` (rad) — panda safety uses the same in ``ford.h``; PSCM enforces in firmware.
FORD_DBC_PATH_ANGLE_MIN = -0.5
FORD_DBC_PATH_ANGLE_MAX = 0.5235

# Auto-cal state persistence cadence: losing a save costs at most this much evidence.

# --- Anti-weave smoothing (FordAngleSmoothing) -------------------------------------------
# All constants, semantics, and filter math live in angle_smoothing.AngleSmoother (pure,
# unit-tested). Menu 1.0 = stock/no smoothing, bit-identical to the toggle being off.


# PSCM d_ref (m) vs speed (m/s) — 6 points; above ~55.6 m/s use plateau + optional cap to 5 m.
_PSCM_DREF_SPEEDS_MS = (0.0, 4.17, 27.78, 41.67, 50.0, 55.56)
_PSCM_DREF_M = (0.5, 0.95, 1.4, 2.075, 2.75, 3.875)

# Default blend ratio validated on F-150 fleet data (0.5s lookup time).
_FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT = 0.50

# Variable lookup time (VLT): curvature_lookup_time adapts to speed and curvature magnitude.
# t_lookup = t_base + t_extra_max × speed_factor(v) × kappa_factor(|κ|)
# t_base = liveDelay.lateralDelay + DT_MDL — always matches the planner's pre-compensation floor.
# Extra lookahead collapses toward zero at high speed (PSCM responds faster)
# and at large curvature (prevents blend importing a "start unwinding" signal too early).
_DT_MDL = 0.05                       # model loop period (matches common/realtime.py)
_VLT_T_EXTRA_MAX = 0.10              # max extra lookahead above t_base
_VLT_V_LOW_MS   = 25.0 * 0.44704    # 25 mph — full extra lookahead at or below this speed
_VLT_V_HIGH_MS  = 55.0 * 0.44704    # 55 mph — no extra lookahead at or above this speed
_VLT_KAPPA_FULL  = 0.005             # 1/m — full extra lookahead below this curvature (200m+ radius)
_VLT_KAPPA_TAPER = 0.020             # 1/m — no extra lookahead above this curvature (50m radius)

# Rate cap on path_angle magnitude DECREASE during PSCM LimitReached (rad/call = 0.40 rad/s).
# Both model and planner naturally drop path_angle ~0.36 rad/s at a sharp 90° apex, while the PSCM is
# physically pinned and cannot execute the rapidly falling desired angle. The resulting actual-vs-desired
# gap (up to 47° observed) causes a snap correction the moment the PSCM is released. This cap limits
# the desired-angle drop rate to what the PSCM can reasonably track, at the cost of holding the car
# slightly more in the curve during saturation.
# BluePilot: this strategy runs once per STEER_STEP (CarControllerParams.STEER_STEP=5), i.e. once
# every 5th 100Hz control tick = 20Hz, not every tick. The original 0.004 rad/call value (and its
# "50Hz" comment, corrected above) was authored 2026-05-11 on bp-sid-simple, which had already
# switched STEER_STEP 5->1 (true 100Hz) on 2026-04-22 -- so it was tuned at 100Hz real cadence
# even though its own comment mistakenly said 50Hz. Scaled x5 here to restore the same real-world
# 0.40 rad/s (23 deg/s) unwind rate on this branch's actual 20Hz cadence.
_PSCM_SAT_UNWIND_RATE = 0.02        # rad/call (0.02 * 20Hz = 0.40 rad/s)

# Post-override stall blip. Road test 2026-07-14 (route 886240741b067740/000000bd--feb980680f)
# showed that after driver-touch episodes the Mach-E PSCM keeps reporting InProgress but honors
# path_angle at only ~0.56x (healthy hands-free delivery on the same route: ~0.95 median). The
# current-curvature deviation clip below then pins kappa_cmd at measured + CURVATURE_ERROR, so the
# command can never lead the car enough to overcome the attenuation -- a stall equilibrium the
# driver reads as "not engaging" (wire path_angle flat at ~4 deg for 4.5 s while desired kappa
# climbed to 3x measured, EPS motor current ~0 A). A short mode-0 pulse -- the identical
# panda-clean wire pattern the human-turn override sends, no ford.h involvement -- resets the
# PSCM's authority, after which path_angle ramps back in from zero through the soft ROC.
_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick (matches human_turn.py)
_STALL_GAP_MIN = 2.0 * CarControllerParams.CURVATURE_ERROR  # desired must lead measured by 2x the clip tolerance
_STALL_HOLD_S = 0.5          # accumulated clip-binding time before a pulse fires
_STALL_BLIP_FRAMES = 6       # mode-0 pulse length (6 frames @ 20 Hz = 300 ms; PSCM acked mode 0 in ~150 ms on-road)
_STALL_COOLDOWN_S = 2.0      # re-arm delay after a pulse (release ramp + PSCM response time)
_STALL_MAX_BLIPS = 3         # give up on a stuck episode; devLim telemetry keeps recording the stall
# Proactive hand-off blip: any sustained driver press attenuates the PSCM (route 000000be seg 4:
# 3 s of sub-45-deg circle-exit steering left it at ~0x delivery, and the reactive detector's
# fire-after-the-stall-develops timing meant 2.4 s of dead-straight running into the next curve
# before the pulse landed). Firing the same pulse on the falling edge of a sustained press resets
# the PSCM while the car is straight and the command is small -- a 300 ms lateral gap right at
# hand-off, imperceptible, instead of a missed curve. The reactive detector above stays as backstop.
_PRESS_BLIP_MIN_S = 0.5      # press must last this long before its release earns a pulse


def pscm_d_ref_m(v_ego_ms: float) -> float:
  v = max(float(v_ego_ms), 0.0)
  d = float(np.interp(v, _PSCM_DREF_SPEEDS_MS, _PSCM_DREF_M))
  if v > _PSCM_DREF_SPEEDS_MS[-1]:
    # Doc: d_ref table ends at 3.875 m; contribution saturates for high speed — cap at 5 m.
    d = min(5.0, d)
  return d


class LateralAngleExt:
  def __init__(self, CP=None, CP_SP=None):
    # Predicted-curvature blend for path_angle: pred * b + desired * (1-b); b from ``FordPathAngleBlendRatio``
    self.path_angle_blend_ratio = _FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT
    # Max extra VLT above t_base; from ``FordVLTExtraMax`` param
    self.vlt_extra_max = _VLT_T_EXTRA_MAX
    # Telemetry: final path_angle (rad) after limits (see bp_card_publisher)
    self.bp_path_angle_final = 0.0
    # High-speed gain factors: set per-platform via carFingerprint in update_angle_params.
    self.path_angle_gain_lowC_highV = 1.0   # dampening at high speed, low curvature
    self.path_angle_gain_highC_highV = 1.0  # gain at high speed, high curvature
    self.bp_path_angle_gain_lowC_highV = 1.0
    self.bp_path_angle_gain_highC_highV = 1.0
    # User-tunable "feel" multipliers: read from the angle-tuning Params below.
    self.low_speed_curv_factor = 1.0
    self.high_speed_curv_factor = 1.0
    self.user_dampening_factor = 1.0
    self.bp_low_speed_curv_factor = 1.0
    self.bp_high_speed_curv_factor = 1.0
    # BluePilot: angle mode's own lane-change scaling factor, independent of curvature mode's
    # lane_change_factor_high_curv -- angle needs a boost (>1) where curvature needs a cut (<1).
    self.lane_change_factor_high_ang = 1.0
    # Telemetry: variable curvature lookup time used this frame (s)
    self.bp_curvature_lookup_time = _VLT_T_EXTRA_MAX + 0.3725  # warm start at ~0.5s
    # BluePilot: error-clipped kappa path_angle was derived from -- carcontroller.py reads this as
    # shadow_curvature for ford.h's angle-mode deviation check. Actively consumed, not telemetry.
    self.bp_kappa_cmd = 0.0
    # BluePilot: rate-limit diagnostics (controllerStateBP)
    self.bp_angle_rate_limited = False      # path_angle soft-ROC clip actually bit this frame
    self.bp_curvature_rate_limited = False  # equivalent curvature would be rate-limited by curv-mode logic (sim)
    self.bp_curvature_deviation_limited = False  # current_curvature error-clip constrained kappa_cmd this frame
    self.sim_curvature_last = 0.0           # shadow curvature-mode last for the curvatureRateLimited sim
    # Exit detection: track previous desired curvature to sense when planner is actively reducing
    self._desired_curvature_last = 0.0
    # Human-turn override: while the driver manually turns, lateral is forced inactive (mode 0,
    # all-zero signals) instead of winding path_angle into a stale command the PSCM can't cleanly
    # reconcile on release (2-3 s re-engage dead time observed on Mach-E). See module docstring.
    # Note: in CarController this attribute is shared with LateralCurvExt (same mixin instance) --
    # only one lateral strategy runs per frame, so a single detector serves both.
    self.human_turn_detector = HumanTurnDetector()
    self.angle_human_turn_active = False  # read by carcontroller to force mode 0
    # Post-override stall blip state (see module constants). angle_stall_blip_active is read by
    # carcontroller to force mode 0, exactly like angle_human_turn_active.
    self.stall_blip_hold_s = 0.0      # accumulated deviation-clip-binding time toward a pulse
    self.stall_blip_frames_left = 0   # remaining pulse frames; > 0 -> mode 0 on the wire
    self.stall_blip_cooldown_s = 0.0  # re-arm delay after a pulse
    self.stall_blip_count = 0         # pulses fired this stall episode
    self.angle_stall_blip_active = False
    self.press_timer_s = 0.0          # continuous steeringPressed time, for the hand-off blip
    # BluePilot: continuous auto-calibration of the speed factors. The pure estimator lives
    # in angle_autocal.py; ALL lifecycle (arm/disarm, JSON persistence, user-edit debounce,
    # nudge writes, save cadence, errors, telemetry status) lives in AutoCalController —
    # this class only routes frames and adopts returned nudges.
    self.autocal_ctl = AutoCalController(dt=_STEER_DT)
    self._autocal_param_ctr = 100  # >= threshold so the very first call reads params (also gates smoothing reads)
    # BluePilot: anti-weave smoothing (FordAngleSmoothing; see angle_smoothing.py).
    self.smoother = AngleSmoother(dt=_STEER_DT)
    # Telemetry + autocal gate: the command this frame was modified by PSCM authority
    # limits or the DBC clamp — the car could not make the requested turn.
    self.bp_angle_saturated = False


  # -- smoothing compat surface (offline replay tooling sets these directly) ---------------
  @property
  def smoothing_enabled(self) -> bool:
    return self.smoother.enabled

  @smoothing_enabled.setter
  def smoothing_enabled(self, v: bool):
    self.smoother.enabled = bool(v)

  @property
  def smoothing_strength(self) -> float:
    return self.smoother.strength

  @smoothing_strength.setter
  def smoothing_strength(self, v: float):
    self.smoother.strength = float(v)

  def update_angle_params(self, params):
    """Sets per-platform gain defaults and reads user angle-tuning params."""
    self._ensure_lateral_curv_initialized(self.CP)
    fp = getattr(self.CP, 'carFingerprint', '')
    low, high = platform_gains(fp)
    self.path_angle_gain_lowC_highV = low
    self.path_angle_gain_highC_highV = high
    if params is not None and hasattr(params, "get"):
      for attr, key, min_value, max_value in (
        ("low_speed_curv_factor", "FordLowSpeedFactor_ang", 0.5, 1.5),
        ("high_speed_curv_factor", "FordHighSpeedFactor_ang", 0.5, 1.5),
        ("user_dampening_factor", "FordHighSpeedDampening_ang", 0.75, 1.25),
      ):
        try:
          raw = params.get(key, return_default=True)
          if raw is not None and raw != b"":
            setattr(self, attr, float(clip(
              float(raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw), min_value, max_value)))
        except Exception:
          pass
      try:
        raw = params.get("lane_change_factor_high_ang", return_default=True)
        if raw is not None and raw != b"":
          self.lane_change_factor_high_ang = float(clip(
            float(raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw), 0.85, 1.50))
      except Exception:
        pass
      # BluePilot: auto-calibration arm/disarm (checked ~1 Hz; this method runs at 100 Hz)
      self._autocal_param_ctr += 1
      if self._autocal_param_ctr >= 100:
        self._autocal_param_ctr = 0
        try:
          _sm_enabled = bool(params.get_bool("FordAngleSmoothing"))
          raw_strength = params.get("FordAngleSmoothStrength", return_default=True)
          _menu = 1.0 + self.smoother.strength  # keep current on unreadable/empty
          if raw_strength is not None and raw_strength != b"":
            _menu = float(
              raw_strength.decode("utf-8", errors="replace") if isinstance(raw_strength, bytes) else raw_strength)
          self.smoother.configure(_sm_enabled, _menu)
        except Exception:
          pass  # keep the previous values; defaults are enabled / 1.0
        self.autocal_ctl.poll_params(params, self.low_speed_curv_factor,
                                     self.high_speed_curv_factor,
                                     self.path_angle_gain_highC_highV)

  # -- auto-cal telemetry surface (bp_card_publisher reads these off the carcontroller) ----
  @property
  def autocal_enabled(self) -> bool:
    return self.autocal_ctl.enabled

  @property
  def bp_autocal_status(self) -> str:
    return self.autocal_ctl.status

  def _feed_autocal(self, CS, kappa_cmd: float, kappa_meas: float):
    """Build one evidence Frame from the car signals + this frame's limiter flags and hand
    it to the controller. Frame construction (and its signal reads) happens only while the
    calibrator is armed — for everyone else this is one attribute check per frame."""
    if not self.autocal_ctl.enabled:
      return
    ws = CS.out.wheelSpeeds
    ws_vals = (float(ws.fl), float(ws.fr), float(ws.rl), float(ws.rr))
    self.autocal_ctl.feed(
      Frame(v_ego=float(CS.out.vEgoRaw), kappa_cmd=kappa_cmd, kappa_meas=kappa_meas,
            steering_pressed=bool(CS.out.steeringPressed),
            angle_rate_limited=self.bp_angle_rate_limited,
            deviation_limited=self.bp_curvature_deviation_limited,
            saturated=self.bp_angle_saturated,
            driver_torque=float(CS.out.steeringTorque), a_ego=float(CS.out.aEgo),
            ws_spread=max(ws_vals) - min(ws_vals),
            low_factor=self.low_speed_curv_factor, high_factor=self.high_speed_curv_factor,
            lateral_delay=float(self.sm['liveDelay'].lateralDelay)),
      delay_estimated=str(self.sm['liveDelay'].status) == "estimated")

  def _reset_angle_signals(self, CS):
    """Clear wire/telemetry state and the calibration + smoothing filters. Shared by the
    three branches that drop lateral to mode 0 (inactive, human-turn, stall-blip)."""
    self.path_angle_last = 0.0
    self.bp_path_angle_final = 0.0
    self.apply_curvature_last = 0.0
    self.bp_angle_rate_limited = False
    self.bp_curvature_rate_limited = False
    self.bp_curvature_deviation_limited = False
    self.sim_curvature_last = 0.0
    # Shadow tracks measured curvature while inactive: ford.h latches it from every LKA
    # frame, so a stale zero would fail the deviation check on the first re-engage frame.
    self.bp_kappa_cmd = self.get_current_curvature(CS)
    self.precision_type = 1
    self.bp_angle_saturated = False
    self.autocal_ctl.idle()  # steady/staged evidence must not span a lateral discontinuity
    self.smoother.reset()

  @staticmethod
  def _inactive_result() -> LateralResult:
    """All-zero mode-0 result the three inactive branches return."""
    return LateralResult(apply_curvature=0.0, curvature_rate=0.0, path_offset=0.0,
                         path_angle=0.0, ramp_type=0, precision_type=1, lateralUncertainty=0.0)

  def update_angle_strategy(self, CC, CS, actuators, CP):
    """
    Curvature from planner (+ optional predicted blend) → path_angle via ½·κ·d_ref.
    c0 (path_offset) is always zero on the wire -- no centering trim in angle mode. c2 and c3 are zero.
    Blended κ is not passed through Ford c2 rate / DBC limits (those target the curvature actuator).
    """
    self._ensure_lateral_curv_initialized(CP)

    v_ego = float(CS.out.vEgoRaw)
    d_ref = pscm_d_ref_m(v_ego)

    curvature_rate = 0.0
    path_offset = 0.0
    path_angle = 0.0
    ramp_type = 0
    lateral_uncertainty = 0.0
    precision = 1

    if not CC.latActive:
      self._reset_angle_signals(CS)
      self.human_turn_detector.reset()
      self.angle_human_turn_active = False
      self.stall_blip_hold_s = 0.0
      self.stall_blip_frames_left = 0
      self.stall_blip_cooldown_s = 0.0
      self.stall_blip_count = 0
      self.angle_stall_blip_active = False
      self.press_timer_s = 0.0
      return self._inactive_result()

    # Human-turn override: sustained driver press + large wheel angle → force lateral inactive
    # (carcontroller drops mode to 0; all signals are zero on the wire) so path_angle can't wind
    # into a stale command while the driver turns. Always on in angle mode (no param gate) -- the
    # curv-suffixed human-turn toggle belongs to curvature mode's reset strategy, and the Mach-E
    # PSCM re-engage stall this prevents is not something a user should be able to opt out of.
    # On release, no jump seed: path_angle_last is 0, so the normal flow below ramps the command
    # back in through the soft ROC -- generous at human-turn speeds, no panda bypass involved.
    self.angle_human_turn_active = self.human_turn_detector.update(
      True, CS.out.steeringPressed, CS.out.steeringAngleDeg)
    if self.angle_human_turn_active:
      self._reset_angle_signals(CS)
      # Keep exit detection current so resume doesn't compare against a stale pre-turn value.
      self._desired_curvature_last = float(actuators.curvature)
      # A human turn's own mode 0 does the PSCM reset job; only press time after the latch
      # releases should earn a hand-off pulse, so clear the stall/press state here.
      self.stall_blip_hold_s = 0.0
      self.stall_blip_frames_left = 0
      self.stall_blip_cooldown_s = 0.0
      self.stall_blip_count = 0
      self.angle_stall_blip_active = False
      self.press_timer_s = 0.0
      return self._inactive_result()

    # Proactive hand-off blip: the falling edge of a sustained press earns an immediate mode-0
    # pulse (see _PRESS_BLIP_MIN_S) -- resets the PSCM's press-induced attenuation right at
    # hand-off, while the car is straight and the command small, instead of waiting for the
    # reactive stall detector below to watch the car miss the next curve first.
    if CS.out.steeringPressed:
      self.press_timer_s += _STEER_DT
    else:
      if (self.press_timer_s >= _PRESS_BLIP_MIN_S and self.stall_blip_cooldown_s <= 0.0
          and self.stall_blip_frames_left <= 0):
        self.stall_blip_frames_left = _STALL_BLIP_FRAMES
      self.press_timer_s = 0.0

    # Stall-blip pulse in progress: hold lateral inactive (mode 0, all-zero signals -- the same
    # wire pattern as the human-turn override, no ford.h involvement) for _STALL_BLIP_FRAMES so the
    # PSCM drops its post-override attenuation, then release; path_angle ramps back in from zero
    # through the soft ROC exactly like a human-turn release. Detection lives at the end of the
    # normal flow below.
    if self.stall_blip_frames_left > 0:
      self.stall_blip_frames_left -= 1
      self.angle_stall_blip_active = True
      self._reset_angle_signals(CS)
      self._desired_curvature_last = float(actuators.curvature)
      if self.stall_blip_frames_left <= 0:
        self.stall_blip_cooldown_s = _STALL_COOLDOWN_S
      return self._inactive_result()
    self.angle_stall_blip_active = False

    self.precision_type = 1
    precision = 1
    LP = self.lp
    desired_curvature = float(actuators.curvature)

    # Variable lookup time — delay compensation is SPLIT across two horizons (2026-07-23):
    #  - _t_entering (liveDelay capped 0.15s): the horizon for the entering/exiting DECISION
    #    only. The cap is load-bearing for apexes: a deeper decision horizon keeps
    #    kappa_entering True through the apex — the 0.42s-liveDelay era pathology where the
    #    exit-biased blend never engaged and the command flat-lined at max through the apex.
    #    A replay regression across 308 real apexes (routes 0a/0b/12/1b) showed even a
    #    0.25s decision horizon regresses 7.5% of them, so this horizon stays short and the
    #    apex behavior stays identical by construction.
    #  - _t_base (liveDelay capped 0.30s): the model-prediction LEAD in the blend. The true
    #    actuation delay is ~0.29s (liveDelay median, confirmed by command/measurement
    #    cross-correlation on three drives); compensating only 0.15s of it left ~0.14s of
    #    known-but-ignored delay in the loop, driving a ~0.2 Hz closed-loop breathing
    #    (±0.5° at the wheel, in curves and straights alike — measured desired-osc 0.15,
    #    actual-osc 0.21 mrad/m, actual trailing desired by exactly the actuation delay).
    #    Exits stay protected regardless of this deeper lead: the exit-biased blend
    #    collapses the prediction weight to ~15% there.
    _t_entering = float(clip(self.sm['liveDelay'].lateralDelay, 0.1, 0.15)) + _DT_MDL
    _t_base = float(clip(self.sm['liveDelay'].lateralDelay, 0.1, 0.30)) + _DT_MDL
    _speed_factor = float(interp(v_ego, [_VLT_V_LOW_MS, _VLT_V_HIGH_MS], [1.0, 0.0]))
    # Direction-aware kappa factor: on curve ENTRY (model shows more curvature at t_base than planner now),
    # keep full lookahead so pre-steering begins early. On exit/apex, taper by magnitude to prevent unwind.
    _kappa_at_entering = 0.0
    if self.model is not None and len(self.model.orientationRate.z) >= 17:
      _curvatures_ref = np.array(self.model.orientationRate.z) / max(0.01, v_ego)
      # Decision horizon (_t_entering, short by design) — NOT the blend lead horizon.
      _kappa_at_entering = abs(float(interp(_t_entering, ModelConstants.T_IDXS, _curvatures_ref)))
    # Anti-weave: hysteresis so noise straddling the entering/exiting boundary can't flip
    # this boolean (and with it the exit-blend gate) frame to frame near zero curvature.
    _kappa_entering = self.smoother.entering(_kappa_at_entering - abs(desired_curvature),
                                             _kappa_at_entering > abs(desired_curvature))
    if _kappa_entering:
      _kappa_factor = 1.0  # curve deepening ahead: full extra lookahead for gradual entry
    else:
      _kappa_factor = float(interp(abs(desired_curvature), [_VLT_KAPPA_FULL, _VLT_KAPPA_TAPER], [1.0, 0.0]))
    curvature_lookup_time = _t_base + self.vlt_extra_max * _speed_factor * _kappa_factor
    self.bp_curvature_lookup_time = curvature_lookup_time

    predicted_curvature = 0.0
    if self.model is not None and len(self.model.orientationRate.z) >= 17:
      curvatures = np.array(self.model.orientationRate.z) / max(0.01, v_ego)
      predicted_curvature = float(
        interp(curvature_lookup_time, ModelConstants.T_IDXS, curvatures)
      )
    # Anti-weave: low-pass the model prediction to strip frame-to-frame jitter (details
    # in angle_smoothing.prediction — inside the VLT's slack, so no curve-entry cost).
    predicted_curvature = self.smoother.prediction(predicted_curvature)

    b = float(self.path_angle_blend_ratio)
    b = float(clip(b, 0.0, 1.0))

    # Exit-biased blend: near the PSCM authority limit or while the planner is actively
    # reducing curvature (exit detected), drop model prediction weight from 60% → ~15%.
    # This lets the planner's natural unwind dominate instead of being diluted by a model
    # prediction that still sees the curve (→ seg-14 slow unwind) or that snaps when its
    # lookahead window crosses the curve exit (→ seg-17 snap + reverse PSCM hit).
    # Normal gentle curves are unaffected: no PSCM limit, no falling desired → full b=0.60.
    _pscm_lim = getattr(CS, 'lat_ctl_lim_stat', 0)
    # In angle mode, LatCtlLim_D_Stat (→ lat_ctl_lim_stat) does not fire.
    # Previously used angleState.saturated (CtrSat) as a proxy, but CtrSat fires whenever the car
    # lags the commanded path_angle by > 2.5° — which happens during any normal curve entry.
    # That caused a positive-feedback flat-line: under-steer → CtrSat → path_angle frozen → more under-steer.
    # Use DBC-limit proximity instead: only block when path_angle is already near the ±0.5 rad CAN limits,
    # which is the only condition where the anti-snap unwind rate cap makes physical sense.
    _dbc_sat = (self.path_angle_last >= FORD_DBC_PATH_ANGLE_MAX * 0.90 or
                self.path_angle_last <= FORD_DBC_PATH_ANGLE_MIN * 0.90)
    _in_hard_sat = _pscm_lim >= 2 or _dbc_sat
    # BluePilot: per-call delta threshold. The original 0.002 was authored 2026-05-07 on
    # bp-sid-simple (9c3d000fd), which ran STEER_STEP=1 (true 100Hz, switched 2026-04-22) -- so it
    # was tuned to trigger on planner unwind faster than 0.2 (1/m)/s. Scaled x5 here to restore
    # that same real-world trigger rate on this branch's actual 20Hz cadence; unscaled it fired at
    # 0.04 (1/m)/s, collapsing the model blend on mild straightening instead of genuine exits.
    # Same bug class and fix as _PSCM_SAT_UNWIND_RATE and _soft_roc above.
    _desired_falling = abs(desired_curvature) < abs(self._desired_curvature_last) - 0.010
    _on_exit_near_limit = not _kappa_entering and (_pscm_lim >= 1 or _in_hard_sat or _desired_falling)
    _b_target = float(clip(b * 0.25, 0.0, 1.0)) if _on_exit_near_limit else b
    # Anti-weave: slew instead of stepping the exit blend (bounded ramps, no 4x steps).
    b_blend = self.smoother.blend(_b_target)
    requested_curvature = predicted_curvature * b_blend + desired_curvature * (1.0 - b_blend)
    self._desired_curvature_last = desired_curvature

    if self.model is not None:
      self.lane_change = self.model.meta.laneChangeState in (1, 2, 3)
    else:
      self.lane_change = False

    lane_change_factor = interp(
      v_ego, self.lane_change_factor_bp, [self.lane_change_factor_low, self.lane_change_factor_high_ang]
    )
    if self.lane_change and self.model is not None:
      if self.model.meta.laneChangeDirection == 1 and requested_curvature < 0:
        requested_curvature *= lane_change_factor
        precision = 0
      elif self.model.meta.laneChangeDirection == 2 and requested_curvature > 0:
        requested_curvature *= lane_change_factor
        precision = 0
    self.precision_type = precision

    # Use planner / predicted κ directly for the κ → path_angle map; we are not sending κ on CAN.
    kappa_cmd = float(requested_curvature)

    # BluePilot: clip kappa_cmd to current_curvature (measured, from yaw rate) +- CURVATURE_ERROR,
    # mirroring lateral_curv_ext.py's apply_ford_curvature_limits_ext exactly (same formula, same
    # v_ego > 9 gate, same CarControllerParams.CURVATURE_ERROR tolerance). Without this, kappa_cmd
    # (and therefore path_angle, and the shadow_curvature sent to ford.h) can legitimately lead the
    # measured curvature by more than ford.h's angle-error tolerance during normal curve entry/exit
    # -- the shadow-curvature deviation check (ford_shadow_curvature_error_check) would then block
    # routinely, not just on genuine pothole/override divergence. Curvature mode has always clipped
    # here; this brings angle mode's actual steering intent in line with that proven behavior rather
    # than only clipping the value reported to panda (which would make the check a no-op).
    current_curvature = self.get_current_curvature(CS)
    self.bp_curvature_deviation_limited = False
    if v_ego > 9:
      _kappa_cmd_pre_error_clip = kappa_cmd
      kappa_cmd = float(clip(kappa_cmd, current_curvature - CarControllerParams.CURVATURE_ERROR,
                            current_curvature + CarControllerParams.CURVATURE_ERROR))
      # BluePilot: did this clip actually constrain kappa_cmd this frame (deviation from measured,
      # not rate-of-change -- see carcontroller.py)?
      self.bp_curvature_deviation_limited = bool(abs(kappa_cmd - _kappa_cmd_pre_error_clip) > 1e-9)

    lateral_uncertainty = 0.0  # no curvature-limit ladder until angle-mode torque display is defined



    # Speed-interpolated gain: at low speed both curves use 1.0; at high speed the params take effect.
    self.low_gain_calc = interp(v_ego, [V_LOW, V_HIGH],
                                [1.0, self.path_angle_gain_lowC_highV * self.user_dampening_factor])
    self.high_gain_calc = interp(v_ego, [V_LOW, V_HIGH],
                                 [(LOW_ANCHOR_BASE * self.low_speed_curv_factor),
                                  (self.path_angle_gain_highC_highV * self.high_speed_curv_factor)])

    # As the curve grows the signal needs a boost to not understeer. The smoother's
    # asymmetric filter on |kappa| is the PRIMARY anti-weave fix (see angle_smoothing.py).
    # COUPLING: the interp knee top (0.001) is auto-cal's MIN_KAPPA — the calibrator only
    # samples fully inside the high branch. If this band moves, MIN_KAPPA moves with it.
    _kappa_for_gain = self.smoother.kappa_schedule(abs(kappa_cmd))
    self.curvature_factor = interp(_kappa_for_gain, [0.0007, 0.001], [self.low_gain_calc, self.high_gain_calc])

    path_angle_calc = kappa_cmd * v_ego * self.curvature_factor
    path_angle = path_angle_calc


    # PSCM authority limit clamp.
    # On CANFD Fords in angle mode, LatCtlLim_D_Stat does not fire, so _pscm_lim stays 0.
    # _in_hard_sat (computed above) combines _pscm_lim >= 2 with _dbc_sat (path_angle near ±0.5 rad limit).
    # LimitClose (_pscm_lim >= 1 only): block magnitude increases — exit-biased blend provides unwind.
    # Hard saturation (_in_hard_sat): block increases AND rate-limit decreases to _PSCM_SAT_UNWIND_RATE.
    #   Without the decrease cap, model+planner drop path_angle at ~0.36 rad/s at a sharp apex,
    #   driving desired steering 30°+ ahead of actual while the PSCM is pinned, causing a snap when released.
    if _in_hard_sat:
      _last = self.path_angle_last
      _last_mag = abs(_last)
      _curr_mag = abs(path_angle)
      if _curr_mag > _last_mag:  # magnitude growing — block
        path_angle = _last
      elif _last_mag - _curr_mag > _PSCM_SAT_UNWIND_RATE:  # decreasing too fast — rate-limit
        _limited_mag = _last_mag - _PSCM_SAT_UNWIND_RATE
        path_angle = float(_limited_mag if _last >= 0 else -_limited_mag)
    elif _pscm_lim >= 1:  # LimitClose (F150/non-angle-mode only): block increases only
      path_angle = float(clip(path_angle, -abs(self.path_angle_last), abs(self.path_angle_last)))

    _pre_dbc_clamp = path_angle
    path_angle = min(FORD_DBC_PATH_ANGLE_MAX, max(FORD_DBC_PATH_ANGLE_MIN, path_angle))
    # BluePilot: the car cannot make the requested turn this frame — PSCM authority limit
    # active or the DBC clamp bit. Telemetry + a hard no-sample gate for the auto-calibration.
    self.bp_angle_saturated = bool(_in_hard_sat or _pscm_lim >= 1 or path_angle != _pre_dbc_clamp)

    # Soft ROC limit — unconditional, slightly tighter than ford.h, applied before the
    # hardware bypass in ford.h is re-enabled.  Lets us observe whether the limit would
    # suppress control and tune it, while the PSCM still receives the clipped value.
    # BluePilot: this strategy runs once per STEER_STEP (CarControllerParams.STEER_STEP=5), i.e.
    # once every 5th 100Hz control tick = 20Hz, not every tick -- ported "verbatim from bp-sid-simple"
    # (2026-06-13), which runs STEER_STEP=1 (true 100Hz, switched 2026-04-22). The y-values below are
    # scaled x5 from the original [0.011, 0.011, 0.0085, 0.0018] to restore the same real-world rate
    # (63/63/49/10 deg/s at v=9-10/15/25) on this branch's actual 20Hz cadence. See ford.h's
    # FORD_PATH_ANGLE_LIMITS, which must mirror this scaling (x1.02 looser) to stay a true backstop.
    _soft_roc = float(interp(v_ego, [9., 10., 15., 25.], [0.055, 0.055, 0.0425, 0.009]))
    _path_angle_pre_roc = path_angle
    path_angle = float(clip(path_angle,
                            self.path_angle_last - _soft_roc,
                            self.path_angle_last + _soft_roc))
    # BluePilot: did the soft ROC clip actually limit the path_angle we wanted to send this frame?
    self.bp_angle_rate_limited = bool(abs(path_angle - _path_angle_pre_roc) > 1e-9)

    # Anti-weave: 1-LSB wire hold on the outgoing path_angle (kills LSB dither; held
    # frames are zero-ROC and cannot trip panda — details in angle_smoothing.wire).
    path_angle = self.smoother.wire(path_angle)


    # c0 always zero -- no centering trim in angle mode.
    path_offset = 0.0

    # Telemetry / state
    self.bp_path_angle_gain_lowC_highV = self.path_angle_gain_lowC_highV
    self.bp_path_angle_gain_highC_highV = self.path_angle_gain_highC_highV
    self.bp_low_speed_curv_factor = self.low_speed_curv_factor
    self.bp_high_speed_curv_factor = self.high_speed_curv_factor
    self.path_angle_last = path_angle
    self.bp_path_angle_final = path_angle
    self.apply_curvature_last = 0.0
    # BluePilot: the error-clipped kappa path_angle was derived from -- carcontroller.py reads this
    # as shadow_curvature for ford.h's angle-mode deviation check (see fordcan_ext.create_lka_msg).
    # Not just telemetry: an actively-consumed value, unlike the removed *_kappa_cmd_raw stubs.
    # While the driver is pressing (before the human-turn override latches), the clipped planner
    # kappa can't follow the wheel: the driver moves the measured curvature faster than the
    # deviation clip tracks it, so the shadow can exit ford.h's error band mid-curve -- the one
    # in-drive lateral safety block observed across ~3h of replayed road-test routes was exactly
    # this (driver fighting a sustained curve with the mode still enabled). The honest command
    # during a press is the driver's actual curvature.
    self.bp_kappa_cmd = self.get_current_curvature(CS) if CS.out.steeringPressed else kappa_cmd

    # BluePilot: would the equivalent curvature (kappa_cmd) have been rate-limited by curvature-mode's
    # ROC (apply_std_steer_angle_limits)? kappa_cmd is already error-clipped above (same clip
    # curvature mode applies), so only the rate-of-change portion remains to simulate here.
    _equiv_curv_rl = apply_std_steer_angle_limits(kappa_cmd, self.sim_curvature_last, v_ego,
                                                  CS.out.steeringAngleDeg, CC.latActive, BP_ANGLE_LIMITS)
    self.bp_curvature_rate_limited = bool(abs(_equiv_curv_rl - kappa_cmd) > 1e-9)
    self.sim_curvature_last = float(_equiv_curv_rl)

    # Post-override stall detection (mechanism in the module constants' comment). Fires the mode-0
    # blip when, hands-free, desired curvature has led measured by more than 2x the deviation
    # clip's tolerance while the clip was actually binding for _STALL_HOLD_S accumulated seconds.
    # devLim flickers mid-stall (~63% duty on the diagnosis route), so off frames hold the
    # accumulator rather than resetting it; a closed gap or driver press ends the episode.
    self.stall_blip_cooldown_s = max(0.0, self.stall_blip_cooldown_s - _STEER_DT)
    _stall_gap = desired_curvature - current_curvature
    _stalled = (not CS.out.steeringPressed and not self.lane_change and v_ego > 9.0
                and abs(_stall_gap) > _STALL_GAP_MIN
                and abs(desired_curvature) > abs(current_curvature))
    if _stalled:
      if self.bp_curvature_deviation_limited and self.stall_blip_cooldown_s <= 0.0:
        self.stall_blip_hold_s += _STEER_DT
      if self.stall_blip_hold_s >= _STALL_HOLD_S and self.stall_blip_count < _STALL_MAX_BLIPS:
        self.stall_blip_frames_left = _STALL_BLIP_FRAMES
        self.stall_blip_hold_s = 0.0
        self.stall_blip_count += 1
    else:
      self.stall_blip_hold_s = 0.0
      if CS.out.steeringPressed or abs(_stall_gap) < 0.5 * _STALL_GAP_MIN:
        self.stall_blip_count = 0  # episode over: the car is tracking again or the driver took it

    ramp_type = 2

    # Continuous auto-calibration of the speed factors (armed-only; a no-op otherwise).
    # Nudges are written to the factor params — update_angle_params reads them back, so the
    # factors have a single owner here. Human-turn/stall-blip frames never reach this point.
    self._feed_autocal(CS, kappa_cmd, current_curvature)

    return LateralResult(
      apply_curvature=0.0,
      curvature_rate=curvature_rate,
      path_offset=path_offset,
      path_angle=path_angle,
      ramp_type=ramp_type,
      precision_type=self.precision_type,
      lateralUncertainty=lateral_uncertainty,
    )

