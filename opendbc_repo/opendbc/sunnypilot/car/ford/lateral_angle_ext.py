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
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralResult
from opendbc.sunnypilot.car.ford.human_turn import HumanTurnDetector
from opendbc.sunnypilot.car.ford.values_ext import BP_ANGLE_LIMITS
from selfdrive.modeld.constants import ModelConstants

# Hard-coded per-platform gain defaults (not user-tunable).
# CAN vehicles (Escape MK4, Bronco Sport, Explorer, Maverick, Edge)
_GAIN_CAN         = (1.00, 1.15)
# CAN-FD body-on-frame trucks (F-150, Lightning, Expedition, Ranger)
_GAIN_CANFD_BOF   = (0.95, 0.95)
# CAN-FD unibody SUVs (Mustang Mach-E, Escape MK4.5)
_GAIN_CANFD_SUV   = (1.00, 1.05)

_CANFD_BOF_CARS = frozenset({
  CAR.FORD_F_150_MK14,
  CAR.FORD_F_150_LIGHTNING_MK1,
  CAR.FORD_EXPEDITION_MK4,
  CAR.FORD_RANGER_MK2,
})
_CANFD_SUV_CARS = frozenset({
  CAR.FORD_MUSTANG_MACH_E_MK1,
  CAR.FORD_ESCAPE_MK4_5,
})

# BluePilot: low-speed path_angle authority boost (2026-07-20), curvature-gated revision.
# path_angle = kappa * v * factor scales directly with speed, and the gain schedule (curvature_factor
# below) is flat below the 13.5 m/s (30 mph) knee -- so for the same commanded curvature, path_angle
# shrinks the slower you go, with nothing compensating below 30 mph. Below ~20 mph this leaves
# genuine curve commands too weak for the PSCM to track (logged: desired curvature reaching 0.012
# with actual stuck under 0.003 -- see lateral_angle_ext stall-blip module docstring).
#
# A first attempt at this (bp-7.0-lsgt) boosted path_angle by up to 1.51x on speed alone, with no
# regard for how large the underlying command was. Road-tested and made things worse: it boosts
# EVERY path_angle value below 30 mph equally, including near-zero/noise-level values during normal
# straight-line driving -- amplifying exactly the kind of small spurious signal that was already
# causing unprovoked hands-off drift on straight roads (two separate logged incidents, route
# bfef784d32f5351d/00000001--f5fbd93372, ~20:14:49 and ~20:16:05: measured curvature grew on its own
# while desired stayed near zero). Confirmed on-road worse, not just theorized.
#
# This version gates the boost on |requested_curvature| (the model/planner's own pre-clip signal --
# see the in-line comment at the apply site for why NOT kappa_cmd): no boost at all inside the
# deviation clip's own noise band (CURVATURE_ERROR), ramping to full boost only once the command
# clearly represents a real, intentional curve (2x that tolerance -- reusing the stall detector's own
# _STALL_GAP_MIN threshold for consistency rather than inventing a new number). Applied to path_angle
# only, exactly like the original attempt -- kappa_cmd/shadow_curvature is left untouched because
# ford.h's ford_shadow_curvature_error_check deviation-checks it against measured curvature; boosting
# it risks tripping that panda-side check and faulting steering outright, a worse failure than what
# this fixes.
_LSGT_V_HIGH_MS = 13.5                # m/s (30 mph) -- boost = 1.0 at/above this (today's schedule knee)
_LSGT_V_KNEE_MS = 20.0 * 0.44704      # m/s (20 mph) -- boost reaches its peak here, held flat below
_LSGT_PEAK = _LSGT_V_HIGH_MS / _LSGT_V_KNEE_MS   # ~1.51x -- extrapolated authority parity with 30 mph
_LSGT_KAPPA_GATE_LO = CarControllerParams.CURVATURE_ERROR        # 0.002 -- below this, boost = 1.0 (no-op)
_LSGT_KAPPA_GATE_HI = 2.0 * CarControllerParams.CURVATURE_ERROR  # 0.004 -- at/above this, full boost applies


# DBC ``LatCtlPath_An_Actl`` (rad) — panda safety uses the same in ``ford.h``; PSCM enforces in firmware.
FORD_DBC_PATH_ANGLE_MIN = -0.5
FORD_DBC_PATH_ANGLE_MAX = 0.5235


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
#
# Drift branch (2026-07-20): three logged interventions on the same route showed the detector
# missed the mirror case entirely -- hands-off, desired curvature flat/near-zero while *measured*
# curvature grew on its own (car curving when the model wanted straight), twice, because the trigger
# required abs(desired) > abs(current). That's backwards from the classic stall this was built for,
# so it never armed until the driver had already corrected. The detection below now has a second,
# deliberately narrow branch for exactly that case (measured leads AND the model wants ~straight);
# see the in-line comment at the detection site for why it is not simply direction-agnostic.
_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick (matches human_turn.py)
_STALL_GAP_MIN = 2.0 * CarControllerParams.CURVATURE_ERROR  # desired/measured must diverge by 2x the clip tolerance
# A stall is a FRACTIONAL failure, not just an absolute gap: during an honest deep-curve
# entry the car tracks at 0.7-0.85x of a large, fast-rising demand, which clears
# _STALL_GAP_MIN on magnitude alone -- and a mid-curve pulse releases steering exactly
# when the car is already behind (observed on-road: two such fires in one windy section,
# each followed by the driver grabbing the wheel within 0.2 s). True stalls measure
# 0.28-0.59x delivered across every diagnosed route; entry transients 0.64x and above.
# (Ported from upstream PR #148; applies to the classic branch only -- the drift branch
# below has its own near-straight guard and fires when measured LEADS desired.)
_STALL_DELIVERY_FRACTION = 0.65
_STALL_HOLD_S = 0.5          # accumulated divergence time before a pulse fires
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
    # User-tunable "feel" multipliers: read from FordLowSpeedFactor_ang / FordHighSpeedFactor_ang params.
    self.low_speed_curv_factor = 1.0
    self.high_speed_curv_factor = 1.0
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

  def update_angle_params(self, params):
    """Sets per-platform gain defaults and reads user feel-factor params."""
    self._ensure_lateral_curv_initialized(self.CP)
    fp = getattr(self.CP, 'carFingerprint', '')
    if fp in _CANFD_BOF_CARS:
      low, high = _GAIN_CANFD_BOF
    elif fp in _CANFD_SUV_CARS:
      low, high = _GAIN_CANFD_SUV
    else:
      low, high = _GAIN_CAN
    self.path_angle_gain_lowC_highV = low
    self.path_angle_gain_highC_highV = high
    if params is not None and hasattr(params, "get"):
      for attr, key in (("low_speed_curv_factor", "FordLowSpeedFactor_ang"),
                        ("high_speed_curv_factor", "FordHighSpeedFactor_ang")):
        try:
          raw = params.get(key, return_default=True)
          if raw is not None and raw != b"":
            setattr(self, attr, float(clip(
              float(raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw), 0.5, 1.5)))
        except Exception:
          pass
      try:
        raw = params.get("lane_change_factor_high_ang", return_default=True)
        if raw is not None and raw != b"":
          self.lane_change_factor_high_ang = float(clip(
            float(raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else raw), 0.85, 1.50))
      except Exception:
        pass

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
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      # Publish the shadow curvature from the measured curvature while inactive. LKA keeps
      # carrying angle_mode_engaged whenever angle mode is configured (independent of
      # latActive), and ford.h latches the shadow from every LKA frame -- so the latched
      # value must track reality here, not sit at a stale zero. Otherwise the first enabled
      # LMC frame after (re-)engage races LKA's 33Hz shadow latch against LMC's 20Hz enable
      # bit and ford.h's deviation check compares a zero shadow against real measured
      # curvature. (ford.h skips the check while steer_control_enabled is 0, so the value is
      # free to follow the measurement during the inactive period itself.) Upstream PR #144.
      self.bp_kappa_cmd = self.get_current_curvature(CS)
      self.human_turn_detector.reset()
      self.angle_human_turn_active = False
      self.stall_blip_hold_s = 0.0
      self.stall_blip_frames_left = 0
      self.stall_blip_cooldown_s = 0.0
      self.stall_blip_count = 0
      self.angle_stall_blip_active = False
      self.press_timer_s = 0.0
      self.precision_type = 1
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )

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
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      # Truthful shadow during the override (mirrors the inactive path -- see the comment
      # there): the driver is steering, so the honest command is the car's actual curvature,
      # and the panda-latched shadow stays current for the re-engage frame. Upstream PR #144.
      self.bp_kappa_cmd = self.get_current_curvature(CS)
      # Keep exit detection current so resume doesn't compare against a stale pre-turn value.
      self._desired_curvature_last = float(actuators.curvature)
      # A human turn ends any stall episode -- its own mode 0 does the PSCM reset job. That also
      # covers the press so far: only press time accumulated AFTER the latch releases should earn
      # a hand-off pulse.
      self.stall_blip_hold_s = 0.0
      self.stall_blip_frames_left = 0
      self.stall_blip_cooldown_s = 0.0
      self.stall_blip_count = 0
      self.angle_stall_blip_active = False
      self.press_timer_s = 0.0
      self.precision_type = 1
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )

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
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      # Truthful shadow during the blip (see the inactive-path comment). Upstream PR #144.
      self.bp_kappa_cmd = self.get_current_curvature(CS)
      self._desired_curvature_last = float(actuators.curvature)
      self.precision_type = 1
      if self.stall_blip_frames_left <= 0:
        self.stall_blip_cooldown_s = _STALL_COOLDOWN_S
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )
    self.angle_stall_blip_active = False

    self.precision_type = 1
    precision = 1
    LP = self.lp
    desired_curvature = float(actuators.curvature)

    # Variable lookup time: t_base tracks planner pre-compensation; extra tapers on high speed and large curves.
    # Cap liveDelay at 0.15s for VLT purposes. liveDelay can calibrate up to ~420ms on some runs, which inflates
    # VLT to 0.6s and pushes the model lookahead 5m into the curve. At that depth the model sees full peak
    # curvature, kappa_entering stays True, and the exit-biased blend is permanently disabled — causing the car
    # to command max path_angle through the entire apex. 0.15s gives t_base ≤ 0.20s and VLT ≤ 0.33s, restoring
    # the 2.8m lookahead that kept kappa_entering False at the apex in successful earlier runs.
    _t_base = float(clip(self.sm['liveDelay'].lateralDelay, 0.1, 0.15)) + _DT_MDL
    _speed_factor = float(interp(v_ego, [_VLT_V_LOW_MS, _VLT_V_HIGH_MS], [1.0, 0.0]))
    # Direction-aware kappa factor: on curve ENTRY (model shows more curvature at t_base than planner now),
    # keep full lookahead so pre-steering begins early. On exit/apex, taper by magnitude to prevent unwind.
    _kappa_at_t_base = 0.0
    if self.model is not None and len(self.model.orientationRate.z) >= 17:
      _curvatures_ref = np.array(self.model.orientationRate.z) / max(0.01, v_ego)
      _kappa_at_t_base = abs(float(interp(_t_base, ModelConstants.T_IDXS, _curvatures_ref)))
    _kappa_entering = _kappa_at_t_base > abs(desired_curvature)
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
    b_blend = float(clip(b * 0.25, 0.0, 1.0)) if _on_exit_near_limit else b
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
    self.low_gain_calc = interp(v_ego, [13.5, 26.82], [1.0, self.path_angle_gain_lowC_highV])
    self.high_gain_calc = interp(v_ego, [13.5, 26.82], [(1.30 * self.low_speed_curv_factor), (self.path_angle_gain_highC_highV * self.high_speed_curv_factor)])

    # As the curve gets bigger, we will need a little boost to the signal to to not understeer
    self.curvature_factor = interp(abs(kappa_cmd), [0.0007, 0.001], [self.low_gain_calc, self.high_gain_calc])

    path_angle_calc = kappa_cmd * v_ego * self.curvature_factor

    # BluePilot: low-speed path_angle authority boost (LSGT, curvature-gated -- see module constants
    # for why the gate exists and what it replaced). Speed component: 1.0 at/above 30 mph, ramping to
    # ~1.51x by the 20 mph knee and held flat below. Curvature gate: 0.0 inside the deviation clip's
    # own noise band (no-op on near-straight commands), ramping to 1.0 (full speed boost applies) once
    # the command is unambiguously a real curve. Applied to path_angle only -- kappa_cmd/shadow_curvature
    # must stay exactly as clipped above for the panda-side deviation check.
    #
    # Gates on requested_curvature (pre-deviation-clip), NOT kappa_cmd. Caught by offline replay
    # against the logged drift incidents (2026-07-20): kappa_cmd is clamped to current_curvature +-
    # CURVATURE_ERROR once the clip binds, so during an unprovoked drift (desired ~= 0, but measured
    # curvature grows on its own) kappa_cmd gets dragged along with the drifting measurement and can
    # exceed the gate threshold even though the model wants ~straight -- which boosted the drift
    # instead of suppressing it (replay showed path_angle up to ~4 deg worse than unboosted at the
    # peak of the segment-6 incident). requested_curvature is the model/planner's own signal, computed
    # before the clip ever sees measured curvature, so it stays near zero through a drift and only
    # rises for a genuine commanded curve.
    _lsgt_speed_boost = float(interp(v_ego, [_LSGT_V_KNEE_MS, _LSGT_V_HIGH_MS], [_LSGT_PEAK, 1.0]))
    _lsgt_kappa_gate = float(interp(abs(requested_curvature), [_LSGT_KAPPA_GATE_LO, _LSGT_KAPPA_GATE_HI], [0.0, 1.0]))
    low_speed_boost = 1.0 + (_lsgt_speed_boost - 1.0) * _lsgt_kappa_gate
    path_angle_calc *= low_speed_boost

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

    path_angle = min(FORD_DBC_PATH_ANGLE_MAX, max(FORD_DBC_PATH_ANGLE_MIN, path_angle))

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
    # deviation clip tracks it, so the shadow can exit ford.h's error band mid-curve -- the only
    # in-drive lateral safety blocks observed across ~4.5h of replayed routes were exactly this
    # (driver fighting a sustained curve with the mode still enabled). The honest command during
    # a press is the driver's actual curvature. Upstream PR #144.
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
    # Two stall directions (see module docstring):
    #   classic -- "car won't turn enough": the car is delivering under _STALL_DELIVERY_FRACTION of
    #              the demand. Fractional, not just desired-leads-measured: an honest deep-curve
    #              entry transient (0.7-0.85x of a large, fast-rising demand) clears the absolute
    #              gap threshold on magnitude alone, and a mode-0 pulse mid-curve releases steering
    #              exactly when the car is already behind (upstream PR #148's on-road evidence).
    #   drift   -- "car turns when it shouldn't": measured leads while the model wants ~straight.
    # The drift branch is deliberately narrow: it also requires |desired| to be inside the deviation
    # clip's own noise band, i.e. the model is genuinely commanding near-straight. Without that guard,
    # "measured leads desired" also matches normal curve EXITS (planner unwinds ahead of the car, both
    # values still large) -- offline replay of the incident route showed the unguarded version firing
    # 4-6 extra blips per minute in curve-rich stretches, and a 300 ms mode-0 gap mid-exit is itself a
    # hazard. With the guard, replay fires exactly one extra blip per logged drift incident, ~1.4 s
    # before the driver had to intervene, and none during curve entries/exits.
    _stalled_classic = abs(current_curvature) < _STALL_DELIVERY_FRACTION * abs(desired_curvature)
    _stalled_drift = (abs(desired_curvature) <= abs(current_curvature)
                      and abs(desired_curvature) < 1.5 * CarControllerParams.CURVATURE_ERROR)
    _stalled = (not CS.out.steeringPressed and not self.lane_change and v_ego > 9.0
                and abs(_stall_gap) > _STALL_GAP_MIN
                and (_stalled_classic or _stalled_drift))
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


    return LateralResult(
      apply_curvature=0.0,
      curvature_rate=curvature_rate,
      path_offset=path_offset,
      path_angle=path_angle,
      ramp_type=ramp_type,
      precision_type=self.precision_type,
      lateralUncertainty=lateral_uncertainty,
    )
