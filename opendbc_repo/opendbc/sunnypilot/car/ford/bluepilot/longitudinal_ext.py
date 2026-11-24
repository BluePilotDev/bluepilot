"""
BluePilot (full BluePilot only): Ford highway following, from BluePilot 7.

On top of upstream's accel/gas, above 50 mph (off again below 45) it classifies the lead as gaining,
pacing or trailing and limits gas and accel for each, rate-limits the start of braking unless a
collision is close, and replaces upstream's brake request with separate brake and pre-charge
hysteresis. Applies only while openpilot longitudinal is active.

Called at 50 Hz from the longitudinal block of opendbc/car/ford/carcontroller.py. The lead comes
from CarControlSP.leadOne (sunnypilot copies radarState.leadOne into it); the settings from
CarControlSP.params (BPX_LONG_PARAMS), which the host publishes.
"""
import numpy as np

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.bluepilot.published import published_bool

# setting: BP7's default when the host doesn't publish it
BPX_LONG_PARAMS = {
  "disable_BP_long_UI": False,
  "disable_downhill_comp_UI": False,
}

MAX_URBAN_SPEED_MPH = 45.0
FOLLOWING_ACCEL_ROC = 0.002  # max accel change per step in following mode

# brake and pre-charge hysteresis, m/s^2
BRAKE_ACTUATE_TARGET = -0.14
BRAKE_ACTUATE_RELEASE = -0.06
PRECHARGE_ACTUATE_TARGET = -0.12
PRECHARGE_ACTUATE_RELEASE = -0.06


class LongitudinalExt:
  def __init__(self):
    self.bp_accel_last = 0.0
    self.bp_speed_allow = False
    self.op_brake_actuate_last = False

  def update(self, CC, CC_SP, CS, op_accel: float, op_gas: float, accel_due_to_pitch: float):
    """Returns (accel, gas, brake_actuate, precharge_actuate) for the ACCDATA message."""
    disable_bp_long = published_bool(CC_SP, "disable_BP_long_UI", BPX_LONG_PARAMS)
    if published_bool(CC_SP, "disable_downhill_comp_UI", BPX_LONG_PARAMS) and accel_due_to_pitch < 0:
      accel_due_to_pitch = 0.

    accel_pitch_compensated = op_accel + accel_due_to_pitch
    op_brake_actuate = self.op_brake_actuate_last
    if accel_pitch_compensated > BRAKE_ACTUATE_RELEASE or not CC.longActive:
      op_brake_actuate = False
    elif accel_pitch_compensated < BRAKE_ACTUATE_TARGET:
      op_brake_actuate = True

    # speed deadband: engage above 50 mph, disallow below 45 mph
    v_ego_mph = CS.out.vEgo * 2.23694
    if v_ego_mph > MAX_URBAN_SPEED_MPH + 5:
      self.bp_speed_allow = True
    if v_ego_mph < MAX_URBAN_SPEED_MPH:
      self.bp_speed_allow = False

    accel, gas = op_accel, op_gas
    brake_actuate = precharge_actuate = op_brake_actuate
    if not disable_bp_long:
      lead = CC_SP.leadOne if CC_SP.leadOne.status else None
      v_ego = max(CS.out.vEgo, 0.5)
      lead_time_sec = 999.0
      v_rel = v_lead = 0.0
      if lead is not None:
        v_rel, v_lead = float(lead.vRel), float(lead.vLead)
        if lead.dRel > 0:
          lead_time_sec = float(lead.dRel) / v_ego
      lead_time_sec = float(np.clip(lead_time_sec, 0.0, 999.0))

      ttc_sec = 120.0
      if lead is not None:
        ttc_sec = float(lead.dRel) / -v_rel if (lead.dRel > 0 and v_rel < 0) else 60.0
      ttc_sec = float(np.clip(ttc_sec, 0.2, 120.0))

      # gas/accel limits by lead state: gaining, pacing or trailing
      min_gas = max_gas = op_gas
      min_accel = max_accel = op_accel
      if lead is not None:
        if v_rel < -0.1:  # gaining: no gas within 1.5 s
          if lead_time_sec < 1.5:
            min_gas = max_gas = 0.0
        elif v_rel <= 0.1:  # pacing: capped gas
          min_gas, max_gas = 0.0, 0.2 + accel_due_to_pitch
      else:
        # BP7 behavior, kept for the port: with no lead, accel is held at 0 (no braking) while BP
        # longitudinal applies (#16 follow-up)
        min_accel = max_accel = 0

      bp_gas = float(np.clip(op_gas, min_gas, max_gas))
      bp_accel = float(np.clip(op_accel, min_accel, max_accel))

      # dampen the initial brake hit unless a collision is close
      if ttc_sec > 8.0 and lead_time_sec > 0.5:
        bp_accel = float(np.clip(bp_accel, self.bp_accel_last - FOLLOWING_ACCEL_ROC, 999))

      bp_brake_actuate = bp_accel < BRAKE_ACTUATE_TARGET
      bp_precharge_actuate = bp_accel < PRECHARGE_ACTUATE_TARGET

      apply_bp_long = (self.bp_speed_allow and not CS.out.gasPressed and not CS.out.brakePressed and
                       (lead is None or v_lead * 2.23694 > 40.0))
      if apply_bp_long and CC.longActive:
        accel, gas = bp_accel, bp_gas
        brake_actuate, precharge_actuate = bp_brake_actuate, bp_precharge_actuate
      self.bp_accel_last = bp_accel

    # no brake and gas at the same time
    if brake_actuate:
      gas = CarControllerParams.INACTIVE_GAS

    # clip to the panda's ACCDATA limits
    accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    if gas != CarControllerParams.INACTIVE_GAS:
      gas = float(np.clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))

    self.op_brake_actuate_last = op_brake_actuate
    return accel, gas, brake_actuate, precharge_actuate
