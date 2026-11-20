"""Read-only Ford ACC visual feedback from messages already parsed by card."""

import math

from opendbc.car import Bus


def ford_acc_feedback(ci, can_valid: bool, now_ns: int) -> tuple[bool, float]:
  """Return availability and signed demand; never infer commands from vehicle acceleration."""
  if not can_valid or ci.CP.brand != 'ford' or ci.CP.openpilotLongitudinalControl:
    return False, 0.0
  cs = ci.CS.out
  if not cs.cruiseState.enabled or cs.gasPressed or cs.brakePressed:
    return False, 0.0
  cp = ci.can_parsers.get(Bus.pt)
  camera = ci.can_parsers.get(Bus.cam)
  if cp is None or camera is None:
    return False, 0.0
  for parser, message, signal in ((cp, 'EngBrakeData', 'CcOvrrdActv_B_Actl'),
                                  (camera, 'ACCDATA', 'AccPrpl_A_Rq')):
    timestamp = parser.ts_nanos.get(message, {}).get(signal, 0)
    if not timestamp or not 0 <= now_ns - timestamp <= 500_000_000:
      return False, 0.0
  if cp.vl['EngBrakeData']['CcOvrrdActv_B_Actl']:
    return False, 0.0
  request = camera.vl['ACCDATA']
  propulsion = float(request['AccPrpl_A_Rq'])
  braking = float(request['AccBrkTot_A_Rq'])
  if not math.isfinite(propulsion) or not math.isfinite(braking):
    return False, 0.0
  # Ford uses the positive propulsion request for acceleration and the negative
  # total-acceleration request for deceleration. -5 propulsion is an inactive
  # sentinel, not a request for maximum braking. Brake requests take priority.
  if request['AccBrkDecel_B_Rq'] or braking < -0.1:
    return True, -max(0.35, min(1.0, -braking / 2.0))
  if propulsion > 0.1:
    return True, max(0.25, min(1.0, propulsion / 1.5))
  return True, 0.0
