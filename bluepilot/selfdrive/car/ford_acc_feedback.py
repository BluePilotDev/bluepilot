"""Read-only Ford ACC visual feedback from messages already parsed by card."""

from opendbc.car import Bus


def ford_acc_feedback(ci, can_valid: bool, now_ns: int) -> tuple[bool, float]:
  """Return availability and signed feedback; never infer commands from vehicle acceleration."""
  if not can_valid or ci.CP.brand != 'ford' or ci.CP.openpilotLongitudinalControl:
    return False, 0.0
  cs = ci.CS.out
  if not cs.cruiseState.enabled or cs.gasPressed or cs.brakePressed:
    return False, 0.0
  cp = ci.can_parsers.get(Bus.pt)
  if cp is None:
    return False, 0.0
  for message, signal in (('EngBrakeData', 'CcMde_D_Actl'), ('DesiredTorqBrk', 'AccBrkActv_B_Actl')):
    timestamp = cp.ts_nanos.get(message, {}).get(signal, 0)
    if not timestamp or not 0 <= now_ns - timestamp <= 500_000_000:
      return False, 0.0
  engine = cp.vl['EngBrakeData']
  if engine['CcOvrrdActv_B_Actl']:
    return False, 0.0
  mode = int(engine['CcMde_D_Actl'])
  if cp.vl['DesiredTorqBrk']['AccBrkActv_B_Actl'] or mode == 3:
    return True, -1.0
  if mode == 2:
    return True, 1.0
  return True, 0.0
