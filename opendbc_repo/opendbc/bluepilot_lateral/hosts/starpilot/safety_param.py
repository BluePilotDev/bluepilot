"""
BluePilot: the startup safety bits onto StarPilot's Ford safetyParam (HOSTS.md). StarPilot has no second
safety param, so the bits ride its one Ford param at bits 9-14, clear of the bits StarPilot's Ford safety
reads (0-3, and 8 for its AOL's manual transmission). Its own module, so car init imports only values_ext,
not the whole lateral.
"""
from ....car import structs
from ....car.ford.values import FordFlags
from ...core.values_ext import ford_safety_param

BP_SAFETY_PARAM_SHIFT = 9
BP_SAFETY_PARAM_MASK = 0x3F  # ford_safety_param's six bits, before the shift
# StarPilot's Transit steers through Lane_Assist_Data1 alone: no BluePilot lateral
LKA_STEERING = getattr(FordFlags, "LKA_STEERING", 0)


def init_ford_safety_param_starpilot(CP: structs.CarParams, params_dict: dict[str, str]) -> None:
  """Called from StarPilot's Ford interface at car init, after its own safetyParam bits are set. The
  shifted bits reach the panda with the Ford safety mode; StarPilot's ford_init passes them to ford_bp_init."""
  if CP.brand != 'ford' or CP.flags & LKA_STEERING:
    return
  bits = ford_safety_param(CP, params_dict)
  if bits:
    CP.safetyConfigs[-1].safetyParam |= bits << BP_SAFETY_PARAM_SHIFT


def bp_safety_param(CP: structs.CarParams) -> int:
  """The startup bits as ford_safety_param returned them: what BluePilotLateral and ford_bp_init take."""
  if not CP.safetyConfigs:
    return 0
  return (CP.safetyConfigs[-1].safetyParam >> BP_SAFETY_PARAM_SHIFT) & BP_SAFETY_PARAM_MASK


def lateral_inputs_required(CP: structs.CarParams) -> bool:
  """Whether StarPilot's openpilot side fills the per-frame inputs and settings: every Ford the lateral can
  run on, BluePilot lateral on or off."""
  return CP.brand == 'ford' and not CP.flags & LKA_STEERING
