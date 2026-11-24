"""
BluePilot: the startup safety bits onto sunnypilot's CarParamsSP (HOSTS.md). Its own module, so car
init imports only values_ext, not the whole lateral.
"""
from ....car import structs
from ...core.values_ext import ford_safety_param


def init_ford_safety_param_sp(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  """Called from sunnypilot's setup_interfaces at car init. CP_SP.safetyParam reaches the panda as
  current_safety_param_sp (USB 0xdf); lateralInputsRequired tells the host to fill
  CarControlSP.lateralInputs and the per-frame settings, for every Ford (BluePilot lateral on or off)."""
  CP_SP.safetyParam |= ford_safety_param(CP, params_dict)
  if CP.brand == 'ford':
    CP_SP.lateralInputsRequired = True
