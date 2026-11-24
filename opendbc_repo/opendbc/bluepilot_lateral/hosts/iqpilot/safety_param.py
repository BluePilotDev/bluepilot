"""
BluePilot: the startup safety bits onto IQ.Pilot's IQCarParams (HOSTS.md). Its own module, so car
init imports only values_ext, not the whole lateral.
"""
from ...core.values_ext import ford_safety_param


def init_ford_safety_param_iq(CP, CP_IQ, params_dict: dict[str, str]) -> None:
  """Called from IQ.Pilot's apply_iq_car_config at car init. CP_IQ.iqSafetyFlags reaches the panda as
  current_safety_param_iq (USB 0xdf); lateralInputsRequired tells the host to fill
  IQCarControl.lateralInputs and the per-frame settings, for every Ford (BluePilot lateral on or off)."""
  CP_IQ.iqSafetyFlags |= ford_safety_param(CP, params_dict)
  if CP.brand == 'ford':
    CP_IQ.lateralInputsRequired = True
