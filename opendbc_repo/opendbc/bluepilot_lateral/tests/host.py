"""The host's own names, for the tests that touch them: what carries the per-frame inputs and settings, where
its safety bits ride, its adapter (hosts/), and its libsafety setters. Detected from the host's opendbc, so
the same tests run on every host (HOSTS.md, "Running the tests")."""
import importlib.util

from ...car import structs

if hasattr(structs, "CarControlSP"):  # sunnypilot
  from ..hosts.sunnypilot.lateral import BluePilotLateral, params_from_cc_sp as params_from_cc
  from ..hosts.sunnypilot.safety_param import init_ford_safety_param_sp

  NAME = "sunnypilot"
  CarControlX = structs.CarControlSP
  SET_SAFETY_PARAM = "set_current_safety_param_sp"
  SET_CONTROLS_ALLOWED_LAT = "set_controls_allowed_lateral"

  def _init(CP, params_dict):
    CP_SP = structs.CarParamsSP()
    init_ford_safety_param_sp(CP, CP_SP, params_dict)
    return CP_SP.safetyParam, CP_SP.lateralInputsRequired
elif hasattr(structs, "IQCarControl"):  # IQ.Pilot
  from ..hosts.iqpilot.lateral import BluePilotLateral, params_from_cc_iq as params_from_cc
  from ..hosts.iqpilot.safety_param import init_ford_safety_param_iq

  NAME = "iqpilot"
  CarControlX = structs.IQCarControl
  SET_SAFETY_PARAM = "set_current_safety_param_iq"
  SET_CONTROLS_ALLOWED_LAT = "set_controls_allowed_lat"

  def _init(CP, params_dict):
    CP_IQ = structs.IQCarParams()
    init_ford_safety_param_iq(CP, CP_IQ, params_dict)
    return CP_IQ.iqSafetyFlags, CP_IQ.lateralInputsRequired
elif importlib.util.find_spec(__package__.rsplit(".", 2)[0] + ".car.ford.aol"):  # StarPilot: its Ford AOL
  from ..hosts.starpilot.lateral import BluePilotLateral, FrameInputs as CarControlX, params_from_frame as params_from_cc
  from ..hosts.starpilot.safety_param import (BP_SAFETY_PARAM_SHIFT, bp_safety_param, init_ford_safety_param_starpilot,
                                              lateral_inputs_required as lateral_inputs_required_starpilot)

  NAME = "starpilot"
  SET_SAFETY_PARAM = None  # no second param: the bits ride the Ford param (ford_param)
  SET_CONTROLS_ALLOWED_LAT = None  # no lateral-only setter: AOL is off in these tests, so lateral follows controls_allowed

  def _init(CP, params_dict):
    CP.safetyConfigs = [structs.CarParams.SafetyConfig(safetyModel=structs.CarParams.SafetyModel.ford)]
    init_ford_safety_param_starpilot(CP, params_dict)
    return bp_safety_param(CP), lateral_inputs_required_starpilot(CP)
else:
  raise ImportError("unknown host: add it to tests/host.py and hosts/")

__all__ = ["BluePilotLateral", "CarControlX", "params_from_cc"]


def startup_bits(CP, params_dict: dict[str, str]) -> int:
  """What the host's car-init hook gives ford_bp_init (values_ext.ford_safety_param's bits)."""
  return _init(CP, params_dict)[0]


def lateral_inputs_required(CP, params_dict: dict[str, str]) -> bool:
  """Whether the host fills the per-frame inputs and settings for this car."""
  return _init(CP, params_dict)[1]


def set_safety_param(safety, bits: int) -> None:
  """libsafety: the second safety param, the one ford_bp_init reads (current_safety_param_sp / _iq). A host
  without one carries the bits in the Ford param instead (ford_param)."""
  if SET_SAFETY_PARAM is not None:
    getattr(safety, SET_SAFETY_PARAM)(bits)


def ford_param(base: int, bits: int) -> int:
  """The Ford safety param set_safety_hooks takes for these startup bits: unchanged on a host with a second
  param, the bits shifted in on StarPilot."""
  return base | (bits << BP_SAFETY_PARAM_SHIFT) if NAME == "starpilot" else base


def set_controls_allowed_lat(safety, allowed: bool) -> None:
  """libsafety: the host's lateral-only engagement (sunnypilot MADS, IQ.Pilot AOL)."""
  if SET_CONTROLS_ALLOWED_LAT is not None:
    getattr(safety, SET_CONTROLS_ALLOWED_LAT)(allowed)
