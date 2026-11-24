"""
BluePilot: the sunnypilot host adapter for the lateral (HOSTS.md). The Ford CarController builds this
with sunnypilot's types; it hands the core the library's types and nothing sunnypilot-specific.
"""
from ...core import carcontroller_ext
from ...params.param_store import ParamStore


def params_from_cc_sp(CC_SP) -> ParamStore:
  """The settings sunnypilot published this frame (CarControlSP.params entries) as (key, value) pairs."""
  return ParamStore((p.key, p.value) for p in CC_SP.params)


class BluePilotLateral(carcontroller_ext.BluePilotLateral):
  def __init__(self, CP, CP_SP, packer, CAN):
    super().__init__(CP, CP_SP.safetyParam, packer, CAN)

  def update(self, CC_SP, frame):
    """Every frame: the inputs and settings sunnypilot's controlsd put in CarControlSP. Returns whether
    BluePilot lateral is active."""
    return super().update(CC_SP.lateralInputs, params_from_cc_sp(CC_SP), frame)
