"""
BluePilot: the IQ.Pilot host adapter for the lateral (HOSTS.md). The Ford CarController builds this
with IQ.Pilot's types; it hands the core the library's types and nothing IQ-specific.
"""
from ...core import carcontroller_ext
from ...params.param_store import ParamStore


def params_from_cc_iq(CC_IQ) -> ParamStore:
  """The settings IQ.Pilot published this frame (IQCarControl.params entries) as (key, value) pairs."""
  return ParamStore((p.key, p.value) for p in CC_IQ.params)


class BluePilotLateral(carcontroller_ext.BluePilotLateral):
  def __init__(self, CP, CP_IQ, packer, CAN):
    super().__init__(CP, CP_IQ.iqSafetyFlags, packer, CAN)

  def update(self, CC_IQ, frame):
    """Every frame: the inputs and settings IQ.Pilot's controlsd put in IQCarControl. Returns whether
    BluePilot lateral is active."""
    return super().update(CC_IQ.lateralInputs, params_from_cc_iq(CC_IQ), frame)
