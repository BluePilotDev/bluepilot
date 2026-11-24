"""
BluePilot: the StarPilot host adapter for the lateral (HOSTS.md). StarPilot's Ford CarController builds this
from CarParams alone (the startup bits ride its Ford safetyParam, safety_param.py). StarPilot has no
CarControl extension: its openpilot side hands the CarController a FrameInputs each frame instead, carrying
what sunnypilot and IQ.Pilot carry in theirs.
"""
from dataclasses import dataclass, field

from ...core import carcontroller_ext
from ...host_types import LateralInputs
from ...params.param_store import ParamStore
from .safety_param import bp_safety_param


@dataclass
class FrameInputs:
  """This frame's model / vehicle-parameter inputs and settings, filled by StarPilot's openpilot side."""

  @dataclass
  class Param:
    key: str = ""
    value: bytes = b""

  lateralInputs: LateralInputs = field(default_factory=LateralInputs)
  params: list = field(default_factory=list)


def params_from_frame(frame_inputs: FrameInputs) -> ParamStore:
  """The settings StarPilot published this frame (FrameInputs.params entries) as (key, value) pairs."""
  return ParamStore((p.key, p.value) for p in frame_inputs.params)


class BluePilotLateral(carcontroller_ext.BluePilotLateral):
  def __init__(self, CP, packer, CAN):
    super().__init__(CP, bp_safety_param(CP), packer, CAN)

  def update(self, frame_inputs: FrameInputs, frame):
    """Every frame: the inputs and settings StarPilot's openpilot side put in frame_inputs. Returns whether
    BluePilot lateral is active."""
    return super().update(frame_inputs.lateralInputs, params_from_frame(frame_inputs), frame)
