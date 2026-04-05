"""
BluePilot: BlinkerPauseLateral integration for lane change desire helper.

Wraps the sunnypilot BlinkerPauseLateral so that desire_helper.py only needs
a thin init + update call instead of managing imports and instances inline.
"""

from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral


class BPBlinkerPause:
  """Manages blinker-based lane change pause logic for BluePilot."""

  def __init__(self, en_param: str = "BlinkerPauseLaneChange"):
    self.blinker_pause_lateral = BlinkerPauseLateral(en_param=en_param)

  def update(self, carstate) -> bool:
    """Returns True if lane changes should be paused due to blinker logic."""
    self.blinker_pause_lateral.get_params()
    return self.blinker_pause_lateral.update(carstate)
