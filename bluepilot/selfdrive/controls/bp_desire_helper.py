"""
BluePilot: BlinkerPauseLateral integration for lane change desire helper.

Wraps the sunnypilot BlinkerPauseLateral so that desire_helper.py only needs
a thin init + update call instead of managing imports and instances inline.
"""

from openpilot.sunnypilot.selfdrive.controls.lib.blinker_pause_lateral import BlinkerPauseLateral
from openpilot.cereal import log
from opendbc.car.structs import car
from openpilot.common.params import Params
from opendbc.car.ford.values import FordFlags


class BPBlinkerPause:
  """Manages blinker-based lane change pause logic for BluePilot."""

  def __init__(self, en_param: str = "BlinkerPauseLaneChange"):
    self.blinker_pause_lateral = BlinkerPauseLateral(en_param=en_param)

  def update(self, carstate) -> bool:
    """Returns True if lane changes should be paused due to blinker logic."""
    self.blinker_pause_lateral.get_params()
    return self.blinker_pause_lateral.update(carstate)


class BPLaneChangeHold:
  """BluePilot: lane changes on a Ford that holds the blinker for them, the way BlueCruise does.

  On a TRON Ford (2025-26 F-150) with BPLaneChangeBlinkerHold on, the car code sets the camera's turn request in
  Lane_Assist_Data1 while the lane change state isn't off (opendbc/sunnypilot/car/ford/bluepilot/turn_request.py), and
  the steering column module keeps the blinker on, reported back as carState's left/rightBlinker. So:
  - the blinker no longer ends the lane change: one tap carries it through the wait (blind spot, the auto lane change
    timer, a nudge) and the change itself, and the driver cancels by braking, signalling the other way or steering
    against it. A wait longer than PRE_LANE_CHANGE_TIMEOUT gives up;
  - the direction is the one the driver signalled, held for the whole lane change;
  - a finished lane change ends (state off) even though the held blinker is still on, instead of starting another one.
    State off drops the request, the column module drops the blinker within a frame, and a new lane change needs a
    new tap (DesireHelper starts one only on the blinker coming on).
  If the column module ignores the request (it can't be proven off the road), the blinker ends after its flashes and
  the lane change ends with it, as before.
  """
  PRE_LANE_CHANGE_TIMEOUT = 10.0  # s

  def __init__(self, param: str = "BPLaneChangeBlinkerHold"):
    self.params = Params()
    self.param = param
    self.holds_blinker = self.car_holds_blinker(self.params)
    self.enabled = False
    self.param_read_counter = 0
    self.wait = 0.0
    self.read_params()

  @staticmethod
  def car_holds_blinker(params) -> bool:
    """True on the cars the turn request was decoded on: Ford TRON."""
    try:
      cp_bytes = params.get("CarParams")
      if not cp_bytes:
        return False
      with car.CarParams.from_bytes(cp_bytes) as CP:
        return CP.brand == "ford" and bool(CP.flags & FordFlags.TRON)
    except Exception:
      return False

  def read_params(self) -> None:
    self.enabled = self.params.get_bool(self.param)

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  @property
  def active(self) -> bool:
    return self.holds_blinker and self.enabled

  def cancel_pre_lane_change(self, carstate, direction, dt: float) -> bool:
    """Called every model frame while the lane change waits to start. True ends it."""
    if not self.active:
      return False
    self.wait += dt
    left = direction == log.LaneChangeDirection.left
    other_blinker = carstate.rightBlinker if left else carstate.leftBlinker
    against = carstate.steeringPressed and ((carstate.steeringTorque < 0) if left else (carstate.steeringTorque > 0))
    return self.wait > self.PRE_LANE_CHANGE_TIMEOUT or carstate.brakePressed or other_blinker or against

  def update_state(self, lane_change_state) -> None:
    if lane_change_state != log.LaneChangeState.preLaneChange:
      self.wait = 0.0
