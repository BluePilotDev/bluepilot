"""BluePilot: lane changes on a Ford that holds the blinker for them (bp_desire_helper.BPLaneChangeHold).

The truck here is a model of what the 2025-26 F-150 capture showed: the driver's tap lights the blinker for 3 flashes
(TAP_S), and while openpilot's lane change state isn't off the column module keeps it on, one model frame late.
"""
from types import SimpleNamespace as NS

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState, LaneChangeDirection
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

TAP_S = 1.3   # a one-touch tap: 3 flashes
V_EGO = 30.   # m/s


class Truck:
  def __init__(self, hold: bool, timer: int = AutoLaneChangeMode.TWO_SECONDS, bsm_delay: bool = False):
    params = Params()
    params.put("AutoLaneChangeTimer", timer)
    params.put_bool("AutoLaneChangeBsmDelay", bsm_delay)
    params.put_bool("BPLaneChangeBlinkerHold", True)
    self.DH = DesireHelper()
    self.DH._bp_hold.holds_blinker = hold
    self.t = 0.
    self.tap = None           # (direction, start time)
    self.held = False         # the column module's latch, from the previous frame's state
    self.lane_change_prob = 0.
    self.cs = {"steeringPressed": False, "steeringTorque": 0., "brakePressed": False, "leftBlindspot": False, "rightBlindspot": False}
    self.log = []

  def tap_stalk(self, direction):
    self.tap = (direction, self.t)

  def blinkers(self):
    # Ford reports one turn state (TurnLghtSwtch_D_Stat: off, left or right): the driver's latest tap wins over the hold
    if self.tap is not None and self.t - self.tap[1] < TAP_S:
      return self.tap[0] == "left", self.tap[0] == "right"
    if self.held:
      return self.DH.lane_change_direction == LaneChangeDirection.left, self.DH.lane_change_direction == LaneChangeDirection.right
    return False, False

  def run(self, seconds, **cs):
    self.cs.update(cs)
    for _ in range(round(seconds / DT_MDL)):
      left, right = self.blinkers()
      carstate = NS(vEgo=V_EGO, leftBlinker=left, rightBlinker=right, **self.cs)
      # the model: its lane change probability rises while openpilot asks for a lane change, then falls
      if self.DH.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_prob = 0.9 if self.DH.lane_change_timer < 2.0 else 0.0
      self.DH.update(carstate, True, self.lane_change_prob)
      # the column module holds the blinker while the state isn't off (controlsd -> CarControl blinker -> 0x3CA)
      self.held = self.DH._bp_hold.active and self.DH.lane_change_state != LaneChangeState.off
      self.log.append((round(self.t, 2), self.DH.lane_change_state, self.DH.lane_change_direction, left, right))
      self.t += DT_MDL
    return self.DH.lane_change_state

  def states(self):
    return [s for _, s, *_ in self.log]


class TestLaneChangeHold(OpenpilotTestCase):
  def test_one_tap_waits_for_the_blind_spot_then_changes_lanes(self):
    truck = Truck(hold=True)
    truck.tap_stalk("left")
    self.assertEqual(truck.run(4., leftBlindspot=True), LaneChangeState.preLaneChange)  # tap long over, still waiting
    self.assertEqual(truck.DH.lane_change_direction, LaneChangeDirection.left)
    self.assertTrue(truck.log[-1][3])  # the blinker is still on: the truck holds it
    # the 2 s timer ran while it waited: it goes as soon as the blind spot clears
    self.assertEqual(truck.run(0.1, leftBlindspot=False), LaneChangeState.laneChangeStarting)
    self.assertEqual(truck.DH.lane_change_direction, LaneChangeDirection.left)

  def test_blind_spot_delay_waits_a_second_after_it_clears(self):
    truck = Truck(hold=True, bsm_delay=True)
    truck.tap_stalk("left")
    truck.run(4., leftBlindspot=True)
    self.assertEqual(truck.run(0.8, leftBlindspot=False), LaneChangeState.preLaneChange)
    self.assertEqual(truck.run(0.4), LaneChangeState.laneChangeStarting)

  def test_without_the_hold_a_tap_ends_before_the_timer(self):
    # the bug this fixes: 3 flashes are shorter than a 2 s timer, the blinker goes out and the lane change with it
    truck = Truck(hold=False)
    truck.tap_stalk("left")
    truck.run(3.)
    self.assertNotIn(LaneChangeState.laneChangeStarting, truck.states())
    self.assertEqual(truck.DH.lane_change_state, LaneChangeState.off)

  def test_a_finished_lane_change_ends_and_releases_the_blinker(self):
    truck = Truck(hold=True)
    truck.tap_stalk("right")
    truck.run(10.)
    states = truck.states()
    start = states.index(LaneChangeState.laneChangeStarting)
    end = start + states[start:].index(LaneChangeState.off)
    # exactly one lane change: never back to waiting or a second start
    self.assertNotIn(LaneChangeState.preLaneChange, states[end:])
    self.assertNotIn(LaneChangeState.laneChangeStarting, states[end:])
    # the blinker is on through the change and off once it ends
    self.assertTrue(all(r for *_, r in truck.log[start:end]))
    self.assertFalse(any(r for *_, r in truck.log[end + 2:]))

  def test_a_new_tap_after_it_ends_changes_lanes_again(self):
    truck = Truck(hold=True)
    truck.tap_stalk("left")
    truck.run(10.)
    truck.tap_stalk("left")
    truck.run(5.)
    starts = [i for i, (a, b) in enumerate(zip(truck.states(), truck.states()[1:], strict=False))
              if a != LaneChangeState.laneChangeStarting and b == LaneChangeState.laneChangeStarting]
    self.assertEqual(len(starts), 2)

  def test_driver_cancels_while_it_waits(self):
    for name, cs in (("brake", {"brakePressed": True}),
                     ("steering against", {"steeringPressed": True, "steeringTorque": -1.5})):
      with self.subTest(name):
        truck = Truck(hold=True)
        truck.tap_stalk("left")
        truck.run(2., leftBlindspot=True)
        self.assertEqual(truck.run(0.5, **cs), LaneChangeState.off)
        self.assertNotIn(LaneChangeState.laneChangeStarting, truck.states())

  def test_other_blinker_cancels(self):
    truck = Truck(hold=True)
    truck.tap_stalk("left")
    truck.run(2., leftBlindspot=True)
    truck.tap_stalk("right")
    truck.run(0.2)
    self.assertEqual(truck.DH.lane_change_state, LaneChangeState.off)
    self.assertNotIn(LaneChangeDirection.right, [d for _, s, d, *_ in truck.log if s == LaneChangeState.laneChangeStarting])

  def test_gives_up_after_waiting_too_long(self):
    truck = Truck(hold=True)
    truck.tap_stalk("left")
    truck.run(9., leftBlindspot=True)
    self.assertEqual(truck.DH.lane_change_state, LaneChangeState.preLaneChange)
    truck.run(1.5)
    self.assertEqual(truck.DH.lane_change_state, LaneChangeState.off)
    self.assertNotIn(LaneChangeState.laneChangeStarting, truck.states())

  def test_nudge_mode_waits_for_the_nudge(self):
    truck = Truck(hold=True, timer=AutoLaneChangeMode.NUDGE)
    truck.tap_stalk("right")
    self.assertEqual(truck.run(4.), LaneChangeState.preLaneChange)
    self.assertEqual(truck.run(0.2, steeringPressed=True, steeringTorque=-1.5), LaneChangeState.laneChangeStarting)

  def test_setting_off_is_stock(self):
    truck = Truck(hold=True)
    Params().put_bool("BPLaneChangeBlinkerHold", False)
    truck.DH._bp_hold.read_params()
    self.assertFalse(truck.DH._bp_hold.active)

  def test_only_on_a_tron_ford(self):
    self.assertFalse(type(DesireHelper()._bp_hold).car_holds_blinker(NS(get=lambda key: None)))
