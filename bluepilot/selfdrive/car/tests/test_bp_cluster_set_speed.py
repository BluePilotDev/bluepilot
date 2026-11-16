"""With ICBM on a Ford, openpilot's set-speed target follows the truck's cluster set speed at engage and after a
driver Set+/Set- press (#232). Driven through VCruiseHelper.update_v_cruise, one call per card frame (100 Hz),
against a model of what the F-150 does (#232, "What the truck does on a held Set-")."""
from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import INACTIVE_TIMER

from bluepilot.selfdrive.car.bp_cluster_set_speed import SETTLE_TIME

ButtonType = car.CarState.ButtonEvent.Type
SlaState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


class Truck:
  """The truck's cluster set speed and openpilot's target, frame by frame (10 ms)."""

  def __init__(self, brand="ford", icbm=True, is_metric=False, pcm_cruise=True):
    self.helper = VCruiseHelper(car.CarParams(brand=brand, pcmCruise=pcm_cruise), custom.CarParamsSP(pcmCruiseSpeed=not icbm))
    self.is_metric = is_metric
    self.unit = CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS
    self.cluster = 0  # display units, 0 = no set speed shown
    self.enabled = False
    self.limit = 0  # Speed Limit Assist: the limit in display units and the planner's state
    self.sla_state = SlaState.disabled

  def frame(self, *events):
    if self.limit:
      LP_SP = custom.LongitudinalPlanSP.new_message()
      LP_SP.speedLimit.resolver.speedLimitValid = True
      LP_SP.speedLimit.resolver.speedLimitFinalLast = self.limit * self.unit
      LP_SP.speedLimit.assist.state = self.sla_state
      self.helper.update_speed_limit_assist(self.is_metric, LP_SP)
    speed = self.cluster * self.unit
    CS = car.CarState(cruiseState={"available": True, "enabled": self.enabled, "speed": speed, "speedCluster": speed},
                      buttonEvents=[{"type": t, "pressed": p} for t, p in events])
    self.helper.update_v_cruise(CS, self.enabled, self.is_metric)

  def frames(self, n):
    for _ in range(n):
      self.frame()

  @property
  def target(self):
    """openpilot's target in display units, as ICBM and the Speed Limit Assist prompt round it."""
    return round(self.helper.v_cruise_kph * (1 if self.is_metric else CV.KPH_TO_MPH))

  def engage_from_main_off(self, set_speed):
    """Main then Set: the truck reports enabled about 40 ms before it shows the set speed."""
    self.frames(5)
    self.enabled = True
    self.frames(4)
    self.cluster = set_speed
    self.frames(50)

  def press(self, button, steps, release_after):
    """A driver press: the cluster takes each (frame, speed) in steps while the button is held, then the release."""
    self.frame((button, True))
    for i in range(1, release_after):
      self.cluster = dict(steps).get(i, self.cluster)
      self.frame()
    self.frame((button, False))

  def hold(self, button, release_after=70):
    """A held press: 1 unit about 40 ms after the press, the next multiple of 5 at about 0.5 s, released at 0.7 s."""
    sign = 1 if button == ButtonType.accelCruise else -1
    first = self.cluster + sign
    fifth = (first // 5 + (sign > 0)) * 5 if first % 5 else first
    self.press(button, [(4, first), (50, fifth)], release_after)

  def tap(self, button):
    """A short tap: 1 unit about 40 ms after the press, released at 0.12 s."""
    self.press(button, [(4, self.cluster + (1 if button == ButtonType.accelCruise else -1))], 12)


class TestEngage(OpenpilotTestCase):
  def test_engage_from_main_off_takes_the_cluster_set_speed(self):
    truck = Truck()
    truck.engage_from_main_off(35)
    self.assertEqual(truck.target, 35)


class TestDriverPress(OpenpilotTestCase):
  def test_held_set_minus_from_a_multiple_of_5_holds_the_truck_speed(self):
    truck = Truck()
    truck.engage_from_main_off(35)
    truck.hold(ButtonType.decelCruise)
    self.assertEqual(truck.cluster, 30)
    truck.frames(100)
    self.assertEqual(truck.target, 30)

  def test_held_presses_end_on_the_truck_speed(self):
    for button, start, end in ((ButtonType.decelCruise, 33, 30), (ButtonType.decelCruise, 40, 35),
                               (ButtonType.accelCruise, 30, 35), (ButtonType.accelCruise, 35, 40)):
      with self.subTest(button=button, start=start):
        truck = Truck()
        truck.engage_from_main_off(start)
        truck.hold(button)
        truck.frames(100)
        self.assertEqual((truck.cluster, truck.target), (end, end))

  def test_taps_end_on_the_truck_speed(self):
    truck = Truck()
    truck.engage_from_main_off(35)
    for button, end in ((ButtonType.decelCruise, 34), (ButtonType.decelCruise, 33), (ButtonType.accelCruise, 34),
                        (ButtonType.accelCruise, 35), (ButtonType.accelCruise, 36)):
      truck.tap(button)
      truck.frames(100)
      self.assertEqual((truck.cluster, truck.target), (end, end))

  def test_metric(self):
    truck = Truck(is_metric=True)
    truck.engage_from_main_off(60)
    self.assertEqual(truck.target, 60)
    for press, end in ((lambda: truck.tap(ButtonType.decelCruise), 59), (lambda: truck.hold(ButtonType.decelCruise), 55),
                       (lambda: truck.tap(ButtonType.accelCruise), 56), (lambda: truck.hold(ButtonType.accelCruise), 60)):
      press()
      truck.frames(100)
      self.assertEqual((truck.cluster, truck.target), (end, end))


class TestIcbmSteps(OpenpilotTestCase):
  def test_icbm_steps_do_not_move_the_target(self):
    # ICBM's own presses change the cluster without driver button events
    truck = Truck()
    truck.engage_from_main_off(35)
    truck.hold(ButtonType.decelCruise)
    truck.frames(50)
    for speed in (31, 32, 33):
      truck.cluster = speed
      truck.frames(12)
      self.assertEqual(truck.target, 30)

  def test_settle_ends_before_icbm_acts(self):
    self.assertLess(SETTLE_TIME, INACTIVE_TIMER)
    # a truck update just after the release is still taken
    truck = Truck()
    truck.engage_from_main_off(35)
    truck.tap(ButtonType.decelCruise)
    truck.frames(5)
    truck.cluster = 33
    truck.frames(5)
    self.assertEqual(truck.target, 33)
    # a cluster change once ICBM can act (INACTIVE_TIMER after the release) is ICBM's, not the driver's
    truck.frames(round(INACTIVE_TIMER * 100) - 10)
    truck.cluster = 34
    truck.frames(5)
    self.assertEqual(truck.target, 33)


class TestSpeedLimitAssist(OpenpilotTestCase):
  def confirm(self, button, start, limit, is_metric=False):
    """A limit prompt, the driver's confirming tap (the truck moves 1 unit), then the planner confirms on the release."""
    truck = Truck(is_metric=is_metric)
    truck.engage_from_main_off(start)
    truck.limit, truck.sla_state = limit, SlaState.preActive
    truck.frames(10)
    truck.tap(button)
    truck.frames(5)  # the planner sees the release about one plan later
    truck.sla_state = SlaState.active
    truck.frames(100)
    return truck

  def test_confirm_press_sets_the_limit_as_target(self):
    for button, start, limit in ((ButtonType.accelCruise, 32, 35), (ButtonType.decelCruise, 35, 20),
                                 (ButtonType.accelCruise, 19, 35)):
      with self.subTest(button=button, start=start, limit=limit):
        truck = self.confirm(button, start, limit)
        self.assertEqual(truck.target, limit)
    self.assertEqual(self.confirm(ButtonType.decelCruise, 80, 60, is_metric=True).target, 60)

  def test_engage_after_a_confirm_press_takes_the_cluster_set_speed(self):
    truck = self.confirm(ButtonType.accelCruise, 32, 35)
    truck.enabled, truck.cluster, truck.limit = False, 0, 0
    truck.engage_from_main_off(28)
    self.assertEqual(truck.target, 28)

  def test_limit_above_the_set_speed_asks_for_plus_and_set_minus_does_not_confirm(self):
    # drive 2026-11-11 evening: engaged from main-off at 35, a misread 50 mph sign, then a held Set-
    truck = Truck()
    truck.engage_from_main_off(35)
    truck.limit, truck.sla_state = 50, SlaState.preActive
    truck.frames(10)
    self.assertEqual((truck.helper.req_plus, truck.helper.req_minus), (True, False))
    self.assertFalse(truck.helper.update_speed_limit_assist_pre_active_confirmed(ButtonType.decelCruise))
    truck.hold(ButtonType.decelCruise)
    truck.frames(100)
    self.assertEqual((truck.cluster, truck.target), (30, 30))


class TestOtherCars(OpenpilotTestCase):
  def test_other_brand_with_icbm_keeps_sunnypilot_behavior(self):
    truck = Truck(brand="toyota")
    self.assertIsNone(truck.helper.bp_cluster_set_speed)
    truck.engage_from_main_off(35)
    self.assertEqual(truck.target, 158)  # unset
    truck.hold(ButtonType.decelCruise)
    self.assertEqual(truck.target, 90)  # sunnypilot's own steps from the unset target, clipped to 145 kph

  def test_ford_with_icbm_off_reads_the_truck(self):
    truck = Truck(icbm=False)
    self.assertIsNone(truck.helper.bp_cluster_set_speed)
    truck.engage_from_main_off(35)
    truck.hold(ButtonType.decelCruise)
    self.assertEqual(truck.target, 30)

  def test_ford_with_openpilot_longitudinal_is_untouched(self):
    self.assertIsNone(Truck(pcm_cruise=False).helper.bp_cluster_set_speed)
