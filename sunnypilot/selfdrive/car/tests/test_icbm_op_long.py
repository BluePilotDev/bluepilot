"""
BluePilot: ICBM under openpilot longitudinal on Ford.

With ICBM on, openpilot owns the set speed (CP_SP.pcmCruiseSpeed False) even under openpilot
longitudinal, so Speed Limit Assist targets the limit itself instead of asking the driver to park
the cluster at 70/80 mph. Everything else keeps the stock gating.
"""
import pytest

from cereal import custom
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR as FORD
from opendbc.car.hyundai.values import CAR as HYUNDAI
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_with_op_long
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist, ACTIVE_STATES
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


def _setup(car_name, op_long: bool, icbm: bool):
  params = Params()
  params.put_bool("IntelligentCruiseButtonManagement", icbm)
  params.put("SpeedLimitMode", int(Mode.assist))
  params.put_bool("IsMetric", False)
  CarInterface = interfaces[car_name]
  CP = CarInterface.get_non_essential_params(car_name)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
  CI = CarInterface(CP, CP_SP)
  CI.CP.openpilotLongitudinalControl = op_long
  sunnypilot_interfaces.setup_interfaces(CI, params)
  return CI, params


class TestIcbmOpLong:
  def teardown_method(self, method):
    Params().remove("IntelligentCruiseButtonManagement")

  @pytest.mark.parametrize("car_name", [FORD.FORD_EXPEDITION_MK4, FORD.FORD_F_150_MK14, FORD.FORD_EXPLORER_MK6])
  def test_ford_op_long_with_icbm_owns_set_speed(self, car_name):
    CI, params = _setup(car_name, op_long=True, icbm=True)
    assert icbm_with_op_long(CI.CP)
    assert not CI.CP_SP.pcmCruiseSpeed
    assert params.get_bool("IntelligentCruiseButtonManagement")  # not cleaned up

  def test_ford_op_long_without_icbm_is_unchanged(self):
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=True, icbm=False)
    assert CI.CP_SP.pcmCruiseSpeed
    assert SpeedLimitAssist(CI.CP, CI.CP_SP).pcm_op_long

  def test_ford_stock_long_with_icbm_is_unchanged(self):
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=False, icbm=True)
    assert not CI.CP_SP.pcmCruiseSpeed

  def test_other_brand_op_long_still_drops_icbm(self):
    CI, params = _setup(HYUNDAI.HYUNDAI_SONATA, op_long=True, icbm=True)
    assert CI.CP_SP.intelligentCruiseButtonManagementAvailable and not icbm_with_op_long(CI.CP)
    assert CI.CP_SP.pcmCruiseSpeed
    assert not params.get_bool("IntelligentCruiseButtonManagement")

  def test_speed_limit_assist_targets_the_limit_not_70_80(self):
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=True, icbm=True)
    sla = SpeedLimitAssist(CI.CP, CI.CP_SP)
    assert not sla.pcm_op_long
    for limit_mph in (25, 45, 65, 71.5):
      sla._has_speed_limit = True
      sla._speed_limit_final_last = limit_mph * CV.MPH_TO_MS
      sla.update_calculations(35 * CV.MPH_TO_MS)
      assert sla.target_set_speed_conv == round(limit_mph)
      assert sla.target_set_speed_conv not in [round(v * CV.MS_TO_MPH) for v in PCM_LONG_REQUIRED_MAX_SET_SPEED[False]]

  def test_speed_limit_assist_without_icbm_still_asks_for_70_80(self):
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=True, icbm=False)
    sla = SpeedLimitAssist(CI.CP, CI.CP_SP)
    sla._has_speed_limit = True
    sla._speed_limit_final_last = 45 * CV.MPH_TO_MS
    sla.update_calculations(35 * CV.MPH_TO_MS)
    assert sla.target_set_speed_conv == 70

  def test_icbm_controller_runs_under_op_long(self):
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=True, icbm=True)
    icbm = IntelligentCruiseButtonManagement(CI.CP, CI.CP_SP)
    assert not icbm.CP_SP.pcmCruiseSpeed  # run() returns early only when the PCM owns the set speed


class _Obj:
  def __init__(self, **kw):
    self.__dict__.update(kw)


def _cs(v_ego_mph, cluster_mph):
  return _Obj(vEgo=v_ego_mph * CV.MPH_TO_MS, buttonEvents=[],
              cruiseState=_Obj(available=True, enabled=True, speedCluster=cluster_mph * CV.MPH_TO_MS))


_CC = _Obj(enabled=True, cruiseControl=_Obj(override=False, cancel=False, resume=False))


class TestIcbmRaisesSetSpeed:
  """The cluster set speed follows the planner target up, however far it is above the engagement speed,
  once the short guard right after engagement has passed."""

  def _run(self, v_ego_mph, cluster_mph, target_mph, frames=500):
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=True, icbm=True)
    icbm = IntelligentCruiseButtonManagement(CI.CP, CI.CP_SP)
    lp_sp = _Obj(vTarget=target_mph * CV.MPH_TO_MS)
    buttons = []
    for _ in range(frames):
      icbm.run(_cs(v_ego_mph, cluster_mph), _CC, lp_sp, False)
      buttons.append(icbm.cruise_button)
    return icbm, buttons

  @pytest.mark.parametrize("engage_mph, target_mph", [(25, 28), (25, 50), (25, 72), (45, 72)])
  def test_increase_not_capped_by_engagement_speed(self, engage_mph, target_mph):
    icbm, buttons = self._run(engage_mph, engage_mph, target_mph)
    assert icbm.v_target == target_mph
    assert buttons[-1] == SendButtonState.increase

  def _controller(self):
    from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management import controller
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=True, icbm=True)
    return IntelligentCruiseButtonManagement(CI.CP, CI.CP_SP), int(controller.ENGAGE_GUARD_TIME * 100)

  @pytest.mark.parametrize("engage_mph, target_mph, capped_at", [(25, 28, 28), (25, 30, 30), (25, 31, 30), (25, 72, 30), (0, 40, 5)])
  def test_engage_guard_caps_the_target_then_lifts(self, engage_mph, target_mph, capped_at):
    icbm, guard_frames = self._controller()
    lp_sp = _Obj(vTarget=target_mph * CV.MPH_TO_MS)
    cluster = max(engage_mph, 1)
    for _ in range(guard_frames - 1):
      icbm.run(_cs(engage_mph, cluster), _CC, lp_sp, False)
      assert icbm.v_target <= capped_at
    assert icbm.v_target == capped_at
    for _ in range(100):
      icbm.run(_cs(engage_mph, cluster), _CC, lp_sp, False)
    assert icbm.v_target == target_mph
    assert icbm.cruise_button == SendButtonState.increase

  def test_engage_guard_survives_carcontrol_enabling_late(self):
    # the PCM reports engaged before carControl.enabled follows; the guard must still be running after
    icbm, guard_frames = self._controller()
    lp_sp = _Obj(vTarget=72 * CV.MPH_TO_MS)
    cc_late = _Obj(enabled=False, cruiseControl=_Obj(override=False, cancel=False, resume=False))
    for _ in range(20):
      icbm.run(_cs(25, 25), cc_late, lp_sp, False)
      assert icbm.cruise_button == SendButtonState.none
    for _ in range(guard_frames - 30):
      icbm.run(_cs(25, 25), _CC, lp_sp, False)
      assert icbm.v_target == 30

  def test_engage_guard_holds_a_target_that_jumps_mid_ramp(self):
    icbm, _ = self._controller()
    for _ in range(100):
      icbm.run(_cs(25, 25), _CC, _Obj(vTarget=28 * CV.MPH_TO_MS), False)
    assert icbm.cruise_button == SendButtonState.increase
    for _ in range(50):
      icbm.run(_cs(25, 27), _CC, _Obj(vTarget=72 * CV.MPH_TO_MS), False)
      assert icbm.v_target == 30

  def test_engage_guard_never_lowers_a_higher_cluster(self):
    # resumed to a 65 mph set speed while doing 30: the guard must not walk the cluster down to 35
    icbm, guard_frames = self._controller()
    buttons = []
    for _ in range(guard_frames + 100):
      icbm.run(_cs(30, 65), _CC, _Obj(vTarget=65 * CV.MPH_TO_MS), False)
      buttons.append(icbm.cruise_button)
    assert set(buttons) == {SendButtonState.none}

  def test_engage_guard_survives_override_and_restarts_on_re_engage(self):
    icbm, guard_frames = self._controller()
    lp_sp = _Obj(vTarget=72 * CV.MPH_TO_MS)
    cc_override = _Obj(enabled=True, cruiseControl=_Obj(override=True, cancel=False, resume=False))
    for _ in range(50):
      icbm.run(_cs(25, 25), _CC, lp_sp, False)
    for _ in range(50):
      icbm.run(_cs(25, 25), cc_override, lp_sp, False)   # gas pedal: not an engagement change
      assert icbm.v_target == 30
    for _ in range(guard_frames):
      icbm.run(_cs(25, 25), _CC, lp_sp, False)
    assert icbm.v_target == 72
    off = _Obj(vEgo=25 * CV.MPH_TO_MS, buttonEvents=[], cruiseState=_Obj(available=True, enabled=False, speedCluster=25 * CV.MPH_TO_MS))
    for _ in range(10):
      icbm.run(off, _Obj(enabled=False, cruiseControl=_Obj(override=False, cancel=False, resume=False)), lp_sp, False)
    for _ in range(100):
      icbm.run(_cs(25, 25), _CC, lp_sp, False)
      assert icbm.v_target == 30

  def test_decrease(self):
    _, buttons = self._run(45, 45, 30)
    assert buttons[-1] == SendButtonState.decrease

  def test_hold_when_equal(self):
    _, buttons = self._run(45, 45, 45)
    assert set(buttons) == {SendButtonState.none}

  @pytest.mark.parametrize("bad_target_mph", [0, 90, 200])
  @pytest.mark.parametrize("engage_mph, cluster_mph", [(25, 25), (30, 70)])
  def test_invalid_target_holds_the_cluster(self, bad_target_mph, engage_mph, cluster_mph):
    # no presses either way: not up to a bogus target, and not down to the engagement speed
    icbm, buttons = self._run(engage_mph, cluster_mph, bad_target_mph)
    assert icbm.v_target == cluster_mph
    assert set(buttons) == {SendButtonState.none}

  def test_no_increase_from_unset_cluster(self):
    _, buttons = self._run(25, 0, 50)
    assert SendButtonState.increase not in buttons


class TestSpeedLimitAssistAutoConfirm:
  """Ford + ICBM under openpilot longitudinal: the limit is applied with no +/- press, at any speed."""

  def _sla(self, icbm=True, car=FORD.FORD_EXPEDITION_MK4):
    CI, _ = _setup(car, op_long=True, icbm=icbm)
    sla = SpeedLimitAssist(CI.CP, CI.CP_SP)
    sla.enabled = True
    self.states = []
    return sla

  def _step(self, sla, limit_mph, set_mph, frames=40, engaged=True):
    limit = limit_mph * CV.MPH_TO_MS
    for _ in range(frames):
      sla.update(engaged, False, set_mph * CV.MPH_TO_MS, 0., set_mph * CV.MPH_TO_MS, limit, limit, limit_mph > 0, 0., EventsSP())
      sla.enabled = True  # params are re-read periodically; keep Assist mode on for the test
      self.states.append(sla.state)

  @pytest.mark.parametrize("limit_mph", [25, 35, 45, 65])
  def test_first_limit_applies_without_a_press(self, limit_mph):
    sla = self._sla()
    assert sla.auto_confirm
    self._step(sla, limit_mph, set_mph=30)
    assert sla.state in ACTIVE_STATES
    assert SpeedLimitAssistState.preActive not in self.states
    assert sla.output_v_target == pytest.approx(limit_mph * CV.MPH_TO_MS)

  def test_new_limit_below_50_applies_without_a_press(self):
    sla = self._sla()
    self._step(sla, 25, set_mph=25)
    self._step(sla, 45, set_mph=25)
    assert sla.state in ACTIVE_STATES
    assert SpeedLimitAssistState.preActive not in self.states
    assert sla.output_v_target == pytest.approx(45 * CV.MPH_TO_MS)

  def test_manual_set_speed_change_holds_until_next_limit(self):
    sla = self._sla()
    self._step(sla, 45, set_mph=45)
    assert sla.state in ACTIVE_STATES
    self._step(sla, 45, set_mph=52)            # driver bumps the set speed
    assert sla.state == SpeedLimitAssistState.inactive
    self._step(sla, 45, set_mph=52, frames=200)  # same limit: the driver's choice stands
    assert sla.state == SpeedLimitAssistState.inactive
    self._step(sla, 35, set_mph=52)            # a new limit takes over again
    assert sla.state in ACTIVE_STATES
    assert SpeedLimitAssistState.preActive not in self.states

  def test_no_limit_means_not_active(self):
    sla = self._sla()
    self._step(sla, 0, set_mph=40)
    assert sla.state not in ACTIVE_STATES

  def test_disengaged_means_disabled(self):
    sla = self._sla()
    self._step(sla, 45, set_mph=45)
    self._step(sla, 45, set_mph=45, engaged=False, frames=2)
    assert sla.state == SpeedLimitAssistState.disabled

  def test_without_icbm_nothing_is_auto_confirmed(self):
    sla = self._sla(icbm=False)
    assert not sla.auto_confirm and sla.pcm_op_long
    self._step(sla, 45, set_mph=35)
    assert sla.state not in ACTIVE_STATES      # still waits for the 70/80 confirmation

  def test_other_brand_is_not_auto_confirmed(self):
    assert not self._sla(car=HYUNDAI.HYUNDAI_SONATA).auto_confirm


class TestSignToClusterChain:
  """Cruise helper + Speed Limit Assist + ICBM stepped together the way card, plannerd and selfdrived run
  them, with a simulated Ford cluster that moves 1 mph per button press."""

  def _drive(self, limits_mph, engage_mph=30, seconds_per_limit=12):
    from cereal import car
    from openpilot.selfdrive.car.cruise import VCruiseHelper
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=True, icbm=True)
    helper = VCruiseHelper(CI.CP, CI.CP_SP)
    sla = SpeedLimitAssist(CI.CP, CI.CP_SP)
    sla.enabled = True
    icbm = IntelligentCruiseButtonManagement(CI.CP, CI.CP_SP)

    cluster_mph = engage_mph   # the PCM's own setpoint, as shown on the cluster
    enabled_prev = False
    log = []
    for limit_mph in limits_mph:
      limit = limit_mph * CV.MPH_TO_MS
      for frame in range(seconds_per_limit * 100):
        CS = car.CarState.new_message()
        CS.vEgo = engage_mph * CV.MPH_TO_MS
        CS.cruiseState.available = True
        CS.cruiseState.enabled = True
        CS.cruiseState.speed = cluster_mph * CV.MPH_TO_MS
        CS.cruiseState.speedCluster = cluster_mph * CV.MPH_TO_MS

        # plannerd (20 Hz): Speed Limit Assist sees openpilot's own set speed
        if frame % 5 == 0:
          v_cruise_ms = min(helper.v_cruise_cluster_kph, 145) * CV.KPH_TO_MS
          sla.update(True, False, CS.vEgo, 0., v_cruise_ms, limit, limit, True, 0., EventsSP())
          sla.enabled = True
          lp_sp = custom.LongitudinalPlanSP.new_message()
          lp_sp.speedLimit.resolver.speedLimitValid = True
          lp_sp.speedLimit.resolver.speedLimitFinalLast = limit
          lp_sp.speedLimit.assist.state = sla.state
          lp_sp.vTarget = float(min(helper.v_cruise_kph * CV.KPH_TO_MS, sla.output_v_target))

        # card (100 Hz)
        helper.update_speed_limit_assist(False, lp_sp)
        helper.update_v_cruise(CS, True, False)
        if not enabled_prev:
          helper.initialize_v_cruise(CS, False, False)
        enabled_prev = True

        # selfdrived (100 Hz) -> car controller -> PCM: one press moves the cluster 1 mph, at most 10 Hz
        icbm.run(CS, _CC, lp_sp, False)
        if frame % 10 == 0:
          if icbm.cruise_button == SendButtonState.increase:
            cluster_mph += 1
          elif icbm.cruise_button == SendButtonState.decrease:
            cluster_mph -= 1
      log.append((limit_mph, round(helper.v_cruise_kph * CV.KPH_TO_MPH), cluster_mph, sla.state))
    return log

  def test_cluster_follows_each_limit_with_no_driver_input(self):
    log = self._drive([45, 65, 25, 35])
    for limit_mph, set_speed_mph, cluster_mph, state in log:
      assert state in ACTIVE_STATES, log
      assert set_speed_mph == limit_mph, log    # openpilot's set speed is the limit, not 70/80
      assert cluster_mph == limit_mph, log      # and the truck's cluster was walked to it


class TestMinimumSetSpeedUnits:
  def test_imperial_minimum_is_20_mph_in_kph(self):
    from openpilot.selfdrive.car.cruise import VCruiseHelper
    CI, _ = _setup(FORD.FORD_EXPEDITION_MK4, op_long=True, icbm=True)
    helper = VCruiseHelper(CI.CP, CI.CP_SP)
    helper.get_minimum_set_speed(False)
    assert helper.v_cruise_min == pytest.approx(20 * CV.MPH_TO_KPH)
    helper.get_minimum_set_speed(True)
    assert helper.v_cruise_min == 30
