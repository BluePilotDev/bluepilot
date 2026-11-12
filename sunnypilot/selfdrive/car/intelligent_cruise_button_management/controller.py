"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from cereal import car, custom
from opendbc.car import structs, apply_hysteresis
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

ALLOWED_SPEED_THRESHOLD = 1.8  # m/s, ~4 MPH
HYST_GAP = 0.0  # currently disabled; TODO-SP: might need to be brand-specific
INACTIVE_TIMER = 0.4
# BluePilot: for this long after cruise is enabled, don't raise the set speed more than
# ENGAGE_GUARD_MAX_INCREASE (cluster units) above the speed it was enabled at. The planner target can
# be briefly wrong right at engagement, and following it would ramp the set speed up within seconds.
ENGAGE_GUARD_TIME = 3.0  # secs.
ENGAGE_GUARD_MAX_INCREASE = 5


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.v_target_ms_last = 0.0
    self.is_metric = False

    self.cruise_button_timers = CRUISE_BUTTON_TIMER
    
    # BluePilot: Track initial cruise speed when first enabled, in cluster units (mph or km/h)
    self.initial_cruise_speed = 0
    self.cruise_enabled_prev = False
    self.engage_guard_timer = 0

  @property
  def v_cruise_equal(self) -> bool:
    return self.v_target == self.v_cruise_cluster

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    ms_conv = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS

    # BluePilot: Capture initial cruise speed when cruise is first enabled
    # Use current speed (vEgo) as the initial setpoint if planner target is invalid/unreasonable
    cruise_enabled = CS.cruiseState.available and CS.cruiseState.enabled
    if cruise_enabled and not self.cruise_enabled_prev:
      # Cruise just enabled - capture current speed as initial setpoint, in the same cluster
      # units as v_target and v_cruise_cluster (it used to be stored in km/h and compared to mph)
      self.initial_cruise_speed = round(CS.vEgo * speed_conv)
      self.engage_guard_timer = int(ENGAGE_GUARD_TIME / DT_CTRL)
    elif not cruise_enabled:
      # Only the PCM actually dropping cruise ends an engagement. carControl.enabled lags the PCM
      # by a few frames at engagement, and override/resume are temporary, so none of those reset this.
      self.initial_cruise_speed = 0
      self.engage_guard_timer = 0
    else:
      self.engage_guard_timer = max(0, self.engage_guard_timer - 1)
    self.cruise_enabled_prev = cruise_enabled

    self.v_target_ms_last = apply_hysteresis(LP_SP.vTarget, self.v_target_ms_last, HYST_GAP * ms_conv)

    self.v_target = round(self.v_target_ms_last * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

    # BluePilot: a target of 0, or at the top of the range (an unset set speed is clipped to it),
    # is not something to chase. Hold the cluster where it is: no presses either way.
    MAX_REASONABLE_TARGET = 145 if self.is_metric else 90
    if self.v_target >= MAX_REASONABLE_TARGET or self.v_target == 0:
      self.v_target = self.v_cruise_cluster

    # BluePilot: engage guard. For ENGAGE_GUARD_TIME after the PCM engages, the target is capped at
    # ENGAGE_GUARD_MAX_INCREASE above the engagement speed. Capping the target (not just the entry
    # into "increasing") means a target that jumps mid-ramp is held too. The cap never drops below
    # the current cluster speed, so the guard can't cause a decrease by itself.
    if self.engage_guard_timer > 0:
      self.v_target = min(self.v_target, max(self.initial_cruise_speed + ENGAGE_GUARD_MAX_INCREASE, self.v_cruise_cluster))

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_cruise_equal:
              self.state = State.holding

            elif self.v_target > self.v_cruise_cluster:
              # BluePilot: Prevent ICBM from increasing speed when cruise is first enabled
              # If cluster speed is 0 or very low, don't increase - wait for user to set initial speed
              # Also cap target to reasonable maximum (145 kph / 90 mph)
              # (the engage guard and invalid targets are handled in update_calculations)
              if self.v_cruise_cluster == 0:
                # Don't increase - stay in preActive or go to holding
                self.state = State.holding
              else:
                self.state = State.increasing

            elif self.v_target < self.v_cruise_cluster and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

        # HOLDING
        elif self.state == State.holding:
          if not self.v_cruise_equal:
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    # BluePilot: Clear button timers when cruise is disabled to prevent stale presses
    # This ensures that when cruise is re-enabled, ICBM doesn't see stale button presses
    if not ready:
      for k in self.cruise_button_timers:
        self.cruise_button_timers[k] = 0

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
