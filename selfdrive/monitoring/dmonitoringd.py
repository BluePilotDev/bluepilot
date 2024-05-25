#!/usr/bin/env python3
import gc

import cereal.messaging as messaging
from cereal import car
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.controls.lib.events import Events
from openpilot.selfdrive.monitoring.driver_monitor import DriverStatus
from openpilot.selfdrive.monitoring.hands_on_wheel_monitor import HandsOnWheelStatus


def dmonitoringd_thread():
  gc.disable()
  set_realtime_priority(2)

  params = Params()
  pm = messaging.PubMaster(['driverMonitoringState', 'driverMonitoringStateSP'])
  sm = messaging.SubMaster(['driverStateV2', 'liveCalibration', 'carState', 'controlsState', 'modelV2'], poll='driverStateV2')

  driver_status = DriverStatus(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"))
  hands_on_wheel_status = HandsOnWheelStatus()

  v_cruise_last = 0
  driver_engaged = False
  steering_wheel_engaged = False
  hands_on_wheel_monitoring_enabled = params.get_bool("HandsOnWheelMonitoring")

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()
    if not sm.updated['driverStateV2']:
      continue

    # Get interaction
    if sm.updated['carState']:
      v_cruise = sm['carState'].cruiseState.speed
      steering_wheel_engaged = len(sm['carState'].buttonEvents) > 0 or \
        v_cruise != v_cruise_last or \
        sm['carState'].steeringPressed
      driver_engaged = steering_wheel_engaged or sm['carState'].gasPressed
      # Update events and state from hands on wheel monitoring status when steering wheel in engaged
      if steering_wheel_engaged and hands_on_wheel_monitoring_enabled:
        hands_on_wheel_status.update(Events(), True, sm['controlsState'].enabled, sm['carState'].vEgo)
      v_cruise_last = v_cruise

    if sm.updated['modelV2']:
      driver_status.set_policy(sm['modelV2'], sm['carState'].vEgo)

    # Get data from dmonitoringmodeld
    events = Events()

    if sm.all_checks() and len(sm['liveCalibration'].rpyCalib):
      driver_status.update_states(sm['driverStateV2'], sm['liveCalibration'].rpyCalib, sm['carState'].vEgo, sm['controlsState'].enabled)

    # Block engaging after max number of distrations or when alert active
    if driver_status.terminal_alert_cnt >= driver_status.settings._MAX_TERMINAL_ALERTS or \
       driver_status.terminal_time >= driver_status.settings._MAX_TERMINAL_DURATION or \
       driver_status.always_on and driver_status.awareness <= driver_status.threshold_prompt:
      events.add(car.CarEvent.EventName.tooDistracted)

    # Update events from driver state
    driver_status.update_events(events, driver_engaged, sm['controlsState'].enabled,
      sm['carState'].standstill, sm['carState'].gearShifter in [car.CarState.GearShifter.reverse, car.CarState.GearShifter.park], sm['carState'].vEgo)
    # Update events and state from hands on wheel monitoring status
    if hands_on_wheel_monitoring_enabled:
      hands_on_wheel_status.update(events, steering_wheel_engaged, sm['controlsState'].enabled, sm['carState'].vEgo)

    # build driverMonitoringState packet
    dat = messaging.new_message('driverMonitoringState', valid=sm.all_checks())
    dat.driverMonitoringState = {
      "events": events.to_msg(),
      "faceDetected": driver_status.face_detected,
      "isDistracted": driver_status.driver_distracted,
      "distractedType": sum(driver_status.distracted_types),
      "awarenessStatus": driver_status.awareness,
      "posePitchOffset": driver_status.pose.pitch_offseter.filtered_stat.mean(),
      "posePitchValidCount": driver_status.pose.pitch_offseter.filtered_stat.n,
      "poseYawOffset": driver_status.pose.yaw_offseter.filtered_stat.mean(),
      "poseYawValidCount": driver_status.pose.yaw_offseter.filtered_stat.n,
      "stepChange": driver_status.step_change,
      "awarenessActive": driver_status.awareness_active,
      "awarenessPassive": driver_status.awareness_passive,
      "isLowStd": driver_status.pose.low_std,
      "hiStdCount": driver_status.hi_stds,
      "isActiveMode": driver_status.active_monitoring_mode,
      "isRHD": driver_status.wheel_on_right,
    }
    pm.send('driverMonitoringState', dat)

    if sm['driverStateV2'].frameId % 40 == 1:
      driver_status.always_on = params.get_bool("AlwaysOnDM")

    sp_dat = messaging.new_message('driverMonitoringStateSP')
    sp_dat.driverMonitoringStateSP = {
      "handsOnWheelState": hands_on_wheel_status.hands_on_wheel_state,
    }
    pm.send('driverMonitoringStateSP', sp_dat)

    # save rhd virtual toggle every 5 mins
    if (sm['driverStateV2'].frameId % 6000 == 0 and
     driver_status.wheelpos_learner.filtered_stat.n > driver_status.settings._WHEELPOS_FILTER_MIN_COUNT and
     driver_status.wheel_on_right == (driver_status.wheelpos_learner.filtered_stat.M > driver_status.settings._WHEELPOS_THRESHOLD)):
      params.put_bool_nonblocking("IsRhdDetected", driver_status.wheel_on_right)

def main():
  dmonitoringd_thread()


if __name__ == '__main__':
  main()
