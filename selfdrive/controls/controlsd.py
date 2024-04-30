#!/usr/bin/env python3
import os
import math
import numpy as np
import time
import threading
from typing import SupportsFloat

import cereal.messaging as messaging

from cereal import car, log
from cereal.visionipc import VisionIpcClient, VisionStreamType

from panda import ALTERNATIVE_EXPERIENCE

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper, DT_CTRL
from openpilot.common.swaglog import cloudlog

from openpilot.selfdrive.athena.registration import is_registered_device
from openpilot.selfdrive.boardd.boardd import can_list_to_can_capnp
from openpilot.selfdrive.car.car_helpers import get_car, get_startup_event, get_one_can
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.controls.lib.alertmanager import AlertManager, set_offroad_alert
from openpilot.selfdrive.controls.lib.drive_helpers import VCruiseHelper, clip_curvature, get_lag_adjusted_curvature
from openpilot.selfdrive.controls.lib.events import Events, ET
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel
from openpilot.selfdrive.sunnypilot import get_model_generation

from openpilot.system.hardware import HARDWARE
from openpilot.system.version import get_short_branch

SOFT_DISABLE_TIME = 3  # seconds
LDW_MIN_SPEED = 31 * CV.MPH_TO_MS
LANE_DEPARTURE_THRESHOLD = 0.1
CAMERA_OFFSET = 0.04

REPLAY = "REPLAY" in os.environ
SIMULATION = "SIMULATION" in os.environ
TESTING_CLOSET = "TESTING_CLOSET" in os.environ
IGNORE_PROCESSES = {"loggerd", "encoderd", "statsd", "mapd", "gpxd", "gpxd_uploader", "mapd", "otisserv", "fleet_manager"}

ThermalStatus = log.DeviceState.ThermalStatus
State = log.ControlsState.OpenpilotState
PandaType = log.PandaState.PandaType
Desire = log.Desire
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type
SafetyModel = car.CarParams.SafetyModel
GearShifter = car.CarState.GearShifter

IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)
CSID_MAP = {"1": EventName.roadCameraError, "2": EventName.wideRoadCameraError, "0": EventName.driverCameraError}
ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
ACTIVE_STATES = (State.enabled, State.softDisabling, State.overriding)
ENABLED_STATES = (State.preEnabled, *ACTIVE_STATES)


class CarD:
  CI: CarInterfaceBase
  CS: car.CarState

  def __init__(self, CI=None):
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams'])

    self.can_rcv_timeout_counter = 0      # conseuctive timeout count
    self.can_rcv_cum_timeout_counter = 0  # cumulative timeout count

    self.params = Params()

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      get_one_can(self.can_sock)

      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)
      experimental_long_allowed = self.params.get_bool("ExperimentalLongitudinalEnabled")
      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], experimental_long_allowed, num_pandas)
    else:
      self.CI, self.CP = CI, CI.CP

    # set alternative experiences from parameters
    disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
    enable_mads = self.params.get_bool("EnableMads")
    mads_disengage_lateral_on_brake = self.params.get_bool("DisengageLateralOnBrake")
    mads_dlob = enable_mads and mads_disengage_lateral_on_brake
    mads_ndlob = enable_mads and not mads_disengage_lateral_on_brake
    self.CP.alternativeExperience = 0
    if not disengage_on_accelerator:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS
    if mads_dlob:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ENABLE_MADS
    elif mads_ndlob:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.MADS_DISABLE_DISENGAGE_LATERAL_ON_BRAKE

    if self.CP.customStockLongAvailable and self.CP.pcmCruise and self.params.get_bool("CustomStockLong"):
      self.CP.pcmCruiseSpeed = False

    car_recognized = self.CP.carName != 'mock'
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")

    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly

    self.CP.passive = not car_recognized or not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = car.CarParams.SafetyConfig.new_message()
      safety_config.safetyModel = car.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp)

    # Write CarParams for radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes)
    self.params.put_nonblocking("CarParamsCache", cp_bytes)
    self.params.put_nonblocking("CarParamsPersistent", cp_bytes)

  def initialize(self):
    """Initialize CarInterface, once controls are ready"""
    self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])

  def state_update(self, CC: car.CarControl):
    """carState update loop, driven by can"""

    # TODO: This should not depend on carControl

    # Update carState from CAN
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    self.CS = self.CI.update(CC, can_strs)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_timeout_counter += 1
      self.can_rcv_cum_timeout_counter += 1
    else:
      self.can_rcv_timeout_counter = 0

    self.can_rcv_timeout = self.can_rcv_timeout_counter >= 5

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    return self.CS

  def state_publish(self, car_events):
    """carState and carParams publish loop"""

    # TODO: carState should be independent of the event loop

    # carState
    cs_send = messaging.new_message('carState')
    cs_send.valid = self.CS.canValid
    cs_send.carState = self.CS
    cs_send.carState.events = car_events
    self.pm.send('carState', cs_send)

    # carParams - logged every 50 seconds (> 1 per segment)
    if (self.sm.frame % int(50. / DT_CTRL) == 0):
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

  def controls_update(self, CC: car.CarControl):
    """control update loop, driven by carControl"""

    # send car controls over can
    now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
    actuators_output, can_sends = self.CI.apply(CC, now_nanos)
    self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=self.CS.canValid))

    return actuators_output


class Controls:
  def __init__(self, CI=None):
    self.card = CarD(CI)

    self.CP = self.card.CP
    self.CI = self.card.CI

    config_realtime_process(4, Priority.CTRL_HIGH)

    # Ensure the current branch is cached, otherwise the first iteration of controlsd lags
    self.branch = get_short_branch()

    # Setup sockets
    self.pm = messaging.PubMaster(['controlsState', 'carControl', 'onroadEvents', 'controlsStateSP'])

    self.sensor_packets = ["accelerometer", "gyroscope"]
    self.camera_packets = ["roadCameraState", "driverCameraState", "wideRoadCameraState"]

    self.log_sock = messaging.sub_sock('androidLog')

    self.params = Params()

    self.d_camera_hardware_missing = self.params.get_bool("DriverCameraHardwareMissing") and not is_registered_device()
    if self.d_camera_hardware_missing:
      IGNORE_PROCESSES.update({"dmonitoringd", "dmonitoringmodeld"})
      self.camera_packets.remove("driverCameraState")

    ignore = self.sensor_packets + ['testJoystick']
    if SIMULATION:
      ignore += ['driverCameraState', 'managerState']
    if self.d_camera_hardware_missing:
      ignore += ['driverMonitoringState']
    lateral_plan_svs = ['lateralPlanDEPRECATED', 'lateralPlanSPDEPRECATED']
    self.sm = messaging.SubMaster(['deviceState', 'pandaStates', 'peripheralState', 'modelV2', 'liveCalibration',
                                   'driverMonitoringState', 'longitudinalPlan', 'liveLocationKalman',
                                   'managerState', 'liveParameters', 'radarState', 'liveTorqueParameters',
                                   'testJoystick', 'longitudinalPlanSP', 'modelV2SP'] + self.camera_packets + self.sensor_packets + lateral_plan_svs,
                                  ignore_alive=ignore, ignore_avg_freq=ignore+['radarState', 'testJoystick'], ignore_valid=['testJoystick', ],
                                  frequency=int(1/DT_CTRL))

    self.joystick_mode = self.params.get_bool("JoystickDebugMode")

    # read params
    self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
    self.is_metric = self.params.get_bool("IsMetric")
    self.is_ldw_enabled = self.params.get_bool("IsLdwEnabled")
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")

    # detect sound card presence and ensure successful init
    sounds_available = HARDWARE.get_sound_card_online()

    car_recognized = self.CP.carName != 'mock'

    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly

    # cleanup old params
    if not self.CP.experimentalLongitudinalAvailable:
      self.params.remove("ExperimentalLongitudinalEnabled")
    if not self.CP.openpilotLongitudinalControl:
      self.params.remove("ExperimentalMode")

    self.CC = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.AM = AlertManager()
    self.events = Events()

    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)

    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)

    self.initialized = False
    self.state = State.disabled
    self.enabled = False
    self.enabled_long = False
    self.active = False
    self.soft_disable_timer = 0
    self.mismatch_counter = 0
    self.cruise_mismatch_counter = 0
    self.last_blinker_frame = 0
    self.last_steering_pressed_frame = 0
    self.distance_traveled = 0
    self.last_functional_fan_frame = 0
    self.events_prev = []
    self.current_alert_types = [ET.PERMANENT]
    self.logged_comm_issue = None
    self.not_running_prev = None
    self.last_actuators = car.CarControl.Actuators.new_message()
    self.steer_limited = False
    self.desired_curvature = 0.0
    self.experimental_mode = False
    self.v_cruise_helper = VCruiseHelper(self.CP)
    self.recalibrating_seen = False
    self.nn_alert_shown = False
    self.enable_nnff = self.params.get_bool("NNFF")

    self.lane_change_set_timer = int(self.params.get("AutoLaneChangeTimer", encoding="utf8"))
    self.reverse_acc_change = False
    self.dynamic_experimental_control = False

    self.live_torque = self.params.get_bool("LiveTorque")
    self.torqued_override = self.params.get_bool("TorquedOverride")
    self.custom_stock_planner_speed = self.params.get_bool("CustomStockLongPlanner")

    self.enable_mads = self.params.get_bool("EnableMads")
    self.mads_disengage_lateral_on_brake = self.params.get_bool("DisengageLateralOnBrake")
    self.mads_ndlob = self.enable_mads and not self.mads_disengage_lateral_on_brake
    self.process_not_running = False

    self.custom_model, self.model_gen = get_model_generation(self.params)
    self.model_use_lateral_planner = self.custom_model and self.model_gen == 1

    self.can_log_mono_time = 0

    self.startup_event = get_startup_event(car_recognized, controller_available, len(self.CP.carFw) > 0)

    if not sounds_available:
      self.events.add(EventName.soundsUnavailable, static=True)
    if not car_recognized:
      self.events.add(EventName.carUnrecognized, static=True)
      if len(self.CP.carFw) > 0:
        set_offroad_alert("Offroad_CarUnrecognized", True)
      else:
        set_offroad_alert("Offroad_NoFirmware", True)
    elif self.CP.passive:
      self.events.add(EventName.dashcamMode, static=True)

    # controlsd is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

  def set_initial_state(self):
    if REPLAY:
      controls_state = Params().get("ReplayControlsState")
      if controls_state is not None:
        with log.ControlsState.from_bytes(controls_state) as controls_state:
          self.v_cruise_helper.v_cruise_kph = controls_state.vCruise

      if any(ps.controlsAllowed for ps in self.sm['pandaStates']):
        self.state = State.enabled

  def get_road_edge(self, carstate, model_v2):
    # Lane detection by FrogAi
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    if not self.edge_toggle:
      self.road_edge = False
    elif one_blinker:
      # Set the minimum lane threshold to 3.0 meters
      min_lane_threshold = 3.0
      # Set the blinker index based on which signal is on
      blinker_index = 0 if carstate.leftBlinker else 1
      desired_edge = model_v2.roadEdges[blinker_index]
      current_lane = model_v2.laneLines[blinker_index + 1]
      # Check if both the desired lane and the current lane have valid x and y values
      if all([desired_edge.x, desired_edge.y, current_lane.x, current_lane.y]) and len(desired_edge.x) == len(current_lane.x):
        # Interpolate the x and y values to the same length
        x = np.linspace(desired_edge.x[0], desired_edge.x[-1], num=len(desired_edge.x))
        lane_y = np.interp(x, current_lane.x, current_lane.y)
        desired_y = np.interp(x, desired_edge.x, desired_edge.y)
        # Calculate the width of the lane we're wanting to change into
        lane_width = np.abs(desired_y - lane_y)
        # Set road_edge to False if the lane width is not larger than the threshold
        self.road_edge = not (np.amax(lane_width) > min_lane_threshold)
      else:
        self.road_edge = True
    else:
      # Default to setting "road_edge" to False
      self.road_edge = False

  def update_events(self, CS):
    """Compute onroadEvents from carState"""

    self.events.clear()

    # Add joystick event, static on cars, dynamic on nonCars
    if self.joystick_mode:
      self.events.add(EventName.joystickDebug)
      self.startup_event = None

    # Add startup event
    if self.startup_event is not None:
      self.events.add(self.startup_event)
      self.startup_event = None

    # Don't add any more events if not initialized
    if not self.initialized:
      self.events.add(EventName.controlsInitializing)
      return

    # no more events while in dashcam mode
    if self.CP.passive:
      return

    # show alert to indicate whether NNFF is loaded
    if self.enable_nnff and not self.nn_alert_shown and self.sm.frame % 1000 == 0 and \
            self.CP.lateralTuning.which() == 'torque':
      self.nn_alert_shown = True
      self.events.add(EventName.torqueNNLoad)

    # Block resume if cruise never previously enabled
    resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
    if not self.CP.pcmCruise and not self.v_cruise_helper.v_cruise_initialized and resume_pressed:
      self.events.add(EventName.resumeBlocked)

    # Disable on rising edge of accelerator or brake. Also disable on brake when speed > 0
    if (CS.gasPressed and not self.CS_prev.gasPressed and self.disengage_on_accelerator) or \
      (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) or \
      (CS.regenBraking and (not self.CS_prev.regenBraking or not CS.standstill)):
      if CS.cruiseState.enabled or not self.enable_mads:
        self.events.add(EventName.pedalPressed)
      elif not self.mads_ndlob:
        self.events.add(EventName.silentPedalPressed)

    if CS.brakePressed and CS.standstill and CS.cruiseState.enabled:
      self.events.add(EventName.preEnableStandstill)

    if CS.gasPressed and CS.cruiseState.enabled:
      self.events.add(EventName.gasPressedOverride)

    if not self.CP.notCar:
      if not self.d_camera_hardware_missing:
        self.events.add_from_msg(self.sm['driverMonitoringState'].events)
      self.events.add_from_msg(self.sm['longitudinalPlanSP'].events)

    # Add car events, ignore if CAN isn't valid
    if CS.canValid:
      self.events.add_from_msg(CS.events)

    # Create events for temperature, disk space, and memory
    if self.sm['deviceState'].thermalStatus >= ThermalStatus.red:
      self.events.add(EventName.overheat)
    if self.sm['deviceState'].freeSpacePercent < 7 and not SIMULATION:
      # under 7% of space free no enable allowed
      self.events.add(EventName.outOfSpace)
    if self.sm['deviceState'].memoryUsagePercent > 90 and not SIMULATION:
      self.events.add(EventName.lowMemory)

    # TODO: enable this once loggerd CPU usage is more reasonable
    #cpus = list(self.sm['deviceState'].cpuUsagePercent)
    #if max(cpus, default=0) > 95 and not SIMULATION:
    #  self.events.add(EventName.highCpuUsage)

    # Alert if fan isn't spinning for 5 seconds
    if self.sm['peripheralState'].pandaType != log.PandaState.PandaType.unknown:
      if self.sm['peripheralState'].fanSpeedRpm < 500 and self.sm['deviceState'].fanSpeedPercentDesired > 50:
        # allow enough time for the fan controller in the panda to recover from stalls
        if (self.sm.frame - self.last_functional_fan_frame) * DT_CTRL > 15.0:
          self.events.add(EventName.fanMalfunction)
      else:
        self.last_functional_fan_frame = self.sm.frame

    # Handle calibration status
    cal_status = self.sm['liveCalibration'].calStatus
    if cal_status != log.LiveCalibrationData.Status.calibrated:
      if cal_status == log.LiveCalibrationData.Status.uncalibrated:
        self.events.add(EventName.calibrationIncomplete)
      elif cal_status == log.LiveCalibrationData.Status.recalibrating:
        if not self.recalibrating_seen:
          set_offroad_alert("Offroad_Recalibration", True)
        self.recalibrating_seen = True
        self.events.add(EventName.calibrationRecalibrating)
      else:
        self.events.add(EventName.calibrationInvalid)

    # Handle lane change
    lane_change_edge_block = self.sm['lateralPlanSPDEPRECATED'].laneChangeEdgeBlockDEPRECATED if self.model_use_lateral_planner else self.sm['modelV2SP'].laneChangeEdgeBlock
    lane_change_svs = self.sm['lateralPlanDEPRECATED'] if self.model_use_lateral_planner else self.sm['modelV2'].meta
    if lane_change_svs.laneChangeState == LaneChangeState.preLaneChange and lane_change_edge_block:
      self.events.add(EventName.laneChangeRoadEdge)
    elif lane_change_svs.laneChangeState == LaneChangeState.preLaneChange:
      direction = lane_change_svs.laneChangeDirection
      if (CS.leftBlindspot and direction == LaneChangeDirection.left) or \
         (CS.rightBlindspot and direction == LaneChangeDirection.right):
        self.events.add(EventName.laneChangeBlocked)
      else:
        if direction == LaneChangeDirection.left:
          self.events.add(EventName.preLaneChangeLeft)
        else:
          self.events.add(EventName.preLaneChangeRight)
    elif lane_change_svs.laneChangeState in (LaneChangeState.laneChangeStarting,
                                                    LaneChangeState.laneChangeFinishing):
      self.events.add(EventName.laneChange)

    for i, pandaState in enumerate(self.sm['pandaStates']):
      # All pandas must match the list of safetyConfigs, and if outside this list, must be silent or noOutput
      if i < len(self.CP.safetyConfigs):
        safety_mismatch = pandaState.safetyModel != self.CP.safetyConfigs[i].safetyModel or \
                          pandaState.safetyParam != self.CP.safetyConfigs[i].safetyParam or \
                          pandaState.alternativeExperience != self.CP.alternativeExperience
      else:
        safety_mismatch = pandaState.safetyModel not in IGNORED_SAFETY_MODES

      # safety mismatch allows some time for boardd to set the safety mode and publish it back from panda
      if (safety_mismatch and self.sm.frame*DT_CTRL > 10.) or pandaState.safetyRxChecksInvalid or self.mismatch_counter >= 200:
        self.events.add(EventName.controlsMismatch)

      if log.PandaState.FaultType.relayMalfunction in pandaState.faults:
        self.events.add(EventName.relayMalfunction)

    # Handle HW and system malfunctions
    # Order is very intentional here. Be careful when modifying this.
    # All events here should at least have NO_ENTRY and SOFT_DISABLE.
    num_events = len(self.events)

    not_running = {p.name for p in self.sm['managerState'].processes if not p.running and p.shouldBeRunning}
    if self.sm.recv_frame['managerState'] and (not_running - IGNORE_PROCESSES):
      self.events.add(EventName.processNotRunning)
      self.process_not_running = True
      if not_running != self.not_running_prev:
        cloudlog.event("process_not_running", not_running=not_running, error=True)
      self.not_running_prev = not_running
    else:
      if not SIMULATION and not self.rk.lagging:
        if not self.sm.all_alive(self.camera_packets):
          self.events.add(EventName.cameraMalfunction)
          if not self.sm.all_alive(['driverCameraState']) and not self.d_camera_hardware_missing:
            self.d_camera_hardware_missing = True
            self.params.put_bool_nonblocking("DriverCameraHardwareMissing", True)
        elif not self.sm.all_freq_ok(self.camera_packets):
          self.events.add(EventName.cameraFrameRate)
      self.process_not_running = False
    if not REPLAY and self.rk.lagging:
      self.events.add(EventName.controlsdLagging)
    if len(self.sm['radarState'].radarErrors) or (not self.rk.lagging and not self.sm.all_checks(['radarState'])):
      self.events.add(EventName.radarFault)
    if not self.sm.valid['pandaStates']:
      self.events.add(EventName.usbError)
    if CS.canTimeout:
      self.events.add(EventName.canBusMissing)
    elif not CS.canValid:
      self.events.add(EventName.canError)

    # generic catch-all. ideally, a more specific event should be added above instead
    has_disable_events = self.events.contains(ET.NO_ENTRY) and (self.events.contains(ET.SOFT_DISABLE) or self.events.contains(ET.IMMEDIATE_DISABLE))
    no_system_errors = (not has_disable_events) or (len(self.events) == num_events)
    if (not self.sm.all_checks() or self.card.can_rcv_timeout) and no_system_errors:
      if not self.sm.all_alive():
        self.events.add(EventName.commIssue)
      elif not self.sm.all_freq_ok():
        self.events.add(EventName.commIssueAvgFreq)
      else:  # invalid or can_rcv_timeout.
        self.events.add(EventName.commIssue)

      logs = {
        'invalid': [s for s, valid in self.sm.valid.items() if not valid],
        'not_alive': [s for s, alive in self.sm.alive.items() if not alive],
        'not_freq_ok': [s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
        'can_rcv_timeout': self.card.can_rcv_timeout,
      }
      if logs != self.logged_comm_issue:
        cloudlog.event("commIssue", error=True, **logs)
        self.logged_comm_issue = logs
    else:
      self.logged_comm_issue = None

    if not (self.CP.notCar and self.joystick_mode):
      if self.model_use_lateral_planner:
        if not self.sm['lateralPlanDEPRECATED'].mpcSolutionValid:
          self.events.add(EventName.plannerErrorDEPRECATED)
      if not self.sm['liveLocationKalman'].posenetOK:
        self.events.add(EventName.posenetInvalid)
      if not self.sm['liveLocationKalman'].deviceStable:
        self.events.add(EventName.deviceFalling)
      if not self.sm['liveLocationKalman'].inputsOK:
        self.events.add(EventName.locationdTemporaryError)
      if not self.sm['liveParameters'].valid and not TESTING_CLOSET and (not SIMULATION or REPLAY):
        self.events.add(EventName.paramsdTemporaryError)

    # conservative HW alert. if the data or frequency are off, locationd will throw an error
    if any((self.sm.frame - self.sm.recv_frame[s])*DT_CTRL > 10. for s in self.sensor_packets):
      self.events.add(EventName.sensorDataInvalid)

    if not REPLAY:
      # Check for mismatch between openpilot and car's PCM
      cruise_mismatch = CS.cruiseState.enabled and not self.enabled
      self.cruise_mismatch_counter = self.cruise_mismatch_counter + 1 if cruise_mismatch else 0
      if self.cruise_mismatch_counter > int(6. / DT_CTRL):
        self.events.add(EventName.cruiseMismatch)

    # Check for FCW
    stock_long_is_braking = self.enabled and not self.CP.openpilotLongitudinalControl and CS.aEgo < -1.25
    model_fcw = self.sm['modelV2'].meta.hardBrakePredicted and not CS.brakePressed and not stock_long_is_braking
    planner_fcw = self.sm['longitudinalPlan'].fcw and self.enabled
    if planner_fcw or model_fcw:
      self.events.add(EventName.fcw)

    for m in messaging.drain_sock(self.log_sock, wait_for_one=False):
      try:
        msg = m.androidLog.message
        if any(err in msg for err in ("ERROR_CRC", "ERROR_ECC", "ERROR_STREAM_UNDERFLOW", "APPLY FAILED")):
          csid = msg.split("CSID:")[-1].split(" ")[0]
          evt = CSID_MAP.get(csid, None)
          if evt is not None:
            self.events.add(evt)
      except UnicodeDecodeError:
        pass

    # TODO: fix simulator
    if not SIMULATION or REPLAY:
      # Not show in first 1 km to allow for driving out of garage. This event shows after 5 minutes
      if not self.sm['liveLocationKalman'].gpsOK and self.sm['liveLocationKalman'].inputsOK and (self.distance_traveled > 1000):
        self.events.add(EventName.noGps)
      if self.sm['liveLocationKalman'].gpsOK:
        self.distance_traveled = 0
      self.distance_traveled += CS.vEgo * DT_CTRL

      if self.sm['modelV2'].frameDropPerc > 20:
        self.events.add(EventName.modeldLagging)

  def data_sample(self):
    """Receive data from sockets and update carState"""

    CS = self.card.state_update(self.CC)

    self.sm.update(0)

    if not self.initialized:
      all_valid = CS.canValid and self.sm.all_checks()
      timed_out = self.sm.frame * DT_CTRL > 6.
      if all_valid or timed_out or (SIMULATION and not REPLAY):
        available_streams = VisionIpcClient.available_streams("camerad", block=False)
        if VisionStreamType.VISION_STREAM_ROAD not in available_streams:
          self.sm.ignore_alive.append('roadCameraState')
        if VisionStreamType.VISION_STREAM_WIDE_ROAD not in available_streams:
          self.sm.ignore_alive.append('wideRoadCameraState')

        if not self.CP.passive:
          self.card.initialize()

        self.initialized = True
        self.set_initial_state()
        self.params.put_bool_nonblocking("ControlsReady", True)

        cloudlog.event(
          "controlsd.initialized",
          dt=self.sm.frame*DT_CTRL,
          timeout=timed_out,
          canValid=CS.canValid,
          invalid=[s for s, valid in self.sm.valid.items() if not valid],
          not_alive=[s for s, alive in self.sm.alive.items() if not alive],
          not_freq_ok=[s for s, freq_ok in self.sm.freq_ok.items() if not freq_ok],
          error=True,
        )

    # When the panda and controlsd do not agree on controls_allowed
    # we want to disengage openpilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    if not self.enabled:
      self.mismatch_counter = 0

    # All pandas not in silent mode must have controlsAllowed when openpilot is enabled
    if self.enabled and any(not ps.controlsAllowed for ps in self.sm['pandaStates']
           if ps.safetyModel not in IGNORED_SAFETY_MODES):
      self.mismatch_counter += 1

    return CS

  def state_transition(self, CS):
    """Compute conditional state transitions and execute actions on state transitions"""

    self.v_cruise_helper.update_v_cruise(CS, self.enabled_long, self.is_metric, self.reverse_acc_change, self.sm['longitudinalPlanSP'])

    # decrement the soft disable timer at every step, as it's reset on
    # entrance in SOFT_DISABLING state
    self.soft_disable_timer = max(0, self.soft_disable_timer - 1)

    self.current_alert_types = [ET.PERMANENT]

    # ENABLED, SOFT DISABLING, PRE ENABLING, OVERRIDING
    if self.state != State.disabled:
      # user and immediate disable always have priority in a non-disabled state
      if self.events.contains(ET.USER_DISABLE):
        self.state = State.disabled
        self.current_alert_types.append(ET.USER_DISABLE)

      elif self.events.contains(ET.IMMEDIATE_DISABLE):
        self.state = State.disabled
        if CS.gearShifter != GearShifter.park:
          self.current_alert_types.append(ET.IMMEDIATE_DISABLE)

      else:
        # ENABLED
        if self.state == State.enabled:
          if CS.cruiseState.enabled and not self.CS_prev.cruiseState.enabled:
            self.v_cruise_helper.initialize_v_cruise(CS, self.experimental_mode, self.is_metric, self.dynamic_experimental_control)
          # Block resume if cruise never previously enabled
          resume_pressed = any(be.type in (ButtonType.accelCruise, ButtonType.resumeCruise) for be in CS.buttonEvents)
          if not self.CP.pcmCruise and not self.v_cruise_helper.v_cruise_initialized and resume_pressed:
            self.current_alert_types.append(ET.NO_ENTRY)
          if self.events.contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.events.contains(ET.PRE_ENABLE):
            self.current_alert_types.append(ET.PRE_ENABLE)

          elif self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL):
            self.state = State.overriding
            self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]

        # SOFT DISABLING
        elif self.state == State.softDisabling:
          if not self.events.contains(ET.SOFT_DISABLE):
            # no more soft disabling condition, so go back to ENABLED
            self.state = State.enabled

          elif self.soft_disable_timer > 0:
            self.current_alert_types.append(ET.SOFT_DISABLE)

          elif self.soft_disable_timer <= 0:
            self.state = State.disabled

        # PRE ENABLING
        elif self.state == State.preEnabled:
          if not self.events.contains(ET.PRE_ENABLE):
            self.state = State.enabled
          else:
            self.current_alert_types.append(ET.PRE_ENABLE)

        # OVERRIDING
        elif self.state == State.overriding:
          if self.events.contains(ET.SOFT_DISABLE):
            self.state = State.softDisabling
            self.soft_disable_timer = int(SOFT_DISABLE_TIME / DT_CTRL)
            self.current_alert_types.append(ET.SOFT_DISABLE)
          elif not (self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL)):
            self.state = State.enabled
          else:
            self.current_alert_types += [ET.OVERRIDE_LATERAL, ET.OVERRIDE_LONGITUDINAL]
          if CS.cruiseState.enabled and not self.CS_prev.cruiseState.enabled:
            self.v_cruise_helper.initialize_v_cruise(CS, self.experimental_mode, self.is_metric, self.dynamic_experimental_control)

    # DISABLED
    elif self.state == State.disabled:
      if self.events.contains(ET.ENABLE):
        if self.events.contains(ET.NO_ENTRY):
          self.current_alert_types.append(ET.NO_ENTRY)

        else:
          if self.events.contains(ET.PRE_ENABLE):
            self.state = State.preEnabled
          elif self.events.contains(ET.OVERRIDE_LATERAL) or self.events.contains(ET.OVERRIDE_LONGITUDINAL):
            self.state = State.overriding
          else:
            self.state = State.enabled
          self.current_alert_types.append(ET.ENABLE)
          if CS.cruiseState.enabled:
            self.v_cruise_helper.initialize_v_cruise(CS, self.experimental_mode, self.is_metric, self.dynamic_experimental_control)

    # Check if openpilot is engaged and actuators are enabled
    self.enabled = self.state in ENABLED_STATES
    self.enabled_long = self.enabled and CS.cruiseState.enabled
    self.active = self.state in ACTIVE_STATES
    if self.active:
      self.current_alert_types.append(ET.WARNING)

  def state_control(self, CS):
    """Given the state, this function returns a CarControl packet"""

    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1)
    self.VM.update_params(x, sr)

    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and (torque_params.useParams or self.live_torque) and not self.torqued_override:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)

    lat_plan = self.sm['lateralPlanDEPRECATED']
    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']
    blinker_svs = lat_plan if self.model_use_lateral_planner else model_v2.meta

    CC = car.CarControl.new_message()
    CC.enabled = self.enabled

    # Check which actuators can be enabled
    standstill = CS.vEgo <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
    CC.latActive = (self.active or self.mads_ndlob) and not CS.steerFaultTemporary and not CS.steerFaultPermanent and \
                   (not standstill or self.joystick_mode) and CS.madsEnabled and (not CS.brakePressed or self.mads_ndlob) and \
                   (not CS.belowLaneChangeSpeed or (not (((self.sm.frame - self.last_blinker_frame) * DT_CTRL) < 1.0) and
                   not (CS.leftBlinker or CS.rightBlinker))) and CS.latActive and self.sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.calibrated and \
                   not self.process_not_running
    CC.longActive = self.enabled_long and not (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) and not self.events.contains(ET.OVERRIDE_LONGITUDINAL)

    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state

    # Enable blinkers while lane changing
    if blinker_svs.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = blinker_svs.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = blinker_svs.laneChangeDirection == LaneChangeDirection.right

    if CS.leftBlinker or CS.rightBlinker:
      self.last_blinker_frame = self.sm.frame

    # State specific actions

    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset(v_pid=CS.vEgo)

    if not self.joystick_mode:
      # accel PID loop
      pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, self.v_cruise_helper.v_cruise_kph * CV.KPH_TO_MS)
      t_since_plan = (self.sm.frame - self.sm.recv_frame['longitudinalPlan']) * DT_CTRL
      actuators.accel = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan)

      # Steering PID loop and lateral MPC
      if self.model_use_lateral_planner:
        self.desired_curvature = get_lag_adjusted_curvature(self.CP, CS.vEgo, lat_plan.psis, lat_plan.curvatures)
      else:
        self.desired_curvature = clip_curvature(CS.vEgo, self.desired_curvature, model_v2.action.desiredCurvature)
      actuators.curvature = self.desired_curvature
      actuators.steer, actuators.steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                                             self.steer_limited, self.desired_curvature,
                                                                             self.sm['liveLocationKalman'],
                                                                             model_data=model_v2)
      if self.model_use_lateral_planner:
        actuators.curvature = self.desired_curvature
    else:
      lac_log = log.ControlsState.LateralDebugState.new_message()
      if self.sm.recv_frame['testJoystick'] > 0:
        # reset joystick if it hasn't been received in a while
        should_reset_joystick = (self.sm.frame - self.sm.recv_frame['testJoystick'])*DT_CTRL > 0.2
        if not should_reset_joystick:
          joystick_axes = self.sm['testJoystick'].axes
        else:
          joystick_axes = [0.0, 0.0]

        if CC.longActive:
          actuators.accel = 4.0*clip(joystick_axes[0], -1, 1)

        if CC.latActive:
          steer = clip(joystick_axes[1], -1, 1)
          # max angle is 45 for angle-based cars, max curvature is 0.02
          actuators.steer, actuators.steeringAngleDeg, actuators.curvature = steer, steer * 90., steer * -0.02

        lac_log.active = self.active
        lac_log.steeringAngleDeg = CS.steeringAngleDeg
        lac_log.output = actuators.steer
        lac_log.saturated = abs(actuators.steer) >= 0.9

    if CS.steeringPressed:
      self.last_steering_pressed_frame = self.sm.frame
    recent_steer_pressed = (self.sm.frame - self.last_steering_pressed_frame)*DT_CTRL < 2.0

    # Send a "steering required alert" if saturation count has reached the limit
    if lac_log.active and not recent_steer_pressed and not self.CP.notCar and CS.madsEnabled:
      if self.CP.lateralTuning.which() == 'torque' and not self.joystick_mode:
        undershooting = abs(lac_log.desiredLateralAccel) / abs(1e-3 + lac_log.actualLateralAccel) > 1.2
        turning = abs(lac_log.desiredLateralAccel) > 1.0
        good_speed = CS.vEgo > 5
        max_torque = abs(self.last_actuators.steer) > 0.99
        if undershooting and turning and good_speed and max_torque:
          lac_log.active and self.events.add(EventName.steerSaturated)
      elif lac_log.saturated:
        # TODO probably should not use dpath_points but curvature
        dpath_points = lat_plan.dPathPoints if self.model_use_lateral_planner else model_v2.position.y
        if len(dpath_points):
          # Check if we deviated from the path
          # TODO use desired vs actual curvature
          if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
            steering_value = actuators.steeringAngleDeg
          else:
            steering_value = actuators.steer

          left_deviation = steering_value > 0 and dpath_points[0] < -0.20
          right_deviation = steering_value < 0 and dpath_points[0] > 0.20

          if left_deviation or right_deviation:
            self.events.add(EventName.steerSaturated)

    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue

      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)

    return CC, lac_log

  def publish_logs(self, CS, start_time, CC, lac_log):
    """Send actuators and hud commands to the car, send controlsstate and MPC logging"""

    # Orientation and angle rates can be useful for carcontroller
    # Only calibrated (car) frame is relevant for the carcontroller
    orientation_value = list(self.sm['liveLocationKalman'].calibratedOrientationNED.value)
    if len(orientation_value) > 2:
      CC.orientationNED = orientation_value
    angular_rate_value = list(self.sm['liveLocationKalman'].angularVelocityCalibrated.value)
    if len(angular_rate_value) > 2:
      CC.angularVelocity = angular_rate_value

    CC.cruiseControl.override = self.enabled_long and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not self.enabled_long or (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)))
    if self.joystick_mode and self.sm.recv_frame['testJoystick'] > 0 and self.sm['testJoystick'].buttons[0]:
      CC.cruiseControl.cancel = True

    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      CC.cruiseControl.resume = self.enabled_long and CS.cruiseState.standstill and speeds[-1] > 0.1
    CC.vCruise = float(self.v_cruise_helper.v_cruise_kph)

    hudControl = CC.hudControl
    hudControl.setSpeed = float(self.v_cruise_helper.v_cruise_cluster_kph * CV.KPH_TO_MS)
    hudControl.speedVisible = self.enabled_long
    hudControl.lanesVisible = self.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead

    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True

    recent_blinker = (self.sm.frame - self.last_blinker_frame) * DT_CTRL < 5.0  # 5s blinker cooldown
    ldw_allowed = self.is_ldw_enabled and CS.vEgo > LDW_MIN_SPEED and not recent_blinker \
                  and not CC.latActive and self.sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.calibrated

    model_v2 = self.sm['modelV2']
    desire_prediction = model_v2.meta.desirePrediction
    if len(desire_prediction) and ldw_allowed:
      right_lane_visible = model_v2.laneLineProbs[2] > 0.5
      left_lane_visible = model_v2.laneLineProbs[1] > 0.5
      l_lane_change_prob = desire_prediction[Desire.laneChangeLeft]
      r_lane_change_prob = desire_prediction[Desire.laneChangeRight]

      lane_lines = model_v2.laneLines
      l_lane_close = left_lane_visible and (lane_lines[1].y[0] > -(1.08 + CAMERA_OFFSET))
      r_lane_close = right_lane_visible and (lane_lines[2].y[0] < (1.08 - CAMERA_OFFSET))

      hudControl.leftLaneDepart = bool(l_lane_change_prob > LANE_DEPARTURE_THRESHOLD and l_lane_close)
      hudControl.rightLaneDepart = bool(r_lane_change_prob > LANE_DEPARTURE_THRESHOLD and r_lane_close)

    if hudControl.rightLaneDepart or hudControl.leftLaneDepart:
      self.events.add(EventName.ldw)

    clear_event_types = set()
    if ET.WARNING not in self.current_alert_types:
      clear_event_types.add(ET.WARNING)
    if self.enabled and (self.CP.pcmCruise or (not self.CP.pcmCruise and self.v_cruise_helper.v_cruise_initialized)):
      clear_event_types.add(ET.NO_ENTRY)

    alerts = self.events.create_alerts(self.current_alert_types, [self.CP, CS, self.sm, self.is_metric, self.soft_disable_timer])
    self.AM.add_many(self.sm.frame, alerts)
    current_alert = self.AM.process_alerts(self.sm.frame, clear_event_types)
    if current_alert:
      hudControl.visualAlert = current_alert.visual_alert

    if not self.CP.passive and self.initialized:
      self.last_actuators = self.card.controls_update(CC)
      CC.actuatorsOutput = self.last_actuators
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited = abs(CC.actuators.steeringAngleDeg - CC.actuatorsOutput.steeringAngleDeg) > \
                             STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited = abs(CC.actuators.steer - CC.actuatorsOutput.steer) > 1e-2

    force_decel = (self.sm['driverMonitoringState'].awarenessStatus < 0.) or \
                  (self.state == State.softDisabling)

    # Curvature & Steering angle
    lp = self.sm['liveParameters']
    lp_mono_time_svs = 'lateralPlanDEPRECATED' if self.model_use_lateral_planner else 'modelV2'

    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)

    # controlsState
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    controlsState = dat.controlsState
    if current_alert:
      controlsState.alertText1 = current_alert.alert_text_1
      controlsState.alertText2 = current_alert.alert_text_2
      controlsState.alertSize = current_alert.alert_size
      controlsState.alertStatus = current_alert.alert_status
      controlsState.alertBlinkingRate = current_alert.alert_rate
      controlsState.alertType = current_alert.alert_type
      controlsState.alertSound = current_alert.audible_alert

    controlsState.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    controlsState.lateralPlanMonoTime = self.sm.logMonoTime[lp_mono_time_svs]
    controlsState.enabled = not (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) and (self.enabled or CS.cruiseState.enabled) and CS.gearShifter not in [GearShifter.park, GearShifter.reverse]
    controlsState.active = not (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) and (self.active or CS.cruiseState.enabled)
    controlsState.curvature = curvature
    controlsState.desiredCurvature = self.desired_curvature
    controlsState.state = self.state
    controlsState.engageable = not self.events.contains(ET.NO_ENTRY)
    controlsState.longControlState = self.LoC.long_control_state
    controlsState.vPid = float(self.LoC.v_pid)
    controlsState.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    controlsState.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)
    controlsState.upAccelCmd = float(self.LoC.pid.p)
    controlsState.uiAccelCmd = float(self.LoC.pid.i)
    controlsState.ufAccelCmd = float(self.LoC.pid.f)
    controlsState.cumLagMs = -self.rk.remaining * 1000.
    controlsState.startMonoTime = int(start_time * 1e9)
    controlsState.forceDecel = bool(force_decel)
    controlsState.canErrorCounter = self.card.can_rcv_cum_timeout_counter
    controlsState.experimentalMode = self.experimental_mode

    lat_tuning = self.CP.lateralTuning.which()
    if self.joystick_mode:
      controlsState.lateralControlState.debugState = lac_log
    elif self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      controlsState.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      controlsState.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      controlsState.lateralControlState.torqueState = lac_log

    self.pm.send('controlsState', dat)

    dat_sp = messaging.new_message('controlsStateSP')
    controlsStateSP = dat_sp.controlsStateSP

    controlsStateSP.lateralState = lat_tuning

    if self.enable_nnff and lat_tuning == 'torque':
      controlsStateSP.lateralControlState.torqueState = self.LaC.pid_long_sp

    self.pm.send('controlsStateSP', dat_sp)

    car_events = self.events.to_msg()

    self.card.state_publish(car_events)

    # onroadEvents - logged every second or on change
    if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):
      ce_send = messaging.new_message('onroadEvents', len(self.events))
      ce_send.valid = True
      ce_send.onroadEvents = car_events
      self.pm.send('onroadEvents', ce_send)
    self.events_prev = self.events.names.copy()

    # carControl
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

    # copy CarControl to pass to CarInterface on the next iteration
    self.CC = CC

  def step(self):
    start_time = time.monotonic()

    # Sample data from sockets and get a carState
    CS = self.data_sample()
    cloudlog.timestamp("Data sampled")

    self.update_events(CS)
    cloudlog.timestamp("Events updated")

    if not self.CP.passive and self.initialized:
      # Update control state
      self.state_transition(CS)

    # Compute actuators (runs PID loops and lateral MPC)
    CC, lac_log = self.state_control(CS)

    # Publish data
    self.publish_logs(CS, start_time, CC, lac_log)

    self.CS_prev = CS

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and (self.CP.openpilotLongitudinalControl or
                                                                             (not self.CP.pcmCruiseSpeed and self.custom_stock_planner_speed))
      if self.CP.notCar:
        self.joystick_mode = self.params.get_bool("JoystickDebugMode")

      self.lane_change_set_timer = int(self.params.get("AutoLaneChangeTimer", encoding="utf8"))
      self.reverse_acc_change = self.params.get_bool("ReverseAccChange")
      self.dynamic_experimental_control = self.params.get_bool("DynamicExperimentalControl")

      if self.sm.frame % int(2.5 / DT_CTRL) == 0:
        self.live_torque = self.params.get_bool("LiveTorque")
        self.custom_stock_planner_speed = self.params.get_bool("CustomStockLongPlanner")
      time.sleep(0.1)

  def controlsd_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    except SystemExit:
      e.set()
      t.join()


def main():
  controls = Controls()
  controls.controlsd_thread()


if __name__ == "__main__":
  main()
