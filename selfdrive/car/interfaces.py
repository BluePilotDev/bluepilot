import json
import operator
import os
import numpy as np
import time
import tomllib
from abc import abstractmethod, ABC
from difflib import SequenceMatcher
from enum import StrEnum
from typing import Any, NamedTuple
from collections.abc import Callable

from cereal import car
from openpilot.common.basedir import BASEDIR
from openpilot.common.conversions import Conversions as CV
from openpilot.common.simple_kalman import KF1D, get_kalman_gain
from openpilot.common.numpy_fast import clip
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car import apply_hysteresis, gen_empty_fingerprint, scale_rot_inertia, scale_tire_stiffness, STD_CARGO_KG
from openpilot.selfdrive.car.values import Platform
from openpilot.selfdrive.controls.lib.desire_helper import LANE_CHANGE_SPEED_MIN
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_UNSET, get_friction
from openpilot.selfdrive.controls.lib.events import Events
from openpilot.selfdrive.controls.lib.vehicle_model import VehicleModel

ButtonType = car.CarState.ButtonEvent.Type
GearShifter = car.CarState.GearShifter
EventName = car.CarEvent.EventName

MAX_CTRL_SPEED = (V_CRUISE_MAX + 4) * CV.KPH_TO_MS
ACCEL_MAX = 2.0
ACCEL_MIN = -3.5
FRICTION_THRESHOLD = 0.3

TORQUE_PARAMS_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/params.toml')
TORQUE_OVERRIDE_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/override.toml')
TORQUE_SUBSTITUTE_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/substitute.toml')
TORQUE_NN_MODEL_PATH = os.path.join(BASEDIR, 'selfdrive/car/torque_data/lat_models')

# dict used to rename activation functions whose names aren't valid python identifiers
ACTIVATION_FUNCTION_NAMES = {'σ': 'sigmoid'}

GAC_DICT = {1: 1, 2: 2, 3: 3}
FORWARD_GEARS = [GearShifter.drive, GearShifter.low, GearShifter.eco,
                 GearShifter.sport, GearShifter.manumatic, GearShifter.brake]


def similarity(s1: str, s2: str) -> float:
  return SequenceMatcher(None, s1, s2).ratio()


class LatControlInputs(NamedTuple):
  lateral_acceleration: float
  roll_compensation: float
  vego: float
  aego: float


TorqueFromLateralAccelCallbackType = Callable[[LatControlInputs, car.CarParams.LateralTorqueTuning, float, float, bool, bool], float]


def get_torque_params(candidate):
  with open(TORQUE_SUBSTITUTE_PATH, 'rb') as f:
    sub = tomllib.load(f)
  if candidate in sub:
    candidate = sub[candidate]

  with open(TORQUE_PARAMS_PATH, 'rb') as f:
    params = tomllib.load(f)
  with open(TORQUE_OVERRIDE_PATH, 'rb') as f:
    override = tomllib.load(f)

  # Ensure no overlap
  if sum([candidate in x for x in [sub, params, override]]) > 1:
    raise RuntimeError(f'{candidate} is defined twice in torque config')

  if candidate in override:
    out = override[candidate]
  elif candidate in params:
    out = params[candidate]
  else:
    raise NotImplementedError(f"Did not find torque params for {candidate}")
  return {key: out[i] for i, key in enumerate(params['legend'])}


# lateral neural network feedforward
class FluxModel:
  def __init__(self, params_file, zero_bias=False):
    with open(params_file, "r") as f:
      params = json.load(f)

    self.input_size = params["input_size"]
    self.output_size = params["output_size"]
    self.input_mean = np.array(params["input_mean"], dtype=np.float32).T
    self.input_std = np.array(params["input_std"], dtype=np.float32).T
    self.layers = []
    self.friction_override = False

    for layer_params in params["layers"]:
      W = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_W'))], dtype=np.float32).T
      b = np.array(layer_params[next(key for key in layer_params.keys() if key.endswith('_b'))], dtype=np.float32).T
      if zero_bias:
        b = np.zeros_like(b)
      activation = layer_params["activation"]
      for k, v in ACTIVATION_FUNCTION_NAMES.items():
        activation = activation.replace(k, v)
      self.layers.append((W, b, activation))

    self.validate_layers()
    self.check_for_friction_override()

  # Begin activation functions.
  # These are called by name using the keys in the model json file
  @staticmethod
  def sigmoid(x):
    return 1 / (1 + np.exp(-x))

  @staticmethod
  def identity(x):
    return x
  # End activation functions

  def forward(self, x):
    for W, b, activation in self.layers:
      x = getattr(self, activation)(x.dot(W) + b)
    return x

  def evaluate(self, input_array):
    in_len = len(input_array)
    if in_len != self.input_size:
      # If the input is length 2-4, then it's a simplified evaluation.
      # In that case, need to add on zeros to fill out the input array to match the correct length.
      if 2 <= in_len:
        input_array = input_array + [0] * (self.input_size - in_len)
      else:
        raise ValueError(f"Input array length {len(input_array)} must be length 2 or greater")

    input_array = np.array(input_array, dtype=np.float32)

    # Rescale the input array using the input_mean and input_std
    input_array = (input_array - self.input_mean) / self.input_std

    output_array = self.forward(input_array)

    return float(output_array[0, 0])

  def validate_layers(self):
    for W, b, activation in self.layers:
      if not hasattr(self, activation):
        raise ValueError(f"Unknown activation: {activation}")

  def check_for_friction_override(self):
    y = self.evaluate([10.0, 0.0, 0.2])
    self.friction_override = (y < 0.1)


def get_nn_model_path(_car, eps_firmware) -> tuple[str | None, float]:
  def check_nn_path(_check_model):
    _model_path = None
    _max_similarity = -1.0
    for f in os.listdir(TORQUE_NN_MODEL_PATH):
      if f.endswith(".json"):
        model = f.replace(".json", "").replace(f"{TORQUE_NN_MODEL_PATH}/", "")
        similarity_score = similarity(model, _check_model)
        if similarity_score > _max_similarity:
          _max_similarity = similarity_score
          _model_path = os.path.join(TORQUE_NN_MODEL_PATH, f)
    return _model_path, _max_similarity

  if len(eps_firmware) > 3:
    eps_firmware = eps_firmware.replace("\\", "")
    check_model = f"{_car} {eps_firmware}"
  else:
    check_model = _car
  model_path, max_similarity = check_nn_path(check_model)
  if _car not in model_path or 0.0 <= max_similarity < 0.9:
    check_model = _car
    model_path, max_similarity = check_nn_path(check_model)
    if _car not in model_path or 0.0 <= max_similarity < 0.9:
      model_path = None
  return model_path, max_similarity


def get_nn_model(_car, eps_firmware) -> tuple[FluxModel | None, float]:
  model, similarity_score = get_nn_model_path(_car, eps_firmware)
  if model is not None:
    model = FluxModel(model)
  return model, similarity_score


# generic car and radar interfaces

class CarInterfaceBase(ABC):
  def __init__(self, CP, CarController, CarState):
    self.CP = CP
    self.VM = VehicleModel(CP)

    self.frame = 0
    self.steering_unpressed = 0
    self.low_speed_alert = False
    self.no_steer_warning = False
    self.silent_steer_warning = True
    self.v_ego_cluster_seen = False

    self.CS = None
    self.can_parsers = []
    if CarState is not None:
      self.CS = CarState(CP)

      self.cp = self.CS.get_can_parser(CP)
      self.cp_cam = self.CS.get_cam_can_parser(CP)
      self.cp_adas = self.CS.get_adas_can_parser(CP)
      self.cp_body = self.CS.get_body_can_parser(CP)
      self.cp_loopback = self.CS.get_loopback_can_parser(CP)
      self.can_parsers = [self.cp, self.cp_cam, self.cp_adas, self.cp_body, self.cp_loopback]

    self.CC = None
    if CarController is not None:
      self.CC = CarController(self.cp.dbc_name, CP, self.VM)

    self.param_s = Params()
    self.disengage_on_accelerator = self.param_s.get_bool("DisengageOnAccelerator")
    self.enable_mads = self.param_s.get_bool("EnableMads")
    self.mads_disengage_lateral_on_brake = self.param_s.get_bool("DisengageLateralOnBrake")
    self.mads_ndlob = self.enable_mads and not self.mads_disengage_lateral_on_brake
    self.gear_warning = 0
    self.cruise_cancelled_btn = True
    self.acc_mads_combo = self.param_s.get_bool("AccMadsCombo")
    self.below_speed_pause = self.param_s.get_bool("BelowSpeedPause")
    self.prev_acc_mads_combo = False
    self.mads_event_lock = True
    self.gap_button_counter = 0
    self.experimental_mode_hold = False
    self.experimental_mode = self.param_s.get_bool("ExperimentalMode")
    self._frame = 0
    self.op_lookup = {"+": operator.add, "-": operator.sub}
    self.prev_gac_button = False
    self.gac_button_counter = 0
    self.reverse_dm_cam = self.param_s.get_bool("ReverseDmCam")
    self.mads_main_toggle = self.param_s.get_bool("MadsCruiseMain")
    self.lkas_toggle = self.param_s.get_bool("LkasToggle")
    self.last_mads_init = 0.
    self.madsEnabledInit = False
    self.madsEnabledInitPrev = False

    self.lat_torque_nn_model = None
    eps_firmware = str(next((fw.fwVersion for fw in CP.carFw if fw.ecu == "eps"), ""))
    self.has_lateral_torque_nn = self.initialize_lat_torque_nn(CP.carFingerprint, eps_firmware)

  def get_ff_nn(self, x):
    return self.lat_torque_nn_model.evaluate(x)

  def initialize_lat_torque_nn(self, _car, eps_firmware) -> bool:
    self.lat_torque_nn_model, _ = get_nn_model(_car, eps_firmware)
    return self.lat_torque_nn_model is not None and self.param_s.get_bool("NNFF")

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    return ACCEL_MIN, ACCEL_MAX

  @classmethod
  def get_non_essential_params(cls, candidate: Platform):
    """
    Parameters essential to controlling the car may be incomplete or wrong without FW versions or fingerprints.
    """
    return cls.get_params(candidate, gen_empty_fingerprint(), list(), False, False)

  @classmethod
  def get_params(cls, candidate: Platform, fingerprint: dict[int, dict[int, int]], car_fw: list[car.CarParams.CarFw], experimental_long: bool, docs: bool):
    ret = CarInterfaceBase.get_std_params(candidate)

    if hasattr(candidate, "config"):
      if candidate.config.specs is not None:
        ret.mass = candidate.config.specs.mass
        ret.wheelbase = candidate.config.specs.wheelbase
        ret.steerRatio = candidate.config.specs.steerRatio
        ret.centerToFront = ret.wheelbase * candidate.config.specs.centerToFrontRatio
        ret.minEnableSpeed = candidate.config.specs.minEnableSpeed
        ret.minSteerSpeed = candidate.config.specs.minSteerSpeed

    ret = cls._get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs)

    params = Params()
    if ret.steerControlType != car.CarParams.SteerControlType.angle:
      if params.get_bool("EnforceTorqueLateral") or params.get_bool("NNFF"):
        ret = CarInterfaceBase.sp_configure_torque_tune(candidate, ret)
        if params.get_bool("CustomTorqueLateral"):
          ret = CarInterfaceBase.sp_configure_custom_torque_tune(ret, params)

      if ret.lateralTuning.which() == 'torque':
        eps_firmware = str(next((fw.fwVersion for fw in car_fw if fw.ecu == "eps"), ""))
        model, similarity_score = get_nn_model_path(candidate, eps_firmware)
        if model is not None:
          ret.lateralTuning.torque.nnModelName = nn_model_name = os.path.splitext(os.path.basename(model))[0]
          ret.lateralTuning.torque.nnModelFuzzyMatch = (similarity_score < 0.99)
          if 'b\'' in nn_model_name:
            nn_model, _ = nn_model_name.split('b\'')
          else:
            nn_model = nn_model_name
          params.put("NNFFCarModel", nn_model)
        else:
          ret.lateralTuning.torque.nnModelName = "mock"

    # Vehicle mass is published curb weight plus assumed payload such as a human driver; notCars have no assumed payload
    if not ret.notCar:
      ret.mass = ret.mass + STD_CARGO_KG

    # Set params dependent on values set by the car interface
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront, ret.tireStiffnessFactor)

    return ret

  @staticmethod
  @abstractmethod
  def _get_params(ret: car.CarParams, candidate: Platform, fingerprint: dict[int, dict[int, int]],
                  car_fw: list[car.CarParams.CarFw], experimental_long: bool, docs: bool):
    raise NotImplementedError

  @staticmethod
  def init(CP, logcan, sendcan):
    pass

  @staticmethod
  def get_steer_feedforward_default(desired_angle, v_ego):
    # Proportional to realigning tire momentum: lateral acceleration.
    return desired_angle * (v_ego**2)

  def get_steer_feedforward_function(self):
    return self.get_steer_feedforward_default

  def torque_from_lateral_accel_linear(self, latcontrol_inputs: LatControlInputs, torque_params: car.CarParams.LateralTorqueTuning,
                                       lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    # The default is a linear relationship between torque and lateral acceleration (accounting for road roll and steering friction)
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)
    return (latcontrol_inputs.lateral_acceleration / float(torque_params.latAccelFactor)) + friction

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    return self.torque_from_lateral_accel_linear

  # returns a set of default params to avoid repetition in car specific params
  @staticmethod
  def get_std_params(candidate):
    ret = car.CarParams.new_message()
    ret.carFingerprint = candidate

    # Car docs fields
    ret.maxLateralAccel = get_torque_params(candidate)['MAX_LAT_ACCEL_MEASURED']
    ret.autoResumeSng = True  # describes whether car can resume from a stop automatically

    # standard ALC params
    ret.tireStiffnessFactor = 1.0
    ret.steerControlType = car.CarParams.SteerControlType.torque
    ret.minSteerSpeed = 0.
    ret.wheelSpeedFactor = 1.0

    ret.pcmCruise = True     # openpilot's state is tied to the PCM's cruise state on most cars
    ret.pcmCruiseSpeed = True     # openpilot's state is tied to the PCM's cruise speed
    ret.minEnableSpeed = -1. # enable is done by stock ACC, so ignore this
    ret.steerRatioRear = 0.  # no rear steering, at least on the listed cars aboveA
    ret.openpilotLongitudinalControl = False
    ret.stopAccel = -2.0
    ret.stoppingDecelRate = 0.8 # brake_travel/s while trying to stop
    ret.vEgoStopping = 0.5
    ret.vEgoStarting = 0.5
    ret.stoppingControl = True
    ret.longitudinalTuning.deadzoneBP = [0.]
    ret.longitudinalTuning.deadzoneV = [0.]
    ret.longitudinalTuning.kf = 1.
    ret.longitudinalTuning.kpBP = [0.]
    ret.longitudinalTuning.kpV = [1.]
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [1.]
    # TODO estimate car specific lag, use .15s for now
    ret.longitudinalActuatorDelayLowerBound = 0.15
    ret.longitudinalActuatorDelayUpperBound = 0.15
    ret.steerLimitTimer = 1.0
    return ret

  @staticmethod
  def configure_torque_tune(candidate, tune, steering_angle_deadzone_deg=0.0, use_steering_angle=True):
    params = get_torque_params(candidate)

    tune.init('torque')
    tune.torque.useSteeringAngle = use_steering_angle
    tune.torque.kp = 1.0
    tune.torque.kf = 1.0
    tune.torque.ki = 0.1
    tune.torque.friction = params['FRICTION']
    tune.torque.latAccelFactor = params['LAT_ACCEL_FACTOR']
    tune.torque.latAccelOffset = 0.0
    tune.torque.steeringAngleDeadzoneDeg = steering_angle_deadzone_deg

  @staticmethod
  def sp_configure_torque_tune(candidate, ret):
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    return ret

  @staticmethod
  def sp_configure_custom_torque_tune(ret, params):
    ret.lateralTuning.torque.friction = float(params.get("TorqueFriction", encoding="utf8")) * 0.01
    ret.lateralTuning.torque.latAccelFactor = float(params.get("TorqueMaxLatAccel", encoding="utf8")) * 0.01
    return ret

  @abstractmethod
  def _update(self, c: car.CarControl) -> car.CarState:
    pass

  def update(self, c: car.CarControl, can_strings: list[bytes]) -> car.CarState:
    # parse can
    for cp in self.can_parsers:
      if cp is not None:
        cp.update_strings(can_strings)

    # get CarState
    ret = self._update(c)

    ret.canValid = all(cp.can_valid for cp in self.can_parsers if cp is not None)
    ret.canTimeout = any(cp.bus_timeout for cp in self.can_parsers if cp is not None)

    if ret.vEgoCluster == 0.0 and not self.v_ego_cluster_seen:
      ret.vEgoCluster = ret.vEgo
    else:
      self.v_ego_cluster_seen = True

    # Many cars apply hysteresis to the ego dash speed
    if self.CS is not None:
      ret.vEgoCluster = apply_hysteresis(ret.vEgoCluster, self.CS.out.vEgoCluster, self.CS.cluster_speed_hyst_gap)
      if abs(ret.vEgo) < self.CS.cluster_min_speed:
        ret.vEgoCluster = 0.0

    if ret.cruiseState.speedCluster == 0:
      ret.cruiseState.speedCluster = ret.cruiseState.speed

    # copy back for next iteration
    reader = ret.as_reader()
    if self.CS is not None:
      self.CS.out = reader

    return reader

  @abstractmethod
  def apply(self, c: car.CarControl, now_nanos: int) -> tuple[car.CarControl.Actuators, list[bytes]]:
    pass

  def create_common_events(self, cs_out, c, extra_gears=None, pcm_enable=True, allow_enable=True,
                           enable_buttons=(ButtonType.accelCruise, ButtonType.decelCruise)):
    events = Events()

    if cs_out.doorOpen and (c.latActive or c.longActive):
      events.add(EventName.doorOpen)
    if cs_out.seatbeltUnlatched and cs_out.gearShifter != GearShifter.park:
      events.add(EventName.seatbeltNotLatched)
    if cs_out.gearShifter != GearShifter.drive and cs_out.gearShifter not in extra_gears and not \
            (cs_out.gearShifter == GearShifter.unknown and self.gear_warning < int(0.5/DT_CTRL)):
      if cs_out.vEgo < 5:
        events.add(EventName.silentWrongGear)
      else:
        events.add(EventName.wrongGear)
    if cs_out.gearShifter == GearShifter.reverse:
      if not self.reverse_dm_cam and cs_out.vEgo < 5:
        events.add(EventName.spReverseGear)
      elif cs_out.vEgo >= 5:
        events.add(EventName.reverseGear)
    if not cs_out.cruiseState.available:
      events.add(EventName.wrongCarMode)
    if cs_out.espDisabled:
      events.add(EventName.espDisabled)
    if cs_out.stockFcw:
      events.add(EventName.stockFcw)
    if cs_out.stockAeb:
      events.add(EventName.stockAeb)
    if cs_out.vEgo > MAX_CTRL_SPEED:
      events.add(EventName.speedTooHigh)
    if cs_out.cruiseState.nonAdaptive:
      events.add(EventName.wrongCruiseMode)
    if cs_out.brakeHoldActive and self.CP.openpilotLongitudinalControl:
      if cs_out.madsEnabled:
        cs_out.disengageByBrake = True
      if cs_out.cruiseState.enabled:
        events.add(EventName.brakeHold)
      else:
        events.add(EventName.silentBrakeHold)
    if cs_out.parkingBrake:
      events.add(EventName.parkBrake)
    if cs_out.accFaulted:
      events.add(EventName.accFaulted)
    if cs_out.steeringPressed:
      events.add(EventName.steerOverride)

    self.gear_warning = self.gear_warning + 1 if cs_out.gearShifter == GearShifter.unknown else 0

    # Handle button presses
    #for b in cs_out.buttonEvents:
    #  # Enable OP long on falling edge of enable buttons (defaults to accelCruise and decelCruise, overridable per-port)
    #  if not self.CP.pcmCruise and (b.type in enable_buttons and not b.pressed):
    #    events.add(EventName.buttonEnable)
    #  # Disable on rising and falling edge of cancel for both stock and OP long
    #  if b.type == ButtonType.cancel:
    #    events.add(EventName.buttonCancel)

    # Handle permanent and temporary steering faults
    self.steering_unpressed = 0 if cs_out.steeringPressed else self.steering_unpressed + 1
    if cs_out.steerFaultTemporary:
      if cs_out.steeringPressed and (not self.CS.out.steerFaultTemporary or self.no_steer_warning):
        self.no_steer_warning = True
      else:
        self.no_steer_warning = False

        # if the user overrode recently, show a less harsh alert
        if self.silent_steer_warning or cs_out.standstill or self.steering_unpressed < int(1.5 / DT_CTRL):
          self.silent_steer_warning = True
          events.add(EventName.steerTempUnavailableSilent)
        else:
          events.add(EventName.steerTempUnavailable)
    else:
      self.no_steer_warning = False
      self.silent_steer_warning = False
    if cs_out.steerFaultPermanent:
      events.add(EventName.steerUnavailable)

    # we engage when pcm is active (rising edge)
    # enabling can optionally be blocked by the car interface
    if pcm_enable:
      if cs_out.cruiseState.enabled and not self.CS.out.cruiseState.enabled and allow_enable:
        events.add(EventName.pcmEnable)
      elif not cs_out.cruiseState.enabled:
        events.add(EventName.pcmDisable)

    return events

  @staticmethod
  def sp_v_cruise_initialized(v_cruise):
    return v_cruise != V_CRUISE_UNSET

  def get_acc_mads(self, cruiseState_enabled, acc_enabled, mads_enabled):
    if self.acc_mads_combo:
      if not self.prev_acc_mads_combo and (cruiseState_enabled or acc_enabled):
        mads_enabled = True
      self.prev_acc_mads_combo = (cruiseState_enabled or acc_enabled)

    return mads_enabled

  def get_sp_v_cruise_non_pcm_state(self, cs_out, acc_enabled, button_events, vCruise,
                                    enable_buttons=(ButtonType.accelCruise, ButtonType.decelCruise),
                                    resume_button=(ButtonType.accelCruise, ButtonType.resumeCruise)):

    if cs_out.cruiseState.available:
      for b in button_events:
        if not self.CP.pcmCruise or not self.CP.pcmCruiseSpeed:
          if b.type in enable_buttons and not b.pressed:
            acc_enabled = True
        if not self.CP.pcmCruise:
          if b.type in resume_button and not self.sp_v_cruise_initialized(vCruise):
            acc_enabled = False
        if not self.CP.pcmCruiseSpeed:
          if b.type == ButtonType.accelCruise and not cs_out.cruiseState.enabled:
            acc_enabled = False
    else:
      acc_enabled = False

    return acc_enabled, button_events

  def get_sp_cancel_cruise_state(self, mads_enabled, acc_enabled=False):
    mads_enabled = False if not self.enable_mads or self.disengage_on_accelerator else mads_enabled
    return mads_enabled, acc_enabled

  def get_sp_pedal_disengage(self, cs_out):
    accel_pedal = cs_out.gasPressed and not self.CS.out.gasPressed and self.disengage_on_accelerator
    brake = cs_out.brakePressed and (not self.CS.out.brakePressed or not cs_out.standstill)
    regen = cs_out.regenBraking and (not self.CS.out.regenBraking or not cs_out.standstill)
    return accel_pedal or brake or regen

  def get_sp_cruise_main_state(self, cs_out, CS):
    if not CS.control_initialized:
      mads_enabled = False
    elif not self.mads_main_toggle:
      mads_enabled = False
    else:
      mads_enabled = cs_out.cruiseState.available

    return mads_enabled

  def get_sp_started_mads(self, cs_out, CS):
    if not cs_out.cruiseState.available and CS.out.cruiseState.available:
      self.madsEnabledInit = False
      self.madsEnabledInitPrev = False
      return False
    if not self.mads_main_toggle or self.prev_acc_mads_combo:
      return CS.madsEnabled
    if not self.madsEnabledInit and CS.madsEnabled:
      self.madsEnabledInit = True
      self.last_mads_init = time.monotonic()
    if cs_out.gearShifter not in FORWARD_GEARS:
      self.last_mads_init = time.monotonic()
    if self.madsEnabledInit and (not self.madsEnabledInitPrev or cs_out.gearShifter not in FORWARD_GEARS):
      if time.monotonic() < self.last_mads_init + 1.:
        return False
      self.madsEnabledInitPrev = True
      return cs_out.cruiseState.available
    else:
      return CS.madsEnabled

  def get_sp_common_state(self, cs_out, CS, min_enable_speed_pcm=False, gear_allowed=True, gap_button=False):
    cs_out.cruiseState.enabled = CS.accEnabled if not self.CP.pcmCruise or not self.CP.pcmCruiseSpeed or min_enable_speed_pcm else \
                                 cs_out.cruiseState.enabled

    if not self.enable_mads:
      if cs_out.cruiseState.enabled and not CS.out.cruiseState.enabled:
        CS.madsEnabled = True
      elif not cs_out.cruiseState.enabled and CS.out.cruiseState.enabled:
        CS.madsEnabled = False

    self.toggle_exp_mode(gap_button)

    cs_out.belowLaneChangeSpeed = cs_out.vEgo < LANE_CHANGE_SPEED_MIN and self.below_speed_pause

    if cs_out.gearShifter in [GearShifter.park, GearShifter.reverse] or cs_out.doorOpen or \
      (cs_out.seatbeltUnlatched and cs_out.gearShifter != GearShifter.park):
      gear_allowed = False

    cs_out.latActive = gear_allowed

    if not CS.control_initialized:
      CS.control_initialized = True

    # Disable on rising edge of gas or brake. Also disable on brake when speed > 0.
    if (cs_out.gasPressed and not self.CS.out.gasPressed and self.disengage_on_accelerator) or \
      (cs_out.brakePressed and (not self.CS.out.brakePressed or not cs_out.standstill)) or \
      (cs_out.regenBraking and (not self.CS.out.regenBraking or not cs_out.standstill)):
      if CS.madsEnabled:
        CS.disengageByBrake = True

    cs_out.madsEnabled = CS.madsEnabled
    cs_out.accEnabled = CS.accEnabled
    cs_out.disengageByBrake = CS.disengageByBrake
    cs_out.brakeLightsDEPRECATED |= cs_out.brakePressed or cs_out.brakeHoldActive or cs_out.parkingBrake or cs_out.regenBraking

    return cs_out, CS

  def toggle_exp_mode(self, gap_pressed):
    if not self.CP.openpilotLongitudinalControl:
      return None
    if gap_pressed:
      if not self.experimental_mode_hold:
        self.gap_button_counter += 1
        if self.gap_button_counter > 50:
          self.gap_button_counter = 0
          self.experimental_mode_hold = True
          self.param_s.put_bool_nonblocking("ExperimentalMode", not self.experimental_mode)
    else:
      self.gap_button_counter = 0
      self.experimental_mode_hold = False

  def get_sp_gac_state(self, gac_tr, gac_min, gac_max, inc_dec):
    op = self.op_lookup.get(inc_dec)
    gac_tr = op(gac_tr, 1)
    if inc_dec == "+":
      gac_tr = gac_min if gac_tr > gac_max else gac_tr
    else:
      gac_tr = gac_max if gac_tr < gac_min else gac_tr
    return int(gac_tr)

  def get_sp_distance(self, gac_tr, gac_max, gac_dict=None):
    if gac_dict is None:
      gac_dict = GAC_DICT
    return next((key for key, value in gac_dict.items() if value == gac_tr), gac_max)

  def toggle_gac(self, cs_out, CS, gac_button, gac_min, gac_max, gac_default, inc_dec):
    if not self.CP.openpilotLongitudinalControl:
      CS.gac_tr_cluster = gac_default
      if CS.gac_tr != 2:
        CS.gac_tr = 2
        self.param_s.put_nonblocking("LongitudinalPersonality", "2")
      return cs_out, CS
    if gac_button:
      self.gac_button_counter += 1
    elif self.prev_gac_button and not gac_button and self.gac_button_counter < 50:
      self.gac_button_counter = 0
      CS.gac_tr = self.get_sp_gac_state(CS.gac_tr, 0, 2, inc_dec)
      self.param_s.put_nonblocking("LongitudinalPersonality", str(CS.gac_tr))
    else:
      self.gac_button_counter = 0
    CS.gac_tr_cluster = clip(CS.gac_tr + 1, gac_min, gac_max)  # always 1 higher
    self.prev_gac_button = gac_button
    return cs_out, CS

  def create_sp_events(self, CS, cs_out, events, main_enabled=False, allow_enable=True, enable_pressed=False,
                       enable_from_brake=False, enable_pressed_long=False,
                       enable_buttons=(ButtonType.accelCruise, ButtonType.decelCruise)):

    if not cs_out.brakePressed and not cs_out.brakeHoldActive and not cs_out.parkingBrake and not cs_out.regenBraking:
      if cs_out.disengageByBrake and cs_out.madsEnabled:
        enable_pressed = True
        enable_from_brake = True
      CS.disengageByBrake = False
      cs_out.disengageByBrake = False

    for b in cs_out.buttonEvents:
      # Enable OP long on falling edge of enable buttons (defaults to accelCruise and decelCruise, overridable per-port)
      if not self.CP.pcmCruise:
        if b.type in enable_buttons and not b.pressed:
          enable_pressed = True
          enable_pressed_long = True
      # Disable on rising and falling edge of cancel for both stock and OP long
      if b.type == ButtonType.cancel:
        if not cs_out.madsEnabled:
          events.add(EventName.buttonCancel)
        elif not self.cruise_cancelled_btn:
          self.cruise_cancelled_btn = True
          events.add(EventName.manualLongitudinalRequired)
      # do disable on MADS button if ACC is disabled
      if b.type == ButtonType.altButton1 and b.pressed and self.enable_mads:
        if not cs_out.madsEnabled:  # disabled MADS
          if not cs_out.cruiseState.enabled:
            events.add(EventName.buttonCancel)
          else:
            events.add(EventName.manualSteeringRequired)
        else:  # enabled MADS
          if not cs_out.cruiseState.enabled:
            enable_pressed = True
    if self.CP.pcmCruise:
      # do disable on button down
      if main_enabled:
        if any(CS.main_buttons) and not cs_out.cruiseState.enabled:
          if not cs_out.madsEnabled:
            events.add(EventName.buttonCancel)
      # do enable on both accel and decel buttons
      if cs_out.cruiseState.enabled and not CS.out.cruiseState.enabled and allow_enable:
        enable_pressed = True
        enable_pressed_long = True
      elif not cs_out.cruiseState.enabled:
        if not cs_out.madsEnabled:
          events.add(EventName.buttonCancel)
        elif not self.enable_mads:
          cs_out.madsEnabled = False
    if enable_pressed:
      if enable_from_brake:
        events.add(EventName.silentButtonEnable)
      else:
        events.add(EventName.buttonEnable)
    if cs_out.disengageByBrake and not cs_out.standstill and enable_pressed_long:
      events.add(EventName.cruiseEngageBlocked)

    self.cruise_cancelled_btn = False if cs_out.cruiseState.enabled else True

    return events, cs_out

  def sp_update_params(self, CS):
    self.experimental_mode = self.param_s.get_bool("ExperimentalMode")
    CS.gac_tr = int(self.param_s.get("LongitudinalPersonality"))
    self._frame += 1
    if self._frame % 300 == 0:
      self._frame = 0
      self.reverse_dm_cam = self.param_s.get_bool("ReverseDmCam")
    return CS

class RadarInterfaceBase(ABC):
  def __init__(self, CP):
    self.rcp = None
    self.pts = {}
    self.delay = 0
    self.radar_ts = CP.radarTimeStep
    self.frame = 0
    self.no_radar_sleep = 'NO_RADAR_SLEEP' in os.environ

  def update(self, can_strings):
    self.frame += 1
    if (self.frame % int(100 * self.radar_ts)) == 0:
      return car.RadarData.new_message()
    return None


class CarStateBase(ABC):
  def __init__(self, CP):
    self.CP = CP
    self.car_fingerprint = CP.carFingerprint
    self.out = car.CarState.new_message()

    self.cruise_buttons = 0
    self.left_blinker_cnt = 0
    self.right_blinker_cnt = 0
    self.steering_pressed_cnt = 0
    self.left_blinker_prev = False
    self.right_blinker_prev = False
    self.cluster_speed_hyst_gap = 0.0
    self.cluster_min_speed = 0.0  # min speed before dropping to 0

    self.param_s = Params()
    self.accEnabled = False
    self.madsEnabled = False
    self.disengageByBrake = False
    self.mads_enabled = False
    self.prev_mads_enabled = False
    self.control_initialized = False
    self.pcm_cruise_enabled = False
    self.gap_dist_button = 0
    self.gac_tr = int(self.param_s.get("LongitudinalPersonality"))
    self.gac_tr_cluster = clip(int(self.param_s.get("LongitudinalPersonality")), 1, 3)

    Q = [[0.0, 0.0], [0.0, 100.0]]
    R = 0.3
    A = [[1.0, DT_CTRL], [0.0, 1.0]]
    C = [[1.0, 0.0]]
    x0=[[0.0], [0.0]]
    K = get_kalman_gain(DT_CTRL, np.array(A), np.array(C), np.array(Q), R)
    self.v_ego_kf = KF1D(x0=x0, A=A, C=C[0], K=K)

  def update_speed_kf(self, v_ego_raw):
    if abs(v_ego_raw - self.v_ego_kf.x[0][0]) > 2.0:  # Prevent large accelerations when car starts at non zero speed
      self.v_ego_kf.set_x([[v_ego_raw], [0.0]])

    v_ego_x = self.v_ego_kf.update(v_ego_raw)
    return float(v_ego_x[0]), float(v_ego_x[1])

  def get_wheel_speeds(self, fl, fr, rl, rr, unit=CV.KPH_TO_MS):
    factor = unit * self.CP.wheelSpeedFactor

    wheelSpeeds = car.CarState.WheelSpeeds.new_message()
    wheelSpeeds.fl = fl * factor
    wheelSpeeds.fr = fr * factor
    wheelSpeeds.rl = rl * factor
    wheelSpeeds.rr = rr * factor
    return wheelSpeeds

  def update_blinker_from_lamp(self, blinker_time: int, left_blinker_lamp: bool, right_blinker_lamp: bool):
    """Update blinkers from lights. Enable output when light was seen within the last `blinker_time`
    iterations"""
    # TODO: Handle case when switching direction. Now both blinkers can be on at the same time
    self.left_blinker_cnt = blinker_time if left_blinker_lamp else max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = blinker_time if right_blinker_lamp else max(self.right_blinker_cnt - 1, 0)
    return self.left_blinker_cnt > 0, self.right_blinker_cnt > 0

  def update_steering_pressed(self, steering_pressed, steering_pressed_min_count):
    """Applies filtering on steering pressed for noisy driver torque signals."""
    self.steering_pressed_cnt += 1 if steering_pressed else -1
    self.steering_pressed_cnt = clip(self.steering_pressed_cnt, 0, steering_pressed_min_count * 2)
    return self.steering_pressed_cnt > steering_pressed_min_count

  def update_blinker_from_stalk(self, blinker_time: int, left_blinker_stalk: bool, right_blinker_stalk: bool):
    """Update blinkers from stalk position. When stalk is seen the blinker will be on for at least blinker_time,
    or until the stalk is turned off, whichever is longer. If the opposite stalk direction is seen the blinker
    is forced to the other side. On a rising edge of the stalk the timeout is reset."""

    if left_blinker_stalk:
      self.right_blinker_cnt = 0
      if not self.left_blinker_prev:
        self.left_blinker_cnt = blinker_time

    if right_blinker_stalk:
      self.left_blinker_cnt = 0
      if not self.right_blinker_prev:
        self.right_blinker_cnt = blinker_time

    self.left_blinker_cnt = max(self.left_blinker_cnt - 1, 0)
    self.right_blinker_cnt = max(self.right_blinker_cnt - 1, 0)

    self.left_blinker_prev = left_blinker_stalk
    self.right_blinker_prev = right_blinker_stalk

    return bool(left_blinker_stalk or self.left_blinker_cnt > 0), bool(right_blinker_stalk or self.right_blinker_cnt > 0)

  def update_custom_stock_long(self, cruise_button, final_speed_kph, target_speed, v_set_dis, speed_diff, button_type):
    customStockLong = car.CarState.CustomStockLong.new_message()
    customStockLong.cruiseButton = 0 if cruise_button is None else cruise_button
    customStockLong.finalSpeedKph = final_speed_kph
    customStockLong.targetSpeed = target_speed
    customStockLong.vSetDis = v_set_dis
    customStockLong.speedDiff = speed_diff
    customStockLong.buttonType = button_type
    return customStockLong

  @staticmethod
  def parse_gear_shifter(gear: str | None) -> car.CarState.GearShifter:
    if gear is None:
      return GearShifter.unknown

    d: dict[str, car.CarState.GearShifter] = {
      'P': GearShifter.park, 'PARK': GearShifter.park,
      'R': GearShifter.reverse, 'REVERSE': GearShifter.reverse,
      'N': GearShifter.neutral, 'NEUTRAL': GearShifter.neutral,
      'E': GearShifter.eco, 'ECO': GearShifter.eco,
      'T': GearShifter.manumatic, 'MANUAL': GearShifter.manumatic,
      'D': GearShifter.drive, 'DRIVE': GearShifter.drive,
      'S': GearShifter.sport, 'SPORT': GearShifter.sport,
      'L': GearShifter.low, 'LOW': GearShifter.low,
      'B': GearShifter.brake, 'BRAKE': GearShifter.brake,
    }
    return d.get(gear.upper(), GearShifter.unknown)

  @staticmethod
  def get_cam_can_parser(CP):
    return None

  @staticmethod
  def get_adas_can_parser(CP):
    return None

  @staticmethod
  def get_body_can_parser(CP):
    return None

  @staticmethod
  def get_loopback_can_parser(CP):
    return None


INTERFACE_ATTR_FILE = {
  "FINGERPRINTS": "fingerprints",
  "FW_VERSIONS": "fingerprints",
}

# interface-specific helpers

def get_interface_attr(attr: str, combine_brands: bool = False, ignore_none: bool = False) -> dict[str | StrEnum, Any]:
  # read all the folders in selfdrive/car and return a dict where:
  # - keys are all the car models or brand names
  # - values are attr values from all car folders
  result = {}
  for car_folder in sorted([x[0] for x in os.walk(BASEDIR + '/selfdrive/car')]):
    try:
      brand_name = car_folder.split('/')[-1]
      brand_values = __import__(f'openpilot.selfdrive.car.{brand_name}.{INTERFACE_ATTR_FILE.get(attr, "values")}', fromlist=[attr])
      if hasattr(brand_values, attr) or not ignore_none:
        attr_data = getattr(brand_values, attr, None)
      else:
        continue

      if combine_brands:
        if isinstance(attr_data, dict):
          for f, v in attr_data.items():
            result[f] = v
      else:
        result[brand_name] = attr_data
    except (ImportError, OSError):
      pass

  return result


class NanoFFModel:
  def __init__(self, weights_loc: str, platform: str):
    self.weights_loc = weights_loc
    self.platform = platform
    self.load_weights(platform)

  def load_weights(self, platform: str):
    with open(self.weights_loc) as fob:
      self.weights = {k: np.array(v) for k, v in json.load(fob)[platform].items()}

  def relu(self, x: np.ndarray):
    return np.maximum(0.0, x)

  def forward(self, x: np.ndarray):
    assert x.ndim == 1
    x = (x - self.weights['input_norm_mat'][:, 0]) / (self.weights['input_norm_mat'][:, 1] - self.weights['input_norm_mat'][:, 0])
    x = self.relu(np.dot(x, self.weights['w_1']) + self.weights['b_1'])
    x = self.relu(np.dot(x, self.weights['w_2']) + self.weights['b_2'])
    x = self.relu(np.dot(x, self.weights['w_3']) + self.weights['b_3'])
    x = np.dot(x, self.weights['w_4']) + self.weights['b_4']
    return x

  def predict(self, x: list[float], do_sample: bool = False):
    x = self.forward(np.array(x))
    if do_sample:
      pred = np.random.laplace(x[0], np.exp(x[1]) / self.weights['temperature'])
    else:
      pred = x[0]
    pred = pred * (self.weights['output_norm_mat'][1] - self.weights['output_norm_mat'][0]) + self.weights['output_norm_mat'][0]
    return pred
