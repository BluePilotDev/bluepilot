from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.events import ET
from openpilot.selfdrive.car import DT_CTRL
from openpilot.selfdrive.car.ford.values import CarControllerParams, FORD_VEHICLE_TUNINGS

params = Params()

# Define settings_params here | param_name, self_key_name, default_value, param_type
SETTINGS_PARAMS = [
  ["FordPrefSendHandsFreeCanMsg", "send_hands_free_cluster_msg", False, bool],
  ["FordPrefHumanTurnDetectionEnable", "enable_human_turn_detection", True, bool],
  ["FordPrefLaneDepartCanMsg", "send_lane_depart_can_msg", False, bool],
  ["FordPrefDriverMonitorCanMsg", "send_driver_monitor_can_msg", False, bool],
  ["FordLatTuningLowCurvPCFactor", "low_curv_pc_factor", 0.65, float],
  ["FordLatTuningHighCurvPCFactor", "high_curv_pc_factor", 0.45, float],
  ["FordLatTuningHighCurvFactor", "high_curvature_factor", 0.99, float],
  ["FordLatTuningPathLookupTime", "path_lookup_time", None, float],
  ["FordLatTuningResetLookupTime", "reset_lookup_time", None, float],
  ["FordLatTuningLaneChgModifier", "lane_change_factor", None, float],
  ["FordLongTuningBrakeActuatorTarget", "brake_actuator_target", None, float],
  ["FordLongTuningBrakeActuatorStdDevLow", "brake_actuator_stdDevLow", None, float],
  ["FordLongTuningBrakeActuatorStdDevHigh", "brake_actuator_stdDevHigh", None, float],
  ["FordLongTuningPrechargeActuatorTarget", "precharge_actuator_target", None, float],
  ["FordLongTuningPrechargeActuatorStdDevLow", "precharge_actuator_stdDevLow", None, float],
  ["FordLongTuningPrechargeActuatorStdDevHigh", "precharge_actuator_stdDevHigh", None, float],
  ["FordLimitsCurvatureMax", "curvature_max", CarControllerParams.CURVATURE_MAX, float],
  ["FordLimitsCurvatureError", "curvature_error", CarControllerParams.CURVATURE_ERROR, float],
]


def load_initial_cc_pref_params(self_obj): # self_obj is the CarController object (self)
  self_obj.send_hands_free_cluster_msg = get_bool_param("FordPrefSendHandsFreeCanMsg", False)
  self_obj.enable_human_turn_detection = get_bool_param("FordPrefHumanTurnDetectionEnable", True)
  self_obj.send_lane_depart_can_msg = get_bool_param("FordPrefLaneDepartCanMsg", False)
  self_obj.send_driver_monitor_can_msg = get_bool_param("FordPrefDriverMonitorCanMsg", False)
  self_obj.low_curv_pc_factor = get_float_param("FordLatTuningLowCurvPCFactor", 0.4)
  self_obj.high_curv_pc_factor = get_float_param("FordLatTuningHighCurvPCFactor", 0.55)
  self_obj.high_curvature_factor = get_float_param("FordLatTuningHighCurvFactor", 1.00)
  self_obj.curvature_max = get_float_param("FordLimitsCurvatureMax", CarControllerParams.CURVATURE_MAX)
  self_obj.curvature_error = get_float_param("FordLimitsCurvatureError", CarControllerParams.CURVATURE_ERROR)


def initialize_param_defaults(self_obj):
  for param_name, self_key_name, default_value, param_type in SETTINGS_PARAMS:
    # Only process parameters that exist in self_obj
    if hasattr(self_obj, self_key_name):
      default_param_name = f"{param_name}_default"

      # Determine the correct default value
      correct_default = default_value if default_value is not None else getattr(self_obj, self_key_name)

      # Always set the _default param
      set_param_value(default_param_name, correct_default, param_type)

      # Set the main param if it doesn't exist
      if params.get(param_name) is None:
        set_param_value(param_name, correct_default, param_type)

      # Load the current param value to self_obj
      value = get_param_value(param_name, param_type)
      setattr(self_obj, self_key_name, value)


def set_param_value(param_name, value, param_type):
  try:
    print(f"Setting value for {param_name} to {value}")
    cloudlog.debug(f"Setting value for {param_name} to {value}")
    if param_type is bool:
      params.put(param_name, '1' if value else '0')
    elif param_type is int:
      params.put(param_name, str(int(value)))
    elif param_type is float:
      params.put(param_name, str(float(value)))
    else:
      params.put(param_name, str(value))
  except ValueError:
    print(f"Warning: Could not convert {param_name} value '{value}' to {param_type}. Using default.")


def logDebug(message):
  if(get_bool_param("FordPrefEnableDebugLogs", False)):
    print(message)
    cloudlog.debug(message)


def logError(message):
    print(message)
    cloudlog.error(message)


def logWarn(message):
  print(message)
  cloudlog.warn(message)


def update_settings_params(self_obj):
  for param_name, self_key_name, _, param_type in SETTINGS_PARAMS:
    value = get_param_value(param_name, param_type)
    setattr(self_obj, self_key_name, value)


def get_param_value(param_name, param_type):
  if param_type is bool:
    return get_bool_param(param_name)
  elif param_type is int:
    return get_int_param(param_name)
  elif param_type is float:
    return get_float_param(param_name)
  else:
    return get_str_param(param_name)


def get_bool_param(key, default_value=False):
  value = params.get(key, encoding='utf8')
  if value is None:
    return default_value
  return value == '1'


def get_int_param(key, default_value=0):
  value = params.get(key, encoding='utf8')
  if value is None:
    return default_value
  try:
    return int(value)
  except ValueError:
    logWarn(f"Warning: Could not convert {key} value '{value}' to int. Using default.")
    return default_value


def get_float_param(key, default_value=0.0):
  value = params.get(key, encoding='utf8')
  if value is None:
    return default_value
  try:
    return float(value)
  except ValueError:
    logWarn(f"Warning: Could not convert {key} value '{value}' to float. Using default.")

    return default_value


def get_str_param(key, default_value=""):
  value = params.get(key, encoding='utf8')
  return value if value is not None else default_value


def get_ford_vehicle_tuning_carcontroller(carFingerprint):
  if carFingerprint in FORD_VEHICLE_TUNINGS:
    logDebug(f'Matched carFingerprint in CarController | FingerPrint: {carFingerprint}')
    logDebug(f'CarController Tuning: {FORD_VEHICLE_TUNINGS[carFingerprint]}')
    tuning = FORD_VEHICLE_TUNINGS[carFingerprint]
    values = {
      "path_lookup_time": tuning.get("path_lookup_time"),
      "reset_lookup_time": tuning.get("reset_lookup_time"),
      "brake_actuator_target": tuning.get("brake_actuator_target"),
      "brake_actuator_stdDevLow": tuning.get("brake_actuator_stdDevLow"),
      "brake_actuator_stdDevHigh": tuning.get("brake_actuator_stdDevHigh"),
      "precharge_actuator_target": tuning.get("precharge_actuator_target"),
      "precharge_actuator_stdDevLow": tuning.get("precharge_actuator_stdDevLow"),
      "precharge_actuator_stdDevHigh": tuning.get("precharge_actuator_stdDevHigh"),
      "lane_change_factor": tuning.get("lane_change_factor"),
    }
    logDebug(f'CarController Tuning Values: {values}')
    return values
  return None


def get_ford_vehicle_tuning_interface(candidate):
  if candidate in FORD_VEHICLE_TUNINGS:
    logDebug(f'Matched carFingerprint in CarInterface | FingerPrint: {candidate}')
    tuning = FORD_VEHICLE_TUNINGS[candidate]
    values = {
      "steerActuatorDelay": tuning.get("steerActuatorDelay"),
      "steerLimitTimer": tuning.get("steerLimitTimer"),
      "stoppingControl": tuning.get("stoppingControl"),
      "startingState": tuning.get("startingState"),
      "startAccel": tuning.get("startAccel"),
      "stoppingDecelRate": tuning.get("stoppingDecelRate"),
      "longitudinalTuning": tuning.get("longitudinalTuning"),
    }
    logDebug(f'CarInterface Tuning Values: {values}')
    return values
  return None


def hysteresis(current_value, old_value, target, stdDevLow: float, stdDevHigh: float):
  if target - stdDevLow < current_value < target + stdDevHigh:
    result = old_value
  elif current_value <= target - stdDevLow:
    result = 1
  elif current_value >= target + stdDevHigh:
    result = 0

  return result


def actuators_calc(cc_self, brake): # cc_self is the CarController object (self)
  ts = cc_self.frame * DT_CTRL
  brake_actuate = hysteresis(brake, cc_self.brake_actuate_last, cc_self.brake_actuator_target, cc_self.brake_actuator_stdDevLow, cc_self.brake_actuator_stdDevHigh)
  logDebug(f"actuators_calc brake_actuate: {brake} {cc_self.brake_actuate_last} {cc_self.brake_actuator_target - cc_self.brake_actuator_stdDevLow} {cc_self.brake_actuator_target + cc_self.brake_actuator_stdDevHigh} {brake_actuate}")

  cc_self.brake_actuate_last = brake_actuate

  precharge_actuate = hysteresis(
    brake, cc_self.precharge_actuate_last, cc_self.precharge_actuator_target, cc_self.precharge_actuator_stdDevLow, cc_self.precharge_actuator_stdDevHigh
  )
  logDebug(f"actuators_calc precharge_actuate: {brake} {cc_self.precharge_actuate_last} {cc_self.precharge_actuator_target - cc_self.precharge_actuator_stdDevLow} {cc_self.precharge_actuator_target + cc_self.precharge_actuator_stdDevHigh} {precharge_actuate}")

  if precharge_actuate and not cc_self.precharge_actuate_last:
    cc_self.precharge_actuate_ts = ts
  elif not precharge_actuate:
    cc_self.precharge_actuate_ts = 0

  if (
    precharge_actuate
    and not brake_actuate
    and cc_self.precharge_actuate_ts > 0
    and brake > (cc_self.precharge_actuator_target - cc_self.precharge_actuator_stdDevLow)
    and (ts - cc_self.precharge_actuate_ts) > (200 * DT_CTRL)
  ):
    precharge_actuate = False

  cc_self.precharge_actuate_last = precharge_actuate
  logDebug(f"actuators_calc: {precharge_actuate} {brake_actuate}")

  return precharge_actuate, brake_actuate

def get_dm_driver_state(d_state):
  if d_state == "preDriverDistracted/permanent":
    return "preDistracted"
  elif d_state == "promptDriverDistracted/permanent":
    return "promptDistracted"
  elif d_state == "driverDistracted/permanent":
    return "distracted"
  elif d_state == "preDriverUnresponsive/permanent":
    return "preUnresponsive"
  elif d_state == "promptDriverUnresponsive/permanent":
    return "promptUnresponsive"
  elif d_state == "driverUnresponsive/permanent":
    return "unresponsive"
  else:
    return "none"

def get_dm_disable_state(d_state):
  if d_state == ET.USER_DISABLE:
    return "userDisable"
  elif d_state == ET.SOFT_DISABLE:
    return "softDisable"
  elif d_state == ET.IMMEDIATE_DISABLE:
    return "immediateDisable"
  elif d_state == ET.NO_ENTRY:
    return "noEntry"
  else:
    return "none"


def compute_dm_msg_values(hud_control, send_hands_free_cluster_msg):
    tja_msg = 0
    tja_warn = 0

    disableState = get_dm_disable_state(hud_control.alertType)
    driverState = get_dm_driver_state(hud_control.eventType)

    if send_hands_free_cluster_msg:
      if disableState == "noEntry":
        tja_msg = 4  # BlueCruise not available
      elif (driverState in ("distracted", "unresponsive") or disableState in ("softDisable", "immediateDisable")):
        tja_warn = 3  # Resume Control
      elif disableState == "userDisable":
        tja_warn = 1  # Cancelled
      elif driverState == "preDistracted":
        tja_warn = 6  # Watch The Road (no chime)
      elif driverState == "promptDistracted":
        tja_warn = 7  # Watch The Road (chime)
      elif hud_control.leftLaneDepart:
        tja_warn = 5  # Left Lane Departure (chime)
      elif hud_control.rightLaneDepart:
        tja_warn = 4  # Right Lane Departure (chime)
      else:
        tja_warn = 0
    else:
      if disableState == "noEntry":
        tja_msg = 1  # Lane Centering Assist not available
      elif (driverState in ("distracted", "unresponsive") or disableState in ("softDisable", "immediateDisable")):
        tja_warn = 3  # Resume Control
      elif disableState == "userDisable":
        tja_warn = 1  # Cancelled
      else:
        tja_warn = 0

    return tja_msg, tja_warn
