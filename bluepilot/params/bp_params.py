import json
from typing import Any
from openpilot.common.params import Params, ParamKeyFlag, ParamKeyType
from bluepilot.logger.bp_logger import debug, error

# Define the path to the params.json file
PARAMS_JSON_PATH = "/data/openpilot/bluepilot/params/params.json"
SHOW_DEBUG_OUTPUT = False

# Global parameter cache
_params_data = None
_cc_props_cache = {}
_interface_props_cache = {}

# Map the flag names to values using the new ParamKeyFlag enum values
flag_mapping = {
  "PERSISTENT": ParamKeyFlag.PERSISTENT,
  "CLEAR_ON_MANAGER_START": ParamKeyFlag.CLEAR_ON_MANAGER_START,
  "CLEAR_ON_ONROAD_TRANSITION": ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION,
  "CLEAR_ON_OFFROAD_TRANSITION": ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION,
  "DONT_LOG": ParamKeyFlag.DONT_LOG,
  "DEVELOPMENT_ONLY": ParamKeyFlag.DEVELOPMENT_ONLY,
  "CLEAR_ON_IGNITION_ON": ParamKeyFlag.CLEAR_ON_IGNITION_ON,
  "BACKUP": ParamKeyFlag.BACKUP,
  "ALL": ParamKeyFlag.ALL,
}

# Map parameter types to ParamKeyType enum
type_mapping = {
  "string": ParamKeyType.STRING,
  "bool": ParamKeyType.BOOL,
  "int": ParamKeyType.INT,
  "float": ParamKeyType.FLOAT,
  "time": ParamKeyType.TIME,
  "json": ParamKeyType.JSON,
  "bytes": ParamKeyType.BYTES,
}


def log_debug(message: str) -> None:
  if SHOW_DEBUG_OUTPUT:
    debug(message)
    print(message)


def load_params_json() -> dict[str, Any]:
  """Get params data from cache or load from file if not yet loaded"""
  global _params_data

  # Return cached data if available
  if _params_data is not None:
    return _params_data

  # Load file and cache results
  try:
    with open(PARAMS_JSON_PATH) as f:
      _params_data = json.load(f)
      log_debug(f"Successfully loaded params.json from {PARAMS_JSON_PATH}")
      return _params_data
  except FileNotFoundError:
    log_debug(f"Params JSON file not found at {PARAMS_JSON_PATH}")
    return {"params": []}
  except json.JSONDecodeError:
    log_debug(f"Failed to parse JSON from {PARAMS_JSON_PATH}")
    return {"params": []}


def preprocess_params_data() -> None:
  """Preprocess params data for quick lookups by property type"""
  global _cc_props_cache, _interface_props_cache

  # Clear existing caches
  _cc_props_cache.clear()
  _interface_props_cache.clear()

  params_json = load_params_json()

  for param in params_json.get("params", []):
    param_name = param["name"]

    # Cache by ccProp if present
    if "ccProp" in param:
      _cc_props_cache[param["ccProp"]] = param

    # Cache by interfaceProp if present
    if "interfaceProp" in param:
      _interface_props_cache[param["interfaceProp"]] = param


def check_param_exists(params: Params, key: str) -> bool:
  try:
    exists = params.check_key(key)
    log_debug(f"Checked if parameter {key} exists: {exists}")
    return exists
  except Exception as e:
    error(f"Error checking if parameter {key} exists: {e}", True)
    return False


def get_param_value(params: Params, param_name: str, param_type: str, default_value: Any) -> Any:
  try:
    if not check_param_exists(params, param_name):
      log_debug(f"Parameter {param_name} not found, using default value: {default_value}")
      return default_value

    # Use type-specific getters for better type safety
    try:
      if param_type == "bool":
        result = params.get_bool(param_name)
        log_debug(f" - Retrieved bool parameter {param_name}: {result}")
        return result
      elif param_type == "int":
        result = params.get_int(param_name)
        log_debug(f" - Retrieved int parameter {param_name}: {result}")
        return result
      elif param_type == "float":
        result = params.get_float(param_name)
        log_debug(f" - Retrieved float parameter {param_name}: {result}")
        return result
      else:
        # For string and other types, use the generic get method
        value = params.get(param_name)
        if value is None or value == "":
          log_debug(f"Parameter {param_name} is empty or None, using default value: {default_value}")
          return default_value
        log_debug(f" - Retrieved string parameter {param_name}: {value}")
        return value
    except (ValueError, TypeError) as e:
      error(f"Could not convert {param_name} value to {param_type}. Using default: {default_value}. Error: {e}", True)
      return default_value
  except Exception as e:
    error(f"Error getting parameter {param_name}: {e}. Using default: {default_value}", True)
    return default_value


def initialize_custom_params(params: Params) -> None:
  log_debug("Initializing custom parameters")

  # Load params.json and preprocess for efficient lookups
  load_params_json()
  preprocess_params_data()

  # Register and initialize parameters
  for param in _params_data.get("params", []):
    name = param["name"]
    param_type = param["type"]
    default_value = param["default"]
    flags = 0
    flag_list = []

    # Step 1: Register the parameter with the new system
    for flag_str in param.get("flags", []):
      flag_value = flag_mapping.get(flag_str)
      if flag_value is not None:
        flags |= flag_value
        flag_list.append(flag_str)

    log_debug(f"Registering {name} with flags: {flag_list} (value: {flags:#x})")
    try:
      params.register_key(name, flags)
    except Exception as e:
      error(f"Error registering {name}: {e}", True)
      continue  # Skip to next param if registration fails

    # Step 2: Check if parameter exists and its current value
    current_value = None
    try:
      if check_param_exists(params, name):
        # Use type-specific getters for better type safety
        if param_type == "bool":
          current_value = params.get_bool(name)
        elif param_type == "int":
          current_value = params.get_int(name)
        elif param_type == "float":
          current_value = params.get_float(name)
        else:
          current_value = params.get(name)
        log_debug(f"Param {name} exists with value: '{current_value}'")
      else:
        log_debug(f"Param {name} does not exist yet")
    except Exception as e:
      error(f"Error checking existence of {name}: {e}", True)

    # Step 3: Handle default parameter creation if specified
    if param.get("create_default", False):
      default_param_name = f"{name}_default"
      log_debug(f"Creating _default version of {name} with default value: {default_value}")
      try:
        params.register_key(default_param_name, 0)
        default_value_str = (
          "1" if param_type == "bool" and default_value else "0" if param_type == "bool" else str(default_value) if default_value is not None else ""
        )
        if default_value_str:
          params.put(default_param_name, default_value_str)
          log_debug(f"Set {default_param_name} to: '{default_value_str}'")
        else:
          log_debug(f"No value set for {default_param_name} (default is empty)")
      except Exception as e:
        error(f"Error creating/setting {default_param_name}: {e}", True)

    # Step 4: Set the parameter value if it doesn't exist or is empty
    if current_value in (None, ""):
      # Use type-specific putters for better type safety
      try:
        if param_type == "bool":
          params.put_bool(name, bool(default_value))
          log_debug(f"Set {name} to bool default value: {bool(default_value)}")
        elif param_type == "int":
          params.put_int(name, int(default_value) if default_value is not None else 0)
          log_debug(f"Set {name} to int default value: {int(default_value) if default_value is not None else 0}")
        elif param_type == "float":
          params.put_float(name, float(default_value) if default_value is not None else 0.0)
          log_debug(f"Set {name} to float default value: {float(default_value) if default_value is not None else 0.0}")
        else:
          value = str(default_value) if default_value is not None else ""
          if value:
            params.put(name, value)
            log_debug(f"Set {name} to string default value: '{value}'")
          else:
            log_debug(f"No value set for {name} (default is empty)")
      except Exception as e:
        error(f"Error setting {name} to default value: {e}", True)
    else:
      log_debug(f"Retaining existing value for {name}: '{current_value}'")

  log_debug("Custom parameters initialization complete")


def apply_custom_params(obj: Any, prop_key: str, component_type: str) -> None:
  log_debug(f"Applying custom parameters for {component_type} with property key {prop_key}")

  # Use the appropriate cache based on prop_key
  prop_cache = _cc_props_cache if prop_key == "ccProp" else _interface_props_cache

  params = Params()
  params_applied = 0
  params_changed = 0
  changed_params = []

  # Process parameters that match our property key
  for attr_name, param in prop_cache.items():
    param_name = param["name"]
    param_type = param["type"]
    default_value = param["default"]
    min_value = param.get("min")
    max_value = param.get("max")

    try:
      value = get_param_value(params, param_name, param_type, default_value)

      # Clamp int/float values if min/max are specified
      if param_type == "int" and value is not None:
        try:
          value = int(value)
          if min_value is not None and value < min_value:
            value = min_value
            print(f"Clamped {param_name} to min value: {min_value}")
          if max_value is not None and value > max_value:
            value = max_value
            print(f"Clamped {param_name} to max value: {max_value}")
        except ValueError:
          log_debug(f"Invalid int value for {param_name}, using default: {default_value}")
          value = default_value
      elif param_type == "float" and value is not None:
        try:
          value = float(value)
          if min_value is not None and value < min_value:
            value = min_value
            print(f"Clamped {param_name} to min value: {min_value}")
          if max_value is not None and value > max_value:
            value = max_value
            print(f"Clamped {param_name} to max value: {max_value}")
        except ValueError:
          error(f"Invalid float value for {param_name}, using default: {default_value}")
          value = default_value

      # Always set the attribute on the object, even if it doesn't exist
      if hasattr(obj, attr_name):
        initial_val = getattr(obj, attr_name)
        if initial_val != value:
          params_changed += 1
          changed_params.append({"param_name": param_name, "attr_name": attr_name, "old_value": initial_val, "new_value": value})
          log_debug(f"Parameter '{param_name}' changed from '{initial_val}' to '{value}' in {component_type}")
        else:
          log_debug(f"Parameter '{param_name}' unchanged: '{value}' in {component_type}")
      else:
        log_debug(f"Creating new attribute '{attr_name}' with value '{value}' in {component_type}")
        changed_params.append({"param_name": param_name, "attr_name": attr_name, "old_value": "UNSET", "new_value": value})
        params_changed += 1

      # Set the attribute regardless
      setattr(obj, attr_name, value)
      params_applied += 1
    except Exception as e:
      error(f"Error applying parameter {param_name}: {e}", True)

  # Summary of changes
  if params_changed > 0:
    log_debug(f"SUMMARY: {params_changed} parameters changed in {component_type}:")
    for change in changed_params:
      if change["old_value"] != "UNSET":
        print(f"  • {change['param_name']} ({change['attr_name']}): {change['old_value']} → {change['new_value']}")
  else:
    log_debug(f"No parameter values changed in {component_type}")

  log_debug(f"Completed applying {params_applied} custom parameters for {component_type}")


def load_custom_params(obj: Any, component_type: str = "carcontroller") -> None:
  log_debug(f"Loading initial custom parameters for {component_type}")
  apply_custom_params(obj, "ccProp" if component_type == "carcontroller" else "interfaceProp", component_type)
  log_debug(f"Completed loading initial custom parameters for {component_type}")


def update_custom_params(obj: Any, component_type: str = "carcontroller") -> None:
  log_debug(f"Updating custom parameters for {component_type}")
  apply_custom_params(obj, "ccProp" if component_type == "carcontroller" else "interfaceProp", component_type)
  log_debug(f"Completed updating custom parameters for {component_type}")


def apply_interface_params(ret: Any, component_type: str = "interface") -> None:
  log_debug(f"Applying interface parameters for {component_type}")
  apply_custom_params(ret, "interfaceProp", component_type)
  log_debug(f"Completed applying interface parameters for {component_type}")
