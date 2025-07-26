"""
fordcanparser.py - Ford CAN data parsing and publishing
"""

import json
from typing import Any
from dataclasses import dataclass
from enum import Enum, auto
from time import time
from datetime import datetime
import os

import cereal.messaging as messaging
from opendbc.can.parser import CANParser
from opendbc.can.can_define import CANDefine
from opendbc.car.ford.values import DBC
# from openpilot.common.swaglog import cloudlog
from opendbc.car.ford.helpers import get_hev_power_flow_text, get_hev_engine_on_reason_text  # noqa: F401


class SignalType(Enum):
  FLOAT = auto()
  INTEGER = auto()
  ENUM = auto()
  BOOLEAN = auto()


@dataclass
class SignalInfo:
  signal_type: SignalType
  enum_values: dict[int, str] | None = None


class FordCanParser:
  def __init__(self, CP):
    """Initialize DBC parser"""
    try:
      self.dbc_name = DBC[CP.carFingerprint]["pt"]
      self.can_define = CANDefine(self.dbc_name)
      self._signal_info_cache: dict[str, SignalInfo] = {}
      self.last_publish_time = 0.0  # Track last publish time
      self.publish_rate = 0.1  # Publish rate in seconds (10Hz)
      self.block_msgs = {} # Tries to block messages that match these strings
      self.block_msgs_startswith = {  # Tries to block messages that start with these strings
        "GlobalClock_Data",
        "GWM_AutoSar_Net",
        "GWM_HPCM_i_FrP",
        "ElecHorizon_Data1_FD1",
      }
      self.block_msgs_endswith = { } # Tries to block messages that end with these strings
      self.block_sigs = {} # Tries to block signals that match these strings
      self.block_sigs_startswith = { } # Tries to block signals that start with these strings
      self.block_sigs_endswith = { } # Tries to block signals that end with these strings
      print(f"Initialized Ford CAN parser with DBC: {self.dbc_name}")
    except Exception as e:
      # cloudlog.exception("Error initializing Ford CAN parser")
      print(f"Error initializing Ford CAN parser: {e}")
      self.can_define = None


  def _get_signal_info(self, msg_name: str, sig_name: str) -> SignalInfo:
    """Cache and return signal type information"""
    cache_key = f"{msg_name}.{sig_name}"
    if cache_key in self._signal_info_cache:
      return self._signal_info_cache[cache_key]

    try:
      if msg_name in self.can_define.dv and sig_name in self.can_define.dv[msg_name]:
        enum_values = self.can_define.dv[msg_name][sig_name]
        info = SignalInfo(SignalType.ENUM, enum_values)
      else:
        info = SignalInfo(SignalType.FLOAT)

      self._signal_info_cache[cache_key] = info
      return info

    except Exception as e:
      # cloudlog.exception(f"Error determining signal type for {msg_name}.{sig_name}")
      print(f"Error determining signal type for {msg_name}.{sig_name}: {e}")
      return SignalInfo(SignalType.FLOAT)


  def _convert_to_string(self, value: Any, signal_type: SignalType) -> str:
    """
    Convert any value to string representation based on signal type
    Args:
        value: The value to convert
        signal_type: The SignalType enum indicating how to handle the conversion
    Returns:
        str: String representation of the value
    """
    try:
      match signal_type:
        case SignalType.ENUM:
          return str(int(float(value)))
        case SignalType.FLOAT:
          return str(value)
        case SignalType.INTEGER:
          return str(int(value))
        case SignalType.BOOLEAN:
          return str(bool(value))
        case _:
          return str(value)
    except (ValueError, TypeError):
      return "0" if signal_type == SignalType.ENUM else "0.0"


  def _get_signal_metadata(self, msg_name: str, sig_name: str) -> dict:
    metadata = {}
    try:
        # Assuming these methods exist in CANDefine
        metadata['unit'] = self.can_define.get_signal_unit(msg_name, sig_name)
        metadata['sender'] = self.can_define.get_signal_sender(msg_name, sig_name)
    except Exception as e:
        print(f"Error getting metadata for {msg_name}.{sig_name}: {e}")
        metadata['unit'] = ""
        metadata['sender'] = ""
    return metadata


  def publish_can_data(self, cp: CANParser, cp_cam: CANParser, carFingerprint: str) -> None:
    """Publish CAN data with metadata from both main and camera buses"""
    if self.can_define is None or time() - self.last_publish_time < self.publish_rate:
      return

    try:
      can_data = {'metadata': {'main_bus': {}, 'camera_bus': {}}}
      bus_data = [('main_bus', cp), ('camera_bus', cp_cam)]

      for bus_name, parser in bus_data:
        keys = list(parser.vl.keys())
        msg_names = {keys[i]: keys[i + 1] for i in range(0, len(keys), 2) if i + 1 < len(keys)}

        for msg_id, signals in parser.vl.items():
          msg_name = msg_names.get(msg_id, msg_id)
          # Skip blacklisted messages if they exist (allow support for wildcards like "GlobalClock_Data*")
          if msg_name in self.block_msgs or msg_name.startswith(self.block_msgs_startswith) or msg_name.endswith(self.block_msgs_endswith):
            continue

          can_data['metadata'][bus_name][msg_name] = {'signals': {}}

          for sig_name, raw_value in signals.items():
            # Skip blacklisted signals if they exist (allow support for wildcards like "VehicleGGCCData*")
            if sig_name in self.block_sigs or sig_name.startswith(self.block_sigs_startswith) or sig_name.endswith(self.block_sigs_endswith):
              continue

            signal_info = self._get_signal_info(msg_name, sig_name)
            signal_metadata = {'type': signal_info.signal_type.name, 'rawValue': self._convert_to_string(raw_value, signal_info.signal_type)}

            if signal_info.enum_values is not None:
              signal_metadata['enumValues'] = {str(k): str(v) for k, v in signal_info.enum_values.items()}

            if msg_id in parser.ts_nanos and sig_name in parser.ts_nanos[msg_id]:
              signal_metadata['timestamp'] = parser.ts_nanos[msg_id][sig_name]

            can_data['metadata'][bus_name][msg_name]['signals'][sig_name] = signal_metadata

      categorized_data = {}
      try:
        categorizer = FordCanCategorizer()
        categorized_data = categorizer.process_can_data(json.dumps(can_data))
      except Exception as e:
        print(f"Error categorizing CAN data: {e}")

      pm = messaging.PubMaster(['vehicleCanData'])
      msg = messaging.new_message('vehicleCanData')
      msg.vehicleCanData.data = json.dumps(can_data)
      pm.send('vehicleCanData', msg)

      self.save_can_data(can_data, categorized_data, carFingerprint, time())
      self.last_publish_time = time()

    except Exception as e:
      print(f'Error publishing CAN data: {e}')


  def save_can_data(self, can_data: dict, categorized_data: dict, carFingerprint: str, current_time: float) -> None:
    """Save both raw and processed CAN data"""
    try:
      # Create directory path
      data_dir = f'/data/exports/can_data/{carFingerprint}'
      os.makedirs(data_dir, exist_ok=True)

      # Save raw CAN data with timestamp
      raw_filename = f'data_{int(current_time)}.json'
      raw_filepath = os.path.join(data_dir, raw_filename)
      with open(raw_filepath, 'w') as f:
        json.dump(can_data, f)

      # Save processed data without timestamp
      processed_filepath = os.path.join(data_dir, 'processed_data.json')
      with open(processed_filepath, 'w') as f:
        json.dump(categorized_data, f, indent=2)

      # Rotate old raw data files if needed
      try:
        files = [f for f in os.listdir(data_dir) if f.startswith('data_')]
        files.sort(key=lambda x: os.path.getctime(os.path.join(data_dir, x)))
        while len(files) > 5:
          os.remove(os.path.join(data_dir, files.pop(0)))
      except Exception as e:
        print(f"Error rotating files: {e}")

    except Exception as e:
      print(f"Error saving CAN data: {e}")


  def process_dynamic_signals(self, cp, ret, signal_mapping: dict) -> None:
    """
    Process dynamic signals and attach their values to the return object
    Args:
        cp: CAN parser object containing vehicle signals
        ret: Return object to attach values to
        signal_mapping: Dictionary mapping signals to return properties with defaults and optional processing functions
    Example mapping:
    {
      "Cluster_HEV_Data2": {
        "EffWhlLvl2_Pc_Dsply": {"returnProp": "hevThrottleDemandPercent", "default": 0},
        "PwrFlowTxt_D_Dsply": {"returnProp": "hevPowerFlowMode", "default": "", "func": "get_hev_power_flow_text"}
      }
    }
    """
    for main_signal, sub_signals in signal_mapping.items():
      # Skip blacklisted main signals
      if main_signal in self.block_msgs or main_signal.startswith(self.block_msgs_startswith) or main_signal.endswith(self.block_msgs_endswith):
        continue

      try:
        signal_values = cp.vl[main_signal]
        for sub_signal, mapping in sub_signals.items():
          # Skip blacklisted main signals
          if sub_signal in self.block_sigs or sub_signal.startswith(self.block_sigs_startswith) or sub_signal.endswith(self.block_sigs_endswith):
            continue

          return_prop = mapping["returnProp"]
          default_value = mapping["default"]

          # Get the raw value
          raw_value = signal_values.get(sub_signal, default_value)

          # Process value through function if specified
          if "func" in mapping:
            # Get the function from the current module
            process_func = globals().get(mapping["func"])
            if process_func:
              value = process_func(raw_value)
            else:
              value = raw_value
          else:
            value = raw_value

          setattr(ret, return_prop, value)
      except (KeyError, AttributeError):
        # If main signal doesn't exist, set all sub-signal return props to their defaults
        for mapping in sub_signals.values():
          setattr(ret, mapping["returnProp"], mapping["default"])


class FordCanCategorizer:
  def __init__(self):
    self.signal_descriptions = {
      # Powertrain
      "EngVehicleSpThrottle": "Engine throttle position",
      "GearLvrPos_D_Actl": "Actual gear lever position",
      "TrnRng_D_Rq": "Transmission range request",
      # Vehicle dynamics
      "Veh_V_ActlBrk": "Vehicle speed from brake system",
      "VehYaw_W_Actl": "Vehicle yaw rate",
      "VehStop_D_Stat": "Vehicle stop status",
      # Driver assistance
      "CcStat_D_Actl": "Cruise control status",
      "AccEnbl_B_RqDrv": "ACC enable request from driver",
      "AccStopMde_D_Rq": "ACC stop mode request",
      # Add more signal descriptions as needed
    }


    # Define signal categorization mappings
    self.signal_categories = {
      "powertrain": {
        "patterns": [
          "PowertrainData",
          "EngVehicleSpThrottle",
          "PrplWhl",
          "Eng_D_",
          "TrnAin",
          "PwPck",
          "EngAout",
          "EngIdl",
          "EngOil",
          "TrnGear",
          "GearPos",
          "GearLvr",
          "GearEngag",
          "Gear_Shift",
          "EngBrakeData",
          "PwrFlow",
          "EngPw",
          "EngActv",
          "EffWhl",
          "TrnRng",
          "TrnPrkSys",
          "TrnGsm",
          "TrnBtsi",
          "TrnValid",
          "TrnIgn",
        ],
        "description": "Powertrain, transmission, and engine related signals",
      },
      "vehicle_dynamics": {
        "patterns": [
          "BrakeSnData",
          "VehStab",
          "VehYaw",
          "VehRol",
          "VehVert",
          "BrkTot",
          "Veh_V_",
          "VehLong",
          "BrakeSysFeatures",
          "Yaw_Data",
          "VehStop",
          "VehVActl",
          "VehRolComp",
          "VehVertComp",
        ],
        "description": "Vehicle dynamics, stability, and motion control",
      },
      "driver_assistance": {
        "patterns": [
          "Acc",
          "Cmbb",
          "Lscmbb",
          "StabCtl",
          "TracCtl",
          "DesiredTorq",
          "CcStat",
          "CcMde",
          "AccEnbl",
          "AccButtn",
          "AccStopMde",
          "AccDeny",
          "AccEngStat",
          "DrvSlipCtl",
        ],
        "description": "Driver assistance systems including ACC, collision mitigation, and traction control",
      },
      "braking": {
        "patterns": ["Brk", "BrkFluid", "BrkTot", "BPed", "PrkBrk", "Hsa", "AbsActv", "BrkSwtch"],
        "description": "Braking system signals including ABS, parking brake, and brake pedal",
      },
      "steering": {
        "patterns": ["Ste", "EPAS", "SteeringColumn", "DrvSte", "SteeringPinion", "StePw", "StePin", "SteMdule"],
        "description": "Steering system and power steering signals",
      },
      "body": {
        "patterns": ["BodyInfo", "DrStat", "Door", "Body", "PrkBrk", "Hood", "Tgate", "BodySrvc", "RemoteStart", "PlgActv"],
        "description": "Body control modules and door/hood/trunk status",
      },
      "restraints": {
        "patterns": ["RCMStatus", "Buckle", "Rstm", "CrashEvnt", "RILReq", "PassRstrn", "FirstRow", "SecondRow"],
        "description": "Restraint control and occupant safety systems",
      },
      "cluster": {
        "patterns": ["Cluster_Info", "Cluster_HEV", "Dsply", "Dimming", "Backlit", "DimmingLvl", "Day_Night", "Litval"],
        "description": "Instrument cluster and display signals",
      },
      "lane_control": {"patterns": ["Lane_Assist", "LaAct", "LatCtl", "LaSwtch", "TjaButtn"], "description": "Lane assistance and lateral control signals"},
      "lighting": {
        "patterns": ["Lght", "Lamp", "HeadLght", "FogLght", "TurnLght", "ParkLamp", "Backlit"],
        "description": "Vehicle lighting control and status",
      },
      "security": {
        "patterns": ["Key", "Valet", "Lock", "SecureAccess", "IgnKey", "KeyType", "LifeCyc"],
        "description": "Vehicle security, keys, and access control",
      },
      "trailer": {"patterns": ["Trlr", "TrlrHitch", "TrlrBrk", "TrlrSway", "VehVTrlr"], "description": "Trailer control and monitoring"},
      "wipers": {"patterns": ["Wipr", "WiprFront"], "description": "Wiper control and status"},
      "fuel": {"patterns": ["Fuel", "FuelLvl", "FuelPmp", "FuelSecnd", "FuelMaint"], "description": "Fuel system and level monitoring"},
      "hybrid": {"patterns": ["HEV", "Hybrid", "ElPw", "BattCharg", "PwPck"], "description": "Hybrid system specific signals"},
      "climate": {"patterns": ["Climate", "Hvac", "Temp", "AirC"], "description": "Climate control systems"},
    }

  def _get_base_signal_name(self, signal_name: str) -> str:
    """Enhanced base signal name extraction"""
    # Add more common prefixes/suffixes
    prefixes = ['Brk', 'Eng', 'Trn', 'Veh', 'Prk', 'Ste', 'Dr']
    suffixes = ['_D_Stat', '_B_Actl', '_D_Actl', '_B_Rq', '_D_Rq',
                '_Pc_Actl', '_No_Cs', '_No_Cnt']

    base_name = signal_name
    for prefix in prefixes:
        if base_name.startswith(prefix):
            base_name = base_name[len(prefix):]
            break

    for suffix in suffixes:
        if base_name.endswith(suffix):
            base_name = base_name[:-len(suffix)]
            break

    return base_name

  def _determine_category(self, message_name: str, signal_name: str) -> str:
    """Enhanced category determination with priority ordering"""
    # Check specific signal patterns first
    signal_patterns = {
        "braking": ["Brk", "Prk", "Abs"],
        "hybrid": ["ElPw", "HEV", "Rgen"],
        "steering": ["Ste", "EPAS"]
    }

    for category, patterns in signal_patterns.items():
        for pattern in patterns:
            if pattern in signal_name:
                return category

    # Then check message patterns
    for category, info in self.signal_categories.items():
        for pattern in info["patterns"]:
            if pattern in message_name:
                return category

    return "uncategorized"


  def _interpret_signal_value(self, signal_data: dict) -> dict:
    """Convert signal values based on type"""
    raw = signal_data.get("rawValue")
    typ = signal_data.get("type")

    try:
      if typ == "ENUM":
        enum_values = signal_data.get("enumValues", {})
        value = enum_values.get(str(int(float(raw))), raw)
      elif typ == "FLOAT":
        value = float(raw)
      elif typ == "INTEGER":
        value = int(float(raw))
      elif typ == "BOOLEAN":
        value = bool(int(float(raw)))
      else:
        value = raw
    except (ValueError, TypeError):
      value = raw

    return {"type": typ, "value_out": value}


  def _process_timestamp(self, timestamp: int) -> str:
    """Convert CAN timestamp to human readable format"""
    if timestamp:
      return datetime.fromtimestamp(timestamp / 1e9).isoformat()
    return None


  def _get_signal_description(self, signal_name: str) -> str:
    return self.signal_descriptions.get(signal_name, "")


  def categorize_signals(self, can_data: dict[str, Any]) -> dict[str, Any]:
    """Categorize and format CAN signals by message name within categories"""
    categorized = {
        "categories": {},
        "timestamp": int(time() * 1000)
    }

    for category in self.signal_categories:
        categorized["categories"][category] = {}
    categorized["categories"]["uncategorized"] = {}

    try:
        metadata = can_data.get("metadata", {})
        for bus_name in ['main_bus', 'camera_bus']:
            bus = metadata.get(bus_name, {})
            bus_short = "main" if bus_name == "main_bus" else "camera"

            for msg_name, msg_data in bus.items():
                signals = msg_data.get("signals", {})

                for sig_name, sig_data in signals.items():
                    category = self._determine_category(msg_name, sig_name)
                    interpreted = self._interpret_signal_value(sig_data)

                    # Initialize message group if it doesn't exist
                    if msg_name not in categorized["categories"][category]:
                        categorized["categories"][category][msg_name] = {
                            "bus": bus_short,
                            "signals": {}
                        }

                    # Add signal to its message group
                    categorized["categories"][category][msg_name]["signals"][sig_name] = {
                        "type": interpreted["type"],
                        "value": sig_data["rawValue"],
                        "value_out": interpreted["value_out"],
                        "desc": self._get_signal_description(sig_name)
                    }

    except Exception as e:
        print(f"Error categorizing CAN data: {e}")

    return categorized


  def process_can_data(self, json_data: str) -> dict[str, Any]:
    """Process CAN data from JSON string"""
    try:
      can_data = json.loads(json_data)
      return self.categorize_signals(can_data)
    except json.JSONDecodeError as e:
      print(f"Error decoding JSON data: {e}")
      return {}


  def format_output(self, categorized_data: dict[str, Any], format_type: str = "full") -> str:
    """Format the categorized data for output"""
    output = []

    if format_type == "summary":
      # Generate summary with counts per category
      output.append("CAN Data Summary:")
      for category, signals in categorized_data["categories"].items():
        if signals:
          output.append(f"{category}: {len(signals)} signals")

    else:  # full format
      output.append("CAN Data Analysis:")
      for category, signals in categorized_data["categories"].items():
        if signals:
          output.append(f"\n{category.upper()}:")
          for signal_name, signal_info in signals.items():
            value = signal_info["value"]
            human_readable = signal_info["metadata"]["human_readable"]
            timestamp = signal_info["metadata"]["timestamp"]

            signal_str = f"  {signal_name}: {value}"
            if human_readable:
              signal_str += f" ({human_readable})"
            if timestamp:
              signal_str += f" @ {timestamp}"

            output.append(signal_str)

    return "\n".join(output)
