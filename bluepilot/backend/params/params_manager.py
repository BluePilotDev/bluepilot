#!/usr/bin/env python3
"""
BluePilot Backend Params Manager
Safe operations for reading and writing openpilot parameters
"""

import base64
import json
import logging
import os
import re
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Any, Union, Tuple

logger = logging.getLogger(__name__)

# Import Params with fallback for direct file reading
PARAMS_DIR = "/data/params/d"
USE_DIRECT_FILE_READING = False
_PARAM_TYPE_CACHE: Optional[Dict[str, str]] = None
_PARAM_ATTRIBUTES_CACHE: Optional[Dict[str, List[str]]] = None

CAR_PARAM_KEYS = {
    "CarParams", "CarParamsCache", "CarParamsPersistent", "CarParamsPrevRoute",
    "CarParamsSP", "CarParamsSPCache", "CarParamsSPPersistent"
}

try:
    from openpilot.common.params import Params, UnknownKeyName
    logger.info("Successfully imported openpilot.common.params.Params")
except ImportError as e:
    logger.warning(f"Failed to import openpilot Params: {e}. Using direct file reading fallback.")
    USE_DIRECT_FILE_READING = True

    # Fallback Params class that reads directly from filesystem
    class Params:
        def __init__(self, params_dir=PARAMS_DIR):
            self.params_dir = params_dir

        def _read_file(self, key):
            """Read param file directly"""
            try:
                param_path = os.path.join(self.params_dir, key)
                if not os.path.exists(param_path):
                    return None
                with open(param_path, 'rb') as f:
                    return f.read()
            except Exception as e:
                logger.debug(f"Error reading param file {key}: {e}")
                return None

        def get_bool(self, key):
            """Get boolean param"""
            value = self._read_file(key)
            if value is None:
                return False
            # openpilot stores bools as "1" or "0"
            return value == b"1"

        def get_int(self, key):
            value = self._read_file(key)
            if value is None:
                return 0
            try:
                return int(value.decode('utf-8').strip())
            except Exception:
                return 0

        def get_float(self, key):
            value = self._read_file(key)
            if value is None:
                return 0.0
            try:
                return float(value.decode('utf-8').strip())
            except Exception:
                return 0.0

        def get(self, key):
            """Get param as bytes"""
            value = self._read_file(key)
            if value is None:
                return b""
            return value

        def put(self, key, value):
            """Write param to file"""
            try:
                param_path = os.path.join(self.params_dir, key)
                if isinstance(value, str):
                    value = value.encode('utf-8')
                with open(param_path, 'wb') as f:
                    f.write(value)
                return 0
            except Exception as e:
                logger.error(f"Error writing param {key}: {e}")
                return -1

        def put_bool(self, key, value):
            """Write boolean param"""
            return self.put(key, "1" if value else "0")

        def all_keys(self, flag=None):
            try:
                return os.listdir(self.params_dir)
            except Exception:
                return []

        def get_type(self, key):
            return _load_param_type_cache().get(key)

    class UnknownKeyName(Exception):
        pass


# Critical params that should not be modified through the web interface
READONLY_PARAMS = {
    "DongleId",
    "GitCommit",
    "Version",
    "GitBranch",
    "GitRemote",
    "Updated",
    "Passive",
}

# Params that require extra warnings before modification
CRITICAL_PARAMS = {
    "DisableLogging",
    "DisableUpdates",
    "OpenpilotEnabledToggle",
    "LongitudinalPersonality",
}

# Cache for BluePilot panel params (loaded from JSON files)
_BLUEPILOT_PARAMS_CACHE: Optional[set] = None


def _load_bluepilot_params() -> set:
    """Load all param names from BluePilot panel JSON files.

    Params defined in panel JSONs are considered 'BluePilot' params.
    All other params are considered 'System' params.
    """
    global _BLUEPILOT_PARAMS_CACHE
    if _BLUEPILOT_PARAMS_CACHE is not None:
        return _BLUEPILOT_PARAMS_CACHE

    bp_params: set = set()

    # Find panel JSON files
    repo_root = Path(__file__).resolve().parents[3]  # Go up to openpilot root
    panel_dirs = [
        repo_root / "selfdrive" / "ui" / "bluepilot" / "menus",
    ]

    for panel_dir in panel_dirs:
        if not panel_dir.exists():
            continue

        for json_file in panel_dir.glob("*.json"):
            try:
                with open(json_file, 'r') as f:
                    panel_data = json.load(f)

                # Extract params from all controls in all groups
                groups = panel_data.get("groups", [])
                for group in groups:
                    controls = group.get("controls", [])
                    for control in controls:
                        # Get param from control
                        param = control.get("param")
                        if param:
                            bp_params.add(param)

                        # Also check params array (some controls have multiple params)
                        params_list = control.get("params", [])
                        bp_params.update(params_list)

                        # Check options for selection controls
                        options = control.get("options", [])
                        for opt in options:
                            if isinstance(opt, dict):
                                opt_param = opt.get("param")
                                if opt_param:
                                    bp_params.add(opt_param)

                # Also include persistent params and other param arrays
                for key in ["persistentParams", "clearOnManagerStartParams",
                           "clearOnOnroadTransitionParams", "clearOnOffroadTransitionParams"]:
                    bp_params.update(panel_data.get(key, []))

            except (json.JSONDecodeError, IOError) as e:
                logger.debug(f"Error reading panel file {json_file}: {e}")
                continue

    logger.info(f"Loaded {len(bp_params)} BluePilot params from panel JSON files")
    _BLUEPILOT_PARAMS_CACHE = bp_params
    return bp_params


def _load_param_type_cache() -> Dict[str, str]:
    """Parse common/params_keys.h and bluepilot/params/params.json to know declared types."""
    global _PARAM_TYPE_CACHE
    if _PARAM_TYPE_CACHE is not None:
        return _PARAM_TYPE_CACHE

    cache: Dict[str, str] = {}

    # First, load from common/params_keys.h (openpilot core params)
    try:
        repo_root = Path(__file__).resolve().parents[3]  # Go up to openpilot root
        header_path = repo_root / "common" / "params_keys.h"
        if header_path.exists():
            contents = header_path.read_text()
            pattern = re.compile(r'\{"([^"]+)",\s*\{[^,]+,\s*(STRING|BOOL|INT|FLOAT|TIME|JSON|BYTES)')
            for match in pattern.finditer(contents):
                key, type_name = match.groups()
                cache[key] = type_name.lower()
    except Exception as e:
        logger.debug(f"Failed to parse params_keys.h for param types: {e}")

    # Second, load from bluepilot/params/params.json (BluePilot-specific params)
    try:
        repo_root = Path(__file__).resolve().parents[3]  # Go up to openpilot root
        params_json_path = repo_root / "bluepilot" / "params" / "params.json"
        if params_json_path.exists():
            with open(params_json_path, 'r') as f:
                bp_params_data = json.load(f)
            # params.json has structure: {"params": [...]}
            params_list = bp_params_data.get("params", []) if isinstance(bp_params_data, dict) else bp_params_data
            for param in params_list:
                if isinstance(param, dict) and 'name' in param and 'type' in param:
                    cache[param['name']] = param['type'].lower()
            logger.info(f"Loaded {len([p for p in params_list if isinstance(p, dict) and 'name' in p])} param types from bluepilot params.json")
    except Exception as e:
        logger.debug(f"Failed to parse bluepilot params.json for param types: {e}")

    _PARAM_TYPE_CACHE = cache
    return cache


def _load_param_attributes_cache() -> Dict[str, List[str]]:
    """Parse common/params_keys.h to extract ParamKeyAttributes flags.

    Returns a dict mapping param key to list of attribute flags like:
    ['PERSISTENT', 'BACKUP'], ['CLEAR_ON_MANAGER_START'], etc.
    """
    global _PARAM_ATTRIBUTES_CACHE
    if _PARAM_ATTRIBUTES_CACHE is not None:
        return _PARAM_ATTRIBUTES_CACHE

    cache: Dict[str, List[str]] = {}
    try:
        repo_root = Path(__file__).resolve().parents[3]  # Go up to openpilot root
        header_path = repo_root / "common" / "params_keys.h"
        if header_path.exists():
            contents = header_path.read_text()
            # Match pattern: {"ParamName", {FLAGS, TYPE, ...}}
            # FLAGS can be: PERSISTENT | BACKUP | CLEAR_ON_MANAGER_START | DONT_LOG | DEVELOPMENT_ONLY | etc.
            pattern = re.compile(r'\{"([^"]+)",\s*\{([^}]+)\}\}')
            for match in pattern.finditer(contents):
                key, attributes_str = match.groups()
                # Extract the flags portion (everything before the type)
                # Format: FLAGS, TYPE or just TYPE
                parts = [p.strip() for p in attributes_str.split(',')]
                if len(parts) >= 2:
                    flags_str = parts[0]  # First part is flags
                    # Split flags by | and clean up
                    flags = [f.strip() for f in flags_str.split('|')]
                    # Filter out known attribute flags
                    known_flags = ['PERSISTENT', 'BACKUP', 'CLEAR_ON_MANAGER_START',
                                   'CLEAR_ON_ONROAD_TRANSITION', 'CLEAR_ON_OFFROAD_TRANSITION',
                                   'CLEAR_ON_IGNITION_ON', 'DONT_LOG', 'DEVELOPMENT_ONLY']
                    valid_flags = [f for f in flags if f in known_flags]
                    if valid_flags:
                        cache[key] = valid_flags
    except Exception as e:
        logger.debug(f"Failed to parse params_keys.h for param attributes: {e}")

    _PARAM_ATTRIBUTES_CACHE = cache
    return cache


def write_param_direct(key: str, value: Any) -> Tuple[bool, Optional[str]]:
    """Directly write a parameter file when Params API rejects the key."""
    try:
        os.makedirs(PARAMS_DIR, exist_ok=True)

        if isinstance(value, bool):
            data = b"1" if value else b"0"
        else:
            data = str(value).encode('utf-8')

        param_path = os.path.join(PARAMS_DIR, key)
        with open(param_path, 'wb') as f:
            f.write(data)

        logger.info(f"Directly wrote param {key} to {param_path}")
        return True, None

    except Exception as e:
        logger.error(f"Direct param write failed for {key}: {e}")
        return False, str(e)


def categorize_param(key: str) -> str:
    """Determine which category a param belongs to.

    Params defined in BluePilot panel JSON files are 'BluePilot'.
    All other params are 'System'.

    Args:
        key: Parameter key

    Returns:
        Category name ('BluePilot' or 'System')
    """
    bp_params = _load_bluepilot_params()
    if key in bp_params:
        return "BluePilot"
    return "System"


def get_all_params(params: Optional[Params] = None) -> Dict[str, Any]:
    """Get all readable parameters

    Args:
        params: Params instance (creates new one if None)

    Returns:
        Dictionary of all params with metadata
    """
    if params is None:
        params = Params()

    result = {}

    # Try to get all params by listing the params directory
    params_dir = "/data/params/d" if os.path.exists("/data/params/d") else None

    if params_dir and os.path.exists(params_dir):
        # List all param files
        try:
            param_keys = os.listdir(params_dir)
        except Exception as e:
            logger.error(f"Error listing params directory: {e}")
            param_keys = []
    else:
        # Fallback to known params
        param_keys = []
        for category_info in PARAM_CATEGORIES.values():
            param_keys.extend(category_info["params"])

    for key in param_keys:
        result[key] = _build_param_entry(key, params, params_dir)

    return result


def get_param_value(key: str, params: Optional[Params] = None) -> Union[str, bool, int, float, None]:
    """Get a single parameter value with best-effort decoding."""
    entry = _build_param_entry(key, params)
    return entry.get("value")


def determine_param_type(value: Any) -> str:
    """Determine the type of a parameter value."""
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int) and not isinstance(value, bool):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, (bytes, bytearray)):
        return "bytes"
    if isinstance(value, (dict, list)):
        return "json"
    if isinstance(value, datetime):
        return "time"
    return "string"


def _get_param_type_name(params: Params, key: str) -> Optional[str]:
    getter = getattr(params, 'get_type', None)
    if callable(getter):
        try:
            param_type = getter(key)
            if hasattr(param_type, 'name'):
                return param_type.name.lower()
            if isinstance(param_type, str):
                return param_type.lower()
            if isinstance(param_type, int):
                return str(param_type).lower()
        except Exception:
            pass
    return _load_param_type_cache().get(key)


def _normalize_param_value(value: Any, target_type: Optional[str] = None) -> Any:
    """Attempt to coerce common string representations into bool or numeric values."""
    if isinstance(value, str):
        stripped = value.strip()
        lowered = stripped.lower()

        if lowered in ("true", "false"):
            return lowered == "true"

        if target_type == 'bool' and stripped in ("1", "0"):
            return stripped == "1"

        # Try integer conversion
        try:
            if stripped.startswith(('0x', '-0x')):
                return int(stripped, 16)
            if stripped.isdigit() or (stripped.startswith('-') and stripped[1:].isdigit()):
                return int(stripped)
            # Attempt float for strings containing decimal/exponent markers
            if any(ch in stripped for ch in ['.', 'e', 'E']):
                return float(stripped)
        except ValueError:
            pass

    return value


def set_param_value(key: str, value: Any, params: Optional[Params] = None) -> Dict[str, Any]:
    """Set a parameter value with validation

    Args:
        key: Parameter key
        value: New value
        params: Params instance (creates new one if None)

    Returns:
        Result dictionary with success status and message
    """
    if params is None:
        params = Params()

    # Check if param is readonly
    if key in READONLY_PARAMS:
        return {
            "success": False,
            "error": f"Parameter '{key}' is read-only and cannot be modified"
        }

    def success_response():
        return {
            "success": True,
            "message": f"Successfully updated parameter '{key}'",
            "key": key,
            "value": value
        }

    target_type = _get_param_type_name(params, key)
    value = _normalize_param_value(value, target_type)

    try:
        if isinstance(value, bool) or (isinstance(value, str) and value.lower() in ["true", "false"]):
            bool_value = value if isinstance(value, bool) else value.lower() == "true"
            params.put_bool(key, bool_value)
        elif target_type == 'int' and not isinstance(value, bool):
            # For INT params, pass as Python int (Params API will convert to string)
            int_value = int(value) if not isinstance(value, int) else value
            params.put(key, int_value)
        elif target_type == 'float':
            # For FLOAT params, pass as Python float (Params API will convert to string)
            float_value = float(value) if not isinstance(value, float) else value
            params.put(key, float_value)
        else:
            params.put(key, str(value))

        return success_response()

    except UnknownKeyName:
        logger.warning(f"Param '{key}' not recognized by Params API. Falling back to direct write.")
        success, error_message = write_param_direct(key, value)
        if success:
            return success_response()
        return {
            "success": False,
            "error": f"Failed to update parameter: {error_message or 'Unknown error'}"
        }

    except Exception as e:
        logger.error(f"Error setting param {key}: {e}")
        return {
            "success": False,
            "error": f"Failed to update parameter: {str(e)}"
        }


def _build_param_entry(key: str, params: Optional[Params], params_dir: Optional[str] = None) -> Dict[str, Any]:
    if params is None:
        params = Params()

    if params_dir is None:
        params_dir = PARAMS_DIR if os.path.exists(PARAMS_DIR) else None

    # Get ParamKeyAttributes from params_keys.h
    attributes_cache = _load_param_attributes_cache()
    param_attributes = attributes_cache.get(key, [])

    entry: Dict[str, Any] = {
        "key": key,
        "category": categorize_param(key),
        "readonly": key in READONLY_PARAMS,
        "critical": key in CRITICAL_PARAMS,
        "type": "unknown",
        "last_modified": _get_param_mtime(params_dir, key),
        "attributes": param_attributes  # Add ParamKeyAttributes flags
    }

    try:
        param_type = _get_param_type_name(params, key)
        raw_value = _read_param_value(params, key, param_type)
        value, metadata = _format_value_for_response(key, raw_value, param_type)
        entry["value"] = value
        entry["type"] = param_type or determine_param_type(raw_value)
        entry.update(metadata)
    except Exception as e:
        logger.debug(f"Error reading param {key}: {e}")
        entry.update({
            "value": None,
            "type": "unknown",
            "error": str(e)
        })

    return entry


def _read_param_value(params: Params, key: str, param_type: Optional[str]) -> Any:
    """Read a parameter using the most appropriate getter based on its declared type."""
    if param_type == "bool":
        return params.get_bool(key)
    if param_type == "int":
        value = params.get(key)
        if isinstance(value, bytes):
            value = value.decode('utf-8', errors='replace').strip()
        try:
            return int(value) if value else 0
        except (ValueError, TypeError):
            logger.debug(f"Failed to parse param {key} as int from value '{value}'")
            return 0
    if param_type == "float":
        value = params.get(key)
        if isinstance(value, bytes):
            value = value.decode('utf-8', errors='replace').strip()
        try:
            return float(value) if value else 0.0
        except (ValueError, TypeError):
            logger.debug(f"Failed to parse param {key} as float from value '{value}'")
            return 0.0
    value = params.get(key)
    if isinstance(value, (bytes, bytearray)) and param_type in {"string", "json", "time"}:
        value = value.decode('utf-8', errors='replace')
    if param_type == "string" and isinstance(value, bytes):
        return value.decode('utf-8', errors='replace')
    if param_type == "json":
        if isinstance(value, (dict, list)):
            return value
        if isinstance(value, str):
            try:
                return json.loads(value)
            except json.JSONDecodeError:
                return value
    if param_type == "time" and isinstance(value, str):
        try:
            return datetime.fromisoformat(value)
        except ValueError:
            return value
    return value


def _format_value_for_response(key: str, value: Any, param_type: Optional[str]) -> Tuple[Any, Dict[str, Any]]:
    """Prepare a JSON-serializable value plus extra metadata for UI consumption."""
    metadata: Dict[str, Any] = {}

    if value is None:
        return None, metadata

    if param_type == "bytes" or isinstance(value, (bytes, bytearray)):
        raw_bytes = value if isinstance(value, (bytes, bytearray)) else bytes(str(value), 'utf-8')
        byte_length = len(raw_bytes)
        encoded = base64.b64encode(raw_bytes).decode('ascii') if byte_length else None
        decoded = decode_bytes_param(key, raw_bytes)
        preview = decoded.get("preview") if decoded else None
        metadata.update({
            "byte_length": byte_length,
            "raw_value": encoded,
            "raw_format": "base64" if encoded else None,
            "decoded": decoded,
            "value_is_binary": True,
        })
        readable = preview or (f"{byte_length} bytes" if byte_length else "empty bytes")
        return readable, metadata

    if param_type == "json":
        if isinstance(value, (dict, list)):
            metadata["decoded"] = {
                "format": "json",
                "data": value
            }
            return json.dumps(value, ensure_ascii=False), metadata
        return value, metadata

    if param_type == "time" and isinstance(value, datetime):
        metadata["timestamp"] = int(value.timestamp())
        return value.isoformat(), metadata

    return value, metadata


def decode_bytes_param(key: str, raw_bytes: bytes) -> Optional[Dict[str, Any]]:
    """Decode known byte-encoded params into structured data for UI consumption."""
    if not raw_bytes:
        return None

    if key in CAR_PARAM_KEYS:
        try:
            from cereal import car
            with car.CarParams.from_bytes(raw_bytes) as car_params:
                cp_dict = car_params.to_dict()
                summary = {
                    "carName": car_params.carName or "",
                    "carFingerprint": car_params.carFingerprint or "",
                    "carVin": car_params.carVin or "",
                    "openpilotLongitudinalControl": bool(car_params.openpilotLongitudinalControl),
                }
                preview_parts = [part for part in [
                    summary["carName"],
                    summary["carFingerprint"],
                    summary["carVin"]
                ] if part]
                preview = " • ".join(preview_parts) if preview_parts else "CarParams blob"
                return {
                    "format": "carParams",
                    "preview": preview,
                    "summary": summary,
                    "data": cp_dict
                }
        except Exception as e:
            logger.debug(f"Failed to decode CarParams for {key}: {e}")

    try:
        text_value = raw_bytes.decode('utf-8')
        stripped = text_value.strip('\x00').strip()
    except UnicodeDecodeError:
        stripped = None

    if stripped:
        if stripped.startswith('{') or stripped.startswith('['):
            try:
                parsed = json.loads(stripped)
                preview = "JSON object" if isinstance(parsed, dict) else "JSON array"
                return {
                    "format": "json",
                    "preview": preview,
                    "data": parsed
                }
            except json.JSONDecodeError:
                pass
        preview_text = stripped if len(stripped) <= 200 else stripped[:200] + "…"
        return {
            "format": "text",
            "preview": preview_text,
            "data": stripped
        }

    hex_preview = " ".join(f"{byte:02X}" for byte in raw_bytes[:64])
    if hex_preview:
        return {
            "format": "hex",
            "preview": hex_preview
        }

    return None


def _get_param_mtime(params_dir: Optional[str], key: str) -> Optional[float]:
    if not params_dir:
        return None
    try:
        param_path = os.path.join(params_dir, key)
        if os.path.exists(param_path):
            return os.path.getmtime(param_path)
    except Exception as e:
        logger.debug(f"Error getting mtime for param {key}: {e}")
    return None


def get_params_by_category(params: Optional[Params] = None) -> Dict[str, Any]:
    """Get all parameters organized by category

    Args:
        params: Params instance (creates new one if None)

    Returns:
        Dictionary with params organized by category
    """
    all_params = get_all_params(params)

    result = {}
    for category, info in PARAM_CATEGORIES.items():
        result[category] = {
            "name": info["name"],
            "description": info["description"],
            "params": []
        }

    # Organize params into categories
    for param_data in all_params.values():
        category = param_data["category"]
        result[category]["params"].append(param_data)

    # Sort params within each category
    for category_data in result.values():
        category_data["params"].sort(key=lambda p: p["key"])

    return result


def search_params(query: str, params: Optional[Params] = None) -> List[Dict[str, Any]]:
    """Search parameters by key or value

    Args:
        query: Search query
        params: Params instance (creates new one if None)

    Returns:
        List of matching params
    """
    all_params = get_all_params(params)
    query_lower = query.lower()

    results = []
    for param_data in all_params.values():
        # Search in key
        if query_lower in param_data["key"].lower():
            results.append(param_data)
            continue

        # Search in value
        value_str = str(param_data.get("value", "")).lower()
        if query_lower in value_str:
            results.append(param_data)
            continue

        decoded = param_data.get("decoded")
        if decoded:
            try:
                decoded_str = json.dumps(decoded).lower()
                if query_lower in decoded_str:
                    results.append(param_data)
                    continue
            except (TypeError, ValueError):
                pass

    return results
