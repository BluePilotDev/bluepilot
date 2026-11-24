from enum import IntEnum

from openpilot.common.params import Params
from openpilot.common.params import UnknownKeyName


def get_int_enum_param[IntEnumT: IntEnum](
  params: Params,
  key: str,
  enum_type: type[IntEnumT],
  default: IntEnumT,
) -> IntEnumT:
  """Return a validated enum-backed integer param without modifying Params."""
  try:
    raw_value = params.get(key)
  except UnknownKeyName:
    # BluePilot: Allow UI development before common/params_pyx.so has been rebuilt.
    return default

  try:
    return enum_type(raw_value) if raw_value is not None else default
  except (TypeError, ValueError):
    return default


def ensure_int_enum_param_initialized[IntEnumT: IntEnum](
  params: Params,
  key: str,
  enum_type: type[IntEnumT],
  default: IntEnumT,
) -> IntEnumT:
  """Return a validated enum-backed param and persist the default when needed."""
  value = get_int_enum_param(params, key, enum_type, default)

  try:
    raw_value = params.get(key)
  except UnknownKeyName:
    return value

  if raw_value != int(value):
    params.put(key, int(value), block=False)

  return value
