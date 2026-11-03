from enum import IntEnum

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.int_enum_param import (
  ensure_int_enum_param_initialized,
  get_int_enum_param,
)


PARAM_KEY = "BPSteeringWheelIconStyle"


class SteeringWheelIconStyle(IntEnum):
  COMMA_4 = 0
  COMMA_3X = 1


def get_steering_wheel_icon_style(params: Params, device_default: SteeringWheelIconStyle) -> SteeringWheelIconStyle:
  """Return the configured wheel style without modifying Params."""
  return get_int_enum_param(params, PARAM_KEY, SteeringWheelIconStyle, device_default)


def ensure_steering_wheel_icon_style_initialized(
  params: Params,
  device_default: SteeringWheelIconStyle,
) -> SteeringWheelIconStyle:
  """Persist the device default when the wheel-style param is missing or invalid."""
  return ensure_int_enum_param_initialized(params, PARAM_KEY, SteeringWheelIconStyle, device_default)
