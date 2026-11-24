from enum import IntEnum

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.int_enum_param import (
  ensure_int_enum_param_initialized,
  get_int_enum_param,
)


PARAM_KEY = "BPDMStylingChoice"


class DMIconStyle(IntEnum):
  COMMA_4 = 0
  COMMA_3X = 1


def get_dm_icon_style(params: Params, device_default: DMIconStyle) -> DMIconStyle:
  """Return the configured driver-monitoring icon style without modifying Params."""
  return get_int_enum_param(params, PARAM_KEY, DMIconStyle, device_default)


def ensure_dm_icon_style_initialized(params: Params, device_default: DMIconStyle) -> DMIconStyle:
  """Persist the device default when the DM-style param is missing or invalid."""
  return ensure_int_enum_param_initialized(params, PARAM_KEY, DMIconStyle, device_default)
