from enum import IntEnum

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.int_enum_param import get_int_enum_param


PARAM_KEY = "BPModelPathColor"


class ModelPathColor(IntEnum):
  GREEN = 0
  FORD = 1


# Cool blue/cyan progression modeled after Ford's illuminated instrument-cluster road path.
FORD_PATH_EDGE_RGBA = (72, 190, 241, 255)
FORD_PATH_COLOR_RGBA = (
  (25, 135, 220, 170),
  (73, 196, 246, 145),
  (139, 226, 255, 0),
)
FORD_PATH_STOPS = (0.0, 0.52, 1.0)


def get_model_path_color(params: Params) -> ModelPathColor:
  return get_int_enum_param(params, PARAM_KEY, ModelPathColor, ModelPathColor.GREEN)
