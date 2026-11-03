from enum import IntEnum

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib.int_enum_param import get_int_enum_param


CUSTOM_SOUNDS_ENABLED_PARAM = "BPUseCustomSounds"
CUSTOM_SOUNDS_SELECTION_PARAM = "BPCustSoundsSelection"


class CustomSoundSelection(IntEnum):
  COMMA_4 = 0
  COMMA_3X = 1
  TESLA = 2


def get_custom_sound_selection(params: Params) -> CustomSoundSelection:
  """Return a validated custom engagement-sound selection."""
  return get_int_enum_param(
    params,
    CUSTOM_SOUNDS_SELECTION_PARAM,
    CustomSoundSelection,
    CustomSoundSelection.COMMA_4,
  )
