"""Color definitions for onroad UI."""
import pyray as rl
from openpilot.selfdrive.ui.ui_state import UIStatus
from openpilot.selfdrive.ui.sunnypilot.onroad.augmented_road_view import BORDER_COLORS_SP

# BluePilot: Border colors for confidence ball / UI status
BORDER_COLORS = {
  UIStatus.DISENGAGED: rl.Color(0x12, 0x28, 0x39, 0xFF),  # Blue for disengaged state
  UIStatus.OVERRIDE: rl.Color(0x89, 0x92, 0x8D, 0xFF),  # Gray for override state
  UIStatus.ENGAGED: rl.Color(0x16, 0x7F, 0x40, 0xFF),  # Green for engaged state
  **BORDER_COLORS_SP,
}
