import pyray as rl
from collections.abc import Callable

from openpilot.common.time_helpers import system_time_valid
from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigToggle, BigParamControl
from openpilot.selfdrive.ui.mici.widgets.floatbutton import BigParamFloatControl
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import NavWidget
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.widgets.ssh_key import SshKeyAction
from openpilot.common.params import Params

class BluePilotLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)
    self._params = Params()
    self.lane_change_factor_high = float(self._params.get("lane_change_factor_high", return_default=True))

    # ******** Main Scroller ********
    self.show_hands_free_ui = BigParamControl("show hands-free ui", "send_hands_free_cluster_msg")
    self.enable_human_turn_detection = BigParamControl("enable human turn detection", "enable_human_turn_detection")
    self.lane_change_factor_high = BigParamFloatControl("lane change factor high", "lane_change_factor_high", min=0.5, max=1.0)
    self.pc_blend_ratio_high_C = BigParamFloatControl("pc blend ratio high C", "pc_blend_ratio_high_C_UI", min=0.0, max=1.0)
    self.pc_blend_ratio_low_C = BigParamFloatControl("pc blend ratio low C", "pc_blend_ratio_low_C_UI", min=0.0, max=1.0)
    self.enable_lane_positioning = BigParamControl("enable lane positioning", "enable_lane_positioning")
    self.custom_path_offset = BigParamFloatControl("custom path offset", "custom_path_offset", min=-0.5, max=0.5)
    self.enable_adv_lane_pos = BigParamControl("enable adv. lane positioning", "enable_adv_lane_pos")
    self.enable_lane_full_mode = BigParamControl("enable lanefull mode", "enable_lane_full_mode")
    self.custom_profile = BigParamControl("enable custom profile", "custom_profile")
    self.LC_PID_gain = BigParamFloatControl("LC PID gain UI", "LC_PID_gain_UI", min=0.0, max=5.0)

    self._scroller = Scroller([
      self.show_hands_free_ui,
      self.enable_human_turn_detection,
      self.lane_change_factor_high,
      self.pc_blend_ratio_high_C,
      self.pc_blend_ratio_low_C,
      self.enable_lane_positioning,
      self.custom_path_offset,
      self.enable_adv_lane_pos,
      self.enable_lane_full_mode,
      self.custom_profile,
      self.LC_PID_gain,
    ], snap_items=False)

    # Toggle lists
    self._refresh_toggles = (
      ("send_hands_free_cluster_msg", self.show_hands_free_ui),
      ("enable_human_turn_detection", self.enable_human_turn_detection),
      ("enable_lane_positioning", self.enable_lane_positioning),
      ("enable_adv_lane_pos", self.enable_adv_lane_pos),
      ("enable_lane_full_mode", self.enable_lane_full_mode),
      ("custom_profile", self.custom_profile),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _render(self, rect: rl.Rectangle):
    self._scroller.render(rect)

  def _update_toggles(self):
    ui_state.update_params()

    # Refresh toggles from params to mirror external changes
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))