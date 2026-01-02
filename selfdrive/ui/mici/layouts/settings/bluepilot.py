import pyray as rl
from collections.abc import Callable

from openpilot.common.time_helpers import system_time_valid
from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigToggle, BigParamControl
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialog, BigInputDialog
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
    self.enable_lane_positioning = BigParamControl("enable lane positioning", "enable_lane_positioning")
    self.enable_adv_lane_pos = BigParamControl("enable adv. lane positioning", "enable_adv_lane_pos")
    self.enable_lane_full_mode = BigParamControl("enable lanefull mode", "enable_lane_full_mode")
    self.custom_profile = BigParamControl("enable custom profile", "custom_profile")

    def lane_change_factor_high_clicked():
      dlg = BigInputDialog("enter lane change factor high...", str(self.lane_change_factor_high),
                           confirm_callback=lane_change_factor_high_callback, show_special_keys=True)
      gui_app.set_modal_overlay(dlg)

    def lane_change_factor_high_callback(password: str):
      if password:
        try:
          self.lane_change_factor_high = float(password)
          self._params.put_nonblocking("lane_change_factor_high", self.lane_change_factor_high)
          update_lane_change_factor_high_btn()
        except ValueError:
          pass

    def update_lane_change_factor_high_btn():
      self.lane_change_factor_high_btn.set_text(f"lane change factor high [{round(self.lane_change_factor_high,4)}]")

    self.lane_change_factor_high_btn = BigButton("", "")
    update_lane_change_factor_high_btn()
    self.lane_change_factor_high_btn.set_click_callback(lane_change_factor_high_clicked)

    self._scroller = Scroller([
      self.show_hands_free_ui,
      self.enable_human_turn_detection,
      self.lane_change_factor_high_btn,
      self.enable_lane_positioning,
      self.enable_adv_lane_pos,
      self.enable_lane_full_mode,
      self.custom_profile,

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