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

class BluePilotLayoutMici(NavWidget):
  def __init__(self, back_callback: Callable):
    super().__init__()
    self.set_back_callback(back_callback)

    # ******** Main Scroller ********
    self.show_hands_free_ui = BigParamControl("show hands-free ui", "send_hands_free_cluster_msg")

    self._scroller = Scroller([
      self.show_hands_free_ui,
    ], snap_items=False)

    # Toggle lists
    self._refresh_toggles = (
      ("send_hands_free_cluster_msg", self.show_hands_free_ui),
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