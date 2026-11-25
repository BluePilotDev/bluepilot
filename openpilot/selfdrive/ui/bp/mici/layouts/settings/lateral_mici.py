"""BluePilot MICI: Lateral panel — BluePilot or stock steering, lane change pause, mode display."""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP, BigParamControlBP
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamIntControl
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.bluepilot.selfdrive.car.ford_lateral_support import SUPPORT_PARAM, support_row
from openpilot.bluepilot.system.ford_reads.runner import blocker, last_read, last_read_text, launch


class LateralLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

    # How this Ford is steered; a tap shows why, the steering module's part numbers and how the car was matched
    self._support = support_row(None)
    self.support = BigButtonBP("steering support", self._support[0])
    self.support.set_click_callback(lambda: gui_app.push_widget(BigDialog(self._support[0], self._support[1])))
    # Read the Ford's modules (ford_reads.py, read-only); stops openpilot for the read and reboots after it
    self.ford_reads = BigButtonBP("read vehicle modules", "", "icons_mici/settings/device/reboot.png", icon_size=80)
    self.ford_reads.set_click_callback(self._start_ford_reads)
    self.disable_BP_lat = BigParamControlBP("Disable BP Lateral Control", "disable_BP_lat_UI")
    self.disable_lane_change_under_speed = BigParamControlBP(
      "Disable Auto Lane Change Under Speed", "BlinkerPauseLaneChange",
      toggle_callback=lambda state: self.blinker_min_speed.set_enabled(state),
    )
    self.blinker_min_speed = BigParamIntControl(
      "Minimum Speed to Pause Lane Change", "BlinkerMinLateralControlSpeed", min=5, max=50, step=5,
    )
    self.show_lateral_control = BigParamControlBP("Show Lateral Control Mode", "BpShowLateralControl")

    self._scroller.add_widgets([
      self.support,
      self.ford_reads,
      self.disable_lane_change_under_speed,
      self.blinker_min_speed,
      self.show_lateral_control,
      self.disable_BP_lat,
    ])

    self._refresh_toggles = (
      ("disable_BP_lat_UI", self.disable_BP_lat),
      ("BlinkerPauseLaneChange", self.disable_lane_change_under_speed),
      ("BpShowLateralControl", self.show_lateral_control),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  def show_event(self):
    super().show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    self.blinker_min_speed.set_enabled(ui_state.params.get_bool("BlinkerPauseLaneChange"))
    self._support = support_row(ui_state.params.get(SUPPORT_PARAM))
    self.support.set_value(self._support[0])
    last = last_read()
    self.ford_reads.set_value("" if last is None else ("saved" if last.get("ok") else "failed"))

  def _start_ford_reads(self):
    reason = blocker(ui_state.started, ui_state.engaged, ui_state.sm["carState"])
    if reason is not None:
      gui_app.push_widget(BigDialog("read vehicle modules", reason + "\n\n" + last_read_text()))
      return

    def do_read():
      if blocker(ui_state.started, ui_state.engaged, ui_state.sm["carState"]) is None:
        cloudlog.info("BluePilot: starting the Ford module read (ford_reads), the device reboots after it")
        launch()

    icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 64)
    gui_app.push_widget(BigConfirmationDialog("read modules & reboot?", icon, confirm_callback=do_read))
