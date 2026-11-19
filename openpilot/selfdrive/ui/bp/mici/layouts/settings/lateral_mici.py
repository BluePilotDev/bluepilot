"""BluePilot MICI: Lateral panel — BluePilot or stock steering, lane change pause, mode display."""

from collections.abc import Callable

from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigParamControlBP
from openpilot.selfdrive.ui.bp.mici.widgets.floatbutton import BigParamIntControl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets.scroller import NavScroller


class LateralLayoutMici(NavScroller):
  def __init__(self, back_callback: Callable[[], None] | None = None):
    super().__init__()
    if back_callback is not None:
      self.set_back_callback(back_callback)

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
