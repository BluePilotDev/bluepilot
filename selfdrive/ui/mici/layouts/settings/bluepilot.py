import pyray as rl
from collections.abc import Callable

from openpilot.common.time_helpers import system_time_valid
from openpilot.system.ui.widgets.scroller import Scroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigToggle, BigParamControl
from openpilot.system.ui.widgets.label import gui_label, MiciLabel, UnifiedLabel
from openpilot.selfdrive.ui.mici.widgets.floatbutton import BigParamFloatControl
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialogBase
from openpilot.system.ui.lib.application import gui_app, MousePos
from openpilot.system.ui.widgets import NavWidget, DialogResult
from openpilot.selfdrive.ui.layouts.settings.common import restart_needed_callback
from openpilot.selfdrive.ui.ui_state import ui_state
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
    self.enable_lane_positioning = BigParamControl("enable lane positioning", "enable_lane_positioning", tint=rl.GREEN)
    self.custom_path_offset = BigParamFloatControl("in-lane offset", "custom_path_offset", min=-0.5, max=0.5, tint=rl.GREEN)
    self.enable_lane_full_mode = BigParamControl("enable lanefull mode", "enable_lane_full_mode", tint=rl.GREEN)
    self.custom_profile = BigParamControl("use custom tuning profile", "custom_profile", tint=rl.BLUE)
    self.pc_blend_ratio_high_C = BigParamFloatControl("predicted curvature blend ratio high", "pc_blend_ratio_high_C_UI", min=0.0, max=1.0, tint=rl.BLUE)
    self.pc_blend_ratio_low_C = BigParamFloatControl("predicted curvature blend ratio low", "pc_blend_ratio_low_C_UI", min=0.0, max=1.0, tint=rl.BLUE)
    self.LC_PID_gain = BigParamFloatControl("low curvature PID gain", "LC_PID_gain_UI", min=0.0, max=5.0, tint=rl.BLUE)
    self.disable_BP_lat = BigParamControl("disable BP lateral control", "disable_BP_lat_UI")
    self.vbatt_pause_charging = BigParamFloatControl("12V battery limit", "vbatt_pause_charging", min=11.0, max=14.0, step=0.1)

    #self.charging_btn = BigButton("charging", "", "icons_mici/settings/charge_icon.png")
    #self.charging_btn.set_click_callback(lambda: self._show_charging_view())

    self._scroller = Scroller([
      self.show_hands_free_ui,
      self.enable_human_turn_detection,
      self.lane_change_factor_high,
      self.enable_lane_positioning,
      self.custom_path_offset,
      self.enable_lane_full_mode,
      self.custom_profile,
      self.pc_blend_ratio_high_C,
      self.pc_blend_ratio_low_C,
      self.LC_PID_gain,
      self.vbatt_pause_charging,
      self.disable_BP_lat,
    ], snap_items=False)

    # Toggle lists
    self._refresh_toggles = (
      ("send_hands_free_cluster_msg", self.show_hands_free_ui),
      ("enable_human_turn_detection", self.enable_human_turn_detection),
      ("enable_lane_positioning", self.enable_lane_positioning),
      ("enable_lane_full_mode", self.enable_lane_full_mode),
      ("custom_profile", self.custom_profile),
      ("disable_BP_lat_UI", self.disable_BP_lat),
    )

    ui_state.add_offroad_transition_callback(self._update_toggles)

  # def _show_charging_view(self):
  #   dlg = BigChargingDialog()
  #   gui_app.set_modal_overlay(dlg)

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

# class BigChargingDialog(BigDialogBase):
#   def __init__(self):
#     super().__init__(None, None)

#     self._watt_label = MiciLabel("120kW", font_size=90)
#     self._watt_label.set_position(150,75)

#   def _render(self, _):
#     self._watt_label.render()
#     return self._ret

#   def _update_state(self):
#     super()._update_state()
#     if self._swiping_away:
#       self._ret = DialogResult.CANCEL
