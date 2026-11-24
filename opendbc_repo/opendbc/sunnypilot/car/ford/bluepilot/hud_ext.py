"""BluePilot (bp-full): Ford cluster display, from BluePilot 7.

Replaces upstream's UI block in opendbc/car/ford/carcontroller.py on every Ford: IPMA_Data at 1 Hz or on a
change, ACCDATA_3 at 5 Hz or on a change, with BP7's builders (bluepilot/fordcan_ext.py) and BP7's
distance-bar window. The one setting, send_hands_free_cluster_msg, comes from CarControlSP.params.
"""
from opendbc.car.ford.values import CarControllerParams, FordFlags
from opendbc.sunnypilot.car.ford.bluepilot import fordcan_ext
from opendbc.sunnypilot.car.ford.bluepilot.published import published_bool

# setting: BP7's default when the host doesn't publish it
HUD_PARAMS = {
  "send_hands_free_cluster_msg": False,
}
BARS_WINDOW = 400  # frames the distance bars stay shown after a change (BP7: <= 400, so 401 frames)


class HudExt:
  def __init__(self, CP):
    self.CP = CP
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None
    self.send_bars_last = False
    self.send_bars_ts_last = 0

  def update(self, CC, CC_SP, CS, main_on: bool, steer_alert: bool, fcw_alert: bool, packer, CAN, frame: int) -> list:
    hud_control = CC.hudControl
    can_sends = []

    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    if (frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan_ext.create_lkas_ui_msg(packer, CAN, steer_alert, hud_control, CS.lkas_status_stock_values))

    send_bars = False
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = send_bars = True
    # BP7's window, quirks included (#16 follow-ups): ACCDATA_3 every frame while it lasts, a change inside it or on
    # the frame right after it doesn't start a new one, and none for a change at frame 0
    if not self.send_bars_last and send_bars:
      self.send_bars_ts_last = frame
    if self.send_bars_ts_last > 0 and (frame - self.send_bars_ts_last) <= BARS_WINDOW:
      send_ui = send_bars = True

    if (frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      # the BlueCruise display is CAN FD only
      hands_free = bool(self.CP.flags & FordFlags.CANFD) and published_bool(CC_SP, "send_hands_free_cluster_msg", HUD_PARAMS)
      can_sends.append(fordcan_ext.create_acc_ui_msg(packer, CAN, self.CP, main_on, CC.latActive, fcw_alert,
                                                     CS.out.cruiseState.standstill, hud_control,
                                                     CS.acc_tja_status_stock_values, hands_free, send_bars))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars
    self.send_bars_last = send_bars
    return can_sends
