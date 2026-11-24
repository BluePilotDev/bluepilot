"""BluePilot (bp-full): Ford cluster display (sunnypilot/car/ford/bluepilot/hud_ext.py)."""
import unittest
from types import SimpleNamespace as NS

from opendbc.can import CANPacker, CANParser
from opendbc.car import structs
from opendbc.car.ford.values import FordFlags
from opendbc.sunnypilot.car.ford.bluepilot.hud_ext import HudExt

VisualAlert = structs.CarControl.HUDControl.VisualAlert
DBC = "ford_lincoln_base_pt"
CAN = NS(main=0, camera=2)
ACC_UI, LKAS_UI = 0x18A, 0x3D8


def stock(msg, **values):
  out = dict(CANParser(DBC, [(msg, 10)], 0).vl[msg])
  out.update(values)
  return out


def hud(**kw):
  base = dict(leftLaneDepart=False, rightLaneDepart=False, leftLaneVisible=False, rightLaneVisible=False,
              leadVisible=False, leadDistanceBars=3, visualAlert=VisualAlert.none)
  return NS(**{**base, **kw})


def decode(msg):
  address, dat, bus = msg
  name = "ACCDATA_3" if address == ACC_UI else "IPMA_Data"
  parser = CANParser(DBC, [(name, 10)], bus)
  parser.update([(0, [(address, dat, bus)])])
  return parser.vl[name]


class TestHudExt(unittest.TestCase):
  def make(self, canfd=True, op_long=False):
    CP = structs.CarParams()
    CP.flags = int(FordFlags.CANFD) if canfd else 0
    CP.openpilotLongitudinalControl = op_long
    self.hud = HudExt(CP)
    self.packer = CANPacker(DBC)

  def step(self, frame, lat=True, main=True, standstill=False, hands_free=False, hud_control=None, acc_stock=None, published=True):
    hc = hud_control or hud()
    CC = NS(latActive=lat, hudControl=hc)
    CC_SP = NS(params=[NS(key="send_hands_free_cluster_msg", value=b"1" if hands_free else b"0")] if published else [])
    CS = NS(out=NS(cruiseState=NS(available=main, standstill=standstill)),
            acc_tja_status_stock_values=acc_stock or stock("ACCDATA_3"), lkas_status_stock_values=stock("IPMA_Data"))
    steer_alert = hc.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    msgs = self.hud.update(CC, CC_SP, CS, main, steer_alert, hc.visualAlert == VisualAlert.fcw, self.packer, CAN, frame)
    return {m[0]: decode(m) for m in msgs}

  def acc_ui(self, frame=0, **kw):
    return self.step(frame, **kw)[ACC_UI]

  def test_hands_free_shows_bluecruise_on_canfd_only(self):
    self.make(canfd=True)
    self.assertEqual(self.acc_ui(hands_free=True)["Tja_D_Stat"], 7)
    self.make(canfd=True)
    self.assertEqual(self.acc_ui(hands_free=False)["Tja_D_Stat"], 2)
    self.make(canfd=False)
    self.assertEqual(self.acc_ui(hands_free=True)["Tja_D_Stat"], 2)

  def test_hands_free_is_off_when_not_published(self):
    self.make(canfd=True)
    self.assertEqual(self.acc_ui(published=False)["Tja_D_Stat"], 2)

  def test_main_off_is_standby_while_moving(self):
    self.make()
    self.assertEqual(self.acc_ui(lat=False, main=False, standstill=False)["Tja_D_Stat"], 1)
    self.make()
    self.assertEqual(self.acc_ui(lat=False, main=False, standstill=True)["Tja_D_Stat"], 0)

  def test_lane_lines_per_side_also_disengaged(self):
    for lat, main in ((True, True), (False, True), (False, False)):
      self.make()
      lines = self.step(0, lat=lat, main=main, hud_control=hud(leftLaneVisible=True))[LKAS_UI]["LaActvStats_D_Dsply"]
      self.assertEqual(lines, 1 + 10)  # left Available, right Suppress, engaged, main on or main off alike
    self.make()
    lines = self.step(0, hud_control=hud(leftLaneDepart=True, rightLaneVisible=True))[LKAS_UI]["LaActvStats_D_Dsply"]
    self.assertEqual(lines, 4 + 5)  # left Intervene, right Available

  def test_fcw_sounds_and_camera_tja_text_is_cleared(self):
    self.make()
    acc = self.acc_ui(hud_control=hud(visualAlert=VisualAlert.fcw),
                      acc_stock=stock("ACCDATA_3", TjaWarn_D_Rq=3, TjaMsgTxt_D_Dsply=1))
    self.assertEqual((acc["FcwVisblWarn_B_Rq"], acc["FcwAudioWarn_B_Rq"]), (1, 1))
    self.assertEqual((acc["TjaWarn_D_Rq"], acc["TjaMsgTxt_D_Dsply"]), (0, 0))

  def test_lkas_ui_rate(self):
    self.make()
    sent = [f for f in range(301) if LKAS_UI in self.step(f)]
    self.assertEqual(sent, [0, 100, 200, 300])  # 1 Hz with nothing changing

  def test_distance_bar_window(self):
    # BP7: after a bar change, ACCDATA_3 every frame for 401 frames with the bars shown, then 5 Hz without
    self.make(op_long=True)
    for f in range(50):
      self.step(f, hud_control=hud(leadDistanceBars=3))
    sent = {f: m[ACC_UI] for f in range(50, 600) if ACC_UI in (m := self.step(f, hud_control=hud(leadDistanceBars=2)))}
    self.assertEqual(sorted(sent)[:401], list(range(50, 451)))
    self.assertEqual(sorted(sent)[401:], [f for f in range(451, 600) if f % 20 == 0])
    self.assertEqual({sent[f]["AccTGap_B_Dsply"] for f in range(50, 451)}, {1})
    self.assertEqual({sent[f]["AccTGap_B_Dsply"] for f in sorted(sent)[401:]}, {0})

  def test_no_window_for_a_change_at_frame_0(self):
    self.make()
    self.step(0, hud_control=hud(leadDistanceBars=2))  # first frame counts as a change
    self.assertNotIn(ACC_UI, self.step(1, hud_control=hud(leadDistanceBars=2)))


if __name__ == '__main__':
  unittest.main()
