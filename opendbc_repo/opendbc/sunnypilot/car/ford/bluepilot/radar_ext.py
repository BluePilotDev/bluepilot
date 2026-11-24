"""
BluePilot (full BluePilot only): Steer_Assist_Data as a one-track radar for CAN FD Fords, from
BluePilot 7. CAN FD Fords have no radar on the bus; the camera's collision-mitigation object
(Steer_Assist_Data, 0x3D7, camera bus) gives one lead track instead of a vision-only lead.

Called from opendbc/car/ford/radar_interface.py when the platform's radar is
RADAR.STEER_ASSIST_DATA (opendbc/car/ford/values.py).
"""
import collections

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import RADAR

STEER_ASSIST_DATA_ADDR = 0x3d7


def create_steer_assist_data_parser(CP) -> CANParser:
  return CANParser(RADAR.STEER_ASSIST_DATA, [("Steer_Assist_Data", 20)], CanBus(CP).camera)


class SteerAssistRadar:
  def __init__(self):
    self.track_id = 0
    self.v_rel_history: collections.deque = collections.deque(maxlen=20)

  def update(self, msg, pts: dict[int, structs.RadarData.RadarPoint]) -> None:
    d_rel = msg['CmbbObjDistLong_L_Actl']
    if msg['CmbbObjConfdnc_D_Stat'] <= 0:
      pts.pop(0, None)
      return

    new_track = 0 not in pts
    if new_track:
      pts[0] = structs.RadarData.RadarPoint()
      pts[0].trackId = self.track_id
      self.v_rel_history.clear()
      self.track_id += 1

    v_rel = msg['CmbbObjRelLong_V_Actl']
    if not new_track:
      # no relative speed reported: estimate it from the change in distance
      if abs(v_rel) < 1.0e-2:
        self.v_rel_history.append(d_rel - pts[0].dRel)
        v_rel = sum(self.v_rel_history)
      else:
        self.v_rel_history.clear()

      # a jump in speed or distance is a different object
      if abs(pts[0].vRel - v_rel) > 2 or abs(pts[0].dRel - d_rel) > 5:
        pts[0].trackId = self.track_id
        self.v_rel_history.clear()
        self.track_id += 1

    pts[0].dRel = d_rel  # from front of car
    pts[0].yRel = msg['CmbbObjDistLat_L_Actl']  # in car frame's y axis, left is positive
    pts[0].vRel = v_rel
