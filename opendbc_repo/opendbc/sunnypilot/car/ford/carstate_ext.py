"""
Ford car state for platforms without SteeringPinion_Data or PowertrainData_10 (FordFlags.ALT_STEER_ANGLE:
the Edge MK2). Called from opendbc/car/ford/carstate.py.

- Steering angle: the relative pinion angle (SteeringPinion_Data_Alt) plus an offset learned from the
  park assist module's absolute angle (ParkAid_Data) while that one is valid; otherwise the last offset.
- Gear: the gear lever position (TransGearData).
"""
from opendbc.can import CANDefine
from opendbc.car import Bus, structs
from opendbc.car.ford.values import DBC, FordFlags

# ExtSteeringAngleReq2 raw values meaning unknown / invalid
EXT_ANGLE_NOT_AVAILABLE = (32766, 32767)


class CarStateExt:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.steering_angle_offset_deg = 0.0
    if CP.flags & FordFlags.ALT_STEER_ANGLE:
      self.alt_shifter_values = CANDefine(DBC[CP.carFingerprint][Bus.pt]).dv["TransGearData"]["GearLvrPos_D_Actl"]

  @staticmethod
  def alt_steer_angle_messages() -> list[tuple[str, float]]:
    return [
      ("SteeringPinion_Data_Alt", 100),
      ("ParkAid_Data", 50),
      ("TransGearData", 10),
    ]

  def update_alt_steering_angle(self, cp) -> float:
    park_aid = cp.vl["ParkAid_Data"]
    park_aid_valid = (int((park_aid["ExtSteeringAngleReq2"] + 1000) * 10) not in EXT_ANGLE_NOT_AVAILABLE
                      and park_aid["EPASExtAngleStatReq"] == 0
                      and park_aid["ApaSys_D_Stat"] in (0, 1))

    relative_angle = cp.vl["SteeringPinion_Data_Alt"]["StePinRelInit_An_Sns"]
    if park_aid_valid:
      self.steering_angle_offset_deg = park_aid["ExtSteeringAngleReq2"] - relative_angle
    return relative_angle + self.steering_angle_offset_deg

  def alt_gear(self, cp) -> str | None:
    return self.alt_shifter_values.get(cp.vl["TransGearData"]["GearLvrPos_D_Actl"])
