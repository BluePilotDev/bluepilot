from cereal import car
from openpilot.selfdrive.car import PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarInfo
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu

SPEED_FROM_RPM = 0.008587


class CarControllerParams:
  ANGLE_DELTA_BP = [0., 5., 15.]
  ANGLE_DELTA_V = [5., .8, .15]     # windup limit
  ANGLE_DELTA_VU = [5., 3.5, 0.4]   # unwind limit
  LKAS_MAX_TORQUE = 1               # A value of 1 is easy to overpower
  STEER_THRESHOLD = 1.0

  def __init__(self, CP):
    pass


class CAR(Platforms):
  BODY = PlatformConfig(
    "COMMA BODY",
    CarInfo("comma body", package="All"),
    dbc_dict('comma_body', None),
  )


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

CAR_INFO = CAR.create_carinfo_map()
DBC = CAR.create_dbc_map()
