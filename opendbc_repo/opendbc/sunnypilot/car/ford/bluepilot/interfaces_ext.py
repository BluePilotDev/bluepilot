"""
BluePilot (full BluePilot only): Ford longitudinal mode selection, from BluePilot 7.

Every Ford gets the alpha longitudinal toggle, and the toggle decides: on = openpilot
longitudinal, off = the car's own ACC. Upstream only offers the toggle on Fords without a
radar in the DBC (CAN FD) and always runs openpilot longitudinal on the CAN Fords that have one.

Called at the end of CarInterface._get_params() in opendbc/car/ford/interface.py.
"""
from opendbc.car import structs
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import FordFlags, FordSafetyFlags
from opendbc.sunnypilot.car.ford.bluepilot.carstate_ext import FORD_FLAG_BP_HEV_BATTERY_DATA, FORD_FLAG_BP_HEV_CLUSTER_DATA

# Safety param bit (ford.h): restrict a CAN Ford to the stock TX set (no ACCDATA), so the car's
# own ACC runs. Upstream's CAN Ford always gets the longitudinal set, so the bit is BluePilot's
# own and the top bit, as far as possible from where upstream adds Ford flags.
FORD_SAFETY_BP_STOCK_LONG = 0x8000


def apply_ford_ext_params(ret: structs.CarParams, alpha_long: bool, fingerprint) -> None:
  ret.alphaLongitudinalAvailable = True
  ret.openpilotLongitudinalControl = bool(alpha_long)

  safety = ret.safetyConfigs[-1]
  if ret.openpilotLongitudinalControl:
    safety.safetyParam |= FordSafetyFlags.LONG_CONTROL.value
  else:
    safety.safetyParam &= ~FordSafetyFlags.LONG_CONTROL.value
    if not ret.flags & FordFlags.CANFD:
      safety.safetyParam |= FORD_SAFETY_BP_STOCK_LONG

  # hybrid/EV data the car state reads (carstate_ext.py), from the fingerprint: the cluster's
  # hybrid display message, and all three traction battery messages
  main_bus = fingerprint[CanBus(fingerprint=fingerprint).main]
  if 0x365 in main_bus:  # Cluster_HEV_Data2
    ret.flags |= FORD_FLAG_BP_HEV_CLUSTER_DATA
  if all(addr in main_bus for addr in (0x07A, 0x24B, 0x24C)):  # Battery_Traction_1/3/4_FD1
    ret.flags |= FORD_FLAG_BP_HEV_BATTERY_DATA


def apply_ford_ext_params_sp(ret: structs.CarParamsSP) -> None:
  """Called at the end of CarInterface._get_params_sp() in opendbc/car/ford/interface.py."""
  ret.intelligentCruiseButtonManagementAvailable = True  # ICBM: sunnypilot/car/ford/bluepilot/icbm.py
