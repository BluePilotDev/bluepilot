"""
BluePilot Ford interface parameter extensions.

Called from stock interface.py at the end of _get_params() and _get_params_sp()
to apply BluePilot-specific parameter overrides without modifying stock logic.

Includes:
  - HEV flag auto-detection from fingerprint CAN IDs
  - Alpha longitudinal availability policy (always True for all Ford platforms)
  - DELPHI_MRR_64 radar delay configuration
  - Tuning overrides (steerActuatorDelay, longitudinalTuning.kpV)
  - ICBM (Intelligent Cruise Button Management) availability
"""

from opendbc.car import Bus, structs
from opendbc.car.ford.values import DBC, FordFlags, RADAR


def apply_ford_ext_params(ret: structs.CarParams, CP, car_fw, fingerprint) -> None:
  """
  Apply BluePilot parameter overrides to CarParams.

  Called at the end of CarInterface._get_params() after all stock parameter
  setup is complete.

  Args:
    ret: CarParams being built (modified in place)
    CP: CarParams reference (same as ret at this point)
    car_fw: List of CarFw from ECU queries
    fingerprint: Dict of CAN bus fingerprints {bus: {addr: len}}
  """
  from opendbc.car.ford.fordcan import CanBus
  CAN = CanBus(fingerprint=fingerprint)

  # BluePilot: tuning overrides
  ret.steerActuatorDelay = 0.22  # upstream: 0.2
  ret.longitudinalTuning.kpV = [0.]

  # BluePilot: DELPHI_MRR_64 radar support
  candidate = ret.carFingerprint
  if DBC[candidate][Bus.radar] == RADAR.DELPHI_MRR_64:
    ret.radarDelay = 0.1  # 20 Hz / 4 scan modes = 100 ms

  # BluePilot: alpha longitudinal always available for all Ford platforms
  # Upstream only enables for CANFD. We allow CAN vehicles to use Ford ACC too.
  ret.alphaLongitudinalAvailable = True

  # BluePilot: HEV flag auto-detection from CAN fingerprint
  # Cluster_HEV_Data2 (0x365) indicates hybrid cluster data is available
  if 0x365 in fingerprint[CAN.main]:
    ret.flags |= int(FordFlags.HEV_CLUSTER_DATA)

  # Battery_Traction_1 (0x07A), Battery_Traction_3 (0x24B), Battery_Traction_4 (0x24C)
  # All three must be present for full HEV battery telemetry
  if 0x07A in fingerprint[CAN.main] and 0x24B in fingerprint[CAN.main] and 0x24C in fingerprint[CAN.main]:
    ret.flags |= int(FordFlags.HEV_BATTERY_DATA)


def apply_ford_ext_params_sp(ret: structs.CarParamsSP) -> None:
  """
  Apply BluePilot parameter overrides to CarParamsSP.

  Called at the end of CarInterface._get_params_sp() after all stock SP
  parameter setup is complete.

  Args:
    ret: CarParamsSP being built (modified in place)
  """
  # BluePilot: Enable ICBM (Intelligent Cruise Button Management) for all Ford vehicles.
  # ICBM allows openpilot to control cruise speed by emulating button presses.
  # Available when openpilotLongitudinalControl is False (using stock ACC).
  ret.intelligentCruiseButtonManagementAvailable = True
