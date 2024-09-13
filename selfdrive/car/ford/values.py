import copy
import re
from collections import namedtuple
from dataclasses import dataclass, field, replace
from enum import Enum, IntFlag

import panda.python.uds as uds
from cereal import car
from openpilot.selfdrive.car import AngleRateLimit, CarSpecs, dbc_dict, DbcDict, PlatformConfig, Platforms
from openpilot.selfdrive.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column, \
                                                     Device
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, LiveFwVersions, OfflineFwVersions, Request, StdQueries, p16

Ecu = car.CarParams.Ecu
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


class CarControllerParams:
  STEER_STEP = 5        # LateralMotionControl, 20Hz
  LKA_STEP = 3          # Lane_Assist_Data1, 33Hz
  ACC_CONTROL_STEP = 2  # ACCDATA, 50Hz
  LKAS_UI_STEP = 100    # IPMA_Data, 1Hz
  ACC_UI_STEP = 20      # ACCDATA_3, 5Hz
  BUTTONS_STEP = 5      # Steering_Data_FD1, 10Hz, but send twice as fast

  CURVATURE_MAX = 0.01  # Max curvature for steering command, m^-1
  STEER_DRIVER_ALLOWANCE = 1.0  # Driver intervention threshold, Nm

  # Curvature rate limits
  # The curvature signal is limited to 0.003 to 0.009 m^-1/sec by the EPS depending on speed and direction
  ANGLE_RATE_LIMIT_UP = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.0024, 0.0002])
  ANGLE_RATE_LIMIT_DOWN = AngleRateLimit(speed_bp=[5, 25], angle_v=[0.0024, 0.0004])
  CURVATURE_ERROR = 0.01  # ~6 degrees at 10 m/s, ~10 degrees at 35 m/s

  ACCEL_MAX = 2.0               # m/s^2 max acceleration
  ACCEL_MIN = -3.5              # m/s^2 max deceleration
  MIN_GAS = -0.5
  INACTIVE_GAS = -5.0

  def __init__(self, CP):
    pass


class FordConfig:
    BLUECRUISE_CLUSTER_PRESENT = False

class FordFlags(IntFlag):
  # Static flags
  CANFD = 1
  ALT_STEER_ANGLE = 2


class FordFlagsSP(IntFlag):
  SP_ENHANCED_LAT_CONTROL = 1


class RADAR:
  DELPHI_ESR = 'ford_fusion_2018_adas'
  DELPHI_MRR = 'FORD_CADS'
  STEER_ASSIST_DATA = 'ford_lincoln_base_pt'


class Footnote(Enum):
  FOCUS = CarFootnote(
    "Refers only to the Focus Mk4 (C519) available in Europe/China/Taiwan/Australasia, not the Focus Mk3 (C346) in " +
    "North and South America/Southeast Asia.",
    Column.MODEL,
  )


@dataclass
class FordCarDocs(CarDocs):
  package: str = "Co-Pilot360 Assist+"
  hybrid: bool = False
  plug_in_hybrid: bool = False

  def init_make(self, CP: car.CarParams):
    harness = CarHarness.ford_q4 if CP.flags & FordFlags.CANFD else CarHarness.ford_q3
    if CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_MAVERICK_MK1, CAR.FORD_F_150_MK14, CAR.FORD_F_150_LIGHTNING_MK1, CAR.FORD_ESCAPE_MK4_23REFRESH):
      self.car_parts = CarParts([Device.threex_angled_mount, harness])
    else:
      self.car_parts = CarParts([Device.threex, harness])


@dataclass
class FordPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('ford_lincoln_base_pt', RADAR.DELPHI_MRR))

  def init(self):
    for car_docs in list(self.car_docs):
      if car_docs.hybrid:
        name = f"{car_docs.make} {car_docs.model} Hybrid {car_docs.years}"
        self.car_docs.append(replace(copy.deepcopy(car_docs), name=name))
      if car_docs.plug_in_hybrid:
        name = f"{car_docs.make} {car_docs.model} Plug-in Hybrid {car_docs.years}"
        self.car_docs.append(replace(copy.deepcopy(car_docs), name=name))


@dataclass
class FordCANFDPlatformConfig(FordPlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('ford_lincoln_base_pt', RADAR.STEER_ASSIST_DATA))

  def init(self):
    super().init()
    self.flags |= FordFlags.CANFD


class CAR(Platforms):
  FORD_BRONCO_SPORT_MK1 = FordPlatformConfig(
    [FordCarDocs("Ford Bronco Sport 2021-23")],
    CarSpecs(mass=1625, wheelbase=2.67, steerRatio=17.7),
  )
  FORD_EDGE_MK2 = FordPlatformConfig(
    [FordCarDocs("Ford Edge 2022")],
    CarSpecs(mass=1933, steerRatio=15.3, wheelbase=2.824),
    flags=FordFlags.ALT_STEER_ANGLE,
  )
  FORD_ESCAPE_MK4 = FordPlatformConfig(
    [
      FordCarDocs("Ford Escape 2020-22", hybrid=True, plug_in_hybrid=True),
      FordCarDocs("Ford Kuga 2020-22", "Adaptive Cruise Control with Lane Centering", hybrid=True, plug_in_hybrid=True),
    ],
    CarSpecs(mass=1750, wheelbase=2.71, steerRatio=16.7),
  )
  FORD_ESCAPE_MK4_23REFRESH = FordCANFDPlatformConfig(
    [
      FordCarDocs("Ford Escape 2023-24", "Co-Pilot360 Assist 2.0", hybrid=True, plug_in_hybrid=True),
      FordCarDocs("Ford Kuga 2023-24", "Co-Pilot360 Assist 2.0", hybrid=True, plug_in_hybrid=True),
    ],
    CarSpecs(mass=1750, wheelbase=2.71, steerRatio=16.7),
  )
  FORD_EXPLORER_MK6 = FordPlatformConfig(
    [
      FordCarDocs("Ford Explorer 2020-23", hybrid=True),  # Hybrid: Limited and Platinum only
      FordCarDocs("Lincoln Aviator 2020-23", "Co-Pilot360 Plus", plug_in_hybrid=True),  # Hybrid: Grand Touring only
    ],
    CarSpecs(mass=2050, wheelbase=3.025, steerRatio=16.8),
  )
  FORD_F_150_MK14 = FordCANFDPlatformConfig(
    [FordCarDocs("Ford F-150 2022-23", "Co-Pilot360 Active 2.0", hybrid=True)],
    CarSpecs(mass=2000, wheelbase=3.69, steerRatio=17.0),
  )
  FORD_F_150_LIGHTNING_MK1 = FordCANFDPlatformConfig(
    [FordCarDocs("Ford F-150 Lightning 2021-23", "Co-Pilot360 Active 2.0")],
    CarSpecs(mass=2948, wheelbase=3.70, steerRatio=16.9),
  )
  FORD_FOCUS_MK4 = FordPlatformConfig(
    [FordCarDocs("Ford Focus 2018", "Adaptive Cruise Control with Lane Centering", footnotes=[Footnote.FOCUS], hybrid=True)],  # mHEV only
    CarSpecs(mass=1350, wheelbase=2.7, steerRatio=15.0),
  )
  FORD_MAVERICK_MK1 = FordPlatformConfig(
    [
      FordCarDocs("Ford Maverick 2022", "LARIAT Luxury", hybrid=True),
      FordCarDocs("Ford Maverick 2023-24", "Co-Pilot360 Assist", hybrid=True),
    ],
    CarSpecs(mass=1650, wheelbase=3.076, steerRatio=17.0),
  )
  FORD_MUSTANG_MACH_E_MK1 = FordCANFDPlatformConfig(
    [FordCarDocs("Ford Mustang Mach-E 2021-23", "Co-Pilot360 Active 2.0")],
    CarSpecs(mass=2200, wheelbase=2.984, steerRatio=17.0),  # TODO: check steer ratio
  )
  FORD_RANGER_MK2 = FordCANFDPlatformConfig(
    [FordCarDocs("Ford Ranger 2024", "Adaptive Cruise Control with Lane Centering")],
    CarSpecs(mass=2000, wheelbase=3.27, steerRatio=17.0),
  )


# Custom Ford Vehicle Tuning Params (per-fingerprint)
FORD_VEHICLE_TUNINGS = {
  "FORD_F_150_MK14": {
    "brake_actuator_target": -0.04,
    "brake_actuator_stdDevLow": 0.1,
    "brake_actuator_stdDevHigh": 0.0,
    "precharge_actuator_target": -0.04,
    "precharge_actuator_stdDevLow": 0.08,
    "precharge_actuator_stdDevHigh": 0.0,
    "path_lookup_time": 0.25,
    "reset_lookup_time": 0.5,
    "steerActuatorDelay": 0.2,
    "steerLimitTimer": 1.5,
    "stoppingControl": True,
    "startingState": True,
    "startAccel": 1.0,
    "stoppingDecelRate": 0.8,
    "longitudinalTuning": {
      "kpBP": [0.0],
      "kpV": [0.0],
      "kiV": [0.0],
    },
    "lane_change_factor": 0.65,
  },
  "FORD_F_150_LIGHTNING_MK1": {
    "brake_actuator_target": -0.04,
    "brake_actuator_stdDevLow": 0.1,
    "brake_actuator_stdDevHigh": 0.0,
    "precharge_actuator_target": -0.04,
    "precharge_actuator_stdDevLow": 0.08,
    "precharge_actuator_stdDevHigh": 0.0,
    "path_lookup_time": 0.5,
    "reset_lookup_time": 0.5,
    "steerActuatorDelay": 0.02,
    "steerLimitTimer": 1.5,
    "stoppingControl": True,
    "startingState": True,
    "startAccel": 1.0,
    "stoppingDecelRate": 0.8,
    "longitudinalTuning": {
      "kpBP": [0.0],
      "kpV": [0],
      "kiV": [0],
    },
    "lane_change_factor": 0.65,
  },
  "FORD_MUSTANG_MACH_E_MK1": {
    "brake_actuator_target": -0.04,
    "brake_actuator_stdDevLow": 0.1,
    "brake_actuator_stdDevHigh": 0.0,
    "precharge_actuator_target": -0.04,
    "precharge_actuator_stdDevLow": 0.08,
    "precharge_actuator_stdDevHigh": 0.0,
    "path_lookup_time": 0.5,
    "reset_lookup_time": 0.5,
    "steerActuatorDelay": 0.2,
    "steerLimitTimer": 1.5,
    "stoppingControl": True,
    "startingState": True,
    "startAccel": 1.0,
    "stoppingDecelRate": 0.8,
    "longitudinalTuning": {
      "kpBP": [0.0],
      "kpV": [0],
      "kiV": [0],
    },
    "lane_change_factor": 0.65,
  }
}

BUTTONS = [
  Button(car.CarState.ButtonEvent.Type.accelCruise, "Steering_Data_FD1", "CcAslButtnSetIncPress", [1]),
  Button(car.CarState.ButtonEvent.Type.decelCruise, "Steering_Data_FD1", "CcAslButtnSetDecPress", [1]),
  Button(car.CarState.ButtonEvent.Type.cancel, "Steering_Data_FD1", "CcAslButtnCnclPress", [1]),
  Button(car.CarState.ButtonEvent.Type.setCruise, "Steering_Data_FD1", "CcAslButtnSetPress", [1]),
  Button(car.CarState.ButtonEvent.Type.resumeCruise, "Steering_Data_FD1", "CcAsllButtnResPress", [1]),
]


# FW response contains a combined software and part number
# A-Z except no I, O or W
# e.g. NZ6A-14C204-AAA
#      1222-333333-444
# 1 = Model year hint (approximates model year/generation)
# 2 = Platform hint
# 3 = Part number
# 4 = Software version
FW_ALPHABET = b'A-HJ-NP-VX-Z'
# Full firmware version pattern
FW_PATTERN_FULL = re.compile(
    b'^(?P<model_year_hint>[' + FW_ALPHABET + b'])' +
    b'(?P<platform_hint>[0-9' + FW_ALPHABET + b']{3})-' +
    b'(?P<part_number>[0-9' + FW_ALPHABET + b']{5,6})-' +
    b'(?P<software_revision>[' + FW_ALPHABET + b']{2,})\x00*$'
)

# Partial firmware version pattern (excluding software_revision)
FW_PATTERN_PARTIAL = re.compile(
    b'^(?P<model_year_hint>[' + FW_ALPHABET + b'])' +
    b'(?P<platform_hint>[0-9' + FW_ALPHABET + b']{3})-' +
    b'(?P<part_number>[0-9' + FW_ALPHABET + b']{5,6})\x00*$'
)


def get_platform_codes(fw_versions: list[bytes] | set[bytes]) -> set[tuple[bytes, bytes]]:
  codes = set()
  for fw in fw_versions:
    # Try full match first
    full_match = FW_PATTERN_FULL.match(fw)
    if full_match is not None:
      codes.add((full_match.group('platform_hint'), full_match.group('model_year_hint')))
    else:
      # If full match fails, try partial match
      partial_match = FW_PATTERN_PARTIAL.match(fw)
      if partial_match is not None:
        codes.add((partial_match.group('platform_hint'), partial_match.group('model_year_hint')))

  return codes


def match_fw_to_car_fuzzy(live_fw_versions: LiveFwVersions, vin: str, offline_fw_versions: OfflineFwVersions) -> set[str]:
  candidates: set[str] = set()

  for candidate, fws in offline_fw_versions.items():
    # Keep track of ECUs which pass all checks (platform hint, within model year hint range)
    valid_found_ecus = set()
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      # Only check ECUs expected to have platform codes
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue

      # Expected platform codes & model year hints
      codes = get_platform_codes(expected_versions)
      expected_platform_codes = {code for code, _ in codes}
      expected_model_year_hints = {model_year_hint for _, model_year_hint in codes}

      # Found platform codes & model year hints
      codes = get_platform_codes(live_fw_versions.get(addr, set()))
      found_platform_codes = {code for code, _ in codes}
      found_model_year_hints = {model_year_hint for _, model_year_hint in codes}

      # Check platform code matches for any found versions
      if not any(found_platform_code in expected_platform_codes for found_platform_code in found_platform_codes):
        break

      # Check any model year hint within range in the database. Note that some models have more than one
      # platform code per ECU which we don't consider as separate ranges
      if not any(min(expected_model_year_hints) <= found_model_year_hint <= max(expected_model_year_hints) for
                 found_model_year_hint in found_model_year_hints):
        break

      valid_found_ecus.add(addr)

    # If all live ECUs pass all checks for candidate, add it as a match
    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(candidate)

  return candidates


# All of these ECUs must be present and are expected to have platform codes we can match
PLATFORM_CODE_ECUS = (Ecu.abs, Ecu.fwdCamera, Ecu.fwdRadar, Ecu.eps)

DATA_IDENTIFIER_FORD_ASBUILT = 0xDE00

ASBUILT_BLOCKS: list[tuple[int, list]] = [
  (1, [Ecu.debug, Ecu.fwdCamera, Ecu.eps, Ecu.hud]),
  (2, [Ecu.abs, Ecu.debug, Ecu.eps, Ecu.hud]),
  (3, [Ecu.abs, Ecu.debug, Ecu.eps, Ecu.hud]),
  (4, [Ecu.debug, Ecu.fwdCamera, Ecu.hud]),
  (5, [Ecu.debug]),
  (6, [Ecu.debug]),
  (7, [Ecu.debug]),
  (8, [Ecu.debug]),
  (9, [Ecu.debug]),
  (16, [Ecu.debug, Ecu.fwdCamera]),
  (18, [Ecu.fwdCamera]),
  (20, [Ecu.fwdCamera]),
  (21, [Ecu.fwdCamera]),
]


def ford_asbuilt_block_request(block_id: int):
  return bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + p16(DATA_IDENTIFIER_FORD_ASBUILT + block_id - 1)


def ford_asbuilt_block_response(block_id: int):
  return bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + p16(DATA_IDENTIFIER_FORD_ASBUILT + block_id - 1)


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # CAN and CAN FD queries are combined.
    # FIXME: For CAN FD, ECUs respond with frames larger than 8 bytes on the powertrain bus
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.debug, Ecu.engine, Ecu.eps, Ecu.fwdCamera, Ecu.fwdRadar, Ecu.shiftByWire, Ecu.hud],
      logging=True,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.debug, Ecu.engine, Ecu.eps, Ecu.fwdCamera, Ecu.fwdRadar, Ecu.shiftByWire, Ecu.hud],
      bus=0,
      auxiliary=True,
    ),
    *[Request(
      [StdQueries.TESTER_PRESENT_REQUEST, ford_asbuilt_block_request(block_id)],
      [StdQueries.TESTER_PRESENT_RESPONSE, ford_asbuilt_block_response(block_id)],
      whitelist_ecus=ecus,
      bus=0,
      logging=True,
    ) for block_id, ecus in ASBUILT_BLOCKS],
  ],
  extra_ecus=[
    (Ecu.engine, 0x7e0, None),        # Powertrain Control Module
                                      # Note: We are unlikely to get a response from behind the gateway
    (Ecu.shiftByWire, 0x732, None),   # Gear Shift Module
    (Ecu.debug, 0x7d0, None),         # Accessory Protocol Interface Module
    (Ecu.hud, 0x720, None),           # Instrument Cluster Module
  ],
  # Custom fuzzy fingerprinting function using platform and model year hints
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

DBC = CAR.create_dbc_map()
