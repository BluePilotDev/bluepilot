"""
BluePilot: Ford fingerprinting extensions (from BluePilot 7; written to be offered upstream).

- FW_VERSIONS_EXT: firmware seen on BluePilot users' cars that upstream doesn't list yet. Merged
  into the platform tables at the end of opendbc/car/ford/fingerprints.py via merge_fw_versions().
- match_vin_to_car(): a VIN fallback for when the firmware matches nothing, called at the end of
  match_fw_to_car_fuzzy() in opendbc/car/ford/values.py.
"""

from opendbc.car.ford.values import CAR
from opendbc.car.structs import CarParams
from opendbc.car.vin import Vin, is_valid_vin

Ecu = CarParams.Ecu

FW_VERSIONS_EXT = {
  CAR.FORD_BRONCO_SPORT_MK1: {
    (Ecu.abs, 0x760, None): [
      b'LX6C-2D053-KF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'LX6C-2D053-KG\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_EDGE_MK2: {
    (Ecu.eps, 0x730, None): [
      b'M2GC-14D003-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'M2GC-2D053-CB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'M2GC-2D053-EA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'JX7T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'KT4T-14F397-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_ESCAPE_MK4_5: {
    (Ecu.eps, 0x730, None): [
      b'PZ11-14D003-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PZ11-14D003-FA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdCamera, 0x706, None): [
      b'PJ6T-14H102-ABE\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PJ6T-14H102-MDC\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PJ6T-14H102-MDF\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PJ6T-14H102-SCD\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PJ6T-14H102-SCG\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_EXPLORER_MK6: {
    (Ecu.eps, 0x730, None): [
      b'R1MC-14D003-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'R1MC-14D003-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'R1MC-14D003-AG\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'R1MC-14D003-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_EXPEDITION_MK4: {
    (Ecu.abs, 0x760, None): [
      b'PL14-2D053-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_F_150_MK14: {
    (Ecu.eps, 0x730, None): [
      b'MB3C-14D003-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_F_150_LIGHTNING_MK1: {
    (Ecu.abs, 0x760, None): [
      b'NL38-2D053-AF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'RL38-2D053-BC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'TL38-2D053-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'ML3T-14D049-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'RB5T-14D049-AB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x730, None): [
      b'NL38-14D003-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NL38-14D003-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_FOCUS_MK4: {
    (Ecu.eps, 0x730, None): [
      b'JX6C-14D003-AL\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'JX6C-14D003-BB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'NX61-2D053-MD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  # From BluePilot 7, which kept only the part numbers of the first Mondeo capture (the capture also held NAK/error
  # responses). Its engine (0x7e0) and debug (0x7d0) entries are left out: upstream's Ford fingerprints hold only
  # ABS, EPS, radar and camera entries.
  CAR.FORD_MONDEO_MK5: {
    (Ecu.fwdCamera, 0x706, None): [
      b'KT4T-14F397-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'KG9C-2D053-DF\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.eps, 0x730, None): [
      b'K2GC-14D003-AJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'JX7T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_MUSTANG_MACH_E_MK1: {
    (Ecu.fwdRadar, 0x764, None): [
      b'ML3T-14D049-AH\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'ML3T-14D049-AK\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
  CAR.FORD_RANGER_MK2: {
    (Ecu.eps, 0x730, None): [
      b'JR3C-14D003-AA\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'NL14-14D003-AC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.abs, 0x760, None): [
      b'MB3C-2D053-AE\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'MB3C-2D053-ZJ\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PB3C-2D053-ZB\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PB3C-2D053-ZC\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
      b'PB9C-2D053-ZG\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
    (Ecu.fwdRadar, 0x764, None): [
      b'JX7T-14D049-AD\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
    ],
  },
}

# VIN position 10; I, O, Q, U, Z are skipped by the standard
MY_2020, MY_2021, MY_2022, MY_2023, MY_2024, MY_2025 = 'L', 'M', 'N', 'P', 'R', 'S'

# VIN tables sourced from https://github.com/commaai/openpilot/issues/31052 and NHTSA vPIC
# F-150 and Lightning share WMI and body codes; position 8 is the powertrain, L/V = electric
#
# W1B/W3L/W5L/W7L are the 2024-25 series codes (Pro / XLT+Flash / Lariat / Platinum), and K/S/7/M
# are the matching 2024-25 SR/ER powertrain codes. Without these a real 2024-25 Lightning VIN
# fails the vds_codes check before position 8 is even read, and falls through to no match. This
# set is the best known mapping, not yet cross-checked against the Ford Pro VIN guide.
F150_VDS_CODES = {'F1C', 'F1E', 'W1C', 'W1E', 'X1C', 'X1E', 'W1R', 'W1P', 'W1S', 'W1T',
                  'W1B', 'W3L', 'W5L', 'W7L'}
F150_ELECTRIC_CODES = {'L', 'V', 'K', 'S', '7', 'M'}
MACH_E_VDS_CODES = {'K1R', 'K1S', 'K2S', 'K3R', 'K3S', 'K4S'}

# platform: (WMIs = VIN positions 1-3, codes matched anywhere in positions 4-7 since Ford's offset
# varies by model, model years = VIN position 10). A platform not listed stays out of VIN matching.
VIN_PLATFORMS = {
  CAR.FORD_BRONCO_SPORT_MK1: ({'3FM'}, {'CR9'}, {MY_2021, MY_2022, MY_2023, MY_2024}),
  CAR.FORD_EDGE_MK2: ({'2FM'}, {'PK4'}, {MY_2022}),
  CAR.FORD_ESCAPE_MK4: ({'1FM'}, {'CU0', 'CU9'}, {MY_2020, MY_2021, MY_2022}),  # CU0 = 4x2, CU9 = 4WD
  # same body codes as the MK4; the model year letter separates them
  CAR.FORD_ESCAPE_MK4_5: ({'1FM'}, {'CU0', 'CU9'}, {MY_2023, MY_2024}),
  # SK7/SK8 (Explorer) and YJ8/YJ9/5J9 (Aviator) are the post-2021 descriptor blocks
  CAR.FORD_EXPLORER_MK6: ({'1FM', '5LM'}, {'5K7', '5K8', '5J7', 'SK7', 'SK8', 'YJ8', 'YJ9', '5J9'},
                          {MY_2020, MY_2021, MY_2022, MY_2023, MY_2024}),
  # JK1 is the MAX; same platform here
  CAR.FORD_EXPEDITION_MK4: ({'1FM'}, {'JU1', 'JU2', 'JK1'}, {MY_2022, MY_2023, MY_2024}),
  CAR.FORD_F_150_MK14: ({'1FT'}, F150_VDS_CODES, {MY_2021, MY_2022, MY_2023}),
  CAR.FORD_F_150_LIGHTNING_MK1: ({'1FT'}, F150_VDS_CODES, {MY_2022, MY_2023, MY_2024, MY_2025}),
  CAR.FORD_MAVERICK_MK1: ({'3FT'}, {'TW8'}, {MY_2022, MY_2023, MY_2024}),
  CAR.FORD_MUSTANG_MACH_E_MK1: ({'3FM'}, MACH_E_VDS_CODES, {MY_2021, MY_2022, MY_2023, MY_2024}),
  # the year set excludes the unsupported 2019-23 Ranger, which shares this code
  CAR.FORD_RANGER_MK2: ({'1FT'}, {'ER4'}, {MY_2024}),
}


def match_vin_to_car(vin: str) -> set[str]:
  """Decode a North American Ford VIN to a platform; empty for non-NA or unknown VINs."""
  if not is_valid_vin(vin):
    return set()

  wmi = Vin(vin).wmi
  vds = vin[3:7]        # positions 4-7: line, series, body
  model_year = vin[9]   # position 10

  candidates = {platform for platform, (wmis, vds_codes, years) in VIN_PLATFORMS.items()
                if wmi in wmis and model_year in years and any(code in vds for code in vds_codes)}

  # F-150 and Lightning are indistinguishable by WMI/body code; position 8 is the powertrain
  if {CAR.FORD_F_150_MK14, CAR.FORD_F_150_LIGHTNING_MK1} & candidates:
    electric = vin[7] in F150_ELECTRIC_CODES
    candidates.discard(CAR.FORD_F_150_MK14 if electric else CAR.FORD_F_150_LIGHTNING_MK1)

  return {str(c) for c in candidates}
