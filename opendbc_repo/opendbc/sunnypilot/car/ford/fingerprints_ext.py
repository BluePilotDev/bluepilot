"""
BluePilot Ford firmware version extensions.

Contains FW_VERSIONS for BluePilot-only Ford platforms (Ford Edge MK2).
Merged into the main FW_VERSIONS dict at module load time in
opendbc/car/ford/fingerprints.py via merge_fw_versions().
"""

from opendbc.car.ford.values import CAR
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

# BluePilot-only Ford platform firmware versions
FW_VERSIONS_EXT = {
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
}
