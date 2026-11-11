from opendbc.car.ford.values import CAR
from opendbc.car.fw_versions import FW_VERSION_GROUPS, VERSIONS, match_fw_to_car, match_fw_to_car_exact
from opendbc.car.structs import CarParams


def test_navigator_firmware_pairs():
  navigator = CAR.LINCOLN_NAVIGATOR_MK4
  database = VERSIONS['ford'][navigator]
  older_camera = (CarParams.Ecu.fwdCamera, 0x706, None)
  newer, older = FW_VERSION_GROUPS[navigator]
  for pair in (newer, older):
    inventory = {ecu[1:]: {pair.get(ecu, versions[0])} for ecu, versions in database.items()}
    assert match_fw_to_car_exact(inventory, match_brand='ford') == {navigator}

  inventory = {ecu[1:]: {newer.get(ecu, versions[0])} for ecu, versions in database.items()}
  inventory[older_camera[1:]] = {older[older_camera]}
  assert match_fw_to_car_exact(inventory, match_brand='ford') == {CAR.FORD_EXPEDITION_MK4}


def test_navigator_requires_exact_eps():
  navigator = CAR.LINCOLN_NAVIGATOR_MK4
  database = VERSIONS['ford'][navigator]
  for eps_version in (None, b'NL14-14D003-ZZ'.ljust(24, b'\x00')):
    fw = []
    for (ecu, addr, subaddr), versions in database.items():
      version = eps_version if ecu == CarParams.Ecu.eps else versions[0]
      if version is not None:
        fw.append(CarParams.CarFw(ecu=ecu, address=addr, subAddress=subaddr or 0, brand='ford', fwVersion=version))
    assert navigator not in match_fw_to_car(fw, '00000000000000000')[1]
