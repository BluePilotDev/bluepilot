"""pandad's Rivian check reads the cached CarParams; an unreadable cache must not stop pandad (it would never start)."""
from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase

from openpilot.sunnypilot.selfdrive.pandad.rivian_long_flasher import _is_rivian


def car_params(brand: str) -> bytes:
  return car.CarParams.new_message(brand=brand).to_bytes()


class TestIsRivian(OpenpilotTestCase):
  def setUp(self):
    self.params = Params()
    self.params.remove("CarPlatformBundle")

  def test_a_wrapped_cached_car_params_is_not_a_rivian(self):
    # Another fork's format: a header in front, a length that is not a multiple of 8, so the decode raises.
    wrapped = b"SPCACHE" + car_params("rivian")
    self.params.put("CarParamsPersistent", wrapped, block=True)
    try:
      is_rivian = _is_rivian()
    except ValueError as e:
      self.fail(f"the unreadable cache stopped the check (pandad would never start): {e}")
    self.assertFalse(is_rivian)

  def test_a_readable_cached_car_params_still_names_the_brand(self):
    self.params.put("CarParamsPersistent", car_params("rivian"), block=True)
    self.assertTrue(_is_rivian())
    self.params.put("CarParamsPersistent", car_params("ford"), block=True)
    self.assertFalse(_is_rivian())
