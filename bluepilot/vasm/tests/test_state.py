from types import SimpleNamespace

from openpilot.common.params import UnknownKeyName
from openpilot.bluepilot.vasm.state import combined_blindspots, get_vasm_blindspots


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.values.get(key, False))


def test_vasm_state_expires_after_daemon_stops():
  params = FakeParams({"VASMHeartbeat": 10.0, "VASMLeftActive": True, "VASMRightActive": True})
  assert get_vasm_blindspots(params, now=11.0) == (True, True)
  assert get_vasm_blindspots(params, now=15.0) == (False, False)


def test_combined_blindspots_preserves_oem_detection():
  car_state = SimpleNamespace(leftBlindspot=True, rightBlindspot=False)
  params = FakeParams({"VASMHeartbeat": 10.0, "VASMLeftActive": False, "VASMRightActive": True})
  assert combined_blindspots(car_state, params, now=11.0) == (True, True)


def test_unknown_keys_fail_closed_for_stale_quickboot_build():
  class StaleParams:
    def get(self, key):
      raise UnknownKeyName(key)

  assert get_vasm_blindspots(StaleParams(), now=11.0) == (False, False)
