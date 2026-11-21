from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.bp.lib.blindspot_visuals import tesla_blindspot_lane_active


class FakeSubMaster:
  def __init__(self, left: bool = False, right: bool = False, valid: bool = True):
    self.valid = {"carState": valid}
    self._car_state = SimpleNamespace(leftBlindspot=left, rightBlindspot=right)

  def __getitem__(self, key: str):
    assert key == "carState"
    return self._car_state


@pytest.mark.parametrize(("left", "right", "expected"), [
  (False, False, set()),
  (True, False, {1}),
  (False, True, {2}),
  (True, True, {1, 2}),
])
def test_blindspot_state_highlights_only_matching_inner_lane(left, right, expected):
  sm = FakeSubMaster(left, right)
  assert {i for i in range(4) if tesla_blindspot_lane_active(sm, i)} == expected


def test_invalid_car_state_highlights_no_lane():
  sm = FakeSubMaster(left=True, right=True, valid=False)
  assert not any(tesla_blindspot_lane_active(sm, i) for i in range(4))
