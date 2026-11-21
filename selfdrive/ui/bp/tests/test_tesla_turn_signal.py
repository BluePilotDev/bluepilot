from types import SimpleNamespace

from cereal import log
from openpilot.selfdrive.ui.bp.lib.lane_change_visuals import tesla_lane_change_lane_active
from openpilot.selfdrive.ui.bp.lib.tesla_palette import DARK_PALETTE, LIGHT_PALETTE
from openpilot.selfdrive.ui.bp.mici.onroad.alert_renderer_bp import AlertRendererBP
from openpilot.selfdrive.ui.bp.onroad.tesla_turn_signal import (
  TESLA_TURN_SIGNAL_FRAGMENT_SHADER,
  TeslaBlueTurnSignalController,
  tesla_turn_signal_color,
  tesla_turn_signal_state,
)
from openpilot.selfdrive.ui.mici.onroad.alert_renderer import AlertRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.turn_signal import TurnSignalConfig, TurnSignalController


class FakeSubMaster:
  def __init__(self, *, left_blinker: bool = False, right_blinker: bool = False,
               car_alive: bool = True, car_valid: bool = True,
               lane_change_state=log.LaneChangeState.off,
               lane_change_direction=log.LaneChangeDirection.none,
               model_alive: bool = True, model_valid: bool = True):
    self.alive = {"carState": car_alive, "modelV2": model_alive}
    self.valid = {"carState": car_valid, "modelV2": model_valid}
    self.messages = {
      "carState": SimpleNamespace(leftBlinker=left_blinker, rightBlinker=right_blinker),
      "modelV2": SimpleNamespace(meta=SimpleNamespace(
        laneChangeState=lane_change_state,
        laneChangeDirection=lane_change_direction,
      )),
    }

  def __getitem__(self, service):
    return self.messages[service]


def test_tesla_turn_signals_honor_existing_show_toggle() -> None:
  sm = FakeSubMaster(left_blinker=True)

  assert tesla_turn_signal_state(sm, True) == (True, False)
  assert tesla_turn_signal_state(sm, False) == (False, False)


def test_tesla_turn_signals_reject_invalid_car_state() -> None:
  assert tesla_turn_signal_state(FakeSubMaster(left_blinker=True, car_alive=False), True) == (False, False)
  assert tesla_turn_signal_state(FakeSubMaster(right_blinker=True, car_valid=False), True) == (False, False)


def _rgba(color) -> tuple[int, int, int, int]:
  return color.r, color.g, color.b, color.a


def test_tesla_turn_signals_reuse_stock_shape_layout_and_pulse_controller() -> None:
  assert TurnSignalConfig() == TurnSignalConfig(left_x=80, left_y=190, right_x=80, right_y=190, size=150)
  assert issubclass(TeslaBlueTurnSignalController, TurnSignalController)
  assert issubclass(AlertRendererBP, AlertRenderer)
  assert "source.a" in TESLA_TURN_SIGNAL_FRAGMENT_SHADER


def test_tesla_turn_signal_matches_max_label_blue_in_both_palettes() -> None:
  assert _rgba(tesla_turn_signal_color(0.0)) == _rgba(LIGHT_PALETTE.max_active)
  assert _rgba(tesla_turn_signal_color(1.0)) == _rgba(DARK_PALETTE.max_active)
  assert tesla_turn_signal_color(0.0, 100).a == 100


def test_lane_change_highlight_targets_only_destination_inner_lane() -> None:
  left = FakeSubMaster(
    lane_change_state=log.LaneChangeState.laneChangeStarting,
    lane_change_direction=log.LaneChangeDirection.left,
  )
  right = FakeSubMaster(
    lane_change_state=log.LaneChangeState.laneChangeFinishing,
    lane_change_direction=log.LaneChangeDirection.right,
  )

  assert {i for i in range(4) if tesla_lane_change_lane_active(left, i)} == {1}
  assert {i for i in range(4) if tesla_lane_change_lane_active(right, i)} == {2}


def test_lane_change_highlight_stays_off_while_only_preparing_or_invalid() -> None:
  pre = FakeSubMaster(
    lane_change_state=log.LaneChangeState.preLaneChange,
    lane_change_direction=log.LaneChangeDirection.left,
  )
  invalid = FakeSubMaster(
    lane_change_state=log.LaneChangeState.laneChangeStarting,
    lane_change_direction=log.LaneChangeDirection.left,
    model_valid=False,
  )

  assert not any(tesla_lane_change_lane_active(pre, i) for i in range(4))
  assert not any(tesla_lane_change_lane_active(invalid, i) for i in range(4))
