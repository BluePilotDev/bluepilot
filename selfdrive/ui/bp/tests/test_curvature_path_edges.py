import numpy as np
import pytest

from openpilot.selfdrive.ui.bp.lib.curvature_path_edges import CurvaturePathEdges, bend_strength, edge_ribbons


def road(curvature=0):
  x = np.linspace(0, 60, 33)
  return np.column_stack((x, curvature * x * x / 2, np.zeros_like(x)))


def test_bends_and_straight():
  assert bend_strength(road()) == 0
  assert bend_strength(road(.002)) < bend_strength(road(.008))
  assert bend_strength(road(-.008)) == pytest.approx(bend_strength(road(.008)))


@pytest.mark.parametrize('points', [[], [[0, 0]], road() * np.nan, road()[::-1], road()[:5]])
def test_invalid_or_short_geometry(points):
  assert bend_strength(points) == 0


def test_smoothing_dropout_and_disable():
  state = CurvaturePathEdges()
  start = state.last_time
  first = state.update(True, road(.008), True, start + .05)
  assert 0 < first < bend_strength(road(.008))
  second = state.update(True, road(.008), True, start + .1)
  assert second > first
  assert 0 < state.update(True, road(.008), False, start + .15) < second
  assert state.update(False, road(.008), True, start + .2) == 0


def test_ribbons_preserve_input_and_do_not_bridge_edges():
  points = np.array([[0, 0], [1, 1], [2, 2], [4, 2], [3, 1], [2, 0]], dtype=np.float32)
  original = points.copy()
  ribbons = edge_ribbons(points, 1)
  assert len(ribbons) == 2
  assert all(p.shape == (6, 2) and np.isfinite(p).all() for p in ribbons)
  np.testing.assert_array_equal(points, original)
  assert edge_ribbons(np.array([[np.nan, 0]]), 1) == []
