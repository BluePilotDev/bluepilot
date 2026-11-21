from openpilot.selfdrive.ui.bp.lib.model_path_color import (
  FORD_PATH_COLOR_RGBA,
  FORD_PATH_EDGE_RGBA,
  FORD_PATH_STOPS,
  ModelPathColor,
  get_model_path_color,
)


class FakeParams:
  def __init__(self, value):
    self.value = value

  def get(self, _key):
    return self.value


def test_green_is_default():
  assert get_model_path_color(FakeParams(None)) == ModelPathColor.GREEN
  assert get_model_path_color(FakeParams(99)) == ModelPathColor.GREEN


def test_ford_selection():
  assert get_model_path_color(FakeParams(1)) == ModelPathColor.FORD


def test_ford_gradient_is_blue_and_fades_toward_horizon():
  assert all(blue > red for red, _green, blue, _alpha in FORD_PATH_COLOR_RGBA)
  assert FORD_PATH_COLOR_RGBA[0][3] > FORD_PATH_COLOR_RGBA[1][3] > FORD_PATH_COLOR_RGBA[2][3]
  assert tuple(sorted(FORD_PATH_STOPS)) == FORD_PATH_STOPS
  assert FORD_PATH_EDGE_RGBA[2] > FORD_PATH_EDGE_RGBA[0]
