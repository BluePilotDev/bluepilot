import pytest

from openpilot.bluepilot.vasm.config import normalize_annotation_config


def test_normalize_annotation_config():
  config = normalize_annotation_config({
    "width": "1928",
    "height": 1208,
    "poly_left": [[0, 0], [100, 0], [0, 100]],
    "poly_right": [],
  })
  assert config == {
    "width": 1928,
    "height": 1208,
    "poly_left": [[0.0, 0.0], [100.0, 0.0], [0.0, 100.0]],
    "poly_right": [],
  }


@pytest.mark.parametrize("config", [
  {},
  {"width": 100, "height": 100, "poly_left": [], "poly_right": []},
  {"width": 100, "height": 100, "poly_left": [[0, 0], [1, 1]], "poly_right": []},
  {"width": 100, "height": 100, "poly_left": [[0, 0], [100, 0], [101, 1]], "poly_right": []},
  {"width": 100, "height": 100, "poly_left": [[0, 0], [100, 0], [float("nan"), 1]], "poly_right": []},
])
def test_rejects_invalid_annotation_config(config):
  with pytest.raises(ValueError):
    normalize_annotation_config(config)
