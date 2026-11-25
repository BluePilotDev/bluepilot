import unittest

from openpilot.selfdrive.ui.bp.lib.model_path_color_bp import (
  BP_BLUE_EDGE_HEX,
  BP_BLUE_GRADIENT_RGBA,
  BP_BLUE_GRADIENT_STOPS,
  MIN_VISIBLE_VALUE,
  PRESET_COLORS,
  model_path_color_hex,
  model_path_color_is_visible,
  model_path_color_label,
  model_path_color_rgb,
  normalize_model_path_color,
)

class TestModelPathColor(unittest.TestCase):
  def test_normalizes_presets_and_custom_hex(self):
    assert list(PRESET_COLORS) == ["Stock", "BP Blue", "Blue", "Green", "Purple", "Orange", "Red", "Cyan", "Yellow"]
    assert normalize_model_path_color("blue") == "Blue"
    assert normalize_model_path_color("rainbow") == "Stock"
    assert normalize_model_path_color("12abef") == "#12ABEF"
    assert normalize_model_path_color(b"#abcdef") == "#ABCDEF"
    assert normalize_model_path_color("#000000") == "Stock"
    assert normalize_model_path_color("not-a-color") == "Stock"

  def test_resolves_preset_and_stock_colors(self):
    assert model_path_color_hex("Stock") is None
    assert model_path_color_hex("BP Blue") == BP_BLUE_EDGE_HEX
    assert model_path_color_rgb("BP Blue") == (72, 190, 241)
    assert model_path_color_rgb("Blue") == (0, 134, 233)
    assert model_path_color_rgb("#12ABEF") == (18, 171, 239)

  def test_bp_blue_preserves_ford_gradient(self):
    assert BP_BLUE_GRADIENT_RGBA == (
      (25, 135, 220, 170),
      (73, 196, 246, 145),
      (139, 226, 255, 0),
    )
    assert BP_BLUE_GRADIENT_STOPS == (0.0, 0.52, 1.0)

  def test_custom_label_and_visibility_floor(self):
    assert model_path_color_label("#12abef") == "Custom #12ABEF"
    assert not model_path_color_is_visible("#000000")
    floor_channel = round(MIN_VISIBLE_VALUE * 255)
    assert not model_path_color_is_visible(f"#{floor_channel - 1:02X}0000")
    assert model_path_color_is_visible(f"#{floor_channel:02X}0000")
    assert normalize_model_path_color(f"#{floor_channel:02X}0000") == f"#{floor_channel:02X}0000"
    assert model_path_color_is_visible("#0000FF")
    assert model_path_color_is_visible("#12ABEF")


if __name__ == "__main__":
  unittest.main()
