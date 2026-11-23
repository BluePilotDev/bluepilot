from openpilot.selfdrive.ui.bp.lib.model_path_color import (
  PRESET_COLORS,
  model_path_color_hex,
  model_path_color_is_visible,
  model_path_color_is_rainbow,
  model_path_color_label,
  model_path_color_rgb,
  normalize_model_path_color,
)


def test_normalizes_presets_and_custom_hex():
  assert list(PRESET_COLORS) == ["Stock", "Rainbow", "Blue", "Green", "Purple", "Orange", "Red", "Cyan", "Yellow"]
  assert normalize_model_path_color("blue") == "Blue"
  assert normalize_model_path_color("12abef") == "#12ABEF"
  assert normalize_model_path_color(b"#abcdef") == "#ABCDEF"
  assert normalize_model_path_color("not-a-color") == "Stock"


def test_resolves_preset_and_stock_colors():
  assert model_path_color_hex("Stock") is None
  assert model_path_color_hex("Rainbow") is None
  assert model_path_color_is_rainbow("rainbow")
  assert model_path_color_rgb("Blue") == (0, 134, 233)
  assert model_path_color_rgb("#12ABEF") == (18, 171, 239)


def test_custom_label_and_visibility_floor():
  assert model_path_color_label("#12abef") == "Custom #12ABEF"
  assert not model_path_color_is_visible("#000000")
  assert model_path_color_is_visible("#0000FF")
  assert model_path_color_is_visible("#12ABEF")
