from pathlib import Path

from openpilot.selfdrive.ui.bp.lib.model_path_color import (
  BP_BLUE_EDGE_HEX,
  BP_BLUE_GRADIENT_RGBA,
  BP_BLUE_GRADIENT_STOPS,
  BP_BLUE_VALUE,
  PARAM_KEY,
  PRESET_COLORS,
  model_path_color_hex,
  model_path_color_is_visible,
  model_path_color_label,
  model_path_color_rgb,
  normalize_model_path_color,
)

PARAMS_KEYS_H = Path(__file__).resolve().parents[4] / "common" / "params_keys.h"
BP_SETTINGS = Path(__file__).resolve().parents[1] / "layouts" / "settings" / "bluepilot.py"
SP_VISUALS = Path(__file__).resolve().parents[2] / "sunnypilot" / "layouts" / "settings" / "visuals.py"
TICI_RENDERER = Path(__file__).resolve().parents[1] / "onroad" / "model_renderer_bp.py"
MICI_RENDERER = Path(__file__).resolve().parents[1] / "mici" / "onroad" / "model_renderer_bp.py"


def test_param_is_registered_with_legacy_name_and_default():
  registration = f'{{"{PARAM_KEY}", {{PERSISTENT | BACKUP, STRING, "Stock"}}}}'
  assert registration in PARAMS_KEYS_H.read_text()


def test_selector_stays_in_bluepilot_visuals():
  bp_settings = BP_SETTINGS.read_text()
  sp_visuals = SP_VISUALS.read_text()

  assert "self._model_path_color," in bp_settings
  assert "Enable Tesla Rainbow Mode SunnyPilot Toggle and active theme packs" in bp_settings
  assert "ModelPathColorDialog" not in sp_visuals


def test_both_renderers_reload_remote_param_changes():
  for renderer in (TICI_RENDERER, MICI_RENDERER):
    source = renderer.read_text()
    assert "self._counter % 60 == 0" in source
    assert "self._bp_params.get(MODEL_PATH_COLOR_PARAM, return_default=True)" in source
    assert "if model_path_color != self._model_path_color:" in source
    assert "self._custom_path_gradient = model_path_gradient(self._model_path_color)" in source


def test_normalizes_presets_and_custom_hex():
  assert list(PRESET_COLORS) == ["Stock", "BP Blue", "Blue", "Green", "Purple", "Orange", "Red", "Cyan", "Yellow"]
  assert normalize_model_path_color("blue") == "Blue"
  assert normalize_model_path_color("rainbow") == BP_BLUE_VALUE
  assert normalize_model_path_color("12abef") == "#12ABEF"
  assert normalize_model_path_color(b"#abcdef") == "#ABCDEF"
  assert normalize_model_path_color("not-a-color") == "Stock"


def test_resolves_preset_and_stock_colors():
  assert model_path_color_hex("Stock") is None
  assert model_path_color_hex("BP Blue") == BP_BLUE_EDGE_HEX
  assert model_path_color_rgb("BP Blue") == (72, 190, 241)
  assert model_path_color_rgb("Blue") == (0, 134, 233)
  assert model_path_color_rgb("#12ABEF") == (18, 171, 239)


def test_bp_blue_preserves_ford_gradient():
  assert BP_BLUE_GRADIENT_RGBA == (
    (25, 135, 220, 170),
    (73, 196, 246, 145),
    (139, 226, 255, 0),
  )
  assert BP_BLUE_GRADIENT_STOPS == (0.0, 0.52, 1.0)


def test_custom_label_and_visibility_floor():
  assert model_path_color_label("#12abef") == "Custom #12ABEF"
  assert not model_path_color_is_visible("#000000")
  assert model_path_color_is_visible("#0000FF")
  assert model_path_color_is_visible("#12ABEF")
