"""BluePilot model-path color presets, validation, and gradient helpers."""

from __future__ import annotations

import re

PARAM_KEY = "CustomModelPathColor"
STOCK_VALUE = "Stock"
BP_BLUE_VALUE = "BP Blue"
BP_BLUE_EDGE_HEX = "#48BEF1"
BP_BLUE_GRADIENT_RGBA = (
  (25, 135, 220, 170),
  (73, 196, 246, 145),
  (139, 226, 255, 0),
)
BP_BLUE_GRADIENT_STOPS = (0.0, 0.52, 1.0)
_LEGACY_RAINBOW_VALUE = "Rainbow"

# Keep the historical BP choices while also accepting a literal #RRGGBB custom value.
PRESET_COLORS: dict[str, str | None] = {
  STOCK_VALUE: None,
  BP_BLUE_VALUE: BP_BLUE_EDGE_HEX,
  "Blue": "#0086E9",
  "Green": "#0DF87A",
  "Purple": "#A855F7",
  "Orange": "#FF8A00",
  "Red": "#FF334B",
  "Cyan": "#00D8FF",
  "Yellow": "#FFE044",
}

_HEX_COLOR_RE = re.compile(r"^#?([0-9a-fA-F]{6})$")


def normalize_model_path_color(value: str | bytes | None) -> str:
  """Return a canonical preset name or uppercase #RRGGBB value."""
  if isinstance(value, bytes):
    value = value.decode("utf-8", errors="replace")
  value = (value or STOCK_VALUE).strip()

  preset = next((name for name in PRESET_COLORS if name.lower() == value.lower()), None)
  if preset is not None:
    return preset

  # The picker previously exposed Rainbow; migrate that stored value to its BP Blue replacement.
  if value.lower() == _LEGACY_RAINBOW_VALUE.lower():
    return BP_BLUE_VALUE

  if match := _HEX_COLOR_RE.fullmatch(value):
    return f"#{match.group(1).upper()}"

  return STOCK_VALUE


def model_path_color_hex(value: str | bytes | None) -> str | None:
  """Resolve a preset/custom value to #RRGGBB, or None for stock rendering."""
  normalized = normalize_model_path_color(value)
  return PRESET_COLORS.get(normalized, normalized if normalized.startswith("#") else None)


def model_path_color_rgb(value: str | bytes | None) -> tuple[int, int, int] | None:
  color_hex = model_path_color_hex(value)
  if color_hex is None:
    return None
  return tuple(int(color_hex[i:i + 2], 16) for i in (1, 3, 5))


def model_path_color_label(value: str | bytes | None) -> str:
  normalized = normalize_model_path_color(value)
  return f"Custom {normalized}" if normalized.startswith("#") else normalized


def model_path_color_is_visible(value: str | bytes | None) -> bool:
  """Reject colors dark enough to make the path effectively disappear."""
  rgb = model_path_color_rgb(value)
  if rgb is None:
    return True
  # Allow saturated hues (including pure blue) while rejecting near-black values.
  return max(rgb) >= 60 and sum(rgb) >= 100


def model_path_gradient(value: str | bytes | None):
  """Build the cached BP path gradient for a named or literal custom color."""
  normalized = normalize_model_path_color(value)
  rgb = None if normalized == BP_BLUE_VALUE else model_path_color_rgb(normalized)
  if normalized != BP_BLUE_VALUE and rgb is None:
    return None

  # Keep the parser importable in non-UI tooling where raylib is unavailable.
  import pyray as rl
  from openpilot.system.ui.lib.shader_polygon import Gradient

  if normalized == BP_BLUE_VALUE:
    return Gradient(
      start=(0.0, 1.0),
      end=(0.0, 0.0),
      colors=[rl.Color(*rgba) for rgba in BP_BLUE_GRADIENT_RGBA],
      stops=list(BP_BLUE_GRADIENT_STOPS),
    )

  assert rgb is not None
  r, g, b = rgb
  # A slightly brighter near edge improves depth perception while preserving hue.
  edge = tuple(round(channel + (255 - channel) * 0.18) for channel in rgb)
  return Gradient(
    start=(0.0, 1.0),
    end=(0.0, 0.0),
    colors=[rl.Color(*edge, 190), rl.Color(r, g, b, 150), rl.Color(r, g, b, 30)],
    stops=[0.0, 0.5, 1.0],
  )
