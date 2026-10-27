"""Compact C3X onroad network-strength indicator for BluePilot."""

import pyray as rl


NETWORK_BAR_COUNT = 5
FORD_BLUE = rl.Color(25, 135, 220, 245)
FORD_BLUE_BRIGHT = rl.Color(72, 190, 241, 245)


def network_bar_count(network_type_raw: int, network_strength_raw: int) -> int:
  """Map deviceState network enums to a stable 0-5 bar count."""
  if network_type_raw == 0:  # DeviceState.NetworkType.none
    return 0
  if network_type_raw == 6:  # Ethernet has no radio-strength reading.
    return NETWORK_BAR_COUNT
  # NetworkStrength has four known buckets. Spread them across five visible bars,
  # reserving an empty glyph for the separate `unknown` state.
  if network_strength_raw <= 0:
    return 0
  if network_strength_raw >= 4:
    return NETWORK_BAR_COUNT
  return {1: 1, 2: 2, 3: 4}[network_strength_raw]


def network_mode_label(network_type_raw: int, technology: str = "") -> str:
  """Return the live transport/data mode shown beside the strength bars."""
  if network_type_raw == 0:
    return "OFFLINE"
  if network_type_raw == 1:
    return "WiFi"
  if network_type_raw == 6:
    return "ETH"

  cleaned = technology.strip().replace("_", "-")
  if cleaned:
    normalized = cleaned.lower()
    aliases = {
      "lte": "LTE",
      "lte-a": "LTE-A",
      "nr": "5G",
      "nr5g": "5G",
      "5g-nsa": "5G NSA",
      "5g-sa": "5G SA",
      "hspa+": "HSPA+",
    }
    return aliases.get(normalized, cleaned.upper())[:7]

  return {2: "2G", 3: "3G", 4: "4G", 5: "5G"}.get(network_type_raw, "DATA")


def draw_network_status(rect: rl.Rectangle, bars: int, label: str, font: rl.Font) -> None:
  """Draw a five-bar network glyph and live mode in a bordered black pill."""
  bars = max(0, min(NETWORK_BAR_COUNT, bars))
  rl.draw_rectangle_rounded(rect, 0.25, 6, rl.Color(0, 0, 0, 210))
  rl.draw_rectangle_rounded_lines_ex(rect, 0.25, 6, 3, FORD_BLUE)

  bar_width = 10
  gap = 5
  heights = (7, 13, 19, 26, 33)
  start_x = rect.x + 12
  baseline = rect.y + rect.height - 10
  active = FORD_BLUE_BRIGHT
  inactive = rl.Color(112, 122, 132, 155)

  for index, height in enumerate(heights):
    color = active if index < bars else inactive
    rl.draw_rectangle_rounded(
      rl.Rectangle(start_x + index * (bar_width + gap), baseline - height, bar_width, height),
      0.28, 3, color,
    )

  font_size = 24 if len(label) > 5 else 28
  # This tiny label is measured only when the optional network pill is drawn.
  # Avoid importing the global emoji/font fallback cache into the pure mapping tests.
  text_size = rl.measure_text_ex(font, label, font_size, 0)  # noqa: TID251
  text_x = rect.x + 94 + (rect.width - 104 - text_size.x) / 2
  text_y = rect.y + (rect.height - text_size.y) / 2
  rl.draw_text_ex(font, label, rl.Vector2(text_x, text_y), font_size, 0, rl.WHITE)
