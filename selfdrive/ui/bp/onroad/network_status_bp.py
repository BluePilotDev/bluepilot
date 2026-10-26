"""Compact C3X onroad network-strength indicator for BluePilot."""

import pyray as rl


NETWORK_BAR_COUNT = 4


def network_bar_count(network_type_raw: int, network_strength_raw: int) -> int:
  """Map deviceState network enums to a stable 0-4 bar count."""
  if network_type_raw == 0:  # DeviceState.NetworkType.none
    return 0
  return max(0, min(NETWORK_BAR_COUNT, network_strength_raw))


def draw_network_status(rect: rl.Rectangle, bars: int) -> None:
  """Draw a small, high-contrast cellular-style strength glyph."""
  bars = max(0, min(NETWORK_BAR_COUNT, bars))
  rl.draw_rectangle_rounded(rect, 0.28, 5, rl.Color(0, 0, 0, 155))

  bar_width = 9
  gap = 5
  heights = (7, 12, 18, 25)
  total_width = NETWORK_BAR_COUNT * bar_width + (NETWORK_BAR_COUNT - 1) * gap
  start_x = rect.x + (rect.width - total_width) / 2
  baseline = rect.y + rect.height - 7
  active = rl.Color(102, 190, 255, 245)
  inactive = rl.Color(112, 122, 132, 155)

  for index, height in enumerate(heights):
    color = active if index < bars else inactive
    rl.draw_rectangle_rounded(
      rl.Rectangle(start_x + index * (bar_width + gap), baseline - height, bar_width, height),
      0.28, 3, color,
    )
