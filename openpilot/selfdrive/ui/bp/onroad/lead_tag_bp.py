"""
BluePilot: the lead tag, the radar overlay's form in the rendered world (comma 3X Minimal Driving View).

The camera view's radar overlay is three bordered boxes under a filled chevron. The rendered world already draws
the lead as a car on a blue (radar) or yellow (vision) pad, so there the same numbers go in one small translucent
tag beside the car: the distance large, then the other values on a smaller line. A bar on the tag's edge toward
the car carries the radar or vision color. The tag sits on the car's right, or on its left for a car clearly left
of the view's center (in the lane to your left), and moves across when that side has no room.
"""
from dataclasses import dataclass

import pyray as rl

from openpilot.system.ui.lib.text_measure import measure_text_cached

BIG_FONT, SMALL_FONT, MIN_FONT = 56, 42, 40  # px at overlay size 1.0; never below MIN_FONT (comma 3X legibility)
PAD = 14            # px inside the tag
ACCENT_W = 6        # px: the radar or vision bar on the edge toward the car
GAP = 22            # px between the car and the tag
LINE_GAP = 4        # px between the two text lines
MARGIN = 20         # px kept clear at the view's edges
TOP_CLEAR = 350     # px from the view's top kept for the speed and set speed (as ChevronMetricsBP's inverted mode)
BG = (10, 12, 16)
BG_ALPHA = 150
TEXT = (255, 255, 255)
SUBTEXT = (200, 205, 212)
DOT_GAP = 30        # px between the small line's values, with a dot in the middle (the UI font has no middle dot)
DOT_R = 4


@dataclass(frozen=True)
class LeadBox:
  """The lead car's extent on screen, its pad included (px)."""
  left: float
  top: float
  right: float
  bottom: float

  @property
  def center_x(self) -> float:
    return (self.left + self.right) / 2

  @property
  def center_y(self) -> float:
    return (self.top + self.bottom) / 2


def place_tag(box: LeadBox, w: float, h: float, rect: rl.Rectangle) -> tuple[float, float, bool]:
  """Top-left corner of a w by h tag beside the car, and whether it is on the car's right.

  The right side, unless the car is clearly left of the view's center (more than its own width), so a lead in your
  lane does not swap sides as it drifts across the center. When that side has no room it goes on the other; when
  neither has, it is kept on screen.
  Vertically it is centered on the car, kept below the speed and above the view's bottom edge.
  """
  lo_x, hi_x = rect.x + MARGIN, rect.x + rect.width - MARGIN
  right_x, left_x = box.right + GAP, box.left - GAP - w
  fits_right, fits_left = right_x + w <= hi_x, left_x >= lo_x
  prefer_right = box.right + (box.right - box.left) >= rect.x + rect.width / 2
  on_right = (fits_right or not fits_left) if prefer_right else (fits_right and not fits_left)
  x = right_x if on_right else left_x
  x = min(max(x, lo_x), hi_x - w)
  lo_y, hi_y = rect.y + TOP_CLEAR, rect.y + rect.height - MARGIN - h
  y = min(max(box.center_y - h / 2, lo_y), max(lo_y, hi_y))
  return x, y, on_right


def tag_lines(lines: list[str]) -> tuple[str, list[str]]:
  """The tag's large line (the first value, the distance) and the values on its small line (the rest)."""
  if not lines:
    return "", []
  return lines[0], list(lines[1:])


def draw_lead_tag(font, lines: list[str], box: LeadBox, rect: rl.Rectangle, accent: rl.Color, alpha: float,
                  scale: float = 1.0) -> None:
  big_text, small_parts = tag_lines(lines)
  if not big_text or alpha <= 0.0:
    return
  big_px, small_px = max(MIN_FONT, int(BIG_FONT * scale)), max(MIN_FONT, int(SMALL_FONT * scale))
  big = measure_text_cached(font, big_text, big_px, 0)
  smalls = [measure_text_cached(font, t, small_px, 0) for t in small_parts]
  small_w = sum(m.x for m in smalls) + DOT_GAP * max(0, len(smalls) - 1)
  small_h = max((m.y for m in smalls), default=0.0)
  text_w = max(big.x, small_w)
  text_h = big.y + (LINE_GAP + small_h if smalls else 0)
  w, h = text_w + 2 * PAD + ACCENT_W, text_h + 2 * PAD
  x, y, on_right = place_tag(box, w, h, rect)

  a = max(0.0, min(1.0, alpha))
  rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), 0.25, 8, rl.Color(*BG, int(BG_ALPHA * a)))
  # The bar faces the car: the tag's left edge when it is on the car's right, and the reverse
  bar_x = x if on_right else x + w - ACCENT_W
  bar = rl.Color(accent.r, accent.g, accent.b, int(255 * a))
  rl.draw_rectangle_rounded(rl.Rectangle(bar_x, y + PAD / 2, ACCENT_W, h - PAD), 1.0, 4, bar)

  text_x = x + PAD + (ACCENT_W if on_right else 0)
  rl.draw_text_ex(font, big_text, rl.Vector2(text_x, y + PAD), big_px, 0, rl.Color(*TEXT, int(255 * a)))
  sub = rl.Color(*SUBTEXT, int(255 * a))
  sx, sy = text_x, y + PAD + big.y + LINE_GAP
  for i, (t, m) in enumerate(zip(small_parts, smalls, strict=True)):
    if i:
      rl.draw_circle_v(rl.Vector2(sx - DOT_GAP / 2, sy + small_h / 2), DOT_R, sub)
    rl.draw_text_ex(font, t, rl.Vector2(sx, sy), small_px, 0, sub)
    sx += m.x + DOT_GAP
