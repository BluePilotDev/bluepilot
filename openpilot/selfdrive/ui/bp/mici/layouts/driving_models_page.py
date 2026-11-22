"""
BluePilot MICI: the Driving Models page, the comma four's counterpart of TICI's home card
(bluepilot/ui/widgets/model_info.py). A page of the main scroller, right of home, offroad only.

Two tiles side by side on the 536x240 screen: the big model on the left (JetLink's or a chestnut's, or an empty slot
saying how to add one) and the on-device model on the right, an arrow between them because the on-device model
drives whenever the big one cannot. Each tile has its role, the model's name, its state as a chip, the link's USB speed
as a badge, the one line that matters (why it cannot run, else the build's progress, else the link) and the build's
progress bar. The tile of the model that drives is outlined and tinted in its tone; the standby one is dimmed.

What it shows is driving_models.view (selfdrive/ui/bp/lib/driving_models.py), the same view TICI's card draws, read
by driving_models_reader.ViewReader. Tapping the page opens the Models settings panel.
"""
from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.ui.bp.lib import driving_models as dm
from openpilot.selfdrive.ui.bp.lib.driving_models_reader import ViewReader
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wrap_text import wrap_text
from openpilot.system.ui.widgets import Widget

# sizes on the 536x240 canvas: the name at stock MICI's model text size and up, nothing under the 22 px floor
PAD = 8
GAP = 28            # between the tiles: the fallback arrow
TILE_PAD = 14
ROLE_SIZE = 24
NAME_SIZE = 34      # one line
NAME_WRAP_SIZE = 30  # two lines, when one at NAME_SIZE does not fit
CHIP_SIZE = 26
CHIP_H = 38
CHIP_PAD_X = 12
LINE_SIZE = 24
LINE_GAP = 4
PROGRESS_H = 8
RADIUS = 0.12

ROLES = {dm.BIG: "big model", dm.SMALL: "on-device"}
NO_BIG_NAME = "not set up"
NO_BIG_HINT = "turn on jetlink in bluepilot settings"

TILE_BG = rl.Color(255, 255, 255, 18)
TILE_BORDER = rl.Color(255, 255, 255, 40)
EMPTY_BORDER = rl.Color(255, 255, 255, 50)
NAME_COLOR = rl.Color(255, 255, 255, 230)
DIM_COLOR = rl.Color(255, 255, 255, 120)
STANDBY_COLOR = rl.Color(255, 255, 255, 110)
ARROW_COLOR = rl.Color(255, 255, 255, 90)
PROGRESS_TRACK = rl.Color(255, 255, 255, 36)

TONE_COLORS = {
  dm.DRIVE: rl.Color(80, 211, 50, 255),
  dm.READY: rl.Color(24, 144, 255, 255),
  dm.WORK: rl.Color(255, 176, 32, 255),
  dm.ERROR: rl.Color(234, 70, 70, 255),
  dm.IDLE: rl.Color(170, 170, 170, 255),
  dm.PLAIN: rl.WHITE,
}

ELLIPSIS = "..."  # the UI fonts have no U+2026


def _tint(color: rl.Color, alpha: int) -> rl.Color:
  return rl.Color(color.r, color.g, color.b, alpha)


def _fit(font, text: str, size: int, width: float) -> str:
  """`text`, cut with an ellipsis to fit `width`."""
  if measure_text_cached(font, text, size).x <= width:
    return text
  while text and measure_text_cached(font, text + ELLIPSIS, size).x > width:
    text = text[:-1]
  return text.rstrip() + ELLIPSIS


def name_lines(font, name: str, width: float) -> tuple[int, list[str]]:
  """The model's name as drawn: (size, lines). One line at NAME_SIZE where it fits, else two at NAME_WRAP_SIZE, the
  second cut with an ellipsis. The name's date ("Name (November 04, 2026)") is left out: no room."""
  name = dm.short_name(name)
  if measure_text_cached(font, name, NAME_SIZE).x <= width:
    return NAME_SIZE, [name]
  lines = wrap_text(font, name, NAME_WRAP_SIZE, int(width))
  if len(lines) <= 2:
    return NAME_WRAP_SIZE, [_fit(font, line, NAME_WRAP_SIZE, width) for line in lines]
  return NAME_WRAP_SIZE, [_fit(font, lines[0], NAME_WRAP_SIZE, width), _fit(font, " ".join(lines[1:]), NAME_WRAP_SIZE, width)]


class DrivingModelsPageMici(Widget):
  def __init__(self):
    super().__init__()
    self._click_callback: Callable[[], None] | None = None
    self._reader = ViewReader()
    self._view = self._reader.view

  def set_click_callback(self, callback: Callable[[], None]):
    self._click_callback = callback

  def _update_state(self):
    self._view = self._reader.update()

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    if self._click_callback and rl.check_collision_point_rec(mouse_pos, self._rect):
      self._click_callback()

  def _render(self, rect: rl.Rectangle):
    tile_w = (rect.width - 2 * PAD - GAP) / 2
    tile_h = rect.height - 2 * PAD
    left = rl.Rectangle(rect.x + PAD, rect.y + PAD, tile_w, tile_h)
    right = rl.Rectangle(left.x + tile_w + GAP, left.y, tile_w, tile_h)

    if self._view.big is None:
      self._empty(left)
    else:
      self._tile(self._view.big, left)
    self._arrow(left.x + tile_w, rect.y + rect.height / 2)
    self._tile(self._view.small, right)

  @staticmethod
  def _arrow(x: float, cy: float):
    """From the big model to the on-device one, which drives when the big one cannot."""
    x0, x1, head = x + 6, x + GAP - 6, 9
    rl.draw_line_ex(rl.Vector2(x0, cy), rl.Vector2(x1 - 2, cy), 3, ARROW_COLOR)
    rl.draw_triangle(rl.Vector2(x1 - head, cy - head), rl.Vector2(x1 - head, cy + head), rl.Vector2(x1, cy), ARROW_COLOR)

  @staticmethod
  def _empty(rect: rl.Rectangle):
    """The slot where a big model would be."""
    rl.draw_rectangle_rounded_lines_ex(rect, RADIUS, 10, 2, EMPTY_BORDER)
    x, w = rect.x + TILE_PAD, rect.width - 2 * TILE_PAD
    role_font = gui_app.font(FontWeight.SEMI_BOLD)
    font = gui_app.font(FontWeight.ROMAN)
    y = rect.y + TILE_PAD
    rl.draw_text_ex(role_font, ROLES[dm.BIG], rl.Vector2(x, y), ROLE_SIZE, 0, DIM_COLOR)
    y += ROLE_SIZE + 8
    rl.draw_text_ex(role_font, NO_BIG_NAME, rl.Vector2(x, y), NAME_WRAP_SIZE, 0, DIM_COLOR)
    y += NAME_WRAP_SIZE + 10
    for line in wrap_text(font, NO_BIG_HINT, LINE_SIZE, int(w))[:3]:
      rl.draw_text_ex(font, line, rl.Vector2(x, y), LINE_SIZE, 0, DIM_COLOR)
      y += LINE_SIZE + 4

  @staticmethod
  def _chip(text: str, tone: rl.Color, x: float, y: float, max_w: float, outline: bool = False) -> float:
    """A rounded chip at (x, y); returns its width."""
    font = gui_app.font(FontWeight.BOLD)
    text = _fit(font, text, CHIP_SIZE, max_w - 2 * CHIP_PAD_X)
    size = measure_text_cached(font, text, CHIP_SIZE)
    w = size.x + 2 * CHIP_PAD_X
    chip = rl.Rectangle(x, y, w, CHIP_H)
    rl.draw_rectangle_rounded(chip, 1.0, 12, _tint(tone, 45))
    if outline:
      rl.draw_rectangle_rounded_lines_ex(chip, 1.0, 12, 2, _tint(tone, 150))
    rl.draw_text_ex(font, text, rl.Vector2(x + CHIP_PAD_X, y + (CHIP_H - size.y) / 2), CHIP_SIZE, 0, tone)
    return w

  def _tile(self, tile: dm.Tile, rect: rl.Rectangle):
    tone = TONE_COLORS[tile.tone]
    if tile.driving:
      rl.draw_rectangle_rounded(rect, RADIUS, 10, _tint(tone, 34))
      rl.draw_rectangle_rounded_lines_ex(rect, RADIUS, 10, 3, tone)
    else:
      rl.draw_rectangle_rounded(rect, RADIUS, 10, TILE_BG)
      border = _tint(tone, 120) if tile.role == dm.BIG and tile.tag else TILE_BORDER
      rl.draw_rectangle_rounded_lines_ex(rect, RADIUS, 10, 2, border)

    x, w = rect.x + TILE_PAD, rect.width - 2 * TILE_PAD
    role_font = gui_app.font(FontWeight.SEMI_BOLD)
    font = gui_app.font(FontWeight.ROMAN)
    y = rect.y + TILE_PAD

    rl.draw_text_ex(role_font, ROLES[tile.role], rl.Vector2(x, y), ROLE_SIZE, 0, DIM_COLOR)
    y += ROLE_SIZE + 6

    size, lines = name_lines(role_font, tile.name, w)
    for line in lines:
      rl.draw_text_ex(role_font, line, rl.Vector2(x, y), size, 0, STANDBY_COLOR if tile.standby else NAME_COLOR)
      y += size
    y += 8

    if tile.tag:
      chip_w = self._chip(tile.tag.lower(), tone, x, y, w)
      if tile.badge:
        badge_x = x + chip_w + 8
        if badge_x + measure_text_cached(gui_app.font(FontWeight.BOLD), tile.badge, CHIP_SIZE).x + 2 * CHIP_PAD_X <= x + w:
          self._chip(tile.badge, TONE_COLORS[tile.badge_tone], badge_x, y, x + w - badge_x, outline=True)
      y += CHIP_H + 8

    bottom = rect.y + rect.height - TILE_PAD
    bar_room = (PROGRESS_H + 8) if tile.progress is not None else 0
    # the detail wraps onto as many lines as the tile has left, the last cut with an ellipsis
    if (line := dm.compact_detail(tile)) is not None:
      text, line_tone = line
      color = DIM_COLOR if line_tone in (dm.IDLE, dm.PLAIN) else TONE_COLORS[line_tone]
      room = int((bottom - bar_room - y + LINE_GAP) // (LINE_SIZE + LINE_GAP))
      lines = wrap_text(font, text, LINE_SIZE, int(w))
      if room > 0 and lines:
        if len(lines) > room:
          lines = lines[:room - 1] + [_fit(font, " ".join(lines[room - 1:]), LINE_SIZE, w)]
        for text_line in lines:
          rl.draw_text_ex(font, text_line, rl.Vector2(x, y), LINE_SIZE, 0, color)
          y += LINE_SIZE + LINE_GAP

    if tile.progress is not None:
      bar = rl.Rectangle(x, bottom - PROGRESS_H, w, PROGRESS_H)
      rl.draw_rectangle_rounded(bar, 1.0, 8, PROGRESS_TRACK)
      fill = rl.Rectangle(bar.x, bar.y, max(bar.height, bar.width * min(max(tile.progress, 0.0), 1.0)), bar.height)
      rl.draw_rectangle_rounded(fill, 1.0, 8, tone)
