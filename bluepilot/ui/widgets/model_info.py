"""
BluePilot Driving Models card, on the offroad home screen.

Shows the on-device model and, when one is in play, the big model (JetLink's on a device on the USB-C port, or a
chestnut's): each as a tile with its name, its state, and for JetLink the connection, the link's USB speed and the
build's progress. The tile of the model that drives is outlined and tinted in its tone; a line shows the on-device
model as the big one's fallback. What it shows is
openpilot/selfdrive/ui/bp/lib/driving_models.py's view; this draws it, in whatever rect the home layout gives.
Clickable to open the Models settings panel.
"""

import re
from collections.abc import Callable

import pyray as rl

from openpilot.selfdrive.ui.bp.lib import driving_models
from openpilot.selfdrive.ui.bp.lib.driving_models_reader import ViewReader
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wrap_text import wrap_text
from openpilot.system.ui.widgets import Widget
from bluepilot.ui.lib.colors import BPColors


# Card styling (matching DriveStats). Text is 40 or larger: the comma 3X's 5" 2160x1080 screen.
CARD_BG = rl.Color(38, 38, 38, 255)
CARD_BORDER = rl.Color(255, 255, 255, 26)
TILE_BG = rl.Color(255, 255, 255, 13)
TILE_BORDER = rl.Color(255, 255, 255, 26)
NAME_COLOR = rl.WHITE
DIM_COLOR = rl.Color(150, 150, 150, 255)
MUTED_COLOR = rl.Color(170, 170, 170, 255)
LINE_COLOR = rl.Color(255, 255, 255, 60)
PROGRESS_TRACK = rl.Color(255, 255, 255, 30)

TONE_COLORS = {
  driving_models.DRIVE: rl.Color(80, 211, 50, 255),
  driving_models.READY: BPColors.ACCENT,
  driving_models.WORK: rl.Color(255, 176, 32, 255),
  driving_models.ERROR: rl.Color(234, 70, 70, 255),
  driving_models.IDLE: MUTED_COLOR,
  driving_models.PLAIN: rl.WHITE,
}

# "Name (November 04, 2026)": the date goes on a line of its own, smaller
DATE_PATTERN = re.compile(r'^(.*?)\s*\(([A-Za-z]+\s+\d{1,2},\s+\d{4})\)\s*$')

PAD = 32
TILE_PAD = 26
COMPACT_TILE_PAD = 16
TITLE_SIZE = 56
ROLE_SIZE = 40
CHIP_SIZE = 40
CHIP_PAD_X = 20
CHIP_H = 58
BADGE_SIZE = 40
NAME_SIZE = 52
NAME_MIN_SIZE = 42
INLINE_NAME_SIZE = 46
DATE_SIZE = 40
DATE_GAP = 10
LABEL_SIZE = 40
VALUE_SIZE = 44
NOTE_SIZE = 40
ROW_GAP = 10
ROW_H = 58  # a fact row: as tall as the speed badge
PROGRESS_H = 14
CONNECTOR_H = 76
COMPACT_GAP = 20
LINK_TEXT = "Falls back to"
NO_BIG_TITLE = "No big model"
NO_BIG_HINT = "Turn on JetLink in BluePilot settings to add one."

ELLIPSIS = "..."  # the UI fonts have no U+2026


def _fit(font, text: str, size: int, width: float) -> str:
  """`text`, cut with an ellipsis to fit `width`."""
  if measure_text_cached(font, text, size).x <= width:
    return text
  while text and measure_text_cached(font, text + ELLIPSIS, size).x > width:
    text = text[:-1]
  return text.rstrip() + ELLIPSIS


def _split_name(name: str) -> tuple[str, str]:
  """(the name, its date or "")."""
  m = DATE_PATTERN.match(name)
  return (m.group(1), m.group(2)) if m and m.group(1) else (name, "")


def _tint(color: rl.Color, alpha: int) -> rl.Color:
  return rl.Color(color.r, color.g, color.b, alpha)


class ModelInfoWidget(Widget):
  def __init__(self):
    super().__init__()
    self._click_callback: Callable[[], None] | None = None
    self._reader = ViewReader()
    self._view = self._reader.view

  def set_click_callback(self, callback: Callable[[], None]):
    self._click_callback = callback

  @property
  def has_big_model(self) -> bool:
    return self._view.big is not None

  def _update_state(self):
    self._view = self._reader.update()

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    if self._click_callback and rl.check_collision_point_rec(mouse_pos, self._rect):
      self._click_callback()

  # -- drawing ------------------------------------------------------------------
  # Each block is laid out by one function that measures (draw=False) or draws, so the card can pick its compact
  # form (no dates, no source row, one note, a plain gap between the tiles) before it draws anything.

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rounded(rect, 0.04, 10, CARD_BG)
    rl.draw_rectangle_rounded_lines(rect, 0.04, 10, CARD_BORDER)

    x, w = rect.x + PAD, rect.width - 2 * PAD
    top = self._draw_header(x, rect.y + PAD, w)
    avail = rect.y + rect.height - PAD - top
    compact = self._body(x, top, w, compact=False, draw=False) > avail

    rl.begin_scissor_mode(int(rect.x), int(top), int(rect.width), int(max(avail, 0)))
    self._body(x, top, w, compact=compact, draw=True)
    rl.end_scissor_mode()

  def _draw_header(self, x: float, y: float, w: float) -> float:
    font = gui_app.font(FontWeight.BOLD)
    size = measure_text_cached(font, "Driving Models", TITLE_SIZE)
    rl.draw_text_ex(font, "Driving Models", rl.Vector2(x, y), TITLE_SIZE, 0, rl.WHITE)
    # a chevron: the card opens the Models settings
    cx, cy, half = x + w - 8, y + size.y / 2, 14
    rl.draw_line_ex(rl.Vector2(cx - half, cy - half * 1.4), rl.Vector2(cx, cy), 5, DIM_COLOR)
    rl.draw_line_ex(rl.Vector2(cx, cy), rl.Vector2(cx - half, cy + half * 1.4), 5, DIM_COLOR)
    return y + size.y + 24

  def _body(self, x: float, y: float, w: float, compact: bool, draw: bool) -> float:
    """The tiles from y down; returns their height."""
    view = self._view
    start = y
    if view.big is None:
      y += self._tile(view.small, x, y, w, compact, draw)
      if not compact:  # the empty slot only where there is room for its hint
        y += COMPACT_GAP + self._no_big(x, y + COMPACT_GAP, w, draw)
      return y - start
    y += self._tile(view.big, x, y, w, compact, draw)
    y += self._connector(x, y, w, compact, draw)
    y += self._tile(view.small, x, y, w, compact, draw)
    return y - start

  def _connector(self, x: float, y: float, w: float, compact: bool, draw: bool) -> float:
    """The line from the big model down to the on-device model, which drives when the big one cannot."""
    if compact:
      return COMPACT_GAP
    if draw:
      lx = x + 44
      rl.draw_line_ex(rl.Vector2(lx, y + 6), rl.Vector2(lx, y + CONNECTOR_H - 18), 4, LINE_COLOR)
      rl.draw_triangle(rl.Vector2(lx - 12, y + CONNECTOR_H - 20), rl.Vector2(lx, y + CONNECTOR_H - 6),
                       rl.Vector2(lx + 12, y + CONNECTOR_H - 20), LINE_COLOR)
      font = gui_app.font(FontWeight.NORMAL)
      size = measure_text_cached(font, LINK_TEXT, NOTE_SIZE)
      rl.draw_text_ex(font, LINK_TEXT, rl.Vector2(lx + 32, y + (CONNECTOR_H - size.y) / 2), NOTE_SIZE, 0, DIM_COLOR)
    return CONNECTOR_H

  def _no_big(self, x: float, y: float, w: float, draw: bool) -> float:
    """The empty slot where a big model would be."""
    title_font = gui_app.font(FontWeight.SEMI_BOLD)
    font = gui_app.font(FontWeight.NORMAL)
    inner = w - 2 * TILE_PAD
    lines = wrap_text(font, NO_BIG_HINT, NOTE_SIZE, int(inner))
    h = TILE_PAD + VALUE_SIZE + sum(ROW_GAP + NOTE_SIZE for _ in lines) + TILE_PAD
    if draw:
      rect = rl.Rectangle(x, y, w, h)
      rl.draw_rectangle_rounded_lines_ex(rect, 0.12, 10, 2, TILE_BORDER)
      ty = y + TILE_PAD
      rl.draw_text_ex(title_font, NO_BIG_TITLE, rl.Vector2(x + TILE_PAD, ty), VALUE_SIZE, 0, MUTED_COLOR)
      ty += VALUE_SIZE
      for line in lines:
        ty += ROW_GAP
        rl.draw_text_ex(font, line, rl.Vector2(x + TILE_PAD, ty), NOTE_SIZE, 0, DIM_COLOR)
        ty += NOTE_SIZE
    return h

  @staticmethod
  def _badge(tile, right: float, y: float) -> float:
    """The link's USB speed, right-aligned to `right` on a fact row at `y`; returns the room it takes."""
    if not tile.badge:
      return 0.0
    font = gui_app.font(FontWeight.BOLD)
    color = TONE_COLORS[tile.badge_tone]
    text = measure_text_cached(font, tile.badge, BADGE_SIZE)
    w = text.x + 2 * CHIP_PAD_X
    rect = rl.Rectangle(right - w, y + (VALUE_SIZE - CHIP_H) / 2, w, CHIP_H)
    rl.draw_rectangle_rounded(rect, 1.0, 12, _tint(color, 45))
    rl.draw_rectangle_rounded_lines_ex(rect, 1.0, 12, 2, _tint(color, 150))
    rl.draw_text_ex(font, tile.badge, rl.Vector2(rect.x + CHIP_PAD_X, rect.y + (CHIP_H - text.y) / 2), BADGE_SIZE, 0, color)
    return w + 16

  def _tile(self, tile, x: float, y: float, w: float, compact: bool, draw: bool) -> float:
    tone = TONE_COLORS[tile.tone]
    pad = COMPACT_TILE_PAD if compact else TILE_PAD
    inner_x, inner_w = x + TILE_PAD, w - 2 * TILE_PAD
    role_font = gui_app.font(FontWeight.SEMI_BOLD)
    name_font = gui_app.font(FontWeight.SEMI_BOLD)
    text_font = gui_app.font(FontWeight.NORMAL)
    chip_font = gui_app.font(FontWeight.BOLD)

    name, date = _split_name(tile.name)
    name_size = NAME_SIZE
    while name_size > NAME_MIN_SIZE and measure_text_cached(name_font, name, name_size).x > inner_w:
      name_size -= 2
    facts = [f for f in tile.facts if not (compact and f.label == "Source")]
    # compact keeps only why it cannot run: the chip carries the progress's percentage
    notes = [n for n in tile.notes if n.tone == driving_models.ERROR][:1] if compact else tile.notes
    label_w = max((measure_text_cached(text_font, f.label, LABEL_SIZE).x for f in facts), default=0) + 28

    # compact, the on-device model's name takes its role's place: one row with the chip
    inline = compact and tile.role == driving_models.SMALL

    # heights, top to bottom
    h = pad + CHIP_H + (0 if inline else 10 + name_size)
    if date and not compact:
      h += DATE_GAP + DATE_SIZE
    if facts:
      h += 14 + len(facts) * (ROW_H + ROW_GAP) - ROW_GAP
    if notes:
      h += 14 + len(notes) * (NOTE_SIZE + ROW_GAP) - ROW_GAP
    if tile.progress is not None:
      h += 18 + PROGRESS_H
    h += pad
    if not draw:
      return h

    rect = rl.Rectangle(x, y, w, h)
    if tile.driving:
      rl.draw_rectangle_rounded(rect, 0.1, 10, _tint(tone, 30))
      rl.draw_rectangle_rounded_lines_ex(rect, 0.1, 10, 3, tone)
    else:
      rl.draw_rectangle_rounded(rect, 0.1, 10, TILE_BG)
      border = _tint(tone, 110) if tile.role == driving_models.BIG and tile.tag else TILE_BORDER
      rl.draw_rectangle_rounded_lines_ex(rect, 0.1, 10, 2, border)

    # the role on the left, the state as a chip on the right
    cy = y + pad
    chip_w = 0.0
    if tile.tag:
      tag = _fit(chip_font, tile.tag, CHIP_SIZE, inner_w * 0.6)
      text = measure_text_cached(chip_font, tag, CHIP_SIZE)
      chip_w = text.x + 2 * CHIP_PAD_X
      chip = rl.Rectangle(inner_x + inner_w - chip_w, cy, chip_w, CHIP_H)
      rl.draw_rectangle_rounded(chip, 1.0, 12, _tint(tone, 45))
      rl.draw_text_ex(chip_font, tag, rl.Vector2(chip.x + CHIP_PAD_X, cy + (CHIP_H - text.y) / 2), CHIP_SIZE, 0, tone)
    name_color = DIM_COLOR if tile.standby else NAME_COLOR
    if inline:
      text = _fit(name_font, name, INLINE_NAME_SIZE, inner_w - chip_w - 20)
      text_h = measure_text_cached(name_font, text, INLINE_NAME_SIZE).y
      rl.draw_text_ex(name_font, text, rl.Vector2(inner_x, cy + (CHIP_H - text_h) / 2), INLINE_NAME_SIZE, 0, name_color)
      return h
    role = _fit(role_font, tile.role, ROLE_SIZE, inner_w - chip_w - 20)
    role_h = measure_text_cached(role_font, role, ROLE_SIZE).y
    rl.draw_text_ex(role_font, role, rl.Vector2(inner_x, cy + (CHIP_H - role_h) / 2), ROLE_SIZE, 0, DIM_COLOR)
    cy += CHIP_H + 10

    rl.draw_text_ex(name_font, _fit(name_font, name, name_size, inner_w), rl.Vector2(inner_x, cy), name_size, 0, name_color)
    cy += name_size
    if date and not compact:
      cy += DATE_GAP
      rl.draw_text_ex(text_font, date, rl.Vector2(inner_x, cy), DATE_SIZE, 0, DIM_COLOR)
      cy += DATE_SIZE

    if facts:
      cy += 14
      for i, fact in enumerate(facts):
        ty = cy + (ROW_H - VALUE_SIZE) / 2  # the value's top, centred in the row
        rl.draw_text_ex(text_font, fact.label, rl.Vector2(inner_x, ty + (VALUE_SIZE - LABEL_SIZE) / 2), LABEL_SIZE, 0,
                        DIM_COLOR)
        badge_w = self._badge(tile, inner_x + inner_w, ty) if i == len(facts) - 1 else 0.0
        value = _fit(role_font, fact.value, VALUE_SIZE, inner_w - label_w - badge_w)
        rl.draw_text_ex(role_font, value, rl.Vector2(inner_x + label_w, ty), VALUE_SIZE, 0, TONE_COLORS[fact.tone])
        cy += ROW_H + ROW_GAP
      cy -= ROW_GAP

    if notes:
      cy += 14
      for note in notes:
        rl.draw_text_ex(text_font, _fit(text_font, note.text, NOTE_SIZE, inner_w), rl.Vector2(inner_x, cy), NOTE_SIZE, 0,
                        TONE_COLORS[note.tone])
        cy += NOTE_SIZE + ROW_GAP
      cy -= ROW_GAP

    if tile.progress is not None:
      bar = rl.Rectangle(inner_x, cy + 18, inner_w, PROGRESS_H)
      rl.draw_rectangle_rounded(bar, 1.0, 8, PROGRESS_TRACK)
      fill = rl.Rectangle(bar.x, bar.y, max(bar.height, bar.width * min(max(tile.progress, 0.0), 1.0)), bar.height)
      rl.draw_rectangle_rounded(fill, 1.0, 8, tone)
    return h
