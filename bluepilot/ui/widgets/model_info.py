"""
BluePilot Driving Models card, on the offroad home screen.

Shows the on-device model and, when one is in play, the big model (JetLink's on a device on the USB-C port, or a
chestnut's): each as a tile with its name, its state, and for JetLink the connection, the link's USB speed and the
build's progress. The tile of the model that drives is outlined in its tone. What it shows is
openpilot/selfdrive/ui/bp/lib/driving_models.py's view; this draws it, in whatever rect the home layout gives.
Clickable to open the Models settings panel.
"""

import re
import time
from collections.abc import Callable

import pyray as rl

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.lib import driving_models, jetlink_status
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wrap_text import wrap_text
from openpilot.system.ui.widgets import Widget
from bluepilot.ui.lib.colors import BPColors


# Card styling (matching DriveStats)
CARD_BG = rl.Color(38, 38, 38, 255)
CARD_BORDER = rl.Color(255, 255, 255, 26)
TILE_BG = rl.Color(255, 255, 255, 13)
TILE_BORDER = rl.Color(255, 255, 255, 26)
MODEL_NAME_COLOR = BPColors.ACCENT  # #18b4ff
ROLE_COLOR = rl.Color(160, 160, 160, 255)
DETAIL_COLOR = rl.Color(200, 200, 200, 255)
HINT_COLOR = rl.Color(140, 140, 140, 255)
PROGRESS_TRACK = rl.Color(255, 255, 255, 30)

TONE_COLORS = {
  driving_models.DRIVE: rl.Color(80, 211, 50, 255),
  driving_models.READY: BPColors.ACCENT,
  driving_models.WORK: rl.Color(255, 176, 32, 255),
  driving_models.ERROR: rl.Color(234, 70, 70, 255),
  driving_models.IDLE: rl.Color(150, 150, 150, 255),
}

# Regex to insert newline before date patterns like "(October 03, 2023)"
DATE_PATTERN = re.compile(r'\s+(\([A-Za-z]+\s+\d{1,2},\s+\d{4}\))')

PAD = 28
TILE_PAD = 22
TILE_GAP = 20
TITLE_SIZE = 48
PILL_SIZE = 34
ROLE_SIZE = 32
TAG_SIZE = 34
NAME_SIZE = 46
NAME_MIN_SIZE = 32
DETAIL_SIZE = 34
ACCENT_BAR = 8
PROGRESS_H = 12
NO_BIG_HINT = "Big model: turn on JetLink in BluePilot settings."

# the chestnut pick's name: a params read, once a second at most
CHESTNUT_NAME_PERIOD_S = 1.0


ELLIPSIS = "..."  # the UI fonts have no U+2026


def _fit(font, text: str, size: int, width: float) -> str:
  """`text`, cut with an ellipsis to fit `width`."""
  if measure_text_cached(font, text, size).x <= width:
    return text
  while text and measure_text_cached(font, text + ELLIPSIS, size).x > width:
    text = text[:-1]
  return text.rstrip() + ELLIPSIS


class ModelInfoWidget(Widget):
  def __init__(self):
    super().__init__()
    self._click_callback: Callable[[], None] | None = None
    self._params = Params()
    self._reader = jetlink_status.StatusReader(speed=True)
    self._chestnut_name: str | None = None
    self._chestnut_name_at = -float('inf')
    self._view = driving_models.view(None)

  def set_click_callback(self, callback: Callable[[], None]):
    self._click_callback = callback

  @property
  def has_big_model(self) -> bool:
    return self._view.big is not None

  def _update_state(self):
    status = self._reader.get()
    chestnut_state = getattr(ui_state.chestnut_state, 'value', 'disconnected')
    if not jetlink_status.shown(status) and chestnut_state != 'disconnected':
      self._refresh_chestnut_name()
    self._view = driving_models.view(self._get_active_model_name(), status, self._reader.speed(), chestnut_state,
                                     self._chestnut_name)

  def _refresh_chestnut_name(self):
    now = time.monotonic()
    if now - self._chestnut_name_at < CHESTNUT_NAME_PERIOD_S:
      return
    self._chestnut_name_at = now
    try:
      from openpilot.sunnypilot.models.helpers import get_selected_bundle
      bundle = get_selected_bundle(self._params, "chestnut")
      self._chestnut_name = bundle.displayName if bundle is not None and bundle.displayName else None
    except Exception:
      self._chestnut_name = None

  @staticmethod
  def _get_active_model_name() -> str | None:
    try:
      model_manager = ui_state.sm["modelManagerSP"]
      if model_manager.activeBundle.ref:
        return model_manager.activeBundle.displayName
    except Exception:
      pass
    return None

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    if self._click_callback and rl.check_collision_point_rec(mouse_pos, self._rect):
      self._click_callback()

  # -- drawing ------------------------------------------------------------------

  def _render(self, rect: rl.Rectangle):
    rl.draw_rectangle_rounded(rect, 0.04, 10, CARD_BG)
    rl.draw_rectangle_rounded_lines(rect, 0.04, 10, CARD_BORDER)

    inner_x, inner_w = rect.x + PAD, rect.width - 2 * PAD
    y = rect.y + PAD
    y = self._draw_header(inner_x, y, inner_w)

    tiles = [t for t in (self._view.big, self._view.small) if t is not None]
    bottom = rect.y + rect.height - PAD
    for tile in tiles:
      h = self._tile_height(tile, inner_w)
      h = min(h, bottom - y)
      if h <= TILE_PAD * 2:
        break
      self._draw_tile(tile, rl.Rectangle(inner_x, y, inner_w, h))
      y += h + TILE_GAP

    if self._view.big is None:
      font = gui_app.font(FontWeight.NORMAL)
      for line in wrap_text(font, NO_BIG_HINT, DETAIL_SIZE, int(inner_w)):
        if y + DETAIL_SIZE > bottom:
          break
        rl.draw_text_ex(font, line, rl.Vector2(inner_x, y), DETAIL_SIZE, 0, HINT_COLOR)
        y += DETAIL_SIZE + 10

  def _draw_header(self, x: float, y: float, w: float) -> float:
    title_font = gui_app.font(FontWeight.BOLD)
    title = "Driving Models" if self._view.big is not None else "Driving Model"
    title_size = measure_text_cached(title_font, title, TITLE_SIZE)
    rl.draw_text_ex(title_font, title, rl.Vector2(x, y), TITLE_SIZE, 0, rl.WHITE)

    bottom = y + title_size.y
    if self._view.pill:
      pill_font = gui_app.font(FontWeight.SEMI_BOLD)
      color = TONE_COLORS[self._view.pill_tone]
      text_size = measure_text_cached(pill_font, self._view.pill, PILL_SIZE)
      pill_h = PILL_SIZE + 18
      pill_w = text_size.x + 40
      if pill_w <= w - title_size.x - 24:
        pill = rl.Rectangle(x + w - pill_w, y + (title_size.y - pill_h) / 2, pill_w, pill_h)
        text = self._view.pill
      else:
        text = _fit(pill_font, self._view.pill, PILL_SIZE, w - 40)
        pill_w = measure_text_cached(pill_font, text, PILL_SIZE).x + 40
        pill = rl.Rectangle(x, bottom + 14, pill_w, pill_h)
        bottom = pill.y + pill_h
      rl.draw_rectangle_rounded(pill, 1.0, 12, rl.Color(color.r, color.g, color.b, 45))
      rl.draw_rectangle_rounded_lines(pill, 1.0, 12, rl.Color(color.r, color.g, color.b, 160))
      rl.draw_text_ex(pill_font, text, rl.Vector2(pill.x + 20, pill.y + (pill_h - text_size.y) / 2), PILL_SIZE, 0, color)
    return bottom + 20

  @staticmethod
  def _name_lines(tile) -> list[str]:
    return [line.strip() for line in DATE_PATTERN.sub(r'\n\1', tile.name).split('\n')]

  def _name_size(self, lines: list[str], width: float) -> int:
    font = gui_app.font(FontWeight.SEMI_BOLD)
    size = NAME_SIZE
    while size > NAME_MIN_SIZE and any(measure_text_cached(font, line, size).x > width for line in lines):
      size -= 2
    return size

  def _tile_height(self, tile, width: float) -> float:
    lines = self._name_lines(tile)
    size = self._name_size(lines, width - 2 * TILE_PAD - ACCENT_BAR)
    h = TILE_PAD + ROLE_SIZE + 10 + len(lines) * size * 1.15
    if tile.details:
      h += 8 + len(tile.details) * (DETAIL_SIZE + 10)
    if tile.progress is not None:
      h += 12 + PROGRESS_H
    return h + TILE_PAD

  def _draw_tile(self, tile, rect: rl.Rectangle):
    tone = TONE_COLORS[tile.tone]
    rl.draw_rectangle_rounded(rect, 0.08, 10, TILE_BG)
    rl.draw_rectangle_rounded_lines(rect, 0.08, 10, TILE_BORDER)
    if tile.driving:
      rl.draw_rectangle_rounded_lines_ex(rect, 0.08, 10, 3, tone)
    # the tone's bar down the left edge
    rl.begin_scissor_mode(int(rect.x), int(rect.y), ACCENT_BAR, int(rect.height))
    rl.draw_rectangle_rounded(rect, 0.08, 10, tone if tile.tag else TILE_BORDER)
    rl.end_scissor_mode()

    x = rect.x + ACCENT_BAR + TILE_PAD
    w = rect.width - ACCENT_BAR - 2 * TILE_PAD
    y = rect.y + TILE_PAD

    # the role on the left, the state on the right
    tag_w = 0.0
    if tile.tag:
      tag_font = gui_app.font(FontWeight.BOLD)
      tag = _fit(tag_font, tile.tag, TAG_SIZE, w * 0.55)
      tag_size = measure_text_cached(tag_font, tag, TAG_SIZE)
      tag_x = x + w - tag_size.x
      rl.draw_circle(int(tag_x - 18), int(y + tag_size.y / 2), 8, tone)
      rl.draw_text_ex(tag_font, tag, rl.Vector2(tag_x, y), TAG_SIZE, 0, tone)
      tag_w = tag_size.x + 40
    role_font = gui_app.font(FontWeight.SEMI_BOLD)
    role = _fit(role_font, tile.role, ROLE_SIZE, w - tag_w)
    rl.draw_text_ex(role_font, role, rl.Vector2(x, y + (TAG_SIZE - ROLE_SIZE) / 2), ROLE_SIZE, 0, ROLE_COLOR)
    y += ROLE_SIZE + 10

    lines = self._name_lines(tile)
    size = self._name_size(lines, w)
    name_font = gui_app.font(FontWeight.SEMI_BOLD)
    for line in lines:
      rl.draw_text_ex(name_font, _fit(name_font, line, size, w), rl.Vector2(x, y), size, 0, MODEL_NAME_COLOR)
      y += size * 1.15

    if tile.details:
      y += 8
      detail_font = gui_app.font(FontWeight.NORMAL)
      for line in tile.details:
        if y + DETAIL_SIZE > rect.y + rect.height - TILE_PAD + 4:
          break
        rl.draw_text_ex(detail_font, _fit(detail_font, line, DETAIL_SIZE, w), rl.Vector2(x, y), DETAIL_SIZE, 0, DETAIL_COLOR)
        y += DETAIL_SIZE + 10

    if tile.progress is not None:
      bar = rl.Rectangle(x, min(y + 12, rect.y + rect.height - TILE_PAD - PROGRESS_H), w, PROGRESS_H)
      rl.draw_rectangle_rounded(bar, 1.0, 8, PROGRESS_TRACK)
      fill = rl.Rectangle(bar.x, bar.y, max(bar.height, bar.width * min(max(tile.progress, 0.0), 1.0)), bar.height)
      rl.draw_rectangle_rounded(fill, 1.0, 8, tone)
