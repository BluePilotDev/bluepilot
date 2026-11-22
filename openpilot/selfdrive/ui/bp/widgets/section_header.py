"""Section header widgets for TICI BluePilot settings menu."""
import math
from collections.abc import Callable

import pyray as rl
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.lib.application import gui_app, FontWeight, TextAlignment, TextAlignmentVertical
from openpilot.system.ui.lib.text_measure import measure_text_cached

# Match list item width; height for bold section title
SECTION_HEADER_WIDTH = 600
SECTION_HEADER_HEIGHT = 65
SECTION_HEADER_FONT_SIZE = 52

# Collapsible header: a full-width card, as tall as a list item, so the whole row is the touch target
COLLAPSIBLE_HEADER_HEIGHT = 170
COLLAPSIBLE_CARD_MARGIN_X = 10        # card inset from the scroller's edges
COLLAPSIBLE_CARD_MARGIN_Y = 10        # card inset top and bottom (the gap between two closed sections is twice this)
COLLAPSIBLE_CARD_PAD = 36             # padding inside the card
COLLAPSIBLE_ICON_BOX = 96             # rounded square behind the section icon
COLLAPSIBLE_ICON_SIZE = 64
COLLAPSIBLE_TITLE_FONT_SIZE = 54
COLLAPSIBLE_SUBTITLE_FONT_SIZE = 40
COLLAPSIBLE_COUNT_FONT_SIZE = 40
COLLAPSIBLE_CHEVRON_SIZE = 20         # half-height of the chevron
COLLAPSIBLE_CHEVRON_THICKNESS = 7
COLLAPSIBLE_ACCENT_WIDTH = 10

ACCENT = rl.Color(24, 144, 255, 255)  # BluePilot blue
CARD_BG = rl.Color(40, 42, 46, 255)
CARD_BG_PRESSED = rl.Color(58, 61, 67, 255)
CARD_BG_OPEN = rl.Color(22, 48, 78, 255)
CARD_BG_OPEN_PRESSED = rl.Color(32, 64, 102, 255)
SECTION_BODY_BG = rl.Color(28, 30, 33, 255)  # the card an open section's items sit in
ICON_BOX_BG = rl.Color(255, 255, 255, 20)
ICON_BOX_BG_OPEN = rl.Color(24, 144, 255, 70)
TITLE_COLOR = rl.Color(255, 255, 255, 255)
SUBTITLE_COLOR = rl.Color(160, 164, 172, 255)
COUNT_BG = rl.Color(255, 255, 255, 24)
COUNT_COLOR = rl.Color(200, 204, 212, 255)
CHEVRON_COLOR = rl.Color(220, 224, 232, 255)

# Section body (SectionScroller)
SECTION_ITEM_INSET = 40               # items in an open section are inset this much on each side
SECTION_BODY_PAD_BOTTOM = 16
SECTION_SEPARATOR_COLOR = rl.Color(255, 255, 255, 28)
SECTION_SEPARATOR_PAD = 24


class SectionHeader(Widget):
  """Non-interactive bold section header for dividing menu sections."""

  def __init__(self, title: str):
    super().__init__()
    self._title = title
    self.set_rect(rl.Rectangle(0, 0, SECTION_HEADER_WIDTH, SECTION_HEADER_HEIGHT))

  def _render(self, rect: rl.Rectangle) -> None:
    gui_label(
      rect,
      self._title,
      font_size=SECTION_HEADER_FONT_SIZE,
      font_weight=FontWeight.BOLD,
      color=rl.Color(220, 220, 220, 255),
      alignment=TextAlignment.LEFT,
      alignment_vertical=TextAlignmentVertical.MIDDLE,
      elide_right=True,
    )


class CollapsibleSectionHeader(Widget):
  """Tappable, full-width section card that expands/collapses its associated items.

  Usage:
      header = CollapsibleSectionHeader("Lateral Tuning", subtitle="Steering, lane changes", icon="icons/chffr_wheel.png")
      header.set_items([item1, item2, ...])
      # Returns [header, item1, item2, ...] for the Scroller (a SectionScroller groups them into one card)
  """

  def __init__(self, title: str, initially_expanded: bool = False, subtitle: str = "", icon: str | None = None):
    super().__init__()
    self._title = title
    self._subtitle = subtitle
    self._icon_texture = gui_app.texture(icon, COLLAPSIBLE_ICON_SIZE, COLLAPSIBLE_ICON_SIZE) if icon else None
    self._expanded = initially_expanded
    self._managed_items: list[Widget] = []
    self._item_visible: dict[int, bool] = {}
    # Nested CollapsibleSectionHeaders: a header's own managed_items only cover its direct
    # children, so collapsing it hides those children's *header lines* but not their
    # grandchildren (which the nested header manages independently). Cascade collapse to fix.
    self._nested_headers: list[CollapsibleSectionHeader] = []
    self._toggle_callback: Callable[[CollapsibleSectionHeader], None] | None = None
    self._chevron_angle = 90.0 if initially_expanded else 0.0  # 0 points right (closed), 90 points down (open)
    self._title_font = gui_app.font(FontWeight.BOLD)
    self._text_font = gui_app.font(FontWeight.MEDIUM)
    self.set_rect(rl.Rectangle(0, 0, SECTION_HEADER_WIDTH, COLLAPSIBLE_HEADER_HEIGHT))
    self.set_click_callback(self._toggle)

  @property
  def expanded(self) -> bool:
    return self._expanded

  @property
  def managed_items(self) -> list[Widget]:
    return self._managed_items

  def set_parent_rect(self, parent_rect: rl.Rectangle) -> None:
    # Full width of the list: the whole row is the touch target, not just the title
    super().set_parent_rect(parent_rect)
    self._rect.width = parent_rect.width

  def set_items(self, items: list[Widget]) -> None:
    """Register the items this header controls and apply initial visibility."""
    self._managed_items = list(items)
    self._apply_visibility()

  def set_nested_headers(self, headers: list["CollapsibleSectionHeader"]) -> None:
    """Register nested sub-headers so collapsing this section also collapses them."""
    self._nested_headers = list(headers)

  def set_toggle_callback(self, callback: Callable[["CollapsibleSectionHeader"], None] | None) -> None:
    """Called after the user opens or closes this section (not on collapse())."""
    self._toggle_callback = callback

  def set_item_visible(self, item: Widget, visible: bool) -> None:
    """Control whether a managed item is allowed to show when the section is expanded."""
    self._item_visible[id(item)] = visible
    if self._expanded:
      item.set_visible(visible)

  def _toggle(self) -> None:
    self._expanded = not self._expanded
    self._apply_visibility()
    if not self._expanded:
      for header in self._nested_headers:
        header.collapse()
    if self._toggle_callback is not None:
      self._toggle_callback(self)

  def _apply_visibility(self) -> None:
    for item in self._managed_items:
      item.set_visible(self._expanded and self._item_visible.get(id(item), True))

  def _item_count(self) -> int:
    return sum(1 for item in self._managed_items if self._item_visible.get(id(item), True))

  def collapse(self) -> None:
    self._expanded = False
    for header in self._nested_headers:
      header.collapse()
    self._apply_visibility()

  def show_event(self) -> None:
    # Always start collapsed when the menu is opened so the user sees only headers.
    self.collapse()
    self._chevron_angle = 0.0
    super().show_event()

  def card_rect(self) -> rl.Rectangle:
    return rl.Rectangle(self._rect.x + COLLAPSIBLE_CARD_MARGIN_X, self._rect.y + COLLAPSIBLE_CARD_MARGIN_Y,
                        self._rect.width - 2 * COLLAPSIBLE_CARD_MARGIN_X, self._rect.height - 2 * COLLAPSIBLE_CARD_MARGIN_Y)

  def _draw_chevron(self, cx: float, cy: float) -> None:
    """A thick chevron that turns from right (closed) to down (open)."""
    target = 90.0 if self._expanded else 0.0
    self._chevron_angle += (target - self._chevron_angle) * 0.3
    if abs(target - self._chevron_angle) < 0.5:
      self._chevron_angle = target
    a = math.radians(self._chevron_angle)
    rot = (math.cos(a), math.sin(a))
    s = float(COLLAPSIBLE_CHEVRON_SIZE)
    points = [(-s * 0.5, -s), (s * 0.5, 0.0), (-s * 0.5, s)]  # ">" centered on its tip's midline
    pts = [rl.Vector2(cx + x * rot[0] - y * rot[1], cy + x * rot[1] + y * rot[0]) for x, y in points]
    for p0, p1 in ((pts[0], pts[1]), (pts[1], pts[2])):
      rl.draw_line_ex(p0, p1, COLLAPSIBLE_CHEVRON_THICKNESS, CHEVRON_COLOR)
    for p in pts:  # round the ends and the joint
      rl.draw_circle_v(p, COLLAPSIBLE_CHEVRON_THICKNESS / 2, CHEVRON_COLOR)

  def _render(self, rect: rl.Rectangle) -> None:
    # Skip drawing when scrolled out of view (the scroller still lays it out)
    if self._parent_rect is not None and (rect.y + rect.height <= self._parent_rect.y or
                                          rect.y >= self._parent_rect.y + self._parent_rect.height):
      return

    card = self.card_rect()
    if self._expanded:
      bg = CARD_BG_OPEN_PRESSED if self.is_pressed else CARD_BG_OPEN
    else:
      bg = CARD_BG_PRESSED if self.is_pressed else CARD_BG
    rl.draw_rectangle_rounded(card, 0.22, 12, bg)
    if self._expanded:
      accent = rl.Rectangle(card.x, card.y + 20, COLLAPSIBLE_ACCENT_WIDTH, card.height - 40)
      rl.draw_rectangle_rounded(accent, 1.0, 6, ACCENT)

    x = card.x + COLLAPSIBLE_CARD_PAD
    cy = card.y + card.height / 2

    # Icon in a rounded square
    if self._icon_texture is not None:
      box = rl.Rectangle(x, cy - COLLAPSIBLE_ICON_BOX / 2, COLLAPSIBLE_ICON_BOX, COLLAPSIBLE_ICON_BOX)
      rl.draw_rectangle_rounded(box, 0.3, 10, ICON_BOX_BG_OPEN if self._expanded else ICON_BOX_BG)
      tex = self._icon_texture
      rl.draw_texture_v(tex, rl.Vector2(box.x + (box.width - tex.width) / 2, box.y + (box.height - tex.height) / 2), rl.WHITE)
      x += COLLAPSIBLE_ICON_BOX + COLLAPSIBLE_CARD_PAD

    # Right side: item count pill and the chevron
    right = card.x + card.width - COLLAPSIBLE_CARD_PAD
    chevron_cx = right - COLLAPSIBLE_CHEVRON_SIZE
    self._draw_chevron(chevron_cx, cy)
    right = chevron_cx - COLLAPSIBLE_CHEVRON_SIZE - 32

    count_text = str(self._item_count())
    count_size = measure_text_cached(self._text_font, count_text, COLLAPSIBLE_COUNT_FONT_SIZE)
    pill_h = COLLAPSIBLE_COUNT_FONT_SIZE + 24
    pill_w = max(pill_h, count_size.x + 40)
    pill = rl.Rectangle(right - pill_w, cy - pill_h / 2, pill_w, pill_h)
    rl.draw_rectangle_rounded(pill, 1.0, 12, COUNT_BG)
    rl.draw_text_ex(self._text_font, count_text,
                    rl.Vector2(pill.x + (pill.width - count_size.x) / 2, pill.y + (pill.height - count_size.y) / 2),
                    COLLAPSIBLE_COUNT_FONT_SIZE, 0, COUNT_COLOR)
    text_right = pill.x - 24

    # Title, with the one-line summary under it
    text_w = text_right - x
    if self._subtitle:
      title_h = COLLAPSIBLE_TITLE_FONT_SIZE + 6
      sub_h = COLLAPSIBLE_SUBTITLE_FONT_SIZE + 4
      top = cy - (title_h + sub_h) / 2
      gui_label(rl.Rectangle(x, top, text_w, title_h), self._title, font_size=COLLAPSIBLE_TITLE_FONT_SIZE,
                font_weight=FontWeight.BOLD, color=TITLE_COLOR, alignment=TextAlignment.LEFT,
                alignment_vertical=TextAlignmentVertical.MIDDLE, elide_right=True)
      gui_label(rl.Rectangle(x, top + title_h, text_w, sub_h), self._subtitle, font_size=COLLAPSIBLE_SUBTITLE_FONT_SIZE,
                font_weight=FontWeight.MEDIUM, color=SUBTITLE_COLOR, alignment=TextAlignment.LEFT,
                alignment_vertical=TextAlignmentVertical.MIDDLE, elide_right=True)
    else:
      gui_label(rl.Rectangle(x, card.y, text_w, card.height), self._title, font_size=COLLAPSIBLE_TITLE_FONT_SIZE,
                font_weight=FontWeight.BOLD, color=TITLE_COLOR, alignment=TextAlignment.LEFT,
                alignment_vertical=TextAlignmentVertical.MIDDLE, elide_right=True)


class SectionScroller(Scroller):
  """Scroller for lists of CollapsibleSectionHeader groups.

  An open section's items are drawn inset inside one card under their header, with separators only
  between those items (none around headers). Items before the first header lay out like the stock
  Scroller. scroll_to() brings a widget to the top of the view (clamped to the end of the list).
  """

  def __init__(self, items, line_separator: bool = True):
    super().__init__(items, spacing=0, line_separator=False)
    self._section_separators = line_separator

  def _layout_items(self) -> tuple[list[tuple[Widget, float, CollapsibleSectionHeader | None]], float]:
    """(widget, y relative to the content top, owning header) for each visible item, and the content height."""
    out: list[tuple[Widget, float, CollapsibleSectionHeader | None]] = []
    y = 0.0
    header: CollapsibleSectionHeader | None = None
    prev: Widget | None = None
    for item in self._items:
      if isinstance(item, CollapsibleSectionHeader):
        if header is not None and header.expanded and prev is not None and prev is not header:
          y += SECTION_BODY_PAD_BOTTOM  # room for the open section's card under its last item
        header = item
      if not item.is_visible:
        continue
      out.append((item, y, None if item is header else header))
      y += item.rect.height
      prev = item
    if header is not None and header.expanded and prev is not None and prev is not header:
      y += SECTION_BODY_PAD_BOTTOM
    return out, y

  def scroll_to(self, widget: Widget) -> None:
    layout, content_height = self._layout_items()
    for item, y, _ in layout:
      if item is widget:
        max_scroll = max(0.0, content_height - self._rect.height)
        self.scroll_panel.set_offset(-min(y, max_scroll))
        return

  def _render(self, _):
    layout, content_height = self._layout_items()
    scroll = self.scroll_panel.update(self._rect, rl.Rectangle(0, 0, self._rect.width, content_height))
    inset_rect = rl.Rectangle(self._rect.x + SECTION_ITEM_INSET, self._rect.y,
                              self._rect.width - 2 * SECTION_ITEM_INSET, self._rect.height)

    rl.begin_scissor_mode(int(self._rect.x), int(self._rect.y), int(self._rect.width), int(self._rect.height))

    # Open sections: one card from the header down past its last item
    section_bottom: dict[int, float] = {}
    for item, y, owner in layout:
      if owner is not None:
        section_bottom[id(owner)] = y + item.rect.height
    for item, y, owner in layout:
      if owner is None and isinstance(item, CollapsibleSectionHeader) and id(item) in section_bottom:
        top = self._rect.y + y + scroll + COLLAPSIBLE_CARD_MARGIN_Y
        bottom = self._rect.y + section_bottom[id(item)] + scroll + SECTION_BODY_PAD_BOTTOM - 4
        body = rl.Rectangle(self._rect.x + COLLAPSIBLE_CARD_MARGIN_X, top,
                            self._rect.width - 2 * COLLAPSIBLE_CARD_MARGIN_X, bottom - top)
        if body.y < self._rect.y + self._rect.height and body.y + body.height > self._rect.y:
          rl.draw_rectangle_rounded(body, min(1.0, 60 / max(body.height, 1)), 12, SECTION_BODY_BG)

    prev_owner: CollapsibleSectionHeader | None = None
    for item, y, owner in layout:
      x = self._rect.x + (SECTION_ITEM_INSET if owner is not None else 0)
      top = self._rect.y + y + scroll
      # Separator between two items of the same section
      if self._section_separators and owner is not None and owner is prev_owner:
        rl.draw_line(int(inset_rect.x + SECTION_SEPARATOR_PAD), int(top),
                     int(inset_rect.x + inset_rect.width - SECTION_SEPARATOR_PAD), int(top), SECTION_SEPARATOR_COLOR)
      item.set_position(x, top)
      item.set_parent_rect(inset_rect if owner is not None else self._rect)
      item.render()
      prev_owner = owner

    rl.end_scissor_mode()
