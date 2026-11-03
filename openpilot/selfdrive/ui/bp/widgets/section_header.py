"""Section header widgets for TICI BluePilot settings menu."""
import pyray as rl
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.lib.application import FontWeight

# Match list item width; height for bold section title
SECTION_HEADER_WIDTH = 600
SECTION_HEADER_HEIGHT = 65
SECTION_HEADER_FONT_SIZE = 52

COLLAPSIBLE_HEADER_HEIGHT = 105
COLLAPSIBLE_HEADER_FONT_SIZE = 52
COLLAPSIBLE_PRESS_COLOR = rl.Color(255, 255, 255, 18)
COLLAPSIBLE_ARROW_COLOR = rl.Color(220, 220, 220, 255)
COLLAPSIBLE_ARROW_SIZE = 14   # half-width / half-height of the drawn triangle
COLLAPSIBLE_TITLE_INDENT = 46  # px indent to leave room for arrow


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
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
      alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE,
      elide_right=True,
    )


class CollapsibleSectionHeader(Widget):
  """Tappable section header that expands/collapses its associated items.

  Usage:
      header = CollapsibleSectionHeader("Lateral Tuning")
      header.set_items([item1, item2, ...])
      # Returns [header, item1, item2, ...] for the Scroller
  """

  def __init__(self, title: str, initially_expanded: bool = False):
    super().__init__()
    self._title = title
    self._expanded = initially_expanded
    self._managed_items: list[Widget] = []
    self._item_visible: dict[int, bool] = {}
    # Nested CollapsibleSectionHeaders: a header's own managed_items only cover its direct
    # children, so collapsing it hides those children's *header lines* but not their
    # grandchildren (which the nested header manages independently). Cascade collapse to fix.
    self._nested_headers: list["CollapsibleSectionHeader"] = []
    self.set_rect(rl.Rectangle(0, 0, SECTION_HEADER_WIDTH, COLLAPSIBLE_HEADER_HEIGHT))
    self.set_click_callback(self._toggle)

  def set_items(self, items: list[Widget]) -> None:
    """Register the items this header controls and apply initial visibility."""
    self._managed_items = list(items)
    self._apply_visibility()

  def set_nested_headers(self, headers: list["CollapsibleSectionHeader"]) -> None:
    """Register nested sub-headers so collapsing this section also collapses them."""
    self._nested_headers = list(headers)

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

  def _apply_visibility(self) -> None:
    for item in self._managed_items:
      item.set_visible(self._expanded and self._item_visible.get(id(item), True))

  def collapse(self) -> None:
    self._expanded = False
    for header in self._nested_headers:
      header.collapse()
    self._apply_visibility()

  def show_event(self) -> None:
    # Always start collapsed when the menu is opened so the user sees only headers.
    self.collapse()
    super().show_event()

  def _draw_arrow(self, rect: rl.Rectangle) -> None:
    """Draw a filled triangle arrow using raylib primitives (font-independent)."""
    hw = float(COLLAPSIBLE_ARROW_SIZE)
    hh = float(COLLAPSIBLE_ARROW_SIZE)
    cx = rect.x + hw + 4
    cy = rect.y + rect.height / 2.0

    if self._expanded:
      # Down-pointing triangle ▼ (counter-clockwise winding)
      rl.draw_triangle(
        rl.Vector2(cx - hw, cy - hh * 0.6),
        rl.Vector2(cx,       cy + hh * 0.6),
        rl.Vector2(cx + hw, cy - hh * 0.6),
        COLLAPSIBLE_ARROW_COLOR,
      )
    else:
      # Right-pointing triangle ▶ (counter-clockwise winding: top → bottom → right)
      rl.draw_triangle(
        rl.Vector2(cx - hh * 0.6, cy - hw),
        rl.Vector2(cx - hh * 0.6, cy + hw),
        rl.Vector2(cx + hh * 0.6, cy),
        COLLAPSIBLE_ARROW_COLOR,
      )

  def _render(self, rect: rl.Rectangle) -> None:
    # Subtle press highlight for touch feedback
    if self.is_pressed:
      rl.draw_rectangle_rec(rect, COLLAPSIBLE_PRESS_COLOR)

    self._draw_arrow(rect)

    # Indent title to leave room for the arrow
    title_rect = rl.Rectangle(rect.x + COLLAPSIBLE_TITLE_INDENT, rect.y,
                               rect.width - COLLAPSIBLE_TITLE_INDENT, rect.height)
    gui_label(
      title_rect,
      self._title,
      font_size=COLLAPSIBLE_HEADER_FONT_SIZE,
      font_weight=FontWeight.BOLD,
      color=rl.Color(220, 220, 220, 255),
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
      alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE,
      elide_right=True,
    )
