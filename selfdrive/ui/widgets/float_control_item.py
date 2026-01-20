import pyray as rl
from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import ListItem, ItemAction
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.lib.application import FontWeight, gui_app, MousePos
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached

ITEM_TEXT_FONT_SIZE = 50
ITEM_TEXT_COLOR = rl.WHITE
ITEM_TEXT_VALUE_COLOR = rl.Color(170, 170, 170, 255)
BUTTON_SIZE = 80
BUTTON_SPACING = 7  # Reduced from 20 to ~1/3 (33/3 ≈ 11, but we need some space, so 7)


class FloatControlAction(ItemAction):
  """Action item for float controls with +/- buttons."""
  
  def __init__(self, param: str, min_value: float, max_value: float, step: float, 
               callback: Callable[[float], None] | None = None, enabled: bool | Callable[[], bool] = True,
               suffix: str = ""):
    super().__init__(width=0, enabled=enabled)  # Width 0 means use full width
    self.param = param
    self.min_value = min_value
    self.max_value = max_value
    self.step = step
    self.callback = callback
    self.suffix = suffix
    self.params = Params()
    
    # Create +/- buttons
    self._minus_button = Button(
      "-",
      click_callback=self._decrement,
      font_size=50,
      button_style=ButtonStyle.NORMAL,
      border_radius=10
    )
    self._plus_button = Button(
      "+",
      click_callback=self._increment,
      font_size=50,
      button_style=ButtonStyle.PRIMARY,
      border_radius=10
    )
    
    self._font = gui_app.font(FontWeight.NORMAL)
  
  def _get_value(self) -> float:
    """Get current parameter value."""
    try:
      return float(self.params.get(self.param, return_default=True))
    except (TypeError, ValueError):
      return self.min_value
  
  def _set_value(self, value: float):
    """Set parameter value."""
    # Clamp to min/max
    value = max(self.min_value, min(self.max_value, value))
    self.params.put_nonblocking(self.param, value)
    if self.callback:
      self.callback(value)
  
  def _increment(self):
    """Increment value by step."""
    current = self._get_value()
    self._set_value(current + self.step)
  
  def _decrement(self):
    """Decrement value by step."""
    current = self._get_value()
    self._set_value(current - self.step)
  
  def _render(self, rect: rl.Rectangle) -> bool:
    current_value = self._get_value()
    # Format value based on suffix
    if self.suffix == "V":
      value_text = f"{current_value:.1f}{self.suffix}"
    else:
      value_text = f"{current_value:.2f}{self.suffix}"
    
    # Calculate layout - reduce spacing to 1/3 of original
    # Original was BUTTON_SIZE (80) + BUTTON_SPACING (20) = 100 per side
    # Target is ~33 per side, so BUTTON_SPACING reduced to ~7
    button_y = rect.y + (rect.height - BUTTON_SIZE) / 2
    
    # Check if enabled (handle callable) - use the enabled property from ItemAction
    is_enabled = self.enabled
    
    # Calculate total space needed: buttons + minimal spacing
    total_button_space = BUTTON_SIZE * 2 + BUTTON_SPACING * 2
    # Value width should be just enough for the text, not the full remaining space
    value_text_width = measure_text_cached(self._font, value_text, ITEM_TEXT_FONT_SIZE).x
    # Add small padding around text
    value_width = value_text_width + 20
    
    # Center the value area, with buttons positioned close to it
    total_width = total_button_space + value_width
    start_x = rect.x + (rect.width - total_width) / 2
    
    # Minus button on left
    minus_rect = rl.Rectangle(start_x, button_y, BUTTON_SIZE, BUTTON_SIZE)
    self._minus_button.set_enabled(current_value > self.min_value and is_enabled)
    self._minus_button.render(minus_rect)
    
    # Value in center
    value_rect = rl.Rectangle(
      start_x + BUTTON_SIZE + BUTTON_SPACING,
      rect.y,
      value_width,
      rect.height
    )
    gui_label(
      value_rect,
      value_text,
      font_size=ITEM_TEXT_FONT_SIZE,
      color=ITEM_TEXT_VALUE_COLOR,
      font_weight=FontWeight.NORMAL,
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
      alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE
    )
    
    # Plus button on right
    plus_rect = rl.Rectangle(start_x + BUTTON_SIZE + BUTTON_SPACING + value_width + BUTTON_SPACING, button_y, BUTTON_SIZE, BUTTON_SIZE)
    self._plus_button.set_enabled(current_value < self.max_value and is_enabled)
    self._plus_button.render(plus_rect)
    
    return False
  
  def _handle_mouse_release(self, mouse_pos: MousePos):
    """Handle mouse clicks on buttons."""
    current_value = self._get_value()
    if self.suffix == "V":
      value_text = f"{current_value:.1f}{self.suffix}"
    else:
      value_text = f"{current_value:.2f}{self.suffix}"
    
    button_y = self._rect.y + (self._rect.height - BUTTON_SIZE) / 2
    
    # Calculate button positions (same as in _render)
    value_text_width = measure_text_cached(self._font, value_text, ITEM_TEXT_FONT_SIZE).x
    value_width = value_text_width + 20
    total_button_space = BUTTON_SIZE * 2 + BUTTON_SPACING * 2
    total_width = total_button_space + value_width
    start_x = self._rect.x + (self._rect.width - total_width) / 2
    
    minus_rect = rl.Rectangle(start_x, button_y, BUTTON_SIZE, BUTTON_SIZE)
    if rl.check_collision_point_rec(mouse_pos, minus_rect):
      self._minus_button._handle_mouse_release(mouse_pos)
      return
    
    plus_rect = rl.Rectangle(start_x + BUTTON_SIZE + BUTTON_SPACING + value_width + BUTTON_SPACING, button_y, BUTTON_SIZE, BUTTON_SIZE)
    if rl.check_collision_point_rec(mouse_pos, plus_rect):
      self._plus_button._handle_mouse_release(mouse_pos)
      return
    
    super()._handle_mouse_release(mouse_pos)


def float_control_item(title: str | Callable[[], str], description: str | Callable[[], str] | None = None,
                       param: str = "", min_value: float = 0.0, max_value: float = 1.0, step: float = 0.05,
                       callback: Callable[[float], None] | None = None, enabled: bool | Callable[[], bool] = True,
                       icon: str = "", suffix: str = "") -> ListItem:
  """Create a list item with float control (+/- buttons)."""
  action = FloatControlAction(param, min_value, max_value, step, callback, enabled, suffix)
  return ListItem(title=title, description=description, action_item=action, icon=icon)
