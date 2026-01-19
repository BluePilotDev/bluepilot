import pyray as rl
from collections.abc import Callable

from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.inputbox import InputBox
from openpilot.system.ui.widgets.button import Button, ButtonStyle
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.multilang import tr


class FloatInputDialogTici(Widget):
  """Dialog for inputting float values (TICI version)."""
  
  def __init__(self, title: str, current_value: float, min_value: float = None, 
               max_value: float = None, step: float = 0.05, callback: Callable[[float], None] = None,
               suffix: str = ""):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self.title = title
    self.current_value = current_value
    self.min_value = min_value
    self.max_value = max_value
    self.step = step
    self.callback = callback
    self.suffix = suffix
    
    self._input_box = InputBox(max_text_size=20)
    self._input_box.text = str(current_value)
    
    self._result: DialogResult = DialogResult.NO_ACTION
    
    # Buttons
    self._cancel_button = Button(
      lambda: tr("Cancel"),
      click_callback=lambda: self._set_result(DialogResult.CANCEL),
      button_style=ButtonStyle.NORMAL
    )
    
    self._confirm_button = Button(
      lambda: tr("Confirm"),
      click_callback=self._handle_confirm,
      button_style=ButtonStyle.PRIMARY
    )
    
    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._font_bold = gui_app.font(FontWeight.BOLD)
  
  def _set_result(self, result: DialogResult):
    self._result = result
  
  def _handle_confirm(self):
    """Handle confirm button click."""
    try:
      value = float(self._input_box.text)
      
      # Clamp to min/max if specified
      if self.min_value is not None and value < self.min_value:
        value = self.min_value
      elif self.max_value is not None and value > self.max_value:
        value = self.max_value
      
      if self.callback:
        self.callback(value)
      
      self._set_result(DialogResult.CONFIRM)
    except ValueError:
      # Invalid input, don't close dialog
      pass
  
  def _render(self, rect: rl.Rectangle) -> int:
    # Dialog background
    margin = 200
    dialog_rect = rl.Rectangle(
      rect.x + margin,
      rect.y + margin,
      rect.width - 2 * margin,
      rect.height - 2 * margin
    )
    rl.draw_rectangle_rounded(dialog_rect, 0.02, 20, rl.Color(30, 30, 30, 255))
    
    # Content area
    content_margin = 50
    content_rect = rl.Rectangle(
      dialog_rect.x + content_margin,
      dialog_rect.y + content_margin,
      dialog_rect.width - 2 * content_margin,
      dialog_rect.height - 2 * content_margin
    )
    
    # Title
    title_height = 80
    gui_label(
      rl.Rectangle(content_rect.x, content_rect.y, content_rect.width, title_height),
      self.title,
      font_size=70,
      font_weight=FontWeight.BOLD,
      color=rl.WHITE
    )
    
    # Range hint
    if self.min_value is not None or self.max_value is not None:
      range_text = f"Range: {self.min_value if self.min_value is not None else '-∞'} to {self.max_value if self.max_value is not None else '∞'}"
      gui_label(
        rl.Rectangle(content_rect.x, content_rect.y + title_height + 20, content_rect.width, 50),
        range_text,
        font_size=40,
        font_weight=FontWeight.MEDIUM,
        color=rl.Color(150, 150, 150, 255)
      )
    
    # Input box
    input_height = 120
    input_y = content_rect.y + title_height + 80
    input_rect = rl.Rectangle(
      content_rect.x,
      input_y,
      content_rect.width,
      input_height
    )
    
    # Draw input box background
    rl.draw_rectangle_rounded(input_rect, 0.02, 10, rl.Color(50, 50, 50, 255))
    rl.draw_rectangle_rounded_lines(input_rect, 0.02, 10, 2, rl.Color(100, 100, 100, 255))
    
    # Render input box
    self._input_box.set_rect(input_rect)
    self._input_box._render(input_rect, color=rl.Color(50, 50, 50, 255), 
                           border_color=rl.Color(100, 100, 100, 255), 
                           text_color=rl.WHITE, font_size=60)
    
    # Suffix label if provided
    if self.suffix:
      suffix_x = input_rect.x + input_rect.width - 100
      gui_label(
        rl.Rectangle(suffix_x, input_y, 100, input_height),
        self.suffix,
        font_size=50,
        font_weight=FontWeight.MEDIUM,
        color=rl.Color(200, 200, 200, 255)
      )
    
    # Buttons
    button_height = 100
    button_spacing = 30
    button_y = content_rect.y + content_rect.height - button_height
    button_width = (content_rect.width - button_spacing) / 2
    
    cancel_rect = rl.Rectangle(
      content_rect.x,
      button_y,
      button_width,
      button_height
    )
    self._cancel_button.render(cancel_rect)
    
    confirm_rect = rl.Rectangle(
      content_rect.x + button_width + button_spacing,
      button_y,
      button_width,
      button_height
    )
    self._confirm_button.render(confirm_rect)
    
    return self._result
  
  def _handle_mouse_release(self, mouse_pos):
    """Handle mouse clicks."""
    # Let buttons handle their own clicks
    cancel_rect = self._cancel_button._rect
    if cancel_rect:
      if (cancel_rect.x <= mouse_pos.x <= cancel_rect.x + cancel_rect.width and
          cancel_rect.y <= mouse_pos.y <= cancel_rect.y + cancel_rect.height):
        self._cancel_button._handle_mouse_release(mouse_pos)
        return
    
    confirm_rect = self._confirm_button._rect
    if confirm_rect:
      if (confirm_rect.x <= mouse_pos.x <= confirm_rect.x + confirm_rect.width and
          confirm_rect.y <= mouse_pos.y <= confirm_rect.y + confirm_rect.height):
        self._confirm_button._handle_mouse_release(mouse_pos)
        return
    
    super()._handle_mouse_release(mouse_pos)
