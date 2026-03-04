import math
import pyray as rl
from collections.abc import Callable
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.selfdrive.ui.bp.mici.widgets.keyboard_bp import MiciKeyboardBP
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.widgets.dialog import BigDialogBase

DEBUG = False
PADDING = 20

class BigInputDialogBP(BigDialogBase):
  BACK_TOUCH_AREA_PERCENTAGE = 0.2
  BACKSPACE_RATE = 25  # hz

  def __init__(self,
               hint: str,
               default_text: str = "",
               minimum_length: int = 1,
               confirm_callback: Callable[[str], None] = None,
               show_special_keys: bool = False):
    super().__init__()
    self._hint_label = UnifiedLabel(hint, font_size=35, text_color=rl.Color(255, 255, 255, int(255 * 0.35)),
                                    font_weight=FontWeight.MEDIUM)
    self._keyboard = MiciKeyboardBP(show_special_keys=show_special_keys)
    self._keyboard.set_text(default_text)
    self._minimum_length = minimum_length

    self._backspace_held_time: float | None = None

    self._backspace_img = gui_app.texture("icons_mici/settings/keyboard/backspace.png", 42, 36)
    self._backspace_img_alpha = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)

    self._enter_img = gui_app.texture("icons_mici/settings/keyboard/enter.png", 76, 62)
    self._enter_disabled_img = gui_app.texture("icons_mici/settings/keyboard/enter_disabled.png", 76, 62)
    self._enter_img_alpha = FirstOrderFilter(0, 0.05, 1 / gui_app.target_fps)

    # rects for top buttons
    self._top_left_button_rect = rl.Rectangle(0, 0, 0, 0)
    self._top_right_button_rect = rl.Rectangle(0, 0, 0, 0)

    def confirm_callback_wrapper():
      self._ret = DialogResult.CONFIRM
      if confirm_callback:
        confirm_callback(self._keyboard.text())
    self._confirm_callback = confirm_callback_wrapper

  def _update_state(self):
    super()._update_state()

    last_mouse_event = gui_app.last_mouse_event
    if last_mouse_event.left_down and rl.check_collision_point_rec(last_mouse_event.pos, self._top_right_button_rect) and self._backspace_img_alpha.x > 1:
      if self._backspace_held_time is None:
        self._backspace_held_time = rl.get_time()

      if rl.get_time() - self._backspace_held_time > 0.5:
        if gui_app.frame % round(gui_app.target_fps / self.BACKSPACE_RATE) == 0:
          self._keyboard.backspace()

    else:
      self._backspace_held_time = None

  def _render(self, _):
    text_input_size = 35

    # draw current text so far below everything. text floats left but always stays in view
    text = self._keyboard.text()
    candidate_char = self._keyboard.get_candidate_character()
    text_size = measure_text_cached(gui_app.font(FontWeight.ROMAN), text + candidate_char or self._hint_label.text, text_input_size)
    text_x = PADDING * 2 + self._enter_img.width

    # text needs to move left if we're at the end where right button is
    text_rect = rl.Rectangle(text_x,
                             int(self._rect.y + PADDING),
                             # clip width to right button when in view
                             int(self._rect.width - text_x - PADDING * 2 - self._enter_img.width + 5),  # TODO: why 5?
                             int(text_size.y))

    # draw rounded background for text input
    bg_block_margin = 5
    text_field_rect = rl.Rectangle(text_rect.x - bg_block_margin, text_rect.y - bg_block_margin,
                                   text_rect.width + bg_block_margin * 2, text_input_size + bg_block_margin * 2)

    # draw text input
    # push text left with a gradient on left side if too long
    if text_size.x > text_rect.width:
      text_x -= text_size.x - text_rect.width

    rl.begin_scissor_mode(int(text_rect.x), int(text_rect.y), int(text_rect.width), int(text_rect.height))
    rl.draw_text_ex(gui_app.font(FontWeight.ROMAN), text, rl.Vector2(text_x, text_rect.y), text_input_size, 0, rl.WHITE)

    # draw grayed out character user is hovering over
    if candidate_char:
      candidate_char_size = measure_text_cached(gui_app.font(FontWeight.ROMAN), candidate_char, text_input_size)
      rl.draw_text_ex(gui_app.font(FontWeight.ROMAN), candidate_char,
                      rl.Vector2(min(text_x + text_size.x, text_rect.x + text_rect.width) - candidate_char_size.x, text_rect.y),
                      text_input_size, 0, rl.Color(255, 255, 255, 128))

    rl.end_scissor_mode()

    # draw gradient on left side to indicate more text
    if text_size.x > text_rect.width:
      rl.draw_rectangle_gradient_h(int(text_rect.x), int(text_rect.y), 80, int(text_rect.height),
                                   rl.BLACK, rl.BLANK)

    # draw cursor
    if text:
      blink_alpha = (math.sin(rl.get_time() * 6) + 1) / 2
      cursor_x = min(text_x + text_size.x + 3, text_rect.x + text_rect.width)
      rl.draw_rectangle_rounded(rl.Rectangle(int(cursor_x), int(text_rect.y), 4, int(text_size.y)),
                                1, 4, rl.Color(255, 255, 255, int(255 * blink_alpha)))

    # draw backspace icon with nice fade
    self._backspace_img_alpha.update(255 * bool(text))
    if self._backspace_img_alpha.x > 1:
      color = rl.Color(255, 255, 255, int(self._backspace_img_alpha.x))
      rl.draw_texture(self._backspace_img, int(self._rect.width - self._backspace_img.width - 27), int(text_field_rect.y), color)

    if not text and self._hint_label.text and not candidate_char:
      # draw description if no text entered yet and not drawing candidate char
      self._hint_label.render(text_field_rect)

    # TODO: move to update state
    # make rect take up entire area so it's easier to click
    self._top_left_button_rect = rl.Rectangle(self._rect.x, self._rect.y, text_field_rect.x, self._rect.height - self._keyboard.get_keyboard_height())
    self._top_right_button_rect = rl.Rectangle(text_field_rect.x + text_field_rect.width, self._rect.y,
                                               self._rect.width - (text_field_rect.x + text_field_rect.width), self._top_left_button_rect.height)

    # draw enter button (enabled + disabled states, same as stock BigInputDialog)
    self._enter_img_alpha.update(255 if (len(text) >= self._minimum_length) else 0)
    color = rl.Color(255, 255, 255, int(self._enter_img_alpha.x))
    rl.draw_texture(self._enter_img, int(self._rect.x + 15), int(text_field_rect.y), color)
    color = rl.Color(255, 255, 255, 255 - int(self._enter_img_alpha.x))
    rl.draw_texture(self._enter_disabled_img, int(self._rect.x + 15), int(text_field_rect.y), color)

    # keyboard goes over everything
    self._keyboard.render(self._rect)

    # draw debugging rect bounds
    if DEBUG:
      rl.draw_rectangle_lines_ex(text_field_rect, 1, rl.Color(100, 100, 100, 255))
      rl.draw_rectangle_lines_ex(text_rect, 1, rl.Color(0, 255, 0, 255))
      rl.draw_rectangle_lines_ex(self._top_right_button_rect, 1, rl.Color(0, 255, 0, 255))
      rl.draw_rectangle_lines_ex(self._top_left_button_rect, 1, rl.Color(0, 255, 0, 255))

    return self._ret

  def _handle_mouse_press(self, mouse_pos: MousePos):
    super()._handle_mouse_press(mouse_pos)
    # TODO: need to track where press was so enter and back can activate on release rather than press
    #  or turn into icon widgets :eyes_open:
    # handle backspace icon click
    if rl.check_collision_point_rec(mouse_pos, self._top_right_button_rect) and self._backspace_img_alpha.x > 254:
      self._keyboard.backspace()
    elif rl.check_collision_point_rec(mouse_pos, self._top_left_button_rect) and self._enter_img_alpha.x > 254:
      # handle enter icon click
      self._confirm_callback()