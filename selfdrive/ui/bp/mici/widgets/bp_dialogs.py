"""BP-styled modal dialogs.

Drop-in replacements for selfdrive/ui/mici/widgets/dialog.py:
- BPInfoDialog       <- BigDialog       (info / error toast — tap anywhere to dismiss)
- BPConfirmDialog    <- BigConfirmationDialog  (slide-to-confirm; reuses BigSlider/RedBigSlider internals)
- BPInputDialog      <- BigInputDialog  (text input; reuses MiciKeyboard inside a BP frame)

All sit on the BP radial backdrop and use BP typography.
"""
import pyray as rl
from typing import Union
from collections.abc import Callable

from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.widgets.nav_widget import NavWidget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets.slider import BigSlider, RedBigSlider
from openpilot.system.ui.widgets.mici_keyboard import MiciKeyboard
from openpilot.common.filter_simple import FirstOrderFilter

from openpilot.selfdrive.ui.bp.mici.widgets.bg_radial import BPRadialBackground
from openpilot.selfdrive.ui.bp.mici.widgets import bp_palette as P


from openpilot.system.ui.widgets import Widget

class _BPDialogBase(Widget):
  """Common: BP backdrop + full-screen rect.

  Plain Widget (not NavWidget) — avoids the slide-in animation and the
  full-screen black overlay that NavWidget._layout draws. We handle dismiss
  manually via tap-to-close in _handle_mouse_release.
  """
  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
    self._bg = self._child(BPRadialBackground())

  # NavWidget API shims so callers can still call .dismiss() / .is_dismissing
  @property
  def is_dismissing(self) -> bool:
    return False

  def dismiss(self, callback=None):
    gui_app.pop_widget()
    if callback:
      callback()


# -----------------------------------------------------------------
# Info / error dialog
# -----------------------------------------------------------------
class BPInfoDialog(_BPDialogBase):
  """Tap anywhere outside the card (or on it) to dismiss."""
  def __init__(self, title: str, description: str = "", icon: Union[rl.Texture, None] = None):
    super().__init__()
    self._icon = icon
    self._title = UnifiedLabel(title, font_size=P.FS_NAME, font_weight=FontWeight.BOLD,
                               text_color=P.TEXT, max_width=480, wrap_text=True, line_height=1.05)
    self._desc = UnifiedLabel(description, font_size=P.FS_SUB, font_weight=FontWeight.MEDIUM,
                              text_color=P.MUTED, max_width=480, wrap_text=True, line_height=1.15)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    self.dismiss()

  def _render(self, _):
    r = self._rect
    self._bg.render(r)

    # Card
    pad = 18
    card_w = min(int(r.width * 0.85), 480)
    card_h = int(r.height * 0.78)
    card_x = r.x + (r.width - card_w) / 2
    card_y = r.y + (r.height - card_h) / 2
    card = rl.Rectangle(card_x, card_y, card_w, card_h)
    rl.draw_rectangle_rounded(card, 0.10, 16, P.PANEL_BG)
    rl.draw_rectangle_rounded_lines_ex(card, 0.10, 16, 1, P.PANEL_BORDER)

    text_x = card_x + pad
    text_max_w = card_w - pad * 2

    # Title (top)
    self._title.set_max_width(int(text_max_w))
    title_h = self._title.get_content_height(int(text_max_w)) + P.FS_NAME * 0.3
    self._title.set_rect(rl.Rectangle(text_x, card_y + pad, text_max_w, title_h))
    self._title.render()

    # Description (below title)
    if self._desc.text:
      self._desc.set_max_width(int(text_max_w))
      desc_y = card_y + pad + title_h + 4
      desc_h = self._desc.get_content_height(int(text_max_w)) + P.FS_SUB * 0.5
      self._desc.set_rect(rl.Rectangle(text_x, desc_y, text_max_w, desc_h))
      self._desc.render()


# -----------------------------------------------------------------
# Confirm dialog (slide-to-confirm)
# -----------------------------------------------------------------
class BPConfirmDialog(_BPDialogBase):
  def __init__(self, title: str, icon: rl.Texture, confirm_callback: Callable[[], None],
               exit_on_confirm: bool = True, red: bool = False):
    super().__init__()
    self._confirm_callback = confirm_callback
    self._exit_on_confirm = exit_on_confirm

    Slider = RedBigSlider if red else BigSlider
    self._slider = self._child(Slider(title, icon, confirm_callback=self._on_confirm))
    self._slider.set_enabled(lambda: self.enabled and not self.is_dismissing)

  def _on_confirm(self):
    if self._exit_on_confirm:
      self.dismiss(self._confirm_callback)
    elif self._confirm_callback:
      self._confirm_callback()

  def _update_state(self):
    super()._update_state()
    if self.is_dismissing and not self._slider.confirmed:
      self._slider.reset()

  def _render(self, _):
    r = self._rect
    self._bg.render(r)
    # The slider widget is fixed-size (520x180); center it.
    sw = self._slider.rect.width
    sh = self._slider.rect.height
    sx = r.x + (r.width - sw) / 2
    sy = r.y + (r.height - sh) / 2
    self._slider.render(rl.Rectangle(sx, sy, sw, sh))


# -----------------------------------------------------------------
# Input dialog (BP frame around the existing MICI keyboard)
# -----------------------------------------------------------------
class BPInputDialog(_BPDialogBase):
  BACK_TOUCH_AREA_PERCENTAGE = 0.2  # smaller — most of screen is the keyboard
  BACKSPACE_RATE = 25

  TEXT_BAR_H = 56

  def __init__(self,
               hint: str,
               default_text: str = "",
               minimum_length: int = 1,
               confirm_callback: Callable[[str], None] | None = None,
               auto_return_to_letters: str = ""):
    super().__init__()
    self._hint = hint
    self._minimum_length = minimum_length

    self._keyboard = self._child(MiciKeyboard(auto_return_to_letters=auto_return_to_letters))
    self._keyboard.set_text(default_text)
    self._keyboard.set_enabled(lambda: self.enabled and not self.is_dismissing)

    self._hint_label = UnifiedLabel(hint, font_size=P.FS_SUB, font_weight=FontWeight.MEDIUM,
                                    text_color=P.MUTED, max_width=520, wrap_text=False)
    self._text_label = UnifiedLabel("", font_size=32, font_weight=FontWeight.BOLD,
                                    text_color=P.TEXT, max_width=520, wrap_text=False)

    self._backspace_held_time: float | None = None
    self._backspace_repeated = False
    self._backspace_img = gui_app.texture("icons_mici/settings/keyboard/backspace.png", 36, 30)
    self._enter_img = gui_app.texture("icons_mici/settings/keyboard/enter.png", 56, 46)
    self._enter_disabled_img = gui_app.texture("icons_mici/settings/keyboard/enter_disabled.png", 56, 46)

    self._top_left_button_rect = rl.Rectangle(0, 0, 0, 0)
    self._top_right_button_rect = rl.Rectangle(0, 0, 0, 0)
    self._enter_pressed = False
    self._backspace_pressed = False

    def _do_confirm():
      text = self._keyboard.text()
      self.dismiss((lambda: confirm_callback(text)) if confirm_callback else None)
    self._do_confirm = _do_confirm

  # ---- input ----
  def _handle_mouse_press(self, mouse_pos: MousePos):
    super()._handle_mouse_press(mouse_pos)
    self._backspace_pressed = rl.check_collision_point_rec(mouse_pos, self._top_right_button_rect)
    self._enter_pressed = rl.check_collision_point_rec(mouse_pos, self._top_left_button_rect)
    if self._backspace_pressed:
      self._backspace_repeated = False

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if self._backspace_pressed and rl.check_collision_point_rec(mouse_pos, self._top_right_button_rect):
      # Backspace
      if not self._backspace_repeated:
        self._keyboard.backspace()
      self._backspace_held_time = None
      self._backspace_repeated = False
    elif self._enter_pressed and rl.check_collision_point_rec(mouse_pos, self._top_left_button_rect):
      # Enter
      if len(self._keyboard.text()) >= self._minimum_length:
        self._do_confirm()
    self._enter_pressed = False
    self._backspace_pressed = False

  def _update_state(self):
    super()._update_state()
    if self.is_dismissing:
      self._enter_pressed = False
      self._backspace_pressed = False
      self._backspace_held_time = None
      return

    # Held backspace repeat (mirrors stock BigInputDialog behavior)
    last = gui_app.last_mouse_event
    if last.left_down and self._backspace_pressed and rl.check_collision_point_rec(last.pos, self._top_right_button_rect):
      if self._backspace_held_time is None:
        self._backspace_held_time = rl.get_time()
      if rl.get_time() - self._backspace_held_time > 0.5:
        if gui_app.frame % round(gui_app.target_fps / self.BACKSPACE_RATE) == 0:
          self._keyboard.backspace()
          self._backspace_repeated = True
    else:
      self._backspace_held_time = None
      self._backspace_repeated = False

  def _render(self, _):
    r = self._rect
    self._bg.render(r)

    # ---- Header bar: hint (left), text, backspace (right) ----
    bar_y = r.y + 8
    bar_h = self.TEXT_BAR_H
    pad_x = 14

    # Backspace button (top-right)
    bs_size = 56
    self._top_right_button_rect = rl.Rectangle(
      r.x + r.width - bs_size - pad_x, bar_y, bs_size, bar_h)
    rl.draw_rectangle_rounded(self._top_right_button_rect, 0.30, 8,
                               rl.Color(255, 255, 255, int(0.06 * 255)))
    bs_x = self._top_right_button_rect.x + (bs_size - self._backspace_img.width) / 2
    bs_y = self._top_right_button_rect.y + (bar_h - self._backspace_img.height) / 2
    rl.draw_texture(self._backspace_img, int(bs_x), int(bs_y), P.TEXT)

    # Enter button (top-right just left of backspace) — only when input meets min length
    text = self._keyboard.text()
    enter_enabled = len(text) >= self._minimum_length
    enter_size = 56
    self._top_left_button_rect = rl.Rectangle(
      self._top_right_button_rect.x - enter_size - 8, bar_y, enter_size, bar_h)
    rl.draw_rectangle_rounded(self._top_left_button_rect, 0.30, 8,
                               rl.Color(P.ACCENT.r, P.ACCENT.g, P.ACCENT.b,
                                        int((0.45 if enter_enabled else 0.10) * 255)))
    en_img = self._enter_img if enter_enabled else self._enter_disabled_img
    en_x = self._top_left_button_rect.x + (enter_size - en_img.width) / 2
    en_y = self._top_left_button_rect.y + (bar_h - en_img.height) / 2
    rl.draw_texture(en_img, int(en_x), int(en_y), P.TEXT)

    # Text input area (left of the buttons)
    text_box_w = self._top_left_button_rect.x - r.x - pad_x * 2
    text_box = rl.Rectangle(r.x + pad_x, bar_y, text_box_w, bar_h)
    rl.draw_rectangle_rounded(text_box, 0.30, 8, rl.Color(255, 255, 255, int(0.04 * 255)))

    if not text:
      self._hint_label.set_text(self._hint)
      self._hint_label.set_position(text_box.x + 12, text_box.y + (bar_h - P.FS_SUB) / 2)
      self._hint_label.render()
    else:
      self._text_label.set_text(text if len(text) < 30 else text[-30:])
      self._text_label.set_position(text_box.x + 12, text_box.y + (bar_h - 32) / 2)
      self._text_label.render()

    # ---- Keyboard fills the rest ----
    kb_y = bar_y + bar_h + 6
    kb_h = r.y + r.height - kb_y - 6
    self._keyboard.render(rl.Rectangle(r.x + 4, kb_y, r.width - 8, kb_h))
