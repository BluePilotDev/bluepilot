import pyray as rl
from typing import Union
from collections.abc import Callable
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigParamControl, BigToggle, BigMultiToggle, BigMultiParamToggle
from openpilot.system.ui import text
from openpilot.system.ui.widgets.scroller import DO_ZOOM
from openpilot.system.ui.lib.application import gui_app

SCROLLING_SPEED_PX_S = 50
COMPLICATION_SIZE    = 36
LABEL_COLOR          = rl.WHITE
LABEL_HORIZONTAL_PADDING = 40
COMPLICATION_GREY    = rl.Color(0xAA, 0xAA, 0xAA, 255)
PRESSED_SCALE = 1.15 if DO_ZOOM else 1.07

class BigButtonBP(BigButton):
  def __init__(self, text: str, value: str = "", icon: Union[str, rl.Texture] = "", icon_size: tuple[int, int] = (64, 64),
               scroll: bool = False, tint: rl.Color = rl.WHITE, is_active: Callable[[], bool] = None,
               value_size: int = COMPLICATION_SIZE):
    BigButton.__init__(self, text, value, icon, icon_size, scroll)
    self.tint = tint
    self.get_is_active = is_active

    self._sub_label.set_font_size(value_size)

  def set_checked(self, checked: bool):
    self._checked = checked

  def _load_images(self):
    BigButton._load_images(self)
    self._is_active = gui_app.texture("icons_mici/buttons/toggle_pill_enabled.png", 120, 66, keep_aspect_ratio=False)
    self._is_non_active = gui_app.texture("icons_mici/buttons/toggle_pill_disabled.png", 120, 66, keep_aspect_ratio=False)

  def _render(self, _):
    # draw _txt_default_bg
    txt_bg = self._txt_default_bg
    if not self.enabled:
      txt_bg = self._txt_disabled_bg
    elif self.is_pressed:
      txt_bg = self._txt_pressed_bg

    scale = self._scale_filter.update(PRESSED_SCALE if self.is_pressed else 1.0)
    btn_x = self._rect.x + (self._rect.width * (1 - scale)) / 2
    btn_y = self._rect.y + (self._rect.height * (1 - scale)) / 2
    rl.draw_texture_ex(txt_bg, (btn_x, btn_y), 0, scale, self.tint)

    self._draw_content(btn_y)

    self._draw_active_indicator()

  def _draw_active_indicator(self):
    if self.get_is_active is not None:
      x = self._rect.x + self._rect.width / 2 - self._is_active.width / 2
      y = self._rect.y

      active = self.get_is_active()
      if active:
        rl.draw_texture(self._is_active, int(x), int(y), rl.GREEN)
      else:
        rl.draw_texture(self._is_non_active, int(x), int(y), rl.WHITE)

class BigToggleBP(BigButtonBP, BigToggle):
  def __init__(self, text: str, value: str = "", initial_state: bool = False, toggle_callback: Callable = None,
               tint: rl.Color = rl.WHITE,is_active: Callable[[], bool] = None):
    BigButtonBP.__init__(self, text, value, "", tint=tint, is_active=is_active)
    BigToggle.__init__(self, text, value, initial_state=initial_state, toggle_callback=toggle_callback)

  def _load_images(self):
    BigButtonBP._load_images(self)
    BigToggle._load_images(self)

class BigMultiToggleBP(BigToggleBP, BigMultiToggle):
  def __init__(self, text: str, options: list[str], toggle_callback: Callable = None,
               select_callback: Callable = None, is_active: Callable[[], bool] = None):
    BigToggleBP.__init__(self, text, "", toggle_callback=toggle_callback, is_active=is_active)
    BigMultiToggle.__init__(self, text, options, toggle_callback=toggle_callback, select_callback=select_callback)

  def _load_images(self):
    BigToggleBP._load_images(self)
    BigMultiToggle._load_images(self)

  def _get_label_font_size(self):
    font_size = BigMultiToggle._get_label_font_size(self)
    return font_size - 10

  def _draw_content(self, btn_y: float):
    # don't draw pill from BigToggle
    BigToggleBP._draw_content(self, btn_y)

    checked_idx = self._options.index(self.value)

    x = self._rect.x + self._rect.width - self._txt_enabled_toggle.width
    y = self._rect.y

    num_options = len(self._options)
    for i in range(num_options):
      dist = 35
      if num_options > 4:
        dist = self._rect.height / int(num_options + 1)
      self._draw_pill(x, y, checked_idx == i)
      y += dist

class BigMultiParamToggleBP(BigMultiToggleBP, BigMultiParamToggle):
  def __init__(self, text: str, param: str, options: list[str], toggle_callback: Callable = None,
               select_callback: Callable = None, value_size: int = 30, is_active: Callable[[], bool] = None):
    BigMultiToggleBP.__init__(self, text, options, toggle_callback, select_callback, is_active=is_active)
    BigMultiParamToggle.__init__(self, text, param, options, toggle_callback, select_callback)
    self._sub_label.set_font_size(value_size)


  def _load_images(self):
    BigMultiToggleBP._load_images(self)
    BigMultiParamToggle._load_images(self) if hasattr(BigMultiParamToggle, '_load_images') else None

  def _load_value(self):
    self.set_value(self._options[self._params.get(self._param) or 0])


class BigMultiParamToggleBoolBP(BigMultiParamToggleBP):
  """Like BigMultiParamToggleBP but for a BOOL param: index 0 = False, index 1 = True."""

  def _load_value(self):
    idx = 1 if self._params.get_bool(self._param) else 0
    self.set_value(self._options[idx])

  def _handle_mouse_release(self, mouse_pos):
    # Advance option and update display (BigMultiToggle), but do NOT call BigMultiParamToggle's
    # put_nonblocking(self._param, new_idx) — param is BOOL, so we must use put_bool_nonblocking.
    BigMultiToggle._handle_mouse_release(self, mouse_pos)
    new_idx = self._options.index(self.value)
    self._params.put_bool_nonblocking(self._param, bool(new_idx))


class BigParamControlBP(BigToggleBP, BigParamControl):
  def __init__(self, text: str, param: str, is_active_param: str = None, toggle_callback: Callable = None,
               tint: rl.Color = rl.WHITE):
    BigToggleBP.__init__(self, text, "", toggle_callback=toggle_callback, tint=tint,
                         is_active=(lambda: self.params.get_bool(is_active_param)) if is_active_param is not None else None)
    BigParamControl.__init__(self, text, param, toggle_callback=toggle_callback)
    self.set_checked(self.params.get_bool(self.param, False))

  def _load_images(self):
    BigToggleBP._load_images(self)
    BigParamControl._load_images(self) if hasattr(BigParamControl, '_load_images') else None
