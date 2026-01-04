
import pyray as rl
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigInputDialog
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, MousePos

CONTENT_MARGIN = 20
LINE_L = 40
LINE_W = 8
LABEL_HORIZONTAL_PADDING = 40

class BigParamFloatControl(BigButton):
  def __init__(self, text: str, param: str, is_active_param: str = None, min: float = None, max: float = None, step: float = 0.05, tint: rl.Color = rl.WHITE):
    super().__init__(text, "", tint=tint, is_active=(lambda: Params().get_bool(is_active_param)) if is_active_param is not None else None)
    self.min = min
    self.max = max
    self.step = step

    self._sub_label.font_size = 22

    self.margin = self._rect.width * 0.1
    self.rect_size = LINE_L + 2 * CONTENT_MARGIN

    self.param = param
    self.params = Params()
    self.set_click_callback(self._on_click)
    self.update_label()

  def _on_click(self):
    if self.min is not None or self.max is not None:
      message = f"({self.min}-{self.max})"
    else:
      message = "enter a numberic value..."

    dlg = BigInputDialog(message, str(self.get_param()),
                         confirm_callback=self._callback, show_special_keys=True, minimum_length=0)
    gui_app.set_modal_overlay(dlg)

  def _callback(self, password: str):
    if password:
      try:
        float_value = float(password)
        self.set_param(float_value)
      except ValueError:
        pass
    else:
      #revert to default
      self.params.remove(self.param)
      self.update_label()

  def get_param(self) -> float:
    try:
      return float(self.params.get(self.param, return_default=True))
    except (TypeError, ValueError):
      return 0.0

  def set_param(self, value: float):
    if self.min is not None and value < self.min:
      value = self.min
    elif self.max is not None and value > self.max:
      value = self.max

    self.params.put_nonblocking(self.param, value)
    self.update_label(value)

  def update_label(self, value: float = None):
    if value is None:
      value = self.get_param()
    self.set_value(f"{round(value,4)}")

  def _get_label_font_size(self):
    font_size = super()._get_label_font_size()
    return font_size - 6

  def _render(self, _):
    super()._render(_)

    self.left = self._rect.x + self.margin
    self.right = self._rect.x + self._rect.width - self.margin
    self.top = self._rect.y + self.margin

    self.minus_hit_rect = rl.Rectangle(
      self.left - CONTENT_MARGIN, self.top - self.rect_size / 2, self.rect_size, self.rect_size
    )
    self.plus_hit_rect = rl.Rectangle(
      self.right - self.rect_size / 2 - CONTENT_MARGIN, self.top - self.rect_size / 2, self.rect_size, self.rect_size
    )

    # rl.draw_rectangle_lines_ex(self.minus_hit_rect, 1, rl.RED)
    # rl.draw_rectangle_lines_ex(self.plus_hit_rect, 1, rl.GREEN)

    rl.draw_line_ex((self.left,self.top), (self.left+LINE_L, self.top), LINE_W, rl.WHITE)

    rl.draw_line_ex((self.right-LINE_L,self.top), (self.right, self.top), LINE_W, rl.WHITE)
    m = self.right - LINE_L/2
    rl.draw_line_ex((m,self.top-LINE_L/2), (m, self.top+LINE_L/2), LINE_W, rl.WHITE)



  def minus_clicked(self):
    self.set_param(self.get_param() - self.step)

  def plus_clicked(self):
    self.set_param(self.get_param() + self.step)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if rl.check_collision_point_rec(mouse_pos, self.minus_hit_rect):
      self.minus_clicked()
    elif rl.check_collision_point_rec(mouse_pos, self.plus_hit_rect):
      self.plus_clicked()
    else:
      super()._handle_mouse_release(mouse_pos)
