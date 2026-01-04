
import pyray as rl
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigInputDialog
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app, MousePos

class BigParamFloatControl(BigButton):
  def __init__(self, text: str, param: str, min: float = None, max: float = None, tint: rl.Color = rl.WHITE):
    super().__init__(text, "", tint=tint)
    self.label_text = text
    self.min = min
    self.max = max
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
    self.set_text(f"{self.label_text} [{round(value,4)}]")