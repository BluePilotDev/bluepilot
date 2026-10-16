"""BluePilot: MICI connect backend selector (Comma Connect / Konik Stable / Offline Mode)."""

from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets.scroller import NavScroller
from bluepilot.backend_switch import BACKEND_LABELS, BACKENDS, PARAM_KEY


class ConnectBackendSelectMici(NavScroller):
  """Horizontal panel of connect backend options. Tap one to select and dismiss."""

  def __init__(self, on_selected: Callable[[int], None] | None = None,
               on_dismiss: Callable[[], None] | None = None):
    super().__init__()
    self.set_back_callback(self._on_back)
    self._params = Params()
    self._on_selected = on_selected
    self._on_dismiss = on_dismiss

    for idx, backend in enumerate(BACKENDS):
      btn = BigButtonBP(BACKEND_LABELS[backend], "")
      btn.set_click_callback(lambda i=idx: self._select(i))
      self._scroller.add_widget(btn)

  def _select(self, idx: int):
    self._params.put(PARAM_KEY, idx)
    gui_app.pop_widget()
    if self._on_selected:
      self._on_selected(idx)
    elif self._on_dismiss:
      self._on_dismiss()

  def _on_back(self):
    gui_app.pop_widget()
    if self._on_dismiss:
      self._on_dismiss()
