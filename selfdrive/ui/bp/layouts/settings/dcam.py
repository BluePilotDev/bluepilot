"""C3X submenu for independent DCam activation triggers."""

import pyray as rl

from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets.list_view import toggle_item
from openpilot.system.ui.widgets.scroller_tici import Scroller


class DcamTriggersLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._options = []
    for label, description, key in (
      ('When Turn Signal Active', 'Show DCam view when turn signal active', 'BPCroppedDcamTurnSignal'),
      ('When BSM Active', 'Show DCam view when vehicle Blind-Spot Monitoring System active', 'BPCroppedDcamBSM'),
    ):
      self._options.append(toggle_item(
        tr(label), tr(description), initial_state=self._params.get_bool(key),
        callback=lambda state, param=key: self._params.put_bool(param, state),
        enabled=lambda: self._params.get_bool('BPCroppedDcam'),
      ))
    self._scroller = self._child(Scroller(self._options, line_separator=True))
    self._back = self._child(Button(tr('Back'), click_callback=gui_app.pop_widget))

  def _render(self, rect):
    rl.draw_rectangle_rec(rect, rl.Color(20, 20, 20, 255))
    gui_label(rl.Rectangle(rect.x + 50, rect.y + 40, rect.width - 100, 90), tr('DCam Activation Options'), 64)
    self._scroller.render(rl.Rectangle(rect.x + 50, rect.y + 160, rect.width - 100, rect.height - 350))
    self._back.render(rl.Rectangle(rect.x + 50, rect.y + rect.height - 150, rect.width - 100, 100))
