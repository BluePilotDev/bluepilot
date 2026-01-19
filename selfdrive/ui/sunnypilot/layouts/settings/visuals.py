"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp


class VisualsLayout(Widget):
  def __init__(self):
    super().__init__()

    self._params = Params()
    items = self._initialize_items()
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _initialize_items(self):
    chevron_toggle = toggle_item_sp(
      lambda: tr("Enable Lead Car Information"),
      description=lambda: tr("Enable Lead Car Information"),
      initial_state=self._params.get("ChevronInfo") != 0,
      callback=self._on_enable_chevron,
    )

    return [chevron_toggle]

  def _on_enable_chevron(self,state):
    if state:
      self._params.put("ChevronInfo", 4)
    else:
      self._params.put("ChevronInfo", 0)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    self._scroller.show_event()
