"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.settings import settings as OP
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP as BigButton
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.sunnylink import SunnylinkLayoutMici
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
from openpilot.system.ui.lib.application import gui_app
# BluePilot: START - BP settings tab import
from openpilot.selfdrive.ui.bp.mici.layouts.settings.bluepilot import BluePilotLayoutMici
# BluePilot: END - BP settings tab import

ICON_SIZE = 70


class SettingsLayoutSP(OP.SettingsLayout):
  def __init__(self):
    OP.SettingsLayout.__init__(self)

    sunnylink_panel = SunnylinkLayoutMici(back_callback=gui_app.pop_widget)
    sunnylink_btn = BigButton("sunnylink", "", "icons_mici/settings/developer/ssh.png")
    sunnylink_btn.set_click_callback(lambda: gui_app.push_widget(sunnylink_panel))

    models_panel = ModelsLayoutMici(back_callback=gui_app.pop_widget)
    models_btn = BigButton("models", "", "../../sunnypilot/selfdrive/assets/offroad/icon_models.png")
    models_btn.set_click_callback(lambda: gui_app.push_widget(models_panel))

    # BluePilot: START - BP settings button and panel
    bp_panel = BluePilotLayoutMici(back_callback=gui_app.pop_widget)
    bluepilot_btn = BigButton("bluepilot", "", "icons_mici/settings/car_icon.png", tint=rl.BLUE)
    bluepilot_btn.set_click_callback(lambda: gui_app.push_widget(bp_panel))
    # BluePilot: END - BP settings button and panel

    items = self._scroller._items.copy()

    items.insert(1, sunnylink_btn)
    items.insert(2, models_btn)
    # BluePilot: START - insert BP button into scroller
    items.insert(3, bluepilot_btn)
    # BluePilot: END - insert BP button into scroller
    self._scroller._items.clear()
    for item in items:
      self._scroller.add_widget(item)
