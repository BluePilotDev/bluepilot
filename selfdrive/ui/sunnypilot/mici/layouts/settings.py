"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.layouts.settings import settings as OP
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.sunnylink import SunnylinkLayoutMici
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
import pyray as rl
from openpilot.system.ui.lib.application import gui_app
# BluePilot: vehicle selector, BP settings panel, and BigButtonBP override
from openpilot.common.bluepilot import is_bluepilot
if is_bluepilot():
  from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP as BigButton
  from openpilot.selfdrive.ui.bp.mici.layouts.settings.bluepilot import BluePilotLayoutMici
  from openpilot.selfdrive.ui.bp.mici.layouts.settings.vehicle_mici import VehicleLayoutMici

ICON_SIZE = 70


class SettingsLayoutSP(OP.SettingsLayout):
  def __init__(self):
    OP.SettingsLayout.__init__(self)

    sunnylink_panel = SunnylinkLayoutMici(back_callback=gui_app.pop_widget)
    sunnylink_btn = BigButton("sunnylink", "", gui_app.texture("icons_mici/settings/developer/ssh.png", ICON_SIZE, ICON_SIZE))
    sunnylink_btn.set_click_callback(lambda: gui_app.push_widget(sunnylink_panel))

    models_panel = ModelsLayoutMici(back_callback=gui_app.pop_widget)
    models_btn = BigButton("models", "", gui_app.texture("../../sunnypilot/selfdrive/assets/offroad/icon_models.png", ICON_SIZE, ICON_SIZE))
    models_btn.set_click_callback(lambda: gui_app.push_widget(models_panel))

    items = self._scroller._items.copy()

    items.insert(1, sunnylink_btn)
    items.insert(2, models_btn)

    # BluePilot: insert vehicle fingerprint selector and BP settings buttons
    if is_bluepilot():
      vehicle_panel = VehicleLayoutMici(back_callback=gui_app.pop_widget)
      vehicle_btn = BigButton("vehicle", "", gui_app.texture("../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png", ICON_SIZE, ICON_SIZE))
      vehicle_btn.set_click_callback(lambda: gui_app.push_widget(vehicle_panel))

      bp_panel = BluePilotLayoutMici(back_callback=gui_app.pop_widget)
      bluepilot_btn = BigButton("bluepilot", "", gui_app.texture("icons_mici/settings/car_icon.png", ICON_SIZE, ICON_SIZE))
      bluepilot_btn.set_click_callback(lambda: gui_app.push_widget(bp_panel))

      items.insert(3, vehicle_btn)
      items.insert(4, bluepilot_btn)

    self._scroller._items.clear()
    for item in items:
      self._scroller.add_widget(item)
