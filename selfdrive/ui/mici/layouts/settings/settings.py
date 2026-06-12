from openpilot.common.params import Params
from openpilot.common.bluepilot import is_bluepilot
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.layouts.settings.toggles import TogglesLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.network.network_layout import NetworkLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.device import DeviceLayoutMici, PairBigButton
from openpilot.selfdrive.ui.mici.layouts.settings.developer import DeveloperLayoutMici
from openpilot.selfdrive.ui.mici.layouts.settings.firehose import FirehoseLayout
from openpilot.system.ui.lib.application import gui_app, FontWeight
if is_bluepilot():
  from openpilot.selfdrive.ui.mici.layouts.settings.bluepilot import BluePilotLayoutMici
  from openpilot.selfdrive.ui.mici.layouts.settings.vehicle import VehicleLayoutMici


class SettingsBigButton(BigButton):
  def _get_label_font_size(self):
    return 64


class SettingsLayout(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()

    toggles_panel = TogglesLayoutMici()
    toggles_btn = SettingsBigButton("toggles", "", gui_app.texture("icons_mici/settings.png", 64, 64))
    toggles_btn.set_click_callback(lambda: gui_app.push_widget(toggles_panel))

    network_panel = NetworkLayoutMici()
    network_btn = SettingsBigButton("network", "", gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 76, 56))
    network_btn.set_click_callback(lambda: gui_app.push_widget(network_panel))

    device_panel = DeviceLayoutMici()
    device_btn = SettingsBigButton("device", "", gui_app.texture("icons_mici/settings/device_icon.png", 72, 58))
    device_btn.set_click_callback(lambda: gui_app.push_widget(device_panel))

    developer_panel = DeveloperLayoutMici()
    developer_btn = SettingsBigButton("developer", "", gui_app.texture("icons_mici/settings/developer_icon.png", 64, 60))
    developer_btn.set_click_callback(lambda: gui_app.push_widget(developer_panel))

    firehose_panel = FirehoseLayout()
    firehose_btn = SettingsBigButton("firehose", "", gui_app.texture("icons_mici/settings/firehose.png", 52, 62))
    firehose_btn.set_click_callback(lambda: gui_app.push_widget(firehose_panel))

    # BluePilot: add MICI-local Vehicle and BluePilot settings panels without
    # depending on the sunnypilot SettingsLayoutSP wrapper.
    if is_bluepilot():
      vehicle_panel = VehicleLayoutMici()
      vehicle_btn = SettingsBigButton("vehicle", "", gui_app.texture("../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png", 70, 70))
      vehicle_btn.set_click_callback(lambda: gui_app.push_widget(vehicle_panel))

      bluepilot_panel = BluePilotLayoutMici()
      bluepilot_btn = SettingsBigButton("bluepilot", "", gui_app.texture("icons_mici/settings/car_icon.png", 70, 70))
      bluepilot_btn.set_click_callback(lambda: gui_app.push_widget(bluepilot_panel))
    # End BluePilot

    items = [
      toggles_btn,
      network_btn,
      device_btn,
      PairBigButton(),
      #BigDialogButton("manual", "", "icons_mici/settings/manual_icon.png", "Check out the mici user\nmanual at comma.ai/setup"),
      firehose_btn,
      developer_btn,
    ]
    if is_bluepilot():
      items.insert(3, vehicle_btn)
      items.insert(4, bluepilot_btn)

    self._scroller.add_widgets(items)

    self._font_medium = gui_app.font(FontWeight.MEDIUM)
