import pyray as rl
from enum import IntEnum
import cereal.messaging as messaging
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.sidebar import Sidebar, SIDEBAR_WIDTH
from openpilot.selfdrive.ui.layouts.home import HomeLayout
from openpilot.selfdrive.ui.layouts.settings.settings import SettingsLayout, PanelType
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.ui_state import device, ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.layouts.onboarding import OnboardingWindow

# BluePilot: override sidebar, home layout, onroad overlays, and add debug panel
from openpilot.common.bluepilot import is_bluepilot
if is_bluepilot():
  from bluepilot.ui.widgets.sidebar import SidebarBP as Sidebar
  from bluepilot.ui.lib.constants import BPConstants
  SIDEBAR_WIDTH = BPConstants.SIDEBAR_WIDTH
  from bluepilot.ui.layouts.home_bp import HomeLayoutBP as HomeLayout
  from openpilot.selfdrive.ui.bp.onroad.augmented_road_view_bp import AugmentedRoadViewBP as AugmentedRoadView
  from bluepilot.ui.widgets.debug import ControlsDebugPanel

if gui_app.sunnypilot_ui():
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.settings import SettingsLayoutSP as SettingsLayout


class MainState(IntEnum):
  HOME = 0
  SETTINGS = 1
  ONROAD = 2


class MainLayout(Widget):
  def __init__(self):
    super().__init__()

    self._pm = messaging.PubMaster(['bookmarkButton'])

    self._sidebar = Sidebar()
    self._current_mode = MainState.HOME
    self._prev_onroad = False

    # Initialize layouts
    self._layouts = {MainState.HOME: HomeLayout(), MainState.SETTINGS: SettingsLayout(), MainState.ONROAD: AugmentedRoadView()}

    self._sidebar_rect = rl.Rectangle(0, 0, 0, 0)
    self._content_rect = rl.Rectangle(0, 0, 0, 0)

    # BluePilot: debug panel overlay for onroad view
    if is_bluepilot():
      self._debug_panel = ControlsDebugPanel()
      self._debug_toggled_this_frame = False

    # Set callbacks
    self._setup_callbacks()

    gui_app.push_widget(self)

    # Start onboarding if terms or training not completed, make sure to push after self
    self._onboarding_window = OnboardingWindow()
    if not self._onboarding_window.completed:
      gui_app.push_widget(self._onboarding_window)

  def _render(self, _):
    if is_bluepilot():
      self._debug_toggled_this_frame = False
    self._handle_onroad_transition()
    self._render_main_content()

  def _setup_callbacks(self):
    self._sidebar.set_callbacks(on_settings=self._on_settings_clicked,
                                on_flag=self._on_bookmark_clicked,
                                # BluePilot: sidebar debug and network buttons
                                **({"on_debug": self._on_debug_clicked,
                                    "on_network": lambda: self.open_settings(PanelType.NETWORK)} if is_bluepilot() else {}),
                                open_settings=lambda: self.open_settings(PanelType.TOGGLES))
    self._layouts[MainState.HOME]._setup_widget.set_open_settings_callback(lambda: self.open_settings(PanelType.FIREHOSE))
    self._layouts[MainState.HOME].set_settings_callback(lambda: self.open_settings(PanelType.TOGGLES))
    # BluePilot: model info click opens Models settings panel
    if is_bluepilot() and hasattr(self._layouts[MainState.HOME], 'set_model_settings_callback'):
      self._layouts[MainState.HOME].set_model_settings_callback(lambda: self.open_settings(PanelType.MODELS))
    self._layouts[MainState.SETTINGS].set_callbacks(on_close=self._set_mode_for_state)
    self._layouts[MainState.ONROAD].set_click_callback(self._on_onroad_clicked)
    device.add_interactive_timeout_callback(self._set_mode_for_state)

  def _update_layout_rects(self):
    self._sidebar_rect = rl.Rectangle(self._rect.x, self._rect.y, SIDEBAR_WIDTH, self._rect.height)

    x_offset = SIDEBAR_WIDTH if self._sidebar.is_visible else 0
    self._content_rect = rl.Rectangle(self._rect.y + x_offset, self._rect.y, self._rect.width - x_offset, self._rect.height)

  def _handle_onroad_transition(self):
    if ui_state.started != self._prev_onroad:
      self._prev_onroad = ui_state.started

      self._set_mode_for_state()

  def _set_mode_for_state(self):
    if ui_state.started:
      # Don't hide sidebar from interactive timeout
      if self._current_mode != MainState.ONROAD:
        self._sidebar.set_visible(False)
      self._set_current_layout(MainState.ONROAD)
    else:
      self._set_current_layout(MainState.HOME)
      self._sidebar.set_visible(True)

  def _set_current_layout(self, layout: MainState):
    if layout != self._current_mode:
      self._layouts[self._current_mode].hide_event()
      self._current_mode = layout
      self._layouts[self._current_mode].show_event()

  def open_settings(self, panel_type: PanelType):
    self._layouts[MainState.SETTINGS].set_current_panel(panel_type)
    self._set_current_layout(MainState.SETTINGS)
    self._sidebar.set_visible(False)

  def _on_settings_clicked(self):
    self.open_settings(PanelType.DEVICE)

  def _on_bookmark_clicked(self):
    user_bookmark = messaging.new_message('bookmarkButton')
    user_bookmark.valid = True
    self._pm.send('bookmarkButton', user_bookmark)

  def _on_onroad_clicked(self):
    # BluePilot: suppress onroad clicks when debug panel is visible
    if is_bluepilot() and (self._debug_toggled_this_frame or self._debug_panel.is_panel_visible):
      return
    self._sidebar.set_visible(not self._sidebar.is_visible)

  # BluePilot: toggle onroad debug panel from sidebar button
  def _on_debug_clicked(self):
    self._debug_panel.toggle_visibility()
    self._debug_toggled_this_frame = True

  def _render_main_content(self):
    # Render sidebar
    if self._sidebar.is_visible:
      self._sidebar.render(self._sidebar_rect)

    content_rect = self._content_rect if self._sidebar.is_visible else self._rect
    self._layouts[self._current_mode].render(content_rect)

    # BluePilot: render debug panel overlay on top of onroad view
    if is_bluepilot() and self._current_mode == MainState.ONROAD and self._debug_panel.is_panel_visible:
      self._debug_panel.render(content_rect)
