import pyray as rl
from enum import IntEnum
import cereal.messaging as messaging
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.layouts.sidebar import Sidebar, SIDEBAR_WIDTH  # noqa: F401
from openpilot.selfdrive.ui.layouts.home import HomeLayout
from openpilot.selfdrive.ui.layouts.settings import settings as _settings_mod
from openpilot.selfdrive.ui.layouts.settings.settings import SettingsLayout
from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView  # noqa: F401

# BluePilot: START - BP sidebar, home layout, and onroad overlays
from bluepilot.ui.widgets.sidebar import SidebarBP as Sidebar  # noqa: F811
from bluepilot.ui.lib.constants import BPConstants
SIDEBAR_WIDTH = BPConstants.SIDEBAR_WIDTH  # noqa: F811
from bluepilot.ui.layouts.home_bp import HomeLayoutBP as HomeLayout  # noqa: F811
from openpilot.selfdrive.ui.bp.onroad.augmented_road_view_bp import AugmentedRoadViewBP as AugmentedRoadView  # noqa: F811
from bluepilot.ui.widgets.debug import ControlsDebugPanel
# BluePilot: END - BP sidebar, home layout, and onroad overlays
from openpilot.selfdrive.ui.ui_state import device, ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.selfdrive.ui.layouts.onboarding import OnboardingWindow

if gui_app.sunnypilot_ui():
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.settings import SettingsLayoutSP as SettingsLayout  # noqa: F811


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

    # BluePilot: Debug panel overlay for onroad view
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
    self._debug_toggled_this_frame = False
    self._handle_onroad_transition()
    self._render_main_content()

  def _setup_callbacks(self):
    self._sidebar.set_callbacks(on_settings=self._on_settings_clicked,
                                on_flag=self._on_bookmark_clicked,
                                on_debug=self._on_debug_clicked,
                                on_network=lambda: self.open_settings(_settings_mod.PanelType.NETWORK),
                                open_settings=lambda: self.open_settings(_settings_mod.PanelType.TOGGLES))
    self._layouts[MainState.HOME]._setup_widget.set_open_settings_callback(lambda: self.open_settings(_settings_mod.PanelType.FIREHOSE))
    self._layouts[MainState.HOME].set_settings_callback(lambda: self.open_settings(_settings_mod.PanelType.TOGGLES))
    # BluePilot: model info click -> Models settings panel
    if hasattr(self._layouts[MainState.HOME], 'set_model_settings_callback'):
      self._layouts[MainState.HOME].set_model_settings_callback(lambda: self.open_settings(_settings_mod.PanelType.MODELS))
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

  def open_settings(self, panel_type: _settings_mod.PanelType):
    self._layouts[MainState.SETTINGS].set_current_panel(panel_type)
    self._set_current_layout(MainState.SETTINGS)
    self._sidebar.set_visible(False)

  def _on_settings_clicked(self):
    self.open_settings(_settings_mod.PanelType.DEVICE)

  def _on_bookmark_clicked(self):
    user_bookmark = messaging.new_message('bookmarkButton')
    user_bookmark.valid = True
    self._pm.send('bookmarkButton', user_bookmark)

  def _on_onroad_clicked(self):
    # BluePilot: When debug panel is visible, suppress onroad clicks entirely.
    # The debug panel has its own close button (X). This prevents click-through
    # from the debug panel's tab bar to the onroad view underneath.
    if self._debug_toggled_this_frame or self._debug_panel.is_panel_visible:
      return
    self._sidebar.set_visible(not self._sidebar.is_visible)

  def _on_debug_clicked(self):
    """BluePilot: Toggle the onroad debug panel from the sidebar debug button."""
    self._debug_panel.toggle_visibility()
    self._debug_toggled_this_frame = True

  def _render_main_content(self):
    # Render sidebar
    if self._sidebar.is_visible:
      self._sidebar.render(self._sidebar_rect)

    content_rect = self._content_rect if self._sidebar.is_visible else self._rect
    self._layouts[self._current_mode].render(content_rect)

    # BluePilot: Render debug panel overlay on top of onroad view
    if self._current_mode == MainState.ONROAD and self._debug_panel.is_panel_visible:
      self._debug_panel.render(content_rect)
