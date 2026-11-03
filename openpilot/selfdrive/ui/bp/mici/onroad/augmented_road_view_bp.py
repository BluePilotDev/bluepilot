import time
import pyray as rl
from opendbc.car.structs import car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.system.ui.lib.application import gui_app
from openpilot.selfdrive.ui.mici.onroad import SIDE_PANEL_WIDTH
from openpilot.selfdrive.ui.mici.onroad.augmented_road_view import AugmentedRoadView
from openpilot.selfdrive.ui.bp.mici.onroad.cameraview_bp import MiciCameraViewBP
from openpilot.selfdrive.ui.bp.mici.onroad.model_renderer_bp import ModelRendererBP
from openpilot.selfdrive.ui.bp.onroad.blindspot_renderer import BlindspotRendererMixin
from openpilot.selfdrive.ui.bp.mici.onroad.hud_renderer_bp import MiciHudRendererBP
from openpilot.selfdrive.ui.bp.onroad.driver_state_bp import DriverStateRendererBP
from openpilot.selfdrive.ui.bp.lib.dm_icon_style import DMIconStyle
from openpilot.selfdrive.ui.bp.mici.onroad.complication import MiciComplication
from openpilot.selfdrive.ui.bp.mici.onroad.confidence_ball_bp import ConfidenceBallMiciBP
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log
# BluePilot: swipe-down shortcut to lateral debug screen
from openpilot.selfdrive.ui.bp.mici.onroad.lateral_debug_mici import LateralDebugMici
from openpilot.selfdrive.ui.bp.mici.onroad.rad_racer_mici import RadRacerThemeMici
from openpilot.system.ui.widgets import Widget
# BluePilot: unified theme selector (BPThemePack param)
from openpilot.selfdrive.ui.bp.lib import theme_pack, theme_scene

# BluePilot: Margin to keep confidence ball inside the MICI rounded border
MICI_BALL_BORDER_MARGIN = 25  # half of 50px MICI border thickness

_SWIPE_DOWN_THRESHOLD = 80  # minimum downward travel (px) to trigger lateral debug


class _VerticalSwipeDetector(Widget):
  """Transparent overlay that detects swipe-down gestures.

  Rendered directly inside _render (not via the horizontal Scroller), so it has
  no scroller touch_valid_callback and _handle_mouse_event fires regardless of
  whether the scroller entered MANUAL_SCROLL due to minor horizontal drift.
  Mirrors the BookmarkIcon pattern used for the bookmark swipe.
  """

  def __init__(self, callback):
    super().__init__()
    self._callback = callback
    self._interacting = False
    self._press_x = 0.0
    self._press_y = 0.0
    self._tracking = False

  def interacting(self) -> bool:
    interacting, self._interacting = self._interacting, False
    return interacting

  def _handle_mouse_event(self, mouse_event):
    if mouse_event.left_pressed:
      self._press_x = mouse_event.pos.x
      self._press_y = mouse_event.pos.y
      self._tracking = True
    elif mouse_event.left_released and self._tracking:
      self._tracking = False
      dy = mouse_event.pos.y - self._press_y
      dx = abs(mouse_event.pos.x - self._press_x)
      if dy > _SWIPE_DOWN_THRESHOLD and dy > dx:
        self._interacting = True
        self._callback()

  def _render(self, rect):
    pass  # transparent — visual is handled by parent


class MiciAugmentedRoadViewBP(MiciCameraViewBP, AugmentedRoadView, BlindspotRendererMixin):
  """BluePilot MICI AugmentedRoadView with blindspot indicators, BP HUD, and complication."""

  BLIND_SPOT_WIDTH = 125  # Narrower for MICI's smaller screen

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._init_blindspot()
    self._bp_params = Params()
    self._fade_alpha_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)

    # BluePilot: Replace HUD renderer with BP version (brake coloring + powerflow)
    self._hud_renderer = MiciHudRendererBP()
    self._driver_state_renderer = DriverStateRendererBP(DMIconStyle.COMMA_4)

    # BluePilot: Replace confidence ball with BP version on the left (MADS beam + enhanced coloring)
    self._confidence_ball = ConfidenceBallMiciBP()

    # BluePilot: Add lead car complication widget
    self._complication = MiciComplication()

    self._model_renderer = ModelRendererBP()
    self._lat_debug: LateralDebugMici | None = None
    self._swipe_detector = _VerticalSwipeDetector(self._on_swipe_down)

    # BluePilot: Rad Racer 8-bit theme (MICI-scaled; no gauge cluster on the small screen)
    self._rad_racer_theme = RadRacerThemeMici()

  def _on_swipe_down(self):
    if not ui_state.is_onroad():
      return
    # Guard against double-push. When the car leaves standstill, main.py calls
    # pop_widgets_to() which dismisses LateralDebugMici without invoking back_callback,
    # so a bool flag would get stranded True. Checking widget_in_stack() handles that.
    if self._lat_debug is not None and gui_app.widget_in_stack(self._lat_debug):
      return

    def _dismiss():
      self._lat_debug = None
      gui_app.pop_widget()

    self._lat_debug = LateralDebugMici(back_callback=_dismiss)
    gui_app.push_widget(self._lat_debug)

  def _handle_mouse_release(self, mouse_pos):
    # BluePilot: suppress click-to-home when a swipe-down was detected by the detector
    if not self._swipe_detector.interacting():
      super()._handle_mouse_release(mouse_pos)

  def _render(self, _):
    """Override render to place confidence ball on left, offset driver state, and conditionally hide border."""
    bp_ui_log.tick()  # refresh BPUIDebugLog enabled state (mirrors TICI; MICI had no tick, so the toggle was inert)
    start_draw = time.monotonic()
    self._switch_stream_if_needed(ui_state.sm)
    self._update_calibration()

    # Create inner content area (camera view, excluding side panel)
    self._content_rect = rl.Rectangle(
      self.rect.x,
      self.rect.y,
      self.rect.width - SIDE_PANEL_WIDTH,
      self.rect.height,
    )

    bp_ui_log.scissor("MiciAugRoadView", "begin",
                       x=int(self._content_rect.x), y=int(self._content_rect.y),
                       w=int(self._content_rect.width), h=int(self._content_rect.height))
    rl.begin_scissor_mode(
      int(self._content_rect.x),
      int(self._content_rect.y),
      int(self._content_rect.width),
      int(self._content_rect.height)
    )

    # Render the base camera view. Minimal Driving View suppression lives in MiciCameraViewBP.
    MiciCameraViewBP._render(self, self._content_rect)

    # BluePilot: unified theme scene dispatch — Rad Racer draws inline on MICI (no full
    # takeover: the normal HUD stays); pack scenes get background + foreground passes.
    _scene = theme_scene.active_scene()
    _rad_racer = _scene is not None and _scene.replaces_hud()
    if _rad_racer:
      self._model_renderer.prepare_projection(self._content_rect)
      self._rad_racer_theme.render_background(self._content_rect, self._model_renderer)
    elif _scene is not None:
      _scene.draw_background(self._content_rect, getattr(self, "_bp_hide_camera_view", False))

    # Model overlays
    self._model_renderer.render(self._content_rect)

    # BluePilot: Rad Racer sprites (ego + leads) over the road; no gauge cluster on MICI,
    # so the ego car anchors to the bottom edge of the content rect instead.
    if _rad_racer:
      self._rad_racer_theme.render_foreground(
        self._content_rect, self._model_renderer,
        self._content_rect.y + self._content_rect.height - 4)

    # Fade out bottom overlay (only when engaged)
    fade_alpha = self._fade_alpha_filter.update(ui_state.status != UIStatus.DISENGAGED)
    if fade_alpha > 1e-2:
      rl.draw_texture_ex(self._fade_texture, rl.Vector2(self._content_rect.x, self._content_rect.y), 0.0, 1.0,
                         rl.Color(255, 255, 255, int(255 * fade_alpha)))

    alert_to_render, not_animating_out = self._alert_renderer.will_render()

    # BluePilot: Driver monitor pushed right by ball width
    should_draw_dmoji = (not self._hud_renderer.drawing_top_icons() and ui_state.is_onroad() and
                         (ui_state.status != UIStatus.DISENGAGED or ui_state.always_on_dm))
    self._driver_state_renderer.set_should_draw(should_draw_dmoji)
    self._driver_state_renderer.set_position(self._rect.x + 16, self._rect.y + 10)
    self._driver_state_renderer.render()

    # HUD and alerts
    self._hud_renderer.set_can_draw_top_icons(alert_to_render is None)
    self._hud_renderer.set_wheel_critical_icon(alert_to_render is not None and not not_animating_out and
                                               alert_to_render.visual_alert == car.CarControl.HUDControl.VisualAlert.steerRequired)
    # BluePilot: theme scene foreground — decor + near particles, always under alerts.
    # hero_y_scale lifts the mascot above the MICI fade band + torque bar.
    if _scene is not None and not _rad_racer:
      _scene.hero_y_scale = 0.90
      _scene.draw_foreground(self._content_rect, getattr(self, "_bp_hide_camera_view", False))
    if ui_state.started:
      self._alert_renderer.render(self._content_rect)
    self._hud_renderer.render(self._content_rect)

    bp_ui_log.scissor("MiciAugRoadView", "end")
    rl.end_scissor_mode()

    # BluePilot: Conditionally draw MICI rounded border
    if not self._bp_params.get_bool("BPHideOnroadBorder"):
      rl.draw_rectangle_rounded_lines_ex(self._content_rect, 0.2 * 1.02, 10, 50, rl.BLACK)

    # BluePilot: Blindspot indicators (outside scissor, on screen edges)
    self._draw_blindspot_screen_edges(self.rect, self.BLIND_SPOT_WIDTH)

    # BluePilot: Lead car complication widget
    self._complication.render(self._content_rect)

    ball_rect = rl.Rectangle(
      self._rect.x + self._rect.width - SIDE_PANEL_WIDTH,
      self._content_rect.y,
      SIDE_PANEL_WIDTH,
      self._content_rect.height,
    )
    self._confidence_ball.render(ball_rect)

    # Bookmark icon
    self._bookmark_icon.render(self.rect)

    # BluePilot: swipe-down gesture detector (must render after bookmark icon, same rect)
    self._swipe_detector.render(self.rect)

    # Offroad label
    if not ui_state.started:
      rl.draw_rectangle(int(self.rect.x), int(self.rect.y), int(self.rect.width), int(self.rect.height), rl.Color(0, 0, 0, 175))
      self._offroad_label.render(self._content_rect)
