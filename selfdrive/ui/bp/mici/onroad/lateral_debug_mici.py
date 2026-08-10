"""
BluePilot MICI lateral debug panel.

Shows a live steering angle trend: carControl.actuators.steeringAngleDeg (desired)
vs carState.steeringAngleDeg (actual). Reuses the TICI TimeSeriesGraph infrastructure.

Navigation: pushed via gui_app.push_widget(). Tap anywhere on screen to pop back.
The main layout's standstill/timeout transitions also pop this widget automatically.
"""
import time
import pyray as rl
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.selfdrive.ui.ui_state import device, ui_state
from bluepilot.ui.widgets.debug.debug_colors import DebugColors
from bluepilot.ui.widgets.debug.debug_graph import TimeSeriesGraph, GraphConfig, GraphSeries
from bluepilot.ui.widgets.debug.angle_factor_adjuster import AngleFactorAdjuster
from bluepilot.ui.widgets.debug.autocal_bars import AutoCalBars, poll_status

_ADJUSTER_WIDTH = 110
_ADJUSTER_MARGIN_TOP = 5
_ADJUSTER_MARGIN_RIGHT = 5

# Hold the interactive timeout at 5 minutes while the debug screen is visible so
# the onroad-inactivity timeout doesn't dismiss the screen mid-observation.
_DEBUG_TIMEOUT_S = 300

# Match TICI debug panel update rate
_DATA_PUSH_INTERVAL = 0.05  # 20 Hz
_MAX_DATA_POINTS = 100
_CAL_POLL_INTERVAL = 1.0    # auto-cal status changes at ~1 Hz; polling faster is waste


class LateralDebugMici(Widget):
  """
  Full-screen lateral steering angle trend for MICI.
  Two series: Desired (carControl.actuators.steeringAngleDeg) and
  Actual (carState.steeringAngleDeg). Tap anywhere to dismiss.
  """

  def __init__(self, back_callback):
    super().__init__()
    # Any tap on this widget pops back to the menu, except taps on the angle-factor adjuster
    self._back_callback = back_callback
    self._adjuster = self._child(AngleFactorAdjuster(compact=True))
    self._adjuster_rect = rl.Rectangle(0, 0, 0, 0)

    self._graph = TimeSeriesGraph(
      config=GraphConfig(
        title="Steering Angle",
        y_unit="°",
        max_data_points=_MAX_DATA_POINTS,
        min_scale=5.0,
        # Same 0.4x shrink applied to the legend text below.
        title_font_size=21,
        axis_font_size=16,
        # Lift the Desired/Actual legend clear of the "tap to close" hint below the graph,
        # while staying below the x-axis time labels.
        legend_y_offset=-20,
        # MICI's screen is smaller than TICI's -- half-size beads so they don't overwhelm the line.
        marker_radius=4.5,
      ),
      series=[
        GraphSeries("Desired", DebugColors.DESIRED_GREEN, fill_alpha=25),
        GraphSeries("Actual",  DebugColors.ACTUAL_YELLOW,  fill_alpha=25, beaded=True),
      ]
    )
    self._last_push_time = 0.0
    # Auto-cal band gauges (blue = low band, red = high band); on-device mirror of the
    # phone dashboard, hidden when auto-cal is off.
    self._cal_bars = AutoCalBars()
    self._last_cal_poll = 0.0

  def show_event(self):
    super().show_event()
    device.set_override_interactive_timeout(_DEBUG_TIMEOUT_S)

  def hide_event(self):
    super().hide_event()
    device.set_override_interactive_timeout(None)

  def _update_state(self):
    sm = ui_state.sm
    if sm is None:
      return
    now = time.monotonic()
    if now - self._last_push_time < _DATA_PUSH_INTERVAL:
      return
    desired = 0.0
    actual = 0.0
    try:
      if sm.valid.get('carControl', False):
        desired = sm['carControl'].actuators.steeringAngleDeg
      if sm.valid.get('carState', False):
        actual = sm['carState'].steeringAngleDeg
      self._graph.push_data([desired, actual])
      self._last_push_time = now
    except (KeyError, AttributeError, ValueError):
      pass
    if now - self._last_cal_poll >= _CAL_POLL_INTERVAL:
      self._last_cal_poll = now
      self._cal_bars.update_status(poll_status())

  def _handle_mouse_release(self, mouse_pos):
    if rl.check_collision_point_rec(mouse_pos, self._adjuster_rect):
      return
    self._back_callback()

  def _render(self, rect: rl.Rectangle):
    # Solid background so camera feed doesn't bleed through
    rl.draw_rectangle(int(rect.x), int(rect.y), int(rect.width), int(rect.height),
                      rl.Color(15, 15, 20, 255))

    # Graph fills the whole screen — title, axes, legend are all drawn inside.
    # The legend is lifted via legend_y_offset so it doesn't overlap "tap to close" below.
    self._graph.render(rect)

    # Gauges sit in the strip between the y-axis labels and the plot; armed-only.
    if self._cal_bars.active:
      self._cal_bars.render(rl.Rectangle(rect.x + 52, rect.y + 36,
                                         AutoCalBars.WIDTH, AutoCalBars.HEIGHT))

    # Angle-factor adjuster overlaps the graph's right edge (angle mode only)
    adjuster_w = _ADJUSTER_WIDTH if self._adjuster.is_visible else 0
    self._adjuster_rect = rl.Rectangle(rect.x + rect.width - adjuster_w - _ADJUSTER_MARGIN_RIGHT,
                                       rect.y + _ADJUSTER_MARGIN_TOP,
                                       adjuster_w, rect.height - _ADJUSTER_MARGIN_TOP)
    self._adjuster.render(self._adjuster_rect)

    # Small dismiss hint at bottom-left, outside graph legend area
    font = gui_app.font(FontWeight.NORMAL)
    rl.draw_text_ex(font, "tap to close",
                    rl.Vector2(rect.x + 20, rect.y + rect.height - 40),
                    30, 0, rl.Color(110, 110, 110, 180))
