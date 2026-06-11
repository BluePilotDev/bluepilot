"""BluePilot MICI home — paged carousel-friendly redesign of MiciHomeLayout.

Renders the wordmark + breathing aurora + state pill on a blue/black radial
backdrop. Long-press anywhere on the body (>= 0.5s) toggles ExperimentalMode in
place; a top progress strip fills as you hold. Tap on body opens settings.

Public API mirrors MiciHomeLayout so main.py wiring works unchanged:
- set_callbacks(on_settings, on_alerts, alert_count_callback, max_severity_callback)
"""
import time
import pyray as rl
from cereal import log
from collections.abc import Callable

from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import UnifiedLabel
from openpilot.system.ui.widgets.icon_widget import IconWidget
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.mici.layouts.home import AlertsPill, NetworkIcon, NETWORK_TYPES
from openpilot.selfdrive.ui.bp.mici.widgets.bg_radial import BPRadialBackground
from openpilot.selfdrive.ui.bp.mici.widgets.aurora_wordmark import AuroraWordmark
from openpilot.selfdrive.ui.bp.mici.widgets.state_pill import StatePill
from openpilot.selfdrive.ui.bp.mici.widgets.long_press_progress import LongPressBar
from openpilot.selfdrive.ui.bp.mici.widgets import bp_palette as P

NetworkType = log.DeviceState.NetworkType

LONG_PRESS_DURATION = 0.5
GEAR_HIT_SIZE       = 64    # Touch target — bigger than visible
GEAR_VISIBLE        = 36
HOME_PADDING_X      = 18
META_FONT_SIZE      = P.FS_META


class MiciHomeLayoutBP(Widget):
  def __init__(self):
    super().__init__()
    self._on_settings_click: Callable | None = None
    self._on_alerts_click: Callable | None = None
    self._alert_count_callback: Callable[[], int] | None = None

    # Long-press state
    self._mouse_down_t: float | None = None
    self._did_long_press = False
    self._is_pressed_prev = False

    # Cached for state-pill / wordmark callbacks. Refreshed every frame in
    # _update_state so external param changes (e.g. params web UI) are visible
    # immediately. ExperimentalMode is a single bool — cheap.
    self._experimental_mode = False

    # Children
    self._bg = self._child(BPRadialBackground())
    self._aurora = self._child(AuroraWordmark(get_mode=self._mode))
    self._pill = self._child(StatePill(get_state=self._pill_state))
    self._alerts_pill = self._child(AlertsPill())
    self._long_press_bar = self._child(LongPressBar())

    # Gear (top-right)
    self._gear_icon = self._child(IconWidget(
      "icons_mici/settings.png", (GEAR_VISIBLE, GEAR_VISIBLE), opacity=0.95))

    # Meta strip (bottom-left): network icon + label + branch
    self._network_icon = self._child(NetworkIcon())
    self._meta_label = UnifiedLabel(
      "",
      font_size=META_FONT_SIZE,
      font_weight=FontWeight.MEDIUM,
      text_color=P.MUTED,
      max_width=520,
      wrap_text=False,
    )

  # ---- public API mirroring MiciHomeLayout ----
  def set_callbacks(self, on_settings: Callable | None = None, on_alerts: Callable | None = None,
                    alert_count_callback: Callable[[], int] | None = None,
                    max_severity_callback: Callable[[], int | None] | None = None):
    self._on_settings_click = on_settings
    self._on_alerts_click = on_alerts
    self._alert_count_callback = alert_count_callback
    self._alerts_pill.set_alert_count_callback(alert_count_callback, max_severity_callback)

  # ---- mode helpers ----
  def _mode(self) -> str:
    """Return 'ready' / 'experimental' / 'offline' based on current state."""
    net = ui_state.sm["deviceState"].networkType
    if net == NetworkType.none:
      return "offline"
    return "experimental" if self._experimental_mode else "ready"

  def _pill_state(self) -> tuple[str, str]:
    m = self._mode()
    label = {
      "experimental": "EXPERIMENTAL  ARMED",
      "offline":      "OFFLINE",
    }.get(m, "STANDARD  READY")
    return m, label

  # ---- lifecycle ----
  def show_event(self):
    super().show_event()
    self._update_params()

  def _update_params(self):
    self._experimental_mode = ui_state.params.get_bool("ExperimentalMode")

  def _update_state(self):
    self._update_params()

    # ---- long-press tracking (mirrors stock MiciHomeLayout) ----
    pressed = self.is_pressed and not self._gear_pressed()
    if pressed and not self._is_pressed_prev:
      self._mouse_down_t = time.monotonic()
    elif not pressed and self._is_pressed_prev:
      self._mouse_down_t = None
      self._did_long_press = False
    self._is_pressed_prev = pressed

    if self._mouse_down_t is not None:
      held = time.monotonic() - self._mouse_down_t
      self._long_press_bar.set_progress(held / LONG_PRESS_DURATION)
      if held > LONG_PRESS_DURATION:
        # Mirror stock behavior: only toggle if longitudinal is available and
        # we are NOT onroad (onroad would change behavior mid-drive).
        if ui_state.has_longitudinal_control and not ui_state.started:
          self._experimental_mode = not self._experimental_mode
          ui_state.params.put("ExperimentalMode", self._experimental_mode)
        self._mouse_down_t = None
        self._did_long_press = True
    else:
      self._long_press_bar.set_progress(0.0)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    # Long-press fired? Don't also fire the tap.
    if self._did_long_press:
      self._did_long_press = False
      return
    # Tap on alerts pill → alerts pane.
    if self._point_in_alerts(mouse_pos):
      if self._on_alerts_click:
        self._on_alerts_click()
      return
    # Tap on gear → settings.
    if self._point_in_gear(mouse_pos):
      if self._on_settings_click:
        self._on_settings_click()
      return
    # Tap on body (anywhere else) → also opens settings, matching stock UX.
    if self._on_settings_click:
      self._on_settings_click()

  # ---- helpers ----
  def _gear_rect(self) -> rl.Rectangle:
    """Touch-sized gear hit rect, anchored top-right of self._rect."""
    s = GEAR_HIT_SIZE
    return rl.Rectangle(self._rect.x + self._rect.width - s - 4,
                        self._rect.y + 4, s, s)

  def _point_in_gear(self, p: MousePos) -> bool:
    return rl.check_collision_point_rec(p, self._gear_rect())

  def _point_in_alerts(self, p: MousePos) -> bool:
    has_alerts = self._alert_count_callback and self._alert_count_callback() > 0
    return bool(has_alerts and rl.check_collision_point_rec(p, self._alerts_pill.rect))

  def _gear_pressed(self) -> bool:
    """Is the user currently pressing inside the gear hit rect?"""
    rect = self._gear_rect()
    for ev in gui_app.mouse_events:
      if ev.left_down and rl.check_collision_point_rec(ev.pos, rect):
        return True
    return False

  def _format_meta(self) -> str:
    ds = ui_state.sm["deviceState"]
    net_label = NETWORK_TYPES.get(ds.networkType, "Offline")
    branch = ui_state.params.get("GitBranch") or ""
    if not branch:
      return net_label
    return f"{net_label}    {branch}"

  # ---- render ----
  def _render(self, _):
    r = self._rect
    self._bg.render(r)

    # Aurora + wordmark fill the upper-most region (vertical-center weighted).
    # Carve out ~60% top for wordmark, leaving room for pill + meta below.
    aurora_rect = rl.Rectangle(r.x, r.y, r.width, r.height * 0.62)
    self._aurora.render(aurora_rect)

    # State pill, centered horizontally just below the wordmark.
    # The pill auto-sizes itself in _update_state(); we read last frame's width
    # to center it. First frame may be 1px off-center; corrected next frame.
    pill_w = max(self._pill.rect.width, 1)
    px = r.x + (r.width - pill_w) / 2
    py = r.y + r.height * 0.62 + 2
    self._pill.set_position(px, py)
    self._pill.render()

    # Gear (top right)
    g = self._gear_rect()
    visible_x = g.x + (g.width - GEAR_VISIBLE) / 2
    visible_y = g.y + (g.height - GEAR_VISIBLE) / 2
    self._gear_icon.render(rl.Rectangle(visible_x, visible_y, GEAR_VISIBLE, GEAR_VISIBLE))

    # Alerts pill below the gear so offroad alerts remain reachable.
    alert_rect = self._alerts_pill.rect
    self._alerts_pill.set_position(r.x + r.width - alert_rect.width - 10, g.y + g.height + 8)
    self._alerts_pill.render()

    # Meta strip (bottom-left): network icon + label
    meta_y = r.y + r.height - META_FONT_SIZE - 12
    icon_rect = rl.Rectangle(r.x + HOME_PADDING_X, meta_y - 4, 54, 30)
    self._network_icon.render(icon_rect)
    self._meta_label.set_text(self._format_meta())
    self._meta_label.set_position(icon_rect.x + icon_rect.width + 8, meta_y)
    self._meta_label.render()

    # Long-press progress strip (top edge)
    self._long_press_bar.render(rl.Rectangle(r.x, r.y, r.width, 4))
