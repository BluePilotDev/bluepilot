from __future__ import annotations

import pyray as rl

from collections.abc import Callable

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.mici.widgets.button import BigButton, BigMultiParamToggle, BigMultiToggle, BigParamControl
from openpilot.selfdrive.ui.mici.widgets.dialog import BigInputDialog
from openpilot.selfdrive.ui.mici.layouts.settings.web_server_qr_dialog import WebServerQRDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, MousePos
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.wifi_manager import WifiManager, Network
from openpilot.system.ui.widgets.scroller import NavScroller

CONTENT_MARGIN = 20
LINE_L = 40
LINE_W = 8


class BigBoolParamToggle(BigMultiToggle):
  def __init__(self, text: str, param: str, options: list[str]):
    super().__init__(text, options)
    self._param = param
    self._params = Params()
    self._load_value()

  def _load_value(self):
    self.set_value(self._options[1] if self._params.get_bool(self._param) else self._options[0])

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)
    self._params.put_bool(self._param, self.value == self._options[1], block=True)


class BigNumericParamControl(BigButton):
  def __init__(self, text: str, param: str, *, min_value: float | int | None = None,
               max_value: float | int | None = None, step: float | int = 1,
               formatter: Callable[[float | int], str], parser: Callable[[str], float | int],
               default_value: float | int = 0, is_active: Callable[[], bool] | None = None):
    super().__init__(text, "")
    self._param = param
    self._params = Params()
    self._min_value = min_value
    self._max_value = max_value
    self._step = step
    self._formatter = formatter
    self._parser = parser
    self._default_value = default_value
    self._active_fn = is_active

    self._sub_label.set_font_size(22)
    self._margin = self._rect.width * 0.1
    self._hit_rect_size = LINE_L + 2 * CONTENT_MARGIN
    self.minus_hit_rect = rl.Rectangle(0, 0, 0, 0)
    self.plus_hit_rect = rl.Rectangle(0, 0, 0, 0)

    self.set_click_callback(self._on_click)
    self._refresh_value()

  def _is_active(self) -> bool:
    return self._active_fn() if self._active_fn is not None else True

  def _clamp(self, value: float | int) -> float | int:
    if self._min_value is not None and value < self._min_value:
      value = self._min_value
    if self._max_value is not None and value > self._max_value:
      value = self._max_value
    return value

  def _get_param(self) -> float | int:
    raw = self._params.get(self._param, return_default=True)
    if raw in (None, b"", ""):
      return self._default_value
    try:
      return self._parser(raw.decode("utf-8") if isinstance(raw, bytes) else str(raw))
    except (TypeError, ValueError):
      return self._default_value

  def _set_param(self, value: float | int):
    value = self._clamp(value)
    self._params.put(self._param, value)
    self.set_value(self._formatter(value))

  def _refresh_value(self):
    self.set_value(self._formatter(self._get_param()))

  def _on_click(self):
    if not self._is_active():
      return

    hint = f"({self._min_value}-{self._max_value})" if self._min_value is not None or self._max_value is not None else tr("enter a value...")

    def confirm_callback(text: str):
      if text:
        try:
          self._set_param(self._parser(text))
        except ValueError:
          pass
      else:
        self._params.remove(self._param)
        self._refresh_value()

    gui_app.push_widget(BigInputDialog(hint, str(self._get_param()), minimum_length=0, confirm_callback=confirm_callback))

  def _get_label_font_size(self):
    return super()._get_label_font_size() - 10

  def _draw_content(self, btn_y: float):
    offset = self._hit_rect_size / 3
    self.rect.height -= offset
    super()._draw_content(btn_y + offset)
    self.rect.height += offset

  def _render(self, _):
    super()._render(_)

    left = self._rect.x + self._margin
    right = self._rect.x + self._rect.width - self._margin
    top = self._rect.y + self._margin
    color = rl.WHITE if self.enabled and self._is_active() else rl.Color(255, 255, 255, int(255 * 0.35))

    self.minus_hit_rect = rl.Rectangle(left - CONTENT_MARGIN, top - self._hit_rect_size / 2, self._hit_rect_size, self._hit_rect_size)
    self.plus_hit_rect = rl.Rectangle(right - self._hit_rect_size / 2 - CONTENT_MARGIN, top - self._hit_rect_size / 2, self._hit_rect_size, self._hit_rect_size)

    rl.draw_line_ex((left, top), (left + LINE_L, top), LINE_W, color)
    rl.draw_line_ex((right - LINE_L, top), (right, top), LINE_W, color)
    mid = right - LINE_L / 2
    rl.draw_line_ex((mid, top - LINE_L / 2), (mid, top + LINE_L / 2), LINE_W, color)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if not self._is_active():
      return
    if rl.check_collision_point_rec(mouse_pos, self.minus_hit_rect):
      self._set_param(self._get_param() - self._step)
      return
    if rl.check_collision_point_rec(mouse_pos, self.plus_hit_rect):
      self._set_param(self._get_param() + self._step)
      return
    super()._handle_mouse_release(mouse_pos)


class BigFloatParamControl(BigNumericParamControl):
  def __init__(self, text: str, param: str, *, min_value: float | None = None,
               max_value: float | None = None, step: float = 0.05,
               default_value: float = 0.0, is_active: Callable[[], bool] | None = None):
    super().__init__(text, param, min_value=min_value, max_value=max_value, step=step,
                     formatter=lambda value: f"{round(float(value), 4)}",
                     parser=float, default_value=default_value, is_active=is_active)


class BigIntParamControl(BigNumericParamControl):
  def __init__(self, text: str, param: str, *, min_value: int | None = None,
               max_value: int | None = None, step: int = 1, default_value: int = 0,
               is_active: Callable[[], bool] | None = None):
    super().__init__(text, param, min_value=min_value, max_value=max_value, step=step,
                     formatter=lambda value: str(int(value)),
                     parser=lambda raw: int(float(raw)),
                     default_value=default_value, is_active=is_active)

  def _set_param(self, value: float | int):
    step_value = int(value)
    step_value -= step_value % int(self._step)
    super()._set_param(step_value)


class PreferredNetworkLayoutMici(NavScroller):
  def __init__(self, wifi_manager: WifiManager, saved_networks: list[Network], on_select: Callable[[], None]):
    super().__init__()
    self._params = Params()
    self._wifi_manager = wifi_manager
    self._saved_networks = saved_networks
    self._on_select = on_select
    self._build_buttons()

  def _build_buttons(self):
    items = [self._make_button(tr("none"), "", lambda: self._select(""))]
    for network in self._saved_networks:
      label = network.ssid.strip() or tr("hidden network")
      items.append(self._make_button(label, "", lambda ssid=network.ssid: self._select(ssid)))

    self._scroller._items.clear()
    for item in items:
      self._scroller.add_widget(item)

  @staticmethod
  def _make_button(text: str, value: str, callback: Callable[[], None]) -> BigButton:
    button = BigButton(text, value, gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 76, 56), scroll=True)
    button.set_click_callback(callback)
    return button

  def _select(self, ssid: str):
    self._params.put("WifiFavoriteSSID", ssid)
    if ssid:
      cloudlog.info(f"Set preferred network: {ssid}")
    else:
      cloudlog.info("Cleared preferred network")
    self.dismiss(self._on_select)


class BluePilotLayoutMici(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._wifi_manager = WifiManager()
    self._wifi_manager.set_active(False)
    self._saved_networks: list[Network] = []
    self._wifi_manager.add_callbacks(networks_updated=self._on_network_updated)

    self._preferred_network_btn = BigButton(tr("preferred WiFi network"), "", gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 76, 56), scroll=True)
    self._preferred_network_btn.set_click_callback(self._select_preferred_network)

    self._enable_web_routes = BigParamControl("enable web routes server", "EnableWebRoutesServer")
    self._show_web_routes_qr = BigButton("show QR code", "", gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 76, 56))
    self._show_web_routes_qr.set_click_callback(self._show_qr_dialog)
    self._show_hands_free_ui = BigParamControl("show BlueCruise UI on Cluster", "send_hands_free_cluster_msg")
    self._show_lead_vehicle = BigMultiParamToggle("Lower Right Display", "mici_complication", ["off", "lead car speed", "speed", "lead car distance", "time to lead car"])
    self._show_brake_status = BigParamControl("show brake status", "ShowBrakeStatus")
    self._show_blindspot_ui = BigParamControl("show blindspot overlay", "ShowBlindspotOverlay")
    self._show_hybrid_power_flow = BigParamControl("show hybrid/EV power flow", "FordPrefHybridPowerFlow")
    self._hybrid_power_flow_style = BigBoolParamToggle("hybrid/EV power flow style", "FordPrefHybridPowerFlowAlternate", ["flat", "round"])
    self._rainbow_mode = BigParamControl("rainbow mode", "RainbowMode")
    self._enable_human_turn_detection = BigParamControl("enable human turn detection", "enable_human_turn_detection")
    self._lane_change_factor_high = BigFloatParamControl("lane change factor high", "lane_change_factor_high", min_value=0.5, max_value=1.0, step=0.05)
    self._disable_lane_change_under_speed = BigParamControl("disable auto lane change under speed", "BlinkerPauseLaneChange")
    self._blinker_min_speed = BigIntParamControl("blinker min lane change speed", "BlinkerMinLateralControlSpeed", min_value=5, max_value=50, step=5)
    self._enable_lane_positioning = BigParamControl("enable lane positioning", "enable_lane_positioning")
    self._custom_path_offset = BigFloatParamControl("in-lane offset", "custom_path_offset", min_value=-0.5, max_value=0.5, step=0.05,
                                                    is_active=lambda: self._params.get_bool("enable_lane_positioning"))
    self._enable_lane_full_mode = BigParamControl("enable lanefull mode", "enable_lane_full_mode")
    self._custom_profile = BigParamControl("use custom tuning profile", "custom_profile")
    self._pc_blend_ratio_high = BigFloatParamControl("predicted curvature blend ratio high", "pc_blend_ratio_high_C_UI", min_value=0.0, max_value=1.0, step=0.05,
                                                     is_active=lambda: self._params.get_bool("custom_profile"))
    self._pc_blend_ratio_low = BigFloatParamControl("predicted curvature blend ratio low", "pc_blend_ratio_low_C_UI", min_value=0.0, max_value=1.0, step=0.05,
                                                    is_active=lambda: self._params.get_bool("custom_profile"))
    self._lc_pid_gain = BigFloatParamControl("low curvature PID gain", "LC_PID_gain_UI", min_value=0.0, max_value=5.0, step=0.05,
                                             is_active=lambda: self._params.get_bool("enable_lane_positioning") and self._params.get_bool("custom_profile"))
    self._animate_steering_wheel = BigParamControl("animate steering wheel", "BPAnimateSteeringWheel")
    self._hide_fade = BigParamControl("hide onroad fade", "mici_hide_onroad_fade")
    self._hide_border = BigParamControl("hide screen border", "BPHideOnroadBorder")
    self._disable_bp_lat = BigParamControl("disable BP lateral control", "disable_BP_lat_UI")
    self._disable_bp_long = BigParamControl("bypass BP longitudinal control", "disable_BP_long_UI")
    self._disable_downhill_comp = BigParamControl("disable downhill compensation", "disable_downhill_comp_UI")
    self._clear_model_cache = BigButton("clear crashed model", "", gui_app.texture("icons_mici/settings/device/reboot.png", 64, 70))
    self._clear_model_cache.set_click_callback(self._clear_model_cache_and_reboot)
    self._ui_debug_log = BigParamControl("ui debug logging", "BPUIDebugLog")
    self._vbatt_pause_charging = BigFloatParamControl("12V battery limit", "vbatt_pause_charging", min_value=11.0, max_value=14.0, step=0.1)

    self._scroller.add_widgets([
      self._enable_web_routes,
      self._show_web_routes_qr,
      self._preferred_network_btn,
      self._show_hands_free_ui,
      self._show_lead_vehicle,
      self._show_brake_status,
      self._show_blindspot_ui,
      self._show_hybrid_power_flow,
      self._hybrid_power_flow_style,
      self._rainbow_mode,
      self._enable_human_turn_detection,
      self._lane_change_factor_high,
      self._disable_lane_change_under_speed,
      self._blinker_min_speed,
      self._enable_lane_positioning,
      self._custom_path_offset,
      self._enable_lane_full_mode,
      self._custom_profile,
      self._pc_blend_ratio_high,
      self._pc_blend_ratio_low,
      self._lc_pid_gain,
      self._animate_steering_wheel,
      self._hide_fade,
      self._hide_border,
      self._vbatt_pause_charging,
      self._disable_bp_lat,
      self._disable_bp_long,
      self._disable_downhill_comp,
      self._clear_model_cache,
      self._ui_debug_log,
    ])

    self._refresh_toggles = (
      ("EnableWebRoutesServer", self._enable_web_routes),
      ("send_hands_free_cluster_msg", self._show_hands_free_ui),
      ("FordPrefHybridPowerFlow", self._show_hybrid_power_flow),
      ("ShowBrakeStatus", self._show_brake_status),
      ("ShowBlindspotOverlay", self._show_blindspot_ui),
      ("RainbowMode", self._rainbow_mode),
      ("enable_human_turn_detection", self._enable_human_turn_detection),
      ("BlinkerPauseLaneChange", self._disable_lane_change_under_speed),
      ("enable_lane_positioning", self._enable_lane_positioning),
      ("enable_lane_full_mode", self._enable_lane_full_mode),
      ("custom_profile", self._custom_profile),
      ("disable_BP_lat_UI", self._disable_bp_lat),
      ("disable_BP_long_UI", self._disable_bp_long),
      ("disable_downhill_comp_UI", self._disable_downhill_comp),
      ("BPAnimateSteeringWheel", self._animate_steering_wheel),
      ("BPUIDebugLog", self._ui_debug_log),
      ("mici_hide_onroad_fade", self._hide_fade),
      ("BPHideOnroadBorder", self._hide_border),
    )

    ui_state.add_offroad_transition_callback(self._update_controls)

  def show_event(self):
    super().show_event()
    self._wifi_manager.set_active(True)
    self._update_controls()

  def hide_event(self):
    super().hide_event()
    self._wifi_manager.set_active(False)

  def _update_state(self):
    super()._update_state()
    self._wifi_manager.process_callbacks()
    self._show_lead_vehicle._load_value()
    self._hybrid_power_flow_style._load_value()
    self._update_numeric_labels()
    self._update_buttons()

  def _update_numeric_labels(self):
    for control in (
      self._lane_change_factor_high,
      self._blinker_min_speed,
      self._custom_path_offset,
      self._pc_blend_ratio_high,
      self._pc_blend_ratio_low,
      self._lc_pid_gain,
      self._vbatt_pause_charging,
    ):
      control._refresh_value()

  def _update_controls(self):
    ui_state.update_params()
    for key, item in self._refresh_toggles:
      item.set_checked(ui_state.params.get_bool(key))
    self._update_buttons()

  def _update_buttons(self):
    server_enabled = self._params.get_bool("EnableWebRoutesServer")
    power_flow_enabled = self._params.get_bool("FordPrefHybridPowerFlow")
    lane_positioning_enabled = self._params.get_bool("enable_lane_positioning")
    custom_profile_enabled = self._params.get_bool("custom_profile")

    self._show_web_routes_qr.set_enabled(server_enabled)
    self._hybrid_power_flow_style.set_enabled(power_flow_enabled)
    self._custom_path_offset.set_enabled(lane_positioning_enabled)
    self._enable_lane_full_mode.set_enabled(lane_positioning_enabled)
    self._pc_blend_ratio_high.set_enabled(custom_profile_enabled)
    self._pc_blend_ratio_low.set_enabled(custom_profile_enabled)
    self._lc_pid_gain.set_enabled(lane_positioning_enabled and custom_profile_enabled)
    self._preferred_network_btn.set_enabled(len(self._saved_networks) > 0)
    self._preferred_network_btn.set_value(self._get_preferred_network_display())

  def _show_qr_dialog(self):
    if self._params.get_bool("EnableWebRoutesServer"):
      gui_app.push_widget(WebServerQRDialog(back_callback=gui_app.pop_widget))

  def _clear_model_cache_and_reboot(self):
    def confirm_callback():
      try:
        self._params.remove("ModelRunnerTypeCache")
      except Exception:
        pass
      try:
        self._params.remove("ModelManager_ActiveBundle")
      except Exception:
        pass
      self._params.put_bool("DoReboot", True)
      cloudlog.info("BluePilot: cleared model cache and requested reboot")

    gui_app.push_widget(
      BigInputDialog(
        tr("type REBOOT to clear model cache"),
        "",
        minimum_length=0,
        confirm_callback=lambda text: confirm_callback() if text.strip().upper() == "REBOOT" else None,
      )
    )

  def _on_network_updated(self, networks: list[Network]):
    self._saved_networks = [network for network in networks if self._wifi_manager.is_connection_saved(network.ssid)]
    self._update_buttons()

    favorite_value = self._params.get("WifiFavoriteSSID")
    current_favorite = favorite_value.decode("utf-8", errors="replace").strip("\x00") if isinstance(favorite_value, bytes) else str(favorite_value or "").strip("\x00")
    if current_favorite and current_favorite not in self._wifi_manager._connections:
      self._params.put("WifiFavoriteSSID", "")
      cloudlog.info(f"Cleared preferred network '{current_favorite}' - network no longer saved")

  def _get_preferred_network_display(self) -> str:
    raw = self._params.get("WifiFavoriteSSID")
    value = raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else str(raw or "")
    value = value.strip("\x00").strip()
    return value or tr("none")

  def _select_preferred_network(self):
    if len(self._saved_networks) == 0:
      return
    panel = PreferredNetworkLayoutMici(self._wifi_manager, self._saved_networks, on_select=self._update_buttons)
    gui_app.push_widget(panel)
