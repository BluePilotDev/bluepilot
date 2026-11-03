import pyray as rl
import numpy as np
import time
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.common.params import Params
from opendbc.car import structs
from openpilot.sunnypilot import IntEnumBase
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

FONT_SIZE = 68
DIST_FONT_SIZE = 55
TIME_FONT_SIZE = 50
UNIT_FONT_SIZE = 24
WIDTH = 80
COLOR_DELTA_MS = 4.5  # ~ 10MPH
SHADOW_DEPTH = 3
DELAY = 3.0 #seconds to remove last lead car speed

class ComplicationType(IntEnumBase):
  off = 0
  lead_car_speed = 1
  speed = 2
  lead_car_dist = 3
  lead_car_time = 4

class MiciComplication(Widget):
  def __init__(self):
    super().__init__()
    self.speed: float = 0.0
    self.vRel: float = 0.0
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._color = np.array([255, 255, 255], dtype=float)
    self._slower_color = np.array([255,  160,  0], dtype=float) #orangish
    self._faster_color = np.array([0, 255,  0], dtype=float) #green
    self._font_color: rl.Color = rl.Color(255, 255, 255, 180)
    self._car_state = None
    self._render_type = 1
    self._last_active_time = 0.0

    self.params = Params()

  def _update_state(self):
     self._render_type = self.params.get("mici_complication")
     bp_ui_log.state("MiciComplication", "render_type", self._render_type)

  def _render(self, rect: rl.Rectangle) -> None:
    """Draw the first lead vehicle speed and unit."""
    if self._render_type == ComplicationType.off:
      return

    self.sm = ui_state.sm
    self._car_state = self.sm['carState']

    in_gear = self._car_state.gearShifter != structs.CarState.GearShifter.park and \
      self._car_state.gearShifter != structs.CarState.GearShifter.reverse
    self._radar_state = self.sm['radarState'] if self.sm.valid['radarState'] else None
    self._lead_one = self._radar_state.leadOne if self._radar_state else None
    has_lead_one = self._lead_one.status if self._lead_one else False
    self._render_lead_indicator = self._radar_state is not None and has_lead_one and in_gear

    match self._render_type:
      case ComplicationType.lead_car_speed:
        self._render_lead_speed(rect)
      case ComplicationType.speed:
        # BluePilot: Respect "Speedometer: Hide from Onroad Screen" (HideVEgoUI) from Visuals.
        # Read param directly for immediate response (ui_state.hide_v_ego_ui refreshes every 5s).
        if not self.params.get_bool("HideVEgoUI"):
          self._render_current_speed(rect)
      case ComplicationType.lead_car_dist:
        self._render_lead_dist(rect)
      case ComplicationType.lead_car_time:
        self._render_lead_time(rect)


  def _render_lead_speed(self,rect: rl.Rectangle):
    if self._render_lead_indicator:
      self._last_active_time = time.monotonic()
      speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
      speed_delta = self._lead_one.vRel * speed_conversion
      self.speed = max(0.0, self._car_state.vEgoCluster * speed_conversion + speed_delta)
      self.vRel = self._lead_one.vRel
      fade_ratio = 1.0
    else:
      delay_time = time.monotonic() - self._last_active_time
      if delay_time > DELAY:
        return
      else:
        fade_ratio = 1.0 - (delay_time / DELAY)

    v_delta = np.clip(self.vRel, -COLOR_DELTA_MS, COLOR_DELTA_MS)
    if v_delta <= 0:
        t = (v_delta + COLOR_DELTA_MS) / COLOR_DELTA_MS
        result = (1 - t) * self._slower_color + t * self._color
    else:
        t = v_delta / COLOR_DELTA_MS
        result = (1 - t) * self._color + t * self._faster_color

    color = result.astype(int)
    self._font_color = rl.Color(color[0], color[1], color[2], int(220 * fade_ratio))
    shadow_color = rl.Color(0, 0, 0, int(180 * fade_ratio))

    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZE)
    pos_x = rect.x + rect.width - WIDTH - 5
    speed_pos = rl.Vector2(pos_x + ((WIDTH - speed_text_size.x) / 2), rect.y + rect.height * 0.66 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZE, 0, shadow_color)
    speed_pos.x -= SHADOW_DEPTH
    speed_pos.y -= SHADOW_DEPTH
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZE, 0, self._font_color)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_bold, unit_text, UNIT_FONT_SIZE)
    unit_pos = rl.Vector2(pos_x + WIDTH / 2 - unit_text_size.x / 2, speed_pos.y + speed_text_size.y - 15)
    rl.draw_text_ex(self._font_bold, unit_text, unit_pos, UNIT_FONT_SIZE, 0, shadow_color)
    unit_pos.x -= SHADOW_DEPTH
    unit_pos.y -= SHADOW_DEPTH
    rl.draw_text_ex(self._font_bold, unit_text, unit_pos, UNIT_FONT_SIZE, 0, self._font_color)

    size = 20
    x = pos_x + WIDTH / 2
    y = speed_pos.y - 10
    chevron = [(x + (size * 1.25), y + size), (x, y), (x - (size * 1.25), y + size)]
    rl.draw_triangle_fan(chevron, len(chevron), rl.Color(201, 34, 49, int(150 * fade_ratio)))

  def _render_current_speed(self, rect: rl.Rectangle) -> None:
    # BluePilot: Respect "Speedometer: Hide from Onroad Screen" (HideVEgoUI) from Visuals
    if ui_state.hide_v_ego_ui:
      return
    self._font_color = rl.Color(255, 255, 255, 220)
    shadow_color = rl.Color(0, 0, 0, 180)

    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    speed_text = str(round(max(0.0, self._car_state.vEgoCluster * speed_conversion)))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZE)
    pos_x = rect.x + rect.width - WIDTH - 5
    speed_pos = rl.Vector2(pos_x + ((WIDTH - speed_text_size.x) / 2), rect.y + rect.height * 0.66 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZE, 0, shadow_color)
    speed_pos.x -= SHADOW_DEPTH
    speed_pos.y -= SHADOW_DEPTH
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZE, 0, self._font_color)

    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_bold, unit_text, UNIT_FONT_SIZE)
    unit_pos = rl.Vector2(pos_x + WIDTH / 2 - unit_text_size.x / 2, speed_pos.y + speed_text_size.y - 15)
    rl.draw_text_ex(self._font_bold, unit_text, unit_pos, UNIT_FONT_SIZE, 0, shadow_color)
    unit_pos.x -= SHADOW_DEPTH
    unit_pos.y -= SHADOW_DEPTH
    rl.draw_text_ex(self._font_bold, unit_text, unit_pos, UNIT_FONT_SIZE, 0, self._font_color)

  def _render_lead_dist(self,rect: rl.Rectangle):
    if self._render_lead_indicator:
      self._last_active_time = time.monotonic()
      self.dist = self._lead_one.dRel
      if not ui_state.is_metric:
        self.dist *= 3.28084
      fade_ratio = 1.0
    else:
      delay_time = time.monotonic() - self._last_active_time
      if delay_time > DELAY:
        return
      else:
        fade_ratio = 1.0 - (delay_time / DELAY)

    self._font_color = rl.Color(255, 255, 255, int(220 * fade_ratio))
    shadow_color = rl.Color(0, 0, 0, int(180 * fade_ratio))

    dist_text = str(round(self.dist))
    dist_text_size = measure_text_cached(self._font_bold, dist_text, DIST_FONT_SIZE)
    pos_x = rect.x + rect.width - WIDTH - 5
    speed_pos = rl.Vector2(pos_x + ((WIDTH - dist_text_size.x) / 2), rect.y + rect.height * 0.66 - dist_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, dist_text, speed_pos, DIST_FONT_SIZE, 0, shadow_color)
    speed_pos.x -= SHADOW_DEPTH
    speed_pos.y -= SHADOW_DEPTH
    rl.draw_text_ex(self._font_bold, dist_text, speed_pos, DIST_FONT_SIZE, 0, self._font_color)

    unit_text = tr("m") if ui_state.is_metric else tr("ft")
    unit_text_size = measure_text_cached(self._font_bold, unit_text, UNIT_FONT_SIZE)
    unit_pos = rl.Vector2(pos_x + WIDTH / 2 - unit_text_size.x / 2, speed_pos.y + dist_text_size.y - 8)
    rl.draw_text_ex(self._font_bold, unit_text, unit_pos, UNIT_FONT_SIZE, 0, shadow_color)
    unit_pos.x -= SHADOW_DEPTH
    unit_pos.y -= SHADOW_DEPTH
    rl.draw_text_ex(self._font_bold, unit_text, unit_pos, UNIT_FONT_SIZE, 0, self._font_color)

  def _render_lead_time(self,rect: rl.Rectangle):
    if self._render_lead_indicator and self._lead_one.vRel > 0:
      self._last_active_time = time.monotonic()
      self.dist = self._lead_one.dRel
      self.ttc = (self.dist / self._car_state.vEgoCluster) if (self._car_state.vEgoCluster > 0) else 0.0
      fade_ratio = 1.0
    else:
      delay_time = time.monotonic() - self._last_active_time
      if delay_time > DELAY:
        return
      else:
        fade_ratio = 1.0 - (delay_time / DELAY)

    self._font_color = rl.Color(255, 255, 255, int(220 * fade_ratio))
    shadow_color = rl.Color(0, 0, 0, int(180 * fade_ratio))

    dist_text = f"{self.ttc:.1f}" if (0 < self.ttc < 200) else "---"
    dist_text_size = measure_text_cached(self._font_bold, dist_text, TIME_FONT_SIZE)
    pos_x = rect.x + rect.width - WIDTH - 5
    speed_pos = rl.Vector2(pos_x + ((WIDTH - dist_text_size.x) / 2), rect.y + rect.height * 0.66 - dist_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, dist_text, speed_pos, TIME_FONT_SIZE, 0, shadow_color)
    speed_pos.x -= SHADOW_DEPTH
    speed_pos.y -= SHADOW_DEPTH
    rl.draw_text_ex(self._font_bold, dist_text, speed_pos, TIME_FONT_SIZE, 0, self._font_color)

    unit_text = "sec"
    unit_text_size = measure_text_cached(self._font_bold, unit_text, UNIT_FONT_SIZE)
    unit_pos = rl.Vector2(pos_x + WIDTH / 2 - unit_text_size.x / 2, speed_pos.y + dist_text_size.y - 10)
    rl.draw_text_ex(self._font_bold, unit_text, unit_pos, UNIT_FONT_SIZE, 0, shadow_color)
    unit_pos.x -= SHADOW_DEPTH
    unit_pos.y -= SHADOW_DEPTH
    rl.draw_text_ex(self._font_bold, unit_text, unit_pos, UNIT_FONT_SIZE, 0, self._font_color)