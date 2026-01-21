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

FONT_SIZE = 68
UNIT_FONT_SIZE = 24
WIDTH = 80
COLOR_DELTA_MS = 4.5  # ~ 10MPH
SHADOW_DEPTH = 3
DELAY = 3.0 #seconds to remove last lead car speed

class LeadVehicleRenderer(Widget):
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
    self._should_render = False
    self._last_active_time = 0.0

    self.params = Params()

  def _update_state(self):
     self._should_render = self.params.get_bool("show_lead_speed")

  def _render(self, rect: rl.Rectangle) -> None:
    """Draw the first lead vehicle speed and unit."""
    if not self._should_render:
      return

    sm = ui_state.sm
    self._car_state = sm['carState']

    in_gear = self._car_state.gearShifter != structs.CarState.GearShifter.park and \
      self._car_state.gearShifter != structs.CarState.GearShifter.reverse
    self._radar_state = sm['radarState'] if sm.valid['radarState'] else None
    lead_one = self._radar_state.leadOne if self._radar_state else None
    has_lead_one = lead_one.status if lead_one else False
    render_lead_indicator = self._radar_state is not None and has_lead_one and in_gear

    if render_lead_indicator:
      self._last_active_time = time.monotonic()
      speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
      speed_delta = lead_one.vRel * speed_conversion
      self.speed = max(0.0, self._car_state.vEgoCluster * speed_conversion + speed_delta)
      self.vRel = lead_one.vRel
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
