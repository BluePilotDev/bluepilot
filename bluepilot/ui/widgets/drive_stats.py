"""
BluePilot Drive Stats Widget
Displays all-time and past-week driving statistics (routes, distance, hours)
on the offroad home screen. Ported from the Qt DriveStats widget.
"""

import requests
import threading
import time
import pyray as rl

from openpilot.common.api import api_get
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.lib.api_helpers import get_token
from openpilot.selfdrive.ui.ui_state import ui_state, device
from openpilot.system.athena.registration import UNREGISTERED_DONGLE_ID
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from bluepilot.ui.lib.colors import BPColors


# Card styling colors (matching Qt gradient: #2c2c2c -> #1a1a1a)
CARD_BG = rl.Color(38, 38, 38, 255)
CARD_BORDER = rl.Color(255, 255, 255, 26)
STATS_CONTAINER_BG = rl.Color(255, 255, 255, 13)
STATS_CONTAINER_BORDER = rl.Color(255, 255, 255, 26)
UNIT_COLOR = rl.Color(176, 176, 176, 255)
NUMBER_COLOR = BPColors.ACCENT  # #18b4ff - matching Qt


class DriveStatsWidget(Widget):
  PARAM_KEY = "ApiCache_DriveStats"
  UPDATE_INTERVAL = 30  # seconds

  def __init__(self):
    super().__init__()
    self._params = Params()
    self._session = requests.Session()
    self._stats = self._get_stats()

    self._running = True
    self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
    self._update_thread.start()

  def __del__(self):
    self._running = False
    try:
      if self._update_thread and self._update_thread.is_alive():
        self._update_thread.join(timeout=1.0)
    except Exception:
      pass

  def _get_stats(self):
    stats = self._params.get(self.PARAM_KEY)
    if not stats:
      return {}
    try:
      return stats
    except Exception:
      cloudlog.exception(f"Failed to decode drive stats: {stats}")
      return {}

  def _fetch_drive_stats(self):
    try:
      dongle_id = self._params.get("DongleId")
      if not dongle_id or dongle_id == UNREGISTERED_DONGLE_ID:
        return
      identity_token = get_token(dongle_id)
      response = api_get(f"v1.1/devices/{dongle_id}/stats", access_token=identity_token, session=self._session)
      if response.status_code == 200:
        data = response.json()
        self._stats = data
        self._params.put(self.PARAM_KEY, data)
    except Exception as e:
      cloudlog.error(f"Failed to fetch drive stats: {e}")

  def _update_loop(self):
    while self._running:
      if not ui_state.started and device._awake:
        self._fetch_drive_stats()
      time.sleep(self.UPDATE_INTERVAL)

  def _render(self, rect: rl.Rectangle):
    is_metric = self._params.get_bool("IsMetric")

    all_time = self._stats.get("all", {})
    week = self._stats.get("week", {})

    # Calculate section heights
    spacing = 10
    section_height = (rect.height - spacing) / 2

    # Draw card background
    rl.draw_rectangle_rounded(rect, 0.03, 10, CARD_BG)
    rl.draw_rectangle_rounded_lines(rect, 0.03, 10, CARD_BORDER)

    # Render both stat sections
    top_rect = rl.Rectangle(rect.x, rect.y, rect.width, section_height)
    self._render_stat_section(top_rect, "ALL TIME", all_time, is_metric)

    bottom_rect = rl.Rectangle(rect.x, rect.y + section_height + spacing, rect.width, section_height)
    self._render_stat_section(bottom_rect, "PAST WEEK", week, is_metric)

  def _render_stat_section(self, rect, title, data, is_metric):
    # Calculate scale based on available height (base height: 275px per section)
    scale = min(1.0, max(0.35, rect.height / 275.0))

    padding_x = int(20 * scale)
    padding_top = int(15 * scale)

    title_size = int(max(34, min(50, 50 * scale)))
    number_size = int(max(42, min(66, 66 * scale)))
    unit_size = int(max(28, min(44, 44 * scale)))

    # Title
    title_font = gui_app.font(FontWeight.BOLD)
    title_pos = rl.Vector2(rect.x + padding_x, rect.y + padding_top)
    rl.draw_text_ex(title_font, title, title_pos, title_size, 0, rl.WHITE)

    # Stats container
    container_margin = int(10 * scale)
    container_top = rect.y + padding_top + title_size + int(8 * scale)
    container_height = rect.y + rect.height - container_top - container_margin
    container_rect = rl.Rectangle(
      rect.x + container_margin, container_top,
      rect.width - 2 * container_margin, container_height
    )
    rl.draw_rectangle_rounded(container_rect, 0.08, 10, STATS_CONTAINER_BG)
    rl.draw_rectangle_rounded_lines(container_rect, 0.08, 10, STATS_CONTAINER_BORDER)

    # Values
    routes = int(data.get("routes", 0))
    distance = data.get("distance", 0)
    distance_str = str(int(distance * CV.MPH_TO_KPH)) if is_metric else str(int(distance))
    hours = int(data.get("minutes", 0) / 60)
    dist_unit = "KM" if is_metric else "Miles"

    number_font = gui_app.font(FontWeight.BOLD)
    unit_font = gui_app.font(FontWeight.DISPLAY_REGULAR)

    col_width = container_rect.width / 3

    def draw_column(col_idx, value, unit):
      col_x = container_rect.x + col_width * col_idx
      center_x = col_x + col_width / 2

      # Value (centered)
      val_text = str(value)
      val_size = measure_text_cached(number_font, val_text, number_size)
      val_pos = rl.Vector2(center_x - val_size.x / 2, container_rect.y + container_rect.height * 0.2)
      rl.draw_text_ex(number_font, val_text, val_pos, number_size, 0, NUMBER_COLOR)

      # Unit (centered below value)
      unit_text_size = measure_text_cached(unit_font, unit, unit_size)
      unit_pos = rl.Vector2(center_x - unit_text_size.x / 2, container_rect.y + container_rect.height * 0.6)
      rl.draw_text_ex(unit_font, unit, unit_pos, unit_size, 0, UNIT_COLOR)

    draw_column(0, routes, "Drives")
    draw_column(1, distance_str, dist_unit)
    draw_column(2, hours, "Hours")
