import numpy as np
import pyray as rl
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics, ChevronOptions
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log

# BluePilot: Deadband thresholds for close-proximity mode (in meters)
CLOSE_MODE_THRESHOLD_M = 50.0 * 0.3048   # 50 feet = 15.24 meters
NORMAL_MODE_THRESHOLD_M = 60.0 * 0.3048  # 60 feet = 18.29 meters

# Deadband thresholds when powerflow gauge is active (wider to avoid overlap)
POWERFLOW_UPPER_THRESHOLD_M = 150.0 * 0.3048  # 150 feet = 45.72 meters
POWERFLOW_LOWER_THRESHOLD_M = 100.0 * 0.3048  # 100 feet = 30.48 meters

# BluePilot: Border colors for radar vs vision leads
LEAD_RADAR_GLOW = rl.Color(0, 134, 233, 255)
RADAR_BORDER_COLOR_BASE = rl.Color(0, 100, 200, 255)   # Blue for radar
LEAD_VISION_GLOW = rl.Color(218, 202, 37, 255)
VISION_BORDER_COLOR_BASE = rl.Color(201, 34, 49, 255)   # Red for vision

class ChevronMetricsBP(ChevronMetrics):
  """BluePilot ChevronMetrics with horizontal boxed layout and radar/vision colored borders."""

  def __init__(self):
    super().__init__()
    self._bp_params = Params()
    self._close_mode: bool = False

    # Set by ModelRendererBP before calling draw_lead_status
    self.ford_overlay_enabled: bool = False
    self.lead_is_radar: list[bool] = [False, False]
    self.overlay_scale: float = 1.0

  def should_render(self) -> bool:
    # Render if chevron metrics is enabled OR if Ford overlay is enabled
    result = (ui_state.chevron_metrics != ChevronOptions.OFF or self.ford_overlay_enabled) and self._lead_status_alpha > 0.0
    bp_ui_log.visibility("ChevronMetrics", result, reason=f"ford_overlay={self.ford_overlay_enabled} alpha={self._lead_status_alpha:.2f}")
    return result

  def _draw_lead(self, lead_data, lead_vehicle, v_ego: float, rect: rl.Rectangle, lead_index: int = 0):
    """Draw lead vehicle status with close-proximity mode and boxed layout."""
    if not self.should_render():
      return

    d_rel = lead_data.dRel
    v_rel = lead_data.vRel

    if not lead_vehicle.chevron or len(lead_vehicle.chevron) < 3:
      return

    sz = np.clip((25 * 30) / (d_rel / 3 + 30), 15.0, 30.0) * 2.35 * self.overlay_scale

    text_lines = self._build_text_lines_bp(d_rel, v_rel, v_ego)
    if not text_lines:
      return

    is_radar = self.lead_is_radar[lead_index] if lead_index < len(self.lead_is_radar) else False
    self._render_text_lines_bp(text_lines, lead_vehicle, sz, rect, is_radar)

  def _build_text_lines_bp(self, d_rel: float, v_rel: float, v_ego: float) -> list[str]:
    """Build text lines - Ford overlay forces all 3, otherwise respects setting."""
    if self.ford_overlay_enabled:
      # When Ford overlay is enabled, always show all 3 values
      text_lines = []

      # Distance
      val = max(0.0, d_rel)
      unit = "m" if ui_state.is_metric else "ft"
      if not ui_state.is_metric:
        val *= 3.28084
      text_lines.append(f"{val:.0f} {unit}")

      # Speed
      multiplier = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
      val = max(0.0, (v_rel + v_ego) * multiplier)
      unit = "km/h" if ui_state.is_metric else "mph"
      text_lines.append(f"{val:.0f} {unit}")

      # Lead time
      val = (d_rel / v_ego) if (d_rel > 0 and v_ego > 0) else 0.0
      ttc_text = f"{val:.1f} s" if (0 < val < 200) else "---"
      text_lines.append(ttc_text)

      return text_lines
    else:
      return ChevronMetrics._build_text_lines(d_rel, v_rel, v_ego)

  def _render_text_lines_bp(self, text_lines: list[str], lead_vehicle,
                            sz: float, rect: rl.Rectangle, is_radar: bool):
    """Render text lines with horizontal boxed layout when Ford overlay is active."""

    CHEVRON_H = 40

    margin = 20
    alpha = int(255 * self._lead_status_alpha)
    text_color = rl.Color(255, 255, 255, alpha)
    shadow_color = rl.Color(0, 0, 0, int(200 * self._lead_status_alpha))

    if self.ford_overlay_enabled and len(text_lines) == 3:
      # BluePilot: Horizontal boxed layout with colored borders, scaled by overlay size
      scale = self.overlay_scale
      font_size = int(60 * scale)
      padding = int(12 * scale)
      box_spacing = int(15 * scale)
      box_color = rl.Color(40, 40, 40, int(220 * self._lead_status_alpha))

      chevron_x = lead_vehicle.chevron[1][0]
      chevron_y = lead_vehicle.chevron[1][1]

      # Measure all text sizes
      text_sizes = []
      total_width = 0
      for line in text_lines:
        text_size = measure_text_cached(self._font, line, font_size, 0)
        text_sizes.append(text_size)
        total_width += text_size.x + (padding * 2)
      total_width += box_spacing * (len(text_lines) - 1)

      # Center boxes horizontally on chevron
      start_x = chevron_x - total_width / 2
      current_x = start_x

      text_height = text_sizes[0].y if text_sizes else font_size
      box_height = text_height + (padding * 2)

      # Clamp to screen bounds
      if start_x < margin:
        start_x = margin
        current_x = margin
      elif start_x + total_width > rect.width - margin:
        start_x = rect.width - margin - total_width
        current_x = start_x

      # Border color: blue for radar, red for vision
      if is_radar:
        glow_color = LEAD_RADAR_GLOW
        border_color = rl.Color(RADAR_BORDER_COLOR_BASE.r, RADAR_BORDER_COLOR_BASE.g,
                                RADAR_BORDER_COLOR_BASE.b, alpha)
      else:
        glow_color = LEAD_VISION_GLOW
        border_color = rl.Color(VISION_BORDER_COLOR_BASE.r, VISION_BORDER_COLOR_BASE.g,
                                VISION_BORDER_COLOR_BASE.b, alpha)

      border_thickness = max(2, int(6 * scale))
      y = chevron_y + CHEVRON_H

      box_rects = []
      for line, text_size in zip(text_lines, text_sizes):
        box_width = text_size.x + (padding * 2)

        # Dark grey box
        box_rect = rl.Rectangle(int(current_x), int(y), box_width, box_height)
        box_rects.append(box_rect)
        rl.draw_rectangle_rounded(box_rect, 0.2, 10, box_color)

        # Colored border (drawn on same rect so there's no gap)
        rl.draw_rectangle_rounded_lines_ex(box_rect, 0.2, 10, border_thickness, border_color)

        # Text centered in box
        text_x = int(current_x + padding)
        text_y_pos = int(y + padding)

        rl.draw_text_ex(self._font, line, rl.Vector2(text_x + 2, text_y_pos + 2), font_size, 0, shadow_color)
        rl.draw_text_ex(self._font, line, rl.Vector2(text_x, text_y_pos), font_size, 0, text_color)

        current_x += box_width + box_spacing

      box = None
      if len(box_rects) == 1:
        box = box_rects[0]
      elif len(box_rects) == 3:
        box = box_rects[1]

      if box != None:
        center_x = box.x + box.width / 2
        chevron = [rl.Vector2(center_x, y - CHEVRON_H),
                   rl.Vector2(box.x, y),
                   rl.Vector2(box.x + box.width, y)]
      else:
        chevron = lead_vehicle.glow

      #draw modified chevron
      rl.draw_triangle_fan(chevron, len(chevron), border_color)
      rl.draw_line_ex(chevron[0], chevron[1], border_thickness, glow_color)
      rl.draw_line_ex(chevron[1], chevron[2], border_thickness, glow_color)
      rl.draw_line_ex(chevron[2], chevron[0], border_thickness, glow_color)
      r = border_thickness / 2
      rl.draw_circle_v(chevron[0], r, glow_color)
      rl.draw_circle_v(chevron[1], r, glow_color)
      rl.draw_circle_v(chevron[2], r, glow_color)

    else:
      # Fall back to base vertical stack rendering
      self._render_text_lines(text_lines, chevron_x, chevron_y, sz, rect)

  def draw_lead_status(self, sm, radar_state, rect, lead_vehicles):
    lead_one = radar_state.leadOne
    lead_two = radar_state.leadTwo

    has_lead_one = lead_one.status if lead_one else False
    has_lead_two = lead_two.status if lead_two else False

    self.update_alpha(has_lead_one or has_lead_two)

    if not self.should_render():
      return

    v_ego = sm['carState'].vEgo

    if has_lead_one and lead_vehicles[0].chevron:
      self._draw_lead(lead_one, lead_vehicles[0], v_ego, rect, lead_index=0)

    if has_lead_two and lead_vehicles[1].chevron:
      d_rel_diff = abs(lead_one.dRel - lead_two.dRel) if has_lead_one else float('inf')
      if d_rel_diff > 3.0:
        self._draw_lead(lead_two, lead_vehicles[1], v_ego, rect, lead_index=1)
