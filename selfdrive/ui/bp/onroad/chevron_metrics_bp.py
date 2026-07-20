import numpy as np
import pyray as rl
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.chevron_metrics import ChevronMetrics, ChevronOptions
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.selfdrive.ui.bp.lib.ui_debug_logger import bp_ui_log
# BluePilot: seasonal theme packs (LeadMarker recolors the vision-lead box border)
from openpilot.selfdrive.ui.bp.lib import theme_pack

# BluePilot: Inversion thresholds for radar overlay (close-proximity mode)
# When lead is closer than INVERT_UNDER_M, overlay moves to top with chevron flipped
# Hysteresis: revert to bottom only when lead is farther than NORMAL_OVER_M
INVERT_UNDER_M = 100.0 * 0.3048   # 100 feet = 30.48 m - flip to top
NORMAL_OVER_M = 125.0 * 0.3048    # 125 feet = 38.10 m - revert to bottom

# BluePilot: Vertical offset for inverted layout (keeps overlay on screen, below HUD/speed)
INVERTED_TOP_OFFSET = 350

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
    self._inverted_mode: bool = False

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
    self._render_text_lines_bp(text_lines, lead_vehicle, sz, rect, is_radar, self._inverted_mode)

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
                            sz: float, rect: rl.Rectangle, is_radar: bool, inverted: bool = False):
    """Render text lines with horizontal boxed layout when Ford overlay is active.
    When inverted (close proximity), overlay moves to top and chevron flips upside down."""
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

      text_height = text_sizes[0].y if text_sizes else font_size
      box_height = text_height + (padding * 2)

      # Inverted: center on screen; normal: center on chevron
      if inverted:
        center_x = rect.width / 2
      else:
        center_x = chevron_x
      start_x = center_x - total_width / 2
      current_x = start_x

      # Clamp to screen bounds
      if start_x < margin:
        start_x = margin
        current_x = margin
      elif start_x + total_width > rect.width - margin:
        start_x = rect.width - margin - total_width
        current_x = start_x

      # Inverted: boxes below HUD/speed area (INVERTED_TOP_OFFSET from top); normal: boxes below chevron
      if inverted:
        y = INVERTED_TOP_OFFSET
      else:
        y = chevron_y + CHEVRON_H

      # Border color: blue for radar, red (or theme pack LeadMarker) for vision
      if is_radar:
        glow_color = LEAD_RADAR_GLOW
        border_color = rl.Color(RADAR_BORDER_COLOR_BASE.r, RADAR_BORDER_COLOR_BASE.g,
                                RADAR_BORDER_COLOR_BASE.b, alpha)
      else:
        vision_base, glow_color = VISION_BORDER_COLOR_BASE, LEAD_VISION_GLOW
        pack = theme_pack.get_active_pack()
        if pack is not None:
          pack_marker = pack.rl_colors().get("LeadMarker")
          if pack_marker is not None:
            vision_base = glow_color = pack_marker
        border_color = rl.Color(vision_base.r, vision_base.g, vision_base.b, alpha)

      border_thickness = max(2, int(6 * scale))
      # Gap between triangle outline and box edge so the outline doesn't bleed into the box
      tri_gap = border_thickness / 2 + 1

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
        if inverted:
          # Chevron flipped: wide base at bottom of boxes, apex pointing down
          # Offset base below box bottom so outline doesn't overlap into box
          base_y = y + box_height + tri_gap
          apex_y = base_y + CHEVRON_H
          chevron = [rl.Vector2(center_x, apex_y),
                     rl.Vector2(box.x, base_y),
                     rl.Vector2(box.x + box.width, base_y)]
        else:
          # Normal: apex above (point toward lead), base above box top
          # Offset base above box top so outline doesn't overlap into box
          base_y = y - tri_gap
          chevron = [rl.Vector2(center_x, base_y - CHEVRON_H),
                     rl.Vector2(box.x, base_y),
                     rl.Vector2(box.x + box.width, base_y)]
      else:
        chevron = lead_vehicle.glow

      # Draw triangle connecting chevron to boxes (drawn before boxes are on screen,
      # so z-order is: filled triangle behind, boxes on top)
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

    # BluePilot: Hysteresis for inversion - use closest lead's d_rel
    if has_lead_one or has_lead_two:
      d_rel_closest = min(
        lead_one.dRel if has_lead_one else float('inf'),
        lead_two.dRel if has_lead_two else float('inf'),
      )
      if d_rel_closest < INVERT_UNDER_M:
        self._inverted_mode = True
      elif d_rel_closest > NORMAL_OVER_M:
        self._inverted_mode = False

    if has_lead_one and lead_vehicles[0].chevron:
      self._draw_lead(lead_one, lead_vehicles[0], v_ego, rect, lead_index=0)

    if has_lead_two and lead_vehicles[1].chevron:
      d_rel_diff = abs(lead_one.dRel - lead_two.dRel) if has_lead_one else float('inf')
      if d_rel_diff > 3.0:
        self._draw_lead(lead_two, lead_vehicles[1], v_ego, rect, lead_index=1)
