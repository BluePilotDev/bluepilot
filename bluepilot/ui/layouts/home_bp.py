"""
BluePilot Home Layout
Extends the stock HomeLayout to replace PrimeWidget in the left column
with DriveStats and ModelInfo widgets, and renders badge-styled version
header matching the old Qt OffroadHomeSP layout.
"""

import os
import time
import pyray as rl
from collections.abc import Callable

from openpilot.selfdrive.ui.layouts.home import HomeLayout, HomeLayoutState, HEAD_BUTTON_FONT_SIZE, SPACING
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.multilang import tr, trn
from bluepilot.ui.widgets.drive_stats import DriveStatsWidget
from bluepilot.ui.widgets.model_info import ModelInfoWidget
from bluepilot.ui.widgets.recent_changes import RecentChangesManager
from bluepilot.ui.lib.colors import BPColors
from bluepilot.ui.lib.constants import BPConstants

# Badge styling constants
BADGE_FONT_SIZE = 36
BADGE_TEXT_PAD_LEFT = 20
BADGE_TEXT_PAD_RIGHT = 19
BADGE_SPACING = 8

# Badge definitions: (color, ) - text filled at runtime
BADGE_CONFIGS = [
  BPColors.BADGE_BRAND,
  BPColors.BADGE_BRANCH,
  BPColors.BADGE_COMMIT,
  BPColors.BADGE_DATE,
]


class HomeLayoutBP(HomeLayout):
  def __init__(self):
    super().__init__()
    # BP widgets for the left column (PrimeWidget is still created by super but won't be rendered)
    self._drive_stats = DriveStatsWidget()
    self._model_info = ModelInfoWidget()

    # Read BPVERSION once at startup
    self._bp_version = self._read_bp_version()
    self._badge_parts: tuple[str, ...] | None = None

    # Recent changes auto-show
    self._changes_manager = RecentChangesManager()
    self._startup_time = time.monotonic()
    self._startup_check_done = False
    # Show on offroad transition (2s delay via frame check)
    self._offroad_trigger_time: float = 0
    ui_state.add_offroad_transition_callback(self._on_offroad_transition)

  @staticmethod
  def _read_bp_version() -> str:
    try:
      version_path = os.path.join(os.path.dirname(__file__), '../../../BPVERSION')
      with open(version_path) as f:
        return f.readline().strip()
    except Exception:
      return ""

  def _on_offroad_transition(self):
    """Trigger recent changes check 2s after going offroad."""
    if not ui_state.started:
      self._offroad_trigger_time = time.monotonic() + 2.0

  def _update_state(self):
    super()._update_state()
    now = time.monotonic()

    # Startup auto-show: 3s after init
    if not self._startup_check_done and now - self._startup_time > 3.0:
      self._startup_check_done = True
      print(f"[RecentChanges] Startup trigger fired ({now - self._startup_time:.1f}s after init)")
      self._changes_manager.show_if_needed()

    # Offroad transition auto-show
    if self._offroad_trigger_time > 0 and now > self._offroad_trigger_time:
      self._offroad_trigger_time = 0
      print("[RecentChanges] Offroad transition trigger fired")
      self._changes_manager.show_if_needed()

  def set_model_settings_callback(self, callback: Callable[[], None]):
    """Wire model info click to open Models settings panel."""
    self._model_info.set_click_callback(callback)

  def _get_version_text(self) -> str:
    """Override: parse version into badge parts and return fallback text."""
    result = super()._get_version_text()

    description = self.params.get("UpdaterCurrentDescription")
    if description:
      parts = description.split(" / ")
      if len(parts) >= 4:
        brand_text = f"bluepilot v{self._bp_version}" if self._bp_version else f"bluepilot v{parts[0].strip()}"
        self._badge_parts = (brand_text, parts[1].strip(), parts[2].strip(), parts[3].strip())
      else:
        brand_text = f"bluepilot v{self._bp_version}" if self._bp_version else "bluepilot"
        self._badge_parts = (brand_text,)
    else:
      brand_text = f"bluepilot v{self._bp_version}" if self._bp_version else "bluepilot"
      self._badge_parts = (brand_text,)

    return result

  def _render_header(self):
    """Override: render notification buttons + badge-styled version."""
    font = gui_app.font(FontWeight.MEDIUM)

    # Track how much width notification buttons consume
    buttons_width = 0

    # Update notification button (same as parent)
    if self.update_available:
      buttons_width += self.update_notif_rect.width

      highlight_color = rl.Color(75, 95, 255, 255) if self.current_state == HomeLayoutState.UPDATE else rl.Color(54, 77, 239, 255)
      rl.draw_rectangle_rounded(self.update_notif_rect, 0.3, 10, highlight_color)

      text = tr("UPDATE")
      text_size = measure_text_cached(font, text, HEAD_BUTTON_FONT_SIZE)
      text_x = self.update_notif_rect.x + (self.update_notif_rect.width - text_size.x) // 2
      text_y = self.update_notif_rect.y + (self.update_notif_rect.height - text_size.y) // 2
      rl.draw_text_ex(font, text, rl.Vector2(int(text_x), int(text_y)), HEAD_BUTTON_FONT_SIZE, 0, rl.WHITE)

    # Alert notification button (same as parent)
    if self.alert_count > 0:
      buttons_width += self.alert_notif_rect.width

      highlight_color = rl.Color(255, 70, 70, 255) if self.current_state == HomeLayoutState.ALERTS else rl.Color(226, 44, 44, 255)
      rl.draw_rectangle_rounded(self.alert_notif_rect, 0.3, 10, highlight_color)

      alert_text = trn("{} ALERT", "{} ALERTS", self.alert_count).format(self.alert_count)
      text_size = measure_text_cached(font, alert_text, HEAD_BUTTON_FONT_SIZE)
      text_x = self.alert_notif_rect.x + (self.alert_notif_rect.width - text_size.x) // 2
      text_y = self.alert_notif_rect.y + (self.alert_notif_rect.height - text_size.y) // 2
      rl.draw_text_ex(font, alert_text, rl.Vector2(int(text_x), int(text_y)), HEAD_BUTTON_FONT_SIZE, 0, rl.WHITE)

    # Render version badges (right-aligned)
    if self._badge_parts:
      if buttons_width > 0:
        buttons_width += int(SPACING * 1.5)

      badge_font = gui_app.font(FontWeight.SEMI_BOLD)
      badge_h = BPConstants.BADGE_HEIGHT
      badge_y = self.header_rect.y + (self.header_rect.height - badge_h) // 2

      # Calculate total badges width
      badge_widths = []
      for text in self._badge_parts:
        text_w = measure_text_cached(badge_font, text, BADGE_FONT_SIZE).x
        w = max(BPConstants.BADGE_MIN_WIDTH, int(text_w) + BADGE_TEXT_PAD_LEFT + BADGE_TEXT_PAD_RIGHT)
        badge_widths.append(w)

      total_badges_width = sum(badge_widths) + BADGE_SPACING * (len(badge_widths) - 1)

      # Right-align badges in the available space
      available_right = self.header_rect.x + self.header_rect.width
      badge_x = available_right - total_badges_width

      for i, (text, width) in enumerate(zip(self._badge_parts, badge_widths)):
        accent_color = BADGE_CONFIGS[i] if i < len(BADGE_CONFIGS) else BPColors.BADGE_BRAND
        self._draw_badge(badge_x, badge_y, width, badge_h, text, accent_color, badge_font)
        badge_x += width + BADGE_SPACING

  def _draw_badge(self, x, y, w, h, text, accent_color, font):
    """Draw a single badge card with shadow, background, accent bar, and text."""
    badge_rect = rl.Rectangle(x, y, w, h)
    radius = BPConstants.BADGE_RADIUS
    roundness = radius / (min(w, h) / 2) if min(w, h) > 0 else 0.2

    # Shadow (offset 2px right and down)
    shadow_rect = rl.Rectangle(x + 2, y + 2, w, h)
    rl.draw_rectangle_rounded(shadow_rect, roundness, 10, BPColors.SHADOW)

    # Card background
    rl.draw_rectangle_rounded(badge_rect, roundness, 10, BPColors.CARD_BACKGROUND)

    # Accent bar on left (clipped)
    rl.begin_scissor_mode(int(x), int(y), BPConstants.BADGE_ACCENT_WIDTH, int(h))
    rl.draw_rectangle_rounded(badge_rect, roundness, 10, accent_color)
    rl.end_scissor_mode()

    # Text (white, left-padded)
    text_size = measure_text_cached(font, text, BADGE_FONT_SIZE)
    text_x = x + BADGE_TEXT_PAD_LEFT
    text_y = y + (h - text_size.y) / 2
    rl.draw_text_ex(font, text, rl.Vector2(int(text_x), int(text_y)), BADGE_FONT_SIZE, 0, rl.WHITE)

  def _render_left_column(self):
    """Override: DriveStats instead of PrimeWidget. A paired device has the column to itself; an unpaired one keeps
    the pairing prompt in the right column, so the Driving Models card shares this one."""
    rect = self.left_column_rect
    if ui_state.prime_state.is_paired():
      self._drive_stats.render(rect)
      return

    # the card needs more room with a big model in play (two tiles)
    model_share = 0.55 if self._model_info.has_big_model else 0.3
    stats_height = rect.height * (1 - model_share) - SPACING / 2
    model_height = rect.height * model_share - SPACING / 2

    stats_rect = rl.Rectangle(rect.x, rect.y, rect.width, stats_height)
    self._drive_stats.render(stats_rect)

    model_rect = rl.Rectangle(rect.x, rect.y + stats_height + SPACING, rect.width, model_height)
    self._model_info.render(model_rect)

  def _render_right_column(self):
    """Override: on a paired device the Driving Models card (the on-device model and the big one) takes the column
    in place of upstream's Firehose prompt, which stays in Settings. Unpaired, the pairing prompt keeps it."""
    if ui_state.prime_state.is_paired():
      self._model_info.render(self.right_column_rect)
    else:
      super()._render_right_column()
