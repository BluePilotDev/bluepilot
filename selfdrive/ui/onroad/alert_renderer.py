import time
from typing import Optional
import pyray as rl
from dataclasses import dataclass
from cereal import messaging, log
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
from openpilot.system.hardware import TICI
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import Label

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus

ALERT_MARGIN = 40
ALERT_PADDING = 60
ALERT_LINE_SPACING = 45
ALERT_BORDER_RADIUS = 30

# Pill notification constants (for informational notifications)
PILL_HEIGHT_SINGLE = 110  # Height for single line (increased to accommodate more padding)
PILL_HEIGHT_DOUBLE = 164  # Height for two lines (increased to accommodate more padding)
PILL_PADDING_H = 30  # Horizontal padding
PILL_PADDING_V = 15  # Vertical padding (base padding, additional padding applied in text rendering)
PILL_TOP_MARGIN = 40
PILL_FONT_SIZE = 48
PILL_LINE_SPACING = 8  # Space between two lines
PILL_MAX_CHARS_PER_LINE = 28  # Maximum characters per line before wrapping

# Alert pill constants (3x larger than informational pills)
ALERT_PILL_HEIGHT_SINGLE = 210  # 70 * 3
ALERT_PILL_HEIGHT_DOUBLE = 348  # 116 * 3
ALERT_PILL_PADDING_H = 90  # 30 * 3
ALERT_PILL_PADDING_V = 45  # 15 * 3
ALERT_PILL_FONT_SIZE = 144  # 48 * 3
ALERT_PILL_LINE_SPACING = 24  # 8 * 3
ALERT_PILL_MAX_CHARS_PER_LINE = 28  # Same character limit, but larger font

ALERT_FONT_SMALL = 66
ALERT_FONT_MEDIUM = 74
ALERT_FONT_BIG = 88

ALERT_HEIGHTS = {
  AlertSize.small: 271,
  AlertSize.mid: 420,
}

SELFDRIVE_STATE_TIMEOUT = 5  # Seconds
SELFDRIVE_UNRESPONSIVE_TIMEOUT = 10  # Seconds

# Constants
ALERT_COLORS = {
  AlertStatus.normal: rl.Color(0x15, 0x15, 0x15, 0xF1),      # #151515 with alpha 0xF1
  AlertStatus.userPrompt: rl.Color(0xDA, 0x6F, 0x25, 0xF1),  # #DA6F25 with alpha 0xF1
  AlertStatus.critical: rl.Color(0xC9, 0x22, 0x31, 0xF1),    # #C92231 with alpha 0xF1
}

# Pill notification colors
PILL_BACKGROUND_COLOR = rl.Color(45, 45, 45, 255)  # Dark grey for informational notifications
PILL_ALERT_COLOR = rl.Color(0xDA, 0x6F, 0x25, 0xFF)  # Orange for alerts (matches AlertStatus.userPrompt)


@dataclass
class Alert:
  text1: str = ""
  text2: str = ""
  size: int = 0
  status: int = 0


# Pre-defined alert instances
ALERT_STARTUP_PENDING = Alert(
  text1=tr("sunnypilot Unavailable"),
  text2=tr("Waiting to start"),
  size=AlertSize.mid,
  status=AlertStatus.normal,
)

ALERT_CRITICAL_TIMEOUT = Alert(
  text1=tr("TAKE CONTROL IMMEDIATELY"),
  text2=tr("System Unresponsive"),
  size=AlertSize.full,
  status=AlertStatus.critical,
)

ALERT_CRITICAL_REBOOT = Alert(
  text1=tr("System Unresponsive"),
  text2=tr("Reboot Device"),
  size=AlertSize.mid,
  status=AlertStatus.normal,
)


class AlertRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.font_regular: rl.Font = gui_app.font(FontWeight.NORMAL)
    self.font_bold: rl.Font = gui_app.font(FontWeight.BOLD)

    # font size is set dynamically
    self._full_text1_label = Label("", font_size=0, font_weight=FontWeight.BOLD, text_alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
                                   text_alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_TOP)
    self._full_text2_label = Label("", font_size=ALERT_FONT_BIG, text_alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
                                   text_alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_TOP)

  def get_alert(self, sm: messaging.SubMaster) -> Alert | None:
    """Generate the current alert based on selfdrive state."""
    ss = sm['selfdriveState']

    # Check if selfdriveState messages have stopped arriving
    recv_frame = sm.recv_frame['selfdriveState']
    if not sm.updated['selfdriveState']:
      time_since_onroad = time.monotonic() - ui_state.started_time

      # 1. Never received selfdriveState since going onroad
      waiting_for_startup = recv_frame < ui_state.started_frame
      if waiting_for_startup and time_since_onroad > 5:
        return ALERT_STARTUP_PENDING

      # 2. Lost communication with selfdriveState after receiving it
      if TICI and not waiting_for_startup:
        ss_missing = time.monotonic() - sm.recv_time['selfdriveState']
        if ss_missing > SELFDRIVE_STATE_TIMEOUT:
          if ss.enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < SELFDRIVE_UNRESPONSIVE_TIMEOUT:
            return ALERT_CRITICAL_TIMEOUT
          return ALERT_CRITICAL_REBOOT

    # No alert if size is none
    if ss.alertSize == 0:
      return None

    # Don't get old alert
    if recv_frame < ui_state.started_frame:
      return None

    # Return current alert
    return Alert(text1=ss.alertText1, text2=ss.alertText2, size=ss.alertSize.raw, status=ss.alertStatus.raw)

  def set_speed_right(self, speed_right: int):
    self.speed_right = speed_right

  def _render(self, rect: rl.Rectangle):
    alert = self.get_alert(ui_state.sm)
    if not alert:
      return

    # Check if this is an informational notification (normal status, not full screen)
    is_informational = (alert.status == AlertStatus.normal and alert.size != AlertSize.full)

    # Check if this is an alert that should be rendered as orange pill (userPrompt, not full screen)
    is_alert_pill = (alert.status == AlertStatus.userPrompt and alert.size != AlertSize.full)

    if is_informational:
      # Render as pill-shaped notification in top right
      alert_rect = self._get_pill_rect(rect, alert)
      if alert_rect:
        self._draw_pill_background(alert_rect)
        text_rect = rl.Rectangle(
          alert_rect.x + PILL_PADDING_H,
          alert_rect.y + PILL_PADDING_V,
          alert_rect.width - 2 * PILL_PADDING_H,
          alert_rect.height - 2 * PILL_PADDING_V
        )
        self._draw_pill_text(text_rect, alert)
    elif is_alert_pill:
      # Render as orange pill in middle of screen, 1/3 up from bottom
      alert_rect = self._get_alert_pill_rect(rect, alert)
      if alert_rect:
        self._draw_alert_pill_background(alert_rect)
        text_rect = rl.Rectangle(
          alert_rect.x + ALERT_PILL_PADDING_H,
          alert_rect.y + ALERT_PILL_PADDING_V,
          alert_rect.width - 2 * ALERT_PILL_PADDING_H,
          alert_rect.height - 2 * ALERT_PILL_PADDING_V
        )
        self._draw_alert_pill_text(text_rect, alert)
    else:
      # Render as regular alert (banner style for critical/full screen)
      alert_rect = self._get_alert_rect(rect, alert.size)
      self._draw_background(alert_rect, alert)

      text_rect = rl.Rectangle(
        alert_rect.x + ALERT_PADDING,
        alert_rect.y + ALERT_PADDING,
        alert_rect.width - 2 * ALERT_PADDING,
        alert_rect.height - 2 * ALERT_PADDING
      )
      self._draw_text(text_rect, alert)

  def _get_alert_rect(self, rect: rl.Rectangle, size: int) -> rl.Rectangle:
    if size == AlertSize.full:
      return rect

    h = ALERT_HEIGHTS.get(size, rect.height)
    return rl.Rectangle(rect.x + ALERT_MARGIN, rect.y + rect.height - h + ALERT_MARGIN,
                        rect.width - ALERT_MARGIN * 2, h - ALERT_MARGIN * 2)

  def _get_pill_rect(self, rect: rl.Rectangle, alert: Alert) -> Optional[rl.Rectangle]:
    """Calculate pill-shaped notification rectangle in top right, between speed and steering wheel."""
    # Calculate available space
    # Steering wheel button is at: rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    wheel_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    center_x = self.speed_right

    # Available width is from the right of the speed to to wheel
    available_width = wheel_x - center_x

    if available_width < 100:  # Not enough space
      return None

    # Use text1 for pill (informational notifications typically have short text1)
    text = alert.text1 if alert.text1 else alert.text2
    if not text:
      return None

    # Check if text needs to be split into two lines
    needs_wrapping = len(text) > PILL_MAX_CHARS_PER_LINE

    # Measure text to determine width
    # For wrapping, measure the longest line
    if needs_wrapping:
      # Split text at space near the middle
      words = text.split()
      line1_words = []
      line2_words = []
      current_length = 0

      for word in words:
        test_line = ' '.join(line1_words + [word])
        if len(test_line) <= PILL_MAX_CHARS_PER_LINE:
          line1_words.append(word)
        else:
          line2_words.append(word)

      line1 = ' '.join(line1_words) if line1_words else text[:PILL_MAX_CHARS_PER_LINE]
      line2 = ' '.join(line2_words) if line2_words else text[PILL_MAX_CHARS_PER_LINE:]

      # Measure both lines to find the widest
      line1_size = measure_text_cached(self.font_bold, line1, PILL_FONT_SIZE)
      line2_size = measure_text_cached(self.font_bold, line2, PILL_FONT_SIZE)
      text_width = max(line1_size.x, line2_size.x)
      pill_height = PILL_HEIGHT_DOUBLE
    else:
      text_size = measure_text_cached(self.font_bold, text, PILL_FONT_SIZE)
      text_width = text_size.x
      pill_height = PILL_HEIGHT_SINGLE

    pill_width = min(text_width + 2 * PILL_PADDING_H, available_width)

    # Position: right-aligned, between center and wheel
    # Move 7% further to the right (toward center) by increasing spacing from wheel, plus 10px right
    pill_x = center_x + (wheel_x - center_x) / 2 - pill_width / 2 + 10
    # Vertical center aligns with current speed (header row)
    pill_y = rect.y + UI_CONFIG.header_align_center_y - pill_height / 2

    return rl.Rectangle(pill_x, pill_y, pill_width, pill_height)

  def _draw_background(self, rect: rl.Rectangle, alert: Alert) -> None:
    color = ALERT_COLORS.get(alert.status, ALERT_COLORS[AlertStatus.normal])

    if alert.size != AlertSize.full:
      roundness = ALERT_BORDER_RADIUS / (min(rect.width, rect.height) / 2)
      rl.draw_rectangle_rounded(rect, roundness, 10, color)
    else:
      rl.draw_rectangle_rec(rect, color)

  def _draw_pill_background(self, rect: rl.Rectangle) -> None:
    """Draw pill-shaped background with high roundness (curved sides) - blue for informational."""
    # Very high roundness for pill shape - makes ends more circular (like 1/3 of a circle)
    # Using 0.7-0.8 gives a more pronounced rounded pill effect
    roundness = 0.75
    rl.draw_rectangle_rounded(rect, roundness, 10, PILL_BACKGROUND_COLOR)

  def _draw_alert_pill_background(self, rect: rl.Rectangle) -> None:
    """Draw orange pill-shaped background for alerts."""
    roundness = 0.75
    rl.draw_rectangle_rounded(rect, roundness, 10, PILL_ALERT_COLOR)

  def _get_alert_pill_rect(self, rect: rl.Rectangle, alert: Alert) -> Optional[rl.Rectangle]:
    """Calculate orange pill alert rectangle in middle of screen, 1/3 up from bottom (3x larger)."""
    # Use text1 for pill (alerts typically have short text1)
    text = alert.text1 if alert.text1 else alert.text2
    if not text:
      return None

    # Check if text needs to be split into two lines
    needs_wrapping = len(text) > ALERT_PILL_MAX_CHARS_PER_LINE

    if needs_wrapping:
      # Split text at space near the middle
      words = text.split()
      line1_words = []
      line2_words = []

      for word in words:
        test_line = ' '.join(line1_words + [word])
        if len(test_line) <= ALERT_PILL_MAX_CHARS_PER_LINE:
          line1_words.append(word)
        else:
          line2_words.append(word)

      line1 = ' '.join(line1_words) if line1_words else text[:ALERT_PILL_MAX_CHARS_PER_LINE]
      line2 = ' '.join(line2_words) if line2_words else text[ALERT_PILL_MAX_CHARS_PER_LINE:]

      # Measure both lines to find the widest (using alert font size)
      line1_size = measure_text_cached(self.font_bold, line1, ALERT_PILL_FONT_SIZE)
      line2_size = measure_text_cached(self.font_bold, line2, ALERT_PILL_FONT_SIZE)
      text_width = max(line1_size.x, line2_size.x)
      pill_height = ALERT_PILL_HEIGHT_DOUBLE
    else:
      text_size = measure_text_cached(self.font_bold, text, ALERT_PILL_FONT_SIZE)
      text_width = text_size.x
      pill_height = ALERT_PILL_HEIGHT_SINGLE

    pill_width = text_width + 2 * ALERT_PILL_PADDING_H

    # Position: centered horizontally, 1/3 up from bottom
    pill_x = rect.x + (rect.width - pill_width) / 2
    # 1/3 up from bottom means 2/3 down from top
    pill_y = rect.y + (rect.height * 2 / 3) - (pill_height / 2)

    return rl.Rectangle(pill_x, pill_y, pill_width, pill_height)

  def _draw_alert_pill_text(self, rect: rl.Rectangle, alert: Alert) -> None:
    """Draw text in alert pill (centered, single or two lines, 3x larger)."""
    # Use text1 if available, otherwise text2
    text = alert.text1 if alert.text1 else alert.text2
    if not text:
      return

    # Check if text needs to be split into two lines
    needs_wrapping = len(text) > ALERT_PILL_MAX_CHARS_PER_LINE

    if needs_wrapping:
      # Split text at space near the middle
      words = text.split()
      line1_words = []
      line2_words = []

      for word in words:
        test_line = ' '.join(line1_words + [word])
        if len(test_line) <= ALERT_PILL_MAX_CHARS_PER_LINE:
          line1_words.append(word)
        else:
          line2_words.append(word)

      line1 = ' '.join(line1_words) if line1_words else text[:ALERT_PILL_MAX_CHARS_PER_LINE]
      line2 = ' '.join(line2_words) if line2_words else text[ALERT_PILL_MAX_CHARS_PER_LINE:]

      # Measure both lines (using alert font size)
      line1_size = measure_text_cached(self.font_bold, line1, ALERT_PILL_FONT_SIZE)
      line2_size = measure_text_cached(self.font_bold, line2, ALERT_PILL_FONT_SIZE)

      # Calculate total height needed for two lines
      total_text_height = line1_size.y + ALERT_PILL_LINE_SPACING + line2_size.y

      # Center vertically with extra top and bottom padding
      extra_top_padding = rect.height * 0.05
      extra_bottom_padding = rect.height * 0.08
      available_height = rect.height - extra_top_padding - extra_bottom_padding
      start_y = rect.y + extra_top_padding + (available_height - total_text_height) / 2

      # Draw first line (centered horizontally)
      line1_x = rect.x + (rect.width - line1_size.x) / 2
      rl.draw_text_ex(self.font_bold, line1, rl.Vector2(line1_x, start_y), ALERT_PILL_FONT_SIZE, 0, rl.WHITE)

      # Draw second line (centered horizontally)
      line2_x = rect.x + (rect.width - line2_size.x) / 2
      line2_y = start_y + line1_size.y + ALERT_PILL_LINE_SPACING
      rl.draw_text_ex(self.font_bold, line2, rl.Vector2(line2_x, line2_y), ALERT_PILL_FONT_SIZE, 0, rl.WHITE)
    else:
      # Single line - center text vertically and horizontally
      text_size = measure_text_cached(self.font_bold, text, ALERT_PILL_FONT_SIZE)
      x = rect.x + (rect.width - text_size.x) / 2
      y = rect.y + (rect.height - text_size.y) / 2
      rl.draw_text_ex(self.font_bold, text, rl.Vector2(x, y), ALERT_PILL_FONT_SIZE, 0, rl.WHITE)

  def _draw_pill_text(self, rect: rl.Rectangle, alert: Alert) -> None:
    """Draw text in pill-shaped notification (centered, single or two lines)."""
    # Use text1 if available, otherwise text2
    text = alert.text1 if alert.text1 else alert.text2
    if not text:
      return

    # Check if text needs to be split into two lines
    needs_wrapping = len(text) > PILL_MAX_CHARS_PER_LINE

    if needs_wrapping:
      # Split text at space near the middle
      words = text.split()
      line1_words = []
      line2_words = []

      for word in words:
        test_line = ' '.join(line1_words + [word])
        if len(test_line) <= PILL_MAX_CHARS_PER_LINE:
          line1_words.append(word)
        else:
          line2_words.append(word)

      line1 = ' '.join(line1_words) if line1_words else text[:PILL_MAX_CHARS_PER_LINE]
      line2 = ' '.join(line2_words) if line2_words else text[PILL_MAX_CHARS_PER_LINE:]

      # Measure both lines
      line1_size = measure_text_cached(self.font_bold, line1, PILL_FONT_SIZE)
      line2_size = measure_text_cached(self.font_bold, line2, PILL_FONT_SIZE)

      # Calculate total height needed for two lines
      total_text_height = line1_size.y + PILL_LINE_SPACING + line2_size.y

      # Center vertically with extra top and bottom padding
      # Add extra padding to prevent text from sticking above or below the pill
      # Increased padding percentages for more space (doubled from previous change)
      extra_top_padding = rect.height * 0.25  # Increased from 0.15
      extra_bottom_padding = rect.height * 0.22  # Increased from 0.15 for more bottom space
      available_height = rect.height - extra_top_padding - extra_bottom_padding
      start_y = rect.y + extra_top_padding + (available_height - total_text_height) / 2

      # Draw first line (centered horizontally)
      line1_x = rect.x + (rect.width - line1_size.x) / 2
      rl.draw_text_ex(self.font_bold, line1, rl.Vector2(line1_x, start_y), PILL_FONT_SIZE, 0, rl.WHITE)

      # Draw second line (centered horizontally)
      line2_x = rect.x + (rect.width - line2_size.x) / 2
      line2_y = start_y + line1_size.y + PILL_LINE_SPACING
      rl.draw_text_ex(self.font_bold, line2, rl.Vector2(line2_x, line2_y), PILL_FONT_SIZE, 0, rl.WHITE)
    else:
      # Single line - center text vertically and horizontally with extra padding
      text_size = measure_text_cached(self.font_bold, text, PILL_FONT_SIZE)
      # Add extra top/bottom padding for single line too (doubled from previous change)
      extra_padding = rect.height * 0.25
      available_height = rect.height - (extra_padding * 2)
      x = rect.x + (rect.width - text_size.x) / 2
      y = rect.y + extra_padding + (available_height - text_size.y) / 2
      rl.draw_text_ex(self.font_bold, text, rl.Vector2(x, y), PILL_FONT_SIZE, 0, rl.WHITE)

  def _draw_text(self, rect: rl.Rectangle, alert: Alert) -> None:
    if alert.size == AlertSize.small:
      self._draw_centered(alert.text1, rect, self.font_bold, ALERT_FONT_MEDIUM)

    elif alert.size == AlertSize.mid:
      self._draw_centered(alert.text1, rect, self.font_bold, ALERT_FONT_BIG, center_y=False)
      rect.y += ALERT_FONT_BIG + ALERT_LINE_SPACING
      self._draw_centered(alert.text2, rect, self.font_regular, ALERT_FONT_SMALL, center_y=False)

    else:
      is_long = len(alert.text1) > 15
      font_size1 = 132 if is_long else 177

      top_offset = 200 if is_long or '\n' in alert.text1 else 270
      title_rect = rl.Rectangle(rect.x, rect.y + top_offset, rect.width, 600)
      self._full_text1_label.set_font_size(font_size1)
      self._full_text1_label.set_text(alert.text1)
      self._full_text1_label.render(title_rect)

      bottom_offset = 361 if is_long else 420
      subtitle_rect = rl.Rectangle(rect.x, rect.y + rect.height - bottom_offset, rect.width, 300)
      self._full_text2_label.set_text(alert.text2)
      self._full_text2_label.render(subtitle_rect)

  def _draw_centered(self, text, rect, font, font_size, center_y=True, color=rl.WHITE) -> None:
    text_size = measure_text_cached(font, text, font_size)
    x = rect.x + (rect.width - text_size.x) / 2
    y = rect.y + ((rect.height - text_size.y) / 2 if center_y else 0)
    rl.draw_text_ex(font, text, rl.Vector2(x, y), font_size, 0, color)
