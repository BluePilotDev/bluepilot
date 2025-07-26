#!/usr/bin/env python3
import pyray as rl
import select
import sys
import subprocess
import os

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.widget import Widget
from openpilot.system.ui.text import wrap_text
from openpilot.system.ui.bp_updater import BPUpdaterPanel

# Try to import sunnypilot extensions, fallback to stock
try:
  from openpilot.system.ui.sunnypilot.lib.application import gui_app_sp
  HAS_SUNNYPILOT = True
except ImportError:
  HAS_SUNNYPILOT = False

# Constants
PROGRESS_BAR_WIDTH = 1000
PROGRESS_BAR_HEIGHT = 20
DEGREES_PER_SECOND = 360.0  # one full rotation per second
MARGIN_H = 100
TEXTURE_SIZE = 360
FONT_SIZE = 96
LINE_HEIGHT = 104
ERROR_FONT_SIZE = 48
ERROR_LINE_HEIGHT = 56
BUTTON_HEIGHT = 120
BUTTON_WIDTH = 300
DARKGRAY = (55, 55, 55, 255)
ERROR_BG = (0, 0, 0, 255)
BUTTON_COLOR = (70, 70, 70, 255)
BUTTON_HOVER_COLOR = (100, 100, 100, 255)
FORD_BLUE_COLOR = (0, 63, 127, 255)
FORD_BLUE_HOVER_COLOR = (30, 93, 157, 255)
ORANGE_COLOR = (255, 140, 0, 255)


def clamp(value, min_value, max_value):
  return max(min(value, max_value), min_value)


class BPSpinner(Widget):
  def __init__(self):
    super().__init__()

    # Load textures with fallback
    try:
      if HAS_SUNNYPILOT:
        self._comma_texture = gui_app_sp.sp_texture("images/spinner_sunnypilot.png", TEXTURE_SIZE, TEXTURE_SIZE)
      else:
        self._comma_texture = gui_app.texture("images/spinner_comma.png", TEXTURE_SIZE, TEXTURE_SIZE)
    except Exception:
      self._comma_texture = None

    try:
      self._spinner_texture = gui_app.texture("images/spinner_track.png", TEXTURE_SIZE, TEXTURE_SIZE, alpha_premultiply=True)
    except Exception:
      self._spinner_texture = None

    self._rotation = 0.0
    self._progress: int | None = None
    self._status_text: str = ""
    self._wrapped_lines: list[str] = []

    # BP-specific features
    self._error_mode = False
    self._error_text = ""
    self._output_buffer: list[str] = []
    self._scroll_offset = 0
    self._max_scroll = 0

    # Updater panel
    self._updater_mode = False
    self._updater_panel = None

    # Button states
    self._reboot_button_rect = rl.Rectangle(0, 0, 0, 0)
    self._reboot_hover = False
    self._show_updater_button_rect = rl.Rectangle(0, 0, 0, 0)
    self._show_updater_hover = False

  def set_text(self, text: str) -> None:
    # Handle special BP commands
    if text == "BUILD_FAILED":
      self._enter_error_mode()
      return
    elif text == "BUILD_RETRY":
      self._exit_error_mode()
      return
    elif text == "SHOW_UPDATER":
      self._enter_updater_mode()
      return
    elif text == "EXIT_UPDATER":
      self._exit_updater_mode()
      return

    # Handle progress|text format (BP enhancement)
    if "|" in text:
      parts = text.split("|", 1)
      if parts[0].isdigit():
        self._progress = clamp(int(parts[0]), 0, 100)
        self._status_text = parts[1]
        self._wrapped_lines = wrap_text(self._status_text, FONT_SIZE, gui_app.width - MARGIN_H)
        # Store in output buffer
        if parts[1].strip():
          self._output_buffer.append(parts[1])
        return

    # Stock behavior: progress OR text
    if text.isdigit():
      self._progress = clamp(int(text), 0, 100)
      self._wrapped_lines = []
    else:
      self._progress = None
      self._status_text = text
      self._wrapped_lines = wrap_text(text, FONT_SIZE, gui_app.width - MARGIN_H)
      # Store in output buffer
      if text.strip():
        self._output_buffer.append(text)

    # Limit buffer size
    if len(self._output_buffer) > 500:
      self._output_buffer = self._output_buffer[-500:]

  def _enter_error_mode(self):
    """Switch to error mode showing full build output"""
    self._error_mode = True
    self._error_text = "\n".join(self._output_buffer)

    # Calculate max scroll to position at bottom (latest lines)
    if self._error_text:
      # Use approximate screen dimensions for initial calculation
      screen_height = 1000  # approximate
      text_area_height = screen_height - 200  # approximate available height

      wrapped_lines = []
      for line in self._error_text.split('\n'):
        if line.strip():
          # Approximate line wrapping
          line_count = max(1, len(line) // 80)  # rough estimate
          wrapped_lines.extend([line] * line_count)

      total_text_height = len(wrapped_lines) * ERROR_LINE_HEIGHT
      self._max_scroll = max(0, total_text_height - text_area_height)
      self._scroll_offset = self._max_scroll  # Start at bottom
    else:
      self._scroll_offset = 0

  def _exit_error_mode(self):
    """Exit error mode and return to normal spinner"""
    self._error_mode = False
    self._error_text = ""
    self._output_buffer.clear()
    self._progress = None
    self._status_text = ""
    self._wrapped_lines = []
    self._scroll_offset = 0

  def _enter_updater_mode(self):
    """Switch to updater panel mode"""
    self._updater_mode = True
    if not self._updater_panel:
      self._updater_panel = BPUpdaterPanel()

  def _exit_updater_mode(self):
    """Exit updater panel mode"""
    self._updater_mode = False

  def _handle_input(self):
    """Handle mouse input for buttons in error mode"""
    if self._updater_mode:
      # Let updater panel handle its own input
      return

    if not self._error_mode:
      return

    mouse_pos = rl.get_mouse_position()

    # Check button hover
    self._reboot_hover = rl.check_collision_point_rec(mouse_pos, self._reboot_button_rect)
    # self._show_updater_hover = rl.check_collision_point_rec(mouse_pos, self._show_updater_button_rect)  # TEMPORARILY DISABLED

    # Handle clicks
    if rl.is_mouse_button_pressed(rl.MOUSE_BUTTON_LEFT):
      if self._reboot_hover:
        print("Reboot button clicked")
        try:
          # Try different reboot commands
          result = subprocess.run(["sudo", "reboot"], capture_output=True, text=True, timeout=5)
          if result.returncode != 0:
            subprocess.run(["reboot"], check=False)
        except Exception as e:
          print(f"Reboot failed: {e}")
          print("Development environment - reboot not available")

      # elif self._show_updater_hover:  # TEMPORARILY DISABLED
      #   print("Show Updater button clicked")
      #   self._enter_updater_mode()

    # Handle scrolling
    wheel_move = rl.get_mouse_wheel_move()
    if wheel_move != 0:
      self._scroll_offset -= int(wheel_move * 3)
      self._scroll_offset = clamp(self._scroll_offset, 0, self._max_scroll)

  def _render_error_screen(self, rect: rl.Rectangle):
    """Render the error screen with scrollable text and buttons"""
    # Fill background
    rl.draw_rectangle_rec(rect, ERROR_BG)

    # Title
    title = "Build Failed"
    title_size = measure_text_cached(gui_app.font(), title, FONT_SIZE)
    title_y = 50
    rl.draw_text_ex(gui_app.font(), title, rl.Vector2(rect.width / 2 - title_size.x / 2, title_y),
                    FONT_SIZE, 0.0, ORANGE_COLOR)

    # Error text area
    text_area_y = title_y + FONT_SIZE + 30
    text_area_height = rect.height - text_area_y - BUTTON_HEIGHT - 60

    # Wrap error text
    wrapped_error_lines = []
    if self._error_text:
      for line in self._error_text.split('\n'):
        if line.strip():
          wrapped_error_lines.extend(wrap_text(line, ERROR_FONT_SIZE, rect.width - MARGIN_H))

    # Calculate max scroll
    total_text_height = len(wrapped_error_lines) * ERROR_LINE_HEIGHT
    self._max_scroll = max(0, total_text_height - int(text_area_height))

    # Render visible text lines
    visible_lines = int(text_area_height / ERROR_LINE_HEIGHT) + 1
    start_line = min(self._scroll_offset // ERROR_LINE_HEIGHT, len(wrapped_error_lines))
    end_line = min(start_line + visible_lines, len(wrapped_error_lines))

    for i in range(start_line, end_line):
      line_y = text_area_y + (i - start_line) * ERROR_LINE_HEIGHT - (self._scroll_offset % ERROR_LINE_HEIGHT)
      if line_y >= text_area_y - ERROR_LINE_HEIGHT and line_y <= text_area_y + text_area_height:
        rl.draw_text_ex(gui_app.font(), wrapped_error_lines[i],
                        rl.Vector2(MARGIN_H / 2, line_y),
                        ERROR_FONT_SIZE, 0.0, rl.WHITE)

    # Two buttons: Reboot and Show Updater
    button_y = rect.height - BUTTON_HEIGHT - 40
    button_spacing = 20
    total_button_width = 2 * BUTTON_WIDTH + button_spacing
    start_x = (rect.width - total_button_width) / 2

    # Reboot button (left) - for now, center it by using full width
    self._reboot_button_rect = rl.Rectangle((rect.width - BUTTON_WIDTH) / 2, button_y, BUTTON_WIDTH, BUTTON_HEIGHT)
    reboot_color = FORD_BLUE_HOVER_COLOR if self._reboot_hover else FORD_BLUE_COLOR
    rl.draw_rectangle_rounded(self._reboot_button_rect, 0.4, 20, reboot_color)

    reboot_text = "Reboot"
    reboot_text_size = measure_text_cached(gui_app.font(), reboot_text, ERROR_FONT_SIZE + 8)
    rl.draw_text_ex(gui_app.font(), reboot_text,
                    rl.Vector2(self._reboot_button_rect.x + (BUTTON_WIDTH - reboot_text_size.x) / 2,
                              self._reboot_button_rect.y + (BUTTON_HEIGHT - reboot_text_size.y) / 2),
                    ERROR_FONT_SIZE + 8, 0.0, rl.WHITE)

    # Show Updater button (right) - TEMPORARILY DISABLED
    # self._show_updater_button_rect = rl.Rectangle(start_x + BUTTON_WIDTH + button_spacing, button_y, BUTTON_WIDTH, BUTTON_HEIGHT)
    # updater_color = ORANGE_COLOR if self._show_updater_hover else (180, 100, 0, 255)  # Darker orange
    # rl.draw_rectangle_rounded(self._show_updater_button_rect, 0.4, 20, updater_color)

    # updater_text = "Updater"
    # updater_text_size = measure_text_cached(gui_app.font(), updater_text, ERROR_FONT_SIZE + 8)
    # rl.draw_text_ex(gui_app.font(), updater_text,
    #                 rl.Vector2(self._show_updater_button_rect.x + (BUTTON_WIDTH - updater_text_size.x) / 2,
    #                           self._show_updater_button_rect.y + (BUTTON_HEIGHT - updater_text_size.y) / 2),
    #                 ERROR_FONT_SIZE + 8, 0.0, rl.WHITE)

  def _render(self, rect: rl.Rectangle):
    """Main render method"""
    self._handle_input()

    if self._updater_mode:
      # Render updater panel
      if self._updater_panel:
        self._updater_panel.render(rect)
      return

    if self._error_mode:
      self._render_error_screen(rect)
      return

    # Fixed layout - spinner and progress bar stay in same position
    spacing = 50

    # Center spinner vertically in upper portion of screen
    center_y = rect.height * 0.35  # Fixed position

    # Progress bar position (fixed)
    progress_y = center_y + TEXTURE_SIZE / 2.0 + spacing

    # Text area (fixed, uses remaining space)
    text_area_start = progress_y + PROGRESS_BAR_HEIGHT + 30
    text_area_height = rect.height - text_area_start - 50  # Leave bottom margin

    center = rl.Vector2(rect.width / 2.0, center_y)
    spinner_origin = rl.Vector2(TEXTURE_SIZE / 2.0, TEXTURE_SIZE / 2.0)
    comma_position = rl.Vector2(center.x - TEXTURE_SIZE / 2.0, center.y - TEXTURE_SIZE / 2.0)

    delta_time = rl.get_frame_time()
    self._rotation = (self._rotation + DEGREES_PER_SECOND * delta_time) % 360.0

    # Draw rotating spinner and static comma logo
    if self._spinner_texture:
      rl.draw_texture_pro(self._spinner_texture, rl.Rectangle(0, 0, TEXTURE_SIZE, TEXTURE_SIZE),
                          rl.Rectangle(center.x, center.y, TEXTURE_SIZE, TEXTURE_SIZE),
                          spinner_origin, self._rotation, rl.WHITE)
    if self._comma_texture:
      rl.draw_texture_v(self._comma_texture, comma_position, rl.WHITE)

    # Draw progress bar if available
    if self._progress is not None:
      bar = rl.Rectangle(center.x - PROGRESS_BAR_WIDTH / 2.0, progress_y, PROGRESS_BAR_WIDTH, PROGRESS_BAR_HEIGHT)
      rl.draw_rectangle_rounded(bar, 1, 10, DARKGRAY)
      bar.width *= self._progress / 100.0
      rl.draw_rectangle_rounded(bar, 1, 10, rl.WHITE)

    # Draw status text in fixed area with dynamic scaling to fit all text
    if self._status_text:
      # Constrain text width to max 1080px + margins
      max_content_width = 1080
      text_width = min(rect.width - MARGIN_H, max_content_width)

      # Start with normal font size and get initial wrap
      base_font_size = FONT_SIZE
      base_line_height = LINE_HEIGHT
      wrapped_lines = wrap_text(self._status_text, base_font_size, text_width)

      # Calculate required height for all text
      required_height = len(wrapped_lines) * base_line_height

      # Calculate scale factor to fit all text in available height
      if required_height > text_area_height:
        scale_factor = text_area_height / required_height
        font_size = int(base_font_size * scale_factor)
        line_height = int(base_line_height * scale_factor)

        # Re-wrap with new font size
        wrapped_lines = wrap_text(self._status_text, font_size, text_width)
      else:
        font_size = base_font_size
        line_height = base_line_height

      # Draw all lines (they should all fit now)
      for i, line in enumerate(wrapped_lines):
        text_size = measure_text_cached(gui_app.font(), line, font_size)
        rl.draw_text_ex(gui_app.font(), line,
                        rl.Vector2(center.x - text_size.x / 2, text_area_start + i * line_height),
                        font_size, 0.0, rl.WHITE)


def _read_stdin():
  """Non-blocking read of available lines from stdin."""
  lines = []
  while True:
    rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not rlist:
      break
    line = sys.stdin.readline().strip()
    if line == "":
      break
    lines.append(line)
  return lines


def main():
  gui_app.init_window("BP Spinner")
  bp_spinner = BPSpinner()
  for _ in gui_app.render():
    text_list = _read_stdin()
    if text_list:
      bp_spinner.set_text(text_list[-1])

    bp_spinner.render(rl.Rectangle(0, 0, gui_app.width, gui_app.height))


if __name__ == "__main__":
  main()
