#!/usr/bin/env python3
# BluePilot: verbose build spinner. Extends the stock spinner protocol with:
#   - "<pct>|<text>"  -> progress bar + live status line (the "verbose" build output)
#   - "BUILD_RETRY"   -> clear the error screen and keep building
#   - "BUILD_FAILED"  -> full-screen scrollable build log + a Reboot button
# Driven by common/bp_spinner.py (client) from system/manager/bp_build.py.
# Ported from bp-5.0; the bp_updater panel dependency was dropped and sizing made
# responsive (big_ui) so it renders on MICI/TIZI as well as TICI.
import select
import subprocess
import sys

import pyray as rl

from openpilot.system.ui.lib.application import gui_app, MousePos
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.text import wrap_text
from openpilot.system.ui.widgets import Widget

# Responsive sizing — large (TICI/TIZI) vs small (MICI) screens.
if gui_app.big_ui():
  PROGRESS_BAR_WIDTH = 1000
  PROGRESS_BAR_HEIGHT = 20
  TEXTURE_SIZE = 360
  COMMA_IMAGE_SIZE = 280
  FONT_SIZE = 96
  LINE_HEIGHT = 104
  ERROR_FONT_SIZE = 48
  ERROR_LINE_HEIGHT = 56
  BUTTON_WIDTH = 300
  BUTTON_HEIGHT = 120
  MAX_CONTENT_WIDTH = 1080
else:
  PROGRESS_BAR_WIDTH = 268
  PROGRESS_BAR_HEIGHT = 10
  TEXTURE_SIZE = 140
  COMMA_IMAGE_SIZE = 108
  FONT_SIZE = 48
  LINE_HEIGHT = 56
  ERROR_FONT_SIZE = 26
  ERROR_LINE_HEIGHT = 32
  BUTTON_WIDTH = 200
  BUTTON_HEIGHT = 80
  MAX_CONTENT_WIDTH = 460

DEGREES_PER_SECOND = 360.0  # one full rotation per second
MARGIN_H = 100
DARKGRAY = (55, 55, 55, 255)
ERROR_BG = (0, 0, 0, 255)
FORD_BLUE_COLOR = (0, 63, 127, 255)
FORD_BLUE_HOVER_COLOR = (30, 93, 157, 255)
ORANGE_COLOR = (255, 140, 0, 255)
OUTPUT_BUFFER_MAX = 500
# BluePilot: scons/gcc output lines are frequently full file paths with no spaces or hyphens
# (e.g. /data/openpilot/opendbc_repo/.../carcontroller.py) — text.wrap_text() only breaks on
# whitespace/'-' (re.split(r"(\s+|-)", ...)), so an unbroken path becomes one "word" wider than
# the wrap width and is rendered as-is rather than split. Centering that on screen (center.x -
# text_size.x/2) then starts the line at a deeply negative X, so almost all of it renders off
# the left/right edge — the "oversized, unreadable" text multiple users hit. Hard-cap line length
# before it ever reaches wrap_text() so no single line can produce this regardless of content.
MAX_LINE_CHARS = 100


def clamp(value, min_value, max_value):
  return max(min(value, max_value), min_value)


def _truncate(text: str, max_chars: int = MAX_LINE_CHARS) -> str:
  if len(text) <= max_chars:
    return text
  return text[:max_chars - 1] + "…"


class BPSpinner(Widget):
  def __init__(self):
    super().__init__()

    try:
      self._comma_texture = gui_app.texture("../../sunnypilot/selfdrive/assets/images/spinner_sunnypilot.png",
                                            COMMA_IMAGE_SIZE, COMMA_IMAGE_SIZE)
    except Exception:
      try:
        self._comma_texture = gui_app.texture("images/spinner_comma.png", COMMA_IMAGE_SIZE, COMMA_IMAGE_SIZE)
      except Exception:
        self._comma_texture = None

    try:
      self._spinner_texture = gui_app.texture("images/spinner_track.png", TEXTURE_SIZE, TEXTURE_SIZE, alpha_premultiply=True)
    except Exception:
      self._spinner_texture = None

    self._rotation = 0.0
    self._progress: int | None = None
    self._status_text: str = ""

    # error screen state
    self._error_mode = False
    self._output_buffer: list[str] = []
    self._scroll_offset = 0
    self._max_scroll = 0
    self._reboot_button_rect = rl.Rectangle(0, 0, 0, 0)
    self._reboot_hover = False

  def set_text(self, text: str) -> None:
    if text == "BUILD_FAILED":
      self._error_mode = True
      return
    if text == "BUILD_RETRY":
      self._exit_error_mode()
      return

    # "<pct>|<status line>" — progress bar plus a live status line
    if "|" in text:
      head, tail = text.split("|", 1)
      if head.isdigit():
        self._progress = clamp(int(head), 0, 100)
        self._status_text = _truncate(tail)
        self._append_output(tail)
        return

    if text.isdigit():
      self._progress = clamp(int(text), 0, 100)
    else:
      self._progress = None
      self._status_text = _truncate(text)
      self._append_output(text)

  def _append_output(self, line: str) -> None:
    if line.strip():
      self._output_buffer.append(_truncate(line))
      if len(self._output_buffer) > OUTPUT_BUFFER_MAX:
        self._output_buffer = self._output_buffer[-OUTPUT_BUFFER_MAX:]

  def _exit_error_mode(self) -> None:
    self._error_mode = False
    self._output_buffer.clear()
    self._progress = None
    self._status_text = ""
    self._scroll_offset = 0

  def _handle_input(self) -> None:
    if not self._error_mode:
      return

    # hover feedback (dev mouse only; touchscreens have no hover) + wheel scroll of the log
    mouse_pos = rl.get_mouse_position()
    self._reboot_hover = rl.check_collision_point_rec(mouse_pos, self._reboot_button_rect)

    wheel_move = rl.get_mouse_wheel_move()
    if wheel_move != 0:
      self._scroll_offset = clamp(self._scroll_offset - int(wheel_move * 3 * ERROR_LINE_HEIGHT), 0, self._max_scroll)

  def _handle_mouse_release(self, mouse_pos: MousePos) -> None:
    # framework-dispatched tap handler (works with touch); reboot when the button is tapped
    super()._handle_mouse_release(mouse_pos)
    if not self._error_mode:
      return
    r = self._reboot_button_rect
    if r.x <= mouse_pos.x <= r.x + r.width and r.y <= mouse_pos.y <= r.y + r.height:
      try:
        result = subprocess.run(["sudo", "reboot"], capture_output=True, text=True, timeout=5)
        if result.returncode != 0:
          subprocess.run(["reboot"], check=False)
      except Exception as e:
        print(f"Reboot failed: {e}")

  def _render_error_screen(self, rect: rl.Rectangle) -> None:
    rl.draw_rectangle_rec(rect, ERROR_BG)

    title = "Build Failed"
    title_size = measure_text_cached(gui_app.font(), title, FONT_SIZE)
    title_y = 50
    rl.draw_text_ex(gui_app.font(), title, rl.Vector2(rect.width / 2 - title_size.x / 2, title_y),
                    FONT_SIZE, 0.0, ORANGE_COLOR)

    text_area_y = title_y + FONT_SIZE + 30
    text_area_height = rect.height - text_area_y - BUTTON_HEIGHT - 60

    wrapped_error_lines: list[str] = []
    for line in self._output_buffer:
      wrapped_error_lines.extend(wrap_text(line, ERROR_FONT_SIZE, int(rect.width - MARGIN_H)))

    total_text_height = len(wrapped_error_lines) * ERROR_LINE_HEIGHT
    self._max_scroll = max(0, total_text_height - int(text_area_height))
    # default to showing the tail (the actual error)
    if self._scroll_offset == 0 and self._max_scroll > 0:
      self._scroll_offset = self._max_scroll

    visible_lines = int(text_area_height / ERROR_LINE_HEIGHT) + 1
    start_line = min(self._scroll_offset // ERROR_LINE_HEIGHT, len(wrapped_error_lines))
    end_line = min(start_line + visible_lines, len(wrapped_error_lines))

    for i in range(start_line, end_line):
      line_y = text_area_y + (i - start_line) * ERROR_LINE_HEIGHT - (self._scroll_offset % ERROR_LINE_HEIGHT)
      if text_area_y - ERROR_LINE_HEIGHT <= line_y <= text_area_y + text_area_height:
        rl.draw_text_ex(gui_app.font(), wrapped_error_lines[i], rl.Vector2(MARGIN_H / 2, line_y),
                        ERROR_FONT_SIZE, 0.0, rl.WHITE)

    button_y = rect.height - BUTTON_HEIGHT - 40
    self._reboot_button_rect = rl.Rectangle((rect.width - BUTTON_WIDTH) / 2, button_y, BUTTON_WIDTH, BUTTON_HEIGHT)
    reboot_color = FORD_BLUE_HOVER_COLOR if self._reboot_hover else FORD_BLUE_COLOR
    rl.draw_rectangle_rounded(self._reboot_button_rect, 0.4, 20, reboot_color)

    reboot_text = "Reboot"
    rt_size = measure_text_cached(gui_app.font(), reboot_text, ERROR_FONT_SIZE + 8)
    rl.draw_text_ex(gui_app.font(), reboot_text,
                    rl.Vector2(self._reboot_button_rect.x + (BUTTON_WIDTH - rt_size.x) / 2,
                               self._reboot_button_rect.y + (BUTTON_HEIGHT - rt_size.y) / 2),
                    ERROR_FONT_SIZE + 8, 0.0, rl.WHITE)

  def _render(self, rect: rl.Rectangle) -> None:
    self._handle_input()

    if self._error_mode:
      self._render_error_screen(rect)
      return

    spacing = 50 if gui_app.big_ui() else 20
    center_y = rect.height * 0.35
    progress_y = center_y + TEXTURE_SIZE / 2.0 + spacing
    text_area_start = progress_y + PROGRESS_BAR_HEIGHT + 30
    text_area_height = rect.height - text_area_start - 50

    center = rl.Vector2(rect.width / 2.0, center_y)
    spinner_origin = rl.Vector2(TEXTURE_SIZE / 2.0, TEXTURE_SIZE / 2.0)
    comma_position = rl.Vector2(center.x - COMMA_IMAGE_SIZE / 2.0, center.y - COMMA_IMAGE_SIZE / 2.0)

    self._rotation = (self._rotation + DEGREES_PER_SECOND * rl.get_frame_time()) % 360.0

    if self._spinner_texture:
      rl.draw_texture_pro(self._spinner_texture, rl.Rectangle(0, 0, TEXTURE_SIZE, TEXTURE_SIZE),
                          rl.Rectangle(center.x, center.y, TEXTURE_SIZE, TEXTURE_SIZE),
                          spinner_origin, self._rotation, rl.WHITE)
    if self._comma_texture:
      rl.draw_texture_v(self._comma_texture, comma_position, rl.WHITE)

    if self._progress is not None:
      bar = rl.Rectangle(center.x - PROGRESS_BAR_WIDTH / 2.0, progress_y, PROGRESS_BAR_WIDTH, PROGRESS_BAR_HEIGHT)
      rl.draw_rectangle_rounded(bar, 1, 10, DARKGRAY)
      bar.width *= self._progress / 100.0
      rl.draw_rectangle_rounded(bar, 1, 10, rl.WHITE)

    if self._status_text:
      text_width = min(rect.width - MARGIN_H, MAX_CONTENT_WIDTH)
      # BluePilot: this is a scrolling live-output line, not a headline — use the smaller,
      # already-proven-safe error-log size (FONT_SIZE/LINE_HEIGHT above is reserved for the
      # short, fixed "Build Failed" title). Line length is bounded by _truncate() in set_text(),
      # so this rarely needs the scale-down fallback below, but it stays as a second safety net.
      font_size, line_height = ERROR_FONT_SIZE, ERROR_LINE_HEIGHT
      wrapped_lines = wrap_text(self._status_text, font_size, int(text_width))
      required_height = len(wrapped_lines) * line_height
      if required_height > text_area_height and required_height > 0:
        scale = text_area_height / required_height
        font_size = max(12, int(font_size * scale))
        line_height = max(14, int(line_height * scale))
        wrapped_lines = wrap_text(self._status_text, font_size, int(text_width))

      for i, line in enumerate(wrapped_lines):
        text_size = measure_text_cached(gui_app.font(), line, font_size)
        rl.draw_text_ex(gui_app.font(), line, rl.Vector2(center.x - text_size.x / 2, text_area_start + i * line_height),
                        font_size, 0.0, rl.WHITE)


def _read_stdin() -> list[str]:
  """Non-blocking read of available lines from stdin."""
  lines: list[str] = []
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
    for text in _read_stdin():
      bp_spinner.set_text(text)
    bp_spinner.render(rl.Rectangle(0, 0, gui_app.width, gui_app.height))


if __name__ == "__main__":
  main()
