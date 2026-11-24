#!/usr/bin/env python3
# BluePilot TRON test branch: verbose boot screen. Tails the launch log (launch_chffrplus.sh tees all
# of its output into it) so a URL install that stalls before the build spinner shows *why* instead of
# sitting on the comma logo. Deliberately standalone: only pyray + stdlib, no openpilot imports, so it
# still works when the checkout itself is what's broken.
import os
import signal
import sys
import time

import pyray as rl

LOG = sys.argv[1] if len(sys.argv) > 1 else "/data/bp_boot.log"
FONT_SIZE = 34
LINE_H = 40
MARGIN = 30

running = True


def _stop(*_):
  global running
  running = False


signal.signal(signal.SIGTERM, _stop)
signal.signal(signal.SIGINT, _stop)


def read_tail(path: str, max_bytes: int = 64 * 1024) -> list[str]:
  try:
    with open(path, "rb") as f:
      f.seek(0, os.SEEK_END)
      size = f.tell()
      f.seek(max(0, size - max_bytes))
      data = f.read().decode("utf-8", "replace")
  except OSError as e:
    return [f"(waiting for {path}: {e})"]
  return [l.replace("\t", "  ").replace("\r", "") for l in data.split("\n")]


def wrap(lines: list[str], cols: int) -> list[str]:
  out = []
  for l in lines:
    while len(l) > cols:
      out.append(l[:cols])
      l = "    " + l[cols:]
    out.append(l)
  return out


def main():
  rl.set_trace_log_level(rl.TraceLogLevel.LOG_ERROR)
  rl.init_window(0, 0, "bp bootlog")
  rl.set_target_fps(5)
  w, h = rl.get_screen_width(), rl.get_screen_height()
  if w <= 0 or h <= 0:
    w, h = 2160, 1080
  char_w = max(1, rl.measure_text("M", FONT_SIZE))
  cols = max(20, (w - 2 * MARGIN) // char_w)
  rows = max(1, (h - 2 * MARGIN - LINE_H) // LINE_H)
  start = time.monotonic()

  while running and not rl.window_should_close():
    lines = wrap(read_tail(LOG), cols)[-rows:]
    rl.begin_drawing()
    rl.clear_background(rl.BLACK)
    rl.draw_text(f"BluePilot boot log  ({int(time.monotonic() - start)}s)  {LOG}", MARGIN, MARGIN // 2, FONT_SIZE,
                 rl.Color(70, 140, 255, 255))
    y = MARGIN // 2 + LINE_H
    for l in lines:
      color = rl.Color(255, 110, 110, 255) if any(k in l for k in ("Error", "error", "Traceback", "FAIL", "fatal")) else rl.Color(220, 220, 220, 255)
      rl.draw_text(l, MARGIN, y, FONT_SIZE, color)
      y += LINE_H
    rl.end_drawing()

  rl.close_window()


if __name__ == "__main__":
  main()
