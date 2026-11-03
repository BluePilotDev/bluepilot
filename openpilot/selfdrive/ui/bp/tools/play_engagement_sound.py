#!/usr/bin/env python3
"""Request an engage, disengage, or refusal chime without changing driving state."""

import argparse
import os
import socket
import sys

from opendbc.car.structs import car
from openpilot.selfdrive.ui.bp.soundd_bp import (
  SOUND_TEST_CONTROL_ENV,
  SounddBP,
  _get_output_device,
  get_test_control_socket_path,
)
from openpilot.selfdrive.ui.soundd import SAMPLE_RATE


ALERTS = ("engage", "disengage", "refuse")
AUDIBLE_ALERTS = {
  "engage": car.CarControl.HUDControl.AudibleAlert.engage,
  "disengage": car.CarControl.HUDControl.AudibleAlert.disengage,
  "refuse": car.CarControl.HUDControl.AudibleAlert.refuse,
}


def send_alert(test_socket: socket.socket, alert: str) -> bool:
  try:
    test_socket.sendto(alert.encode(), get_test_control_socket_path())
  except OSError:
    return False
  print(f"Requested the {alert} chime (driving state unchanged).")
  return True


def _direct_output_device(sd) -> int | str | None:
  configured_device = _get_output_device()
  if configured_device is not None:
    return configured_device

  # On macOS, PortAudio's default can be a dock/monitor input-output pair even
  # when the user expects the laptop speakers. Prefer the built-in speakers for
  # this local fallback and print the actual selection below.
  if sys.platform == "darwin":
    for index, device in enumerate(sd.query_devices()):
      name = str(device["name"])
      if device["max_output_channels"] > 0 and "MacBook" in name and "Speakers" in name:
        return index
  return None


def play_direct(alert: str) -> None:
  """Preview the configured sound if the full soundd simulator is not running."""
  import sounddevice as sd

  # Instantiating SounddBP loads the same stock/custom buffers as production,
  # but the direct fallback must not create its own test-control socket.
  test_control = os.environ.pop(SOUND_TEST_CONTROL_ENV, None)
  try:
    soundd = SounddBP()
  finally:
    if test_control is not None:
      os.environ[SOUND_TEST_CONTROL_ENV] = test_control

  try:
    output_device = _direct_output_device(sd)
    device_info = sd.query_devices(output_device, "output")
    print(f"soundd is not running; playing {alert} directly on {device_info['name']}.")
    sd.play(soundd.loaded_sounds[AUDIBLE_ALERTS[alert]], SAMPLE_RATE, device=output_device, blocking=True)
  finally:
    soundd.close()


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("alert", choices=ALERTS, nargs="?", help="Play once and exit; omit for interactive mode")
  args = parser.parse_args()

  with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as test_socket:
    if args.alert is not None:
      if not send_alert(test_socket, args.alert):
        play_direct(args.alert)
      return

    print("Enter 'engage', 'disengage', or 'refuse' to play only that chime; 'q' exits.")
    while True:
      try:
        command = input("sound> ").strip().lower()
      except (EOFError, KeyboardInterrupt):
        print()
        return
      if command in ("q", "quit", "exit"):
        return
      if command in ALERTS:
        if not send_alert(test_socket, command):
          play_direct(command)
      elif command:
        print("Expected 'engage', 'disengage', 'refuse', or 'q'.")


if __name__ == "__main__":
  main()
