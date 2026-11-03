"""BluePilot soundd extension for optional custom alert sounds."""

import hashlib
import os
import socket
import tempfile
import wave

import numpy as np

from opendbc.car.structs import car
from openpilot.common.basedir import BASEDIR
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.bp.lib.custom_sound import (
  CUSTOM_SOUNDS_ENABLED_PARAM,
  CustomSoundSelection,
  get_custom_sound_selection,
)
from openpilot.selfdrive.ui.bp.lib import theme_pack
from openpilot.selfdrive.ui.soundd import MAX_VOLUME, SAMPLE_BUFFER, SAMPLE_RATE, Soundd, sound_list


AudibleAlert = car.CarControl.HUDControl.AudibleAlert
CUSTOM_SOUND_DIR = os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "sounds", "bluepilot")
SOUND_PACK_FILES = {
  CustomSoundSelection.COMMA_4: {
    AudibleAlert.engage: os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "sounds", "engage.wav"),
    AudibleAlert.disengage: os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "sounds", "disengage.wav"),
  },
  CustomSoundSelection.COMMA_3X: {
    AudibleAlert.engage: os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "sounds", "engage_tizi.wav"),
    AudibleAlert.disengage: os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "sounds", "disengage_tizi.wav"),
  },
  CustomSoundSelection.TESLA: {
    AudibleAlert.engage: os.path.join(CUSTOM_SOUND_DIR, "tesla_engage.wav"),
    AudibleAlert.disengage: os.path.join(CUSTOM_SOUND_DIR, "tesla_disengage.wav"),
    AudibleAlert.refuse: os.path.join(CUSTOM_SOUND_DIR, "tesla_warning.wav"),
  },
}
SOUND_OUTPUT_DEVICE_ENV = "BP_SOUNDD_OUTPUT_DEVICE"
SOUND_TEST_CONTROL_ENV = "BP_SOUNDD_TEST_CONTROL"
TEST_ALERTS = {
  b"engage": AudibleAlert.engage,
  b"disengage": AudibleAlert.disengage,
  b"refuse": AudibleAlert.refuse,
}


def _get_output_device() -> int | str | None:
  """Return an optional PortAudio device override for desktop simulation."""
  value = os.getenv(SOUND_OUTPUT_DEVICE_ENV, "").strip()
  if not value:
    return None
  try:
    return int(value)
  except ValueError:
    return value


def get_test_control_socket_path() -> str:
  """Return a short per-prefix path for the optional local test socket."""
  prefix = os.getenv("OPENPILOT_PREFIX", "default")
  prefix_hash = hashlib.sha256(prefix.encode()).hexdigest()[:12]
  return os.path.join(tempfile.gettempdir(), f"bp_soundd_{prefix_hash}.sock")


def _load_mono_sound(path: str) -> np.ndarray:
  """Load the exact PCM format expected by soundd."""
  with wave.open(path, "rb") as wavefile:
    if wavefile.getnchannels() != 1:
      raise ValueError(f"custom sound must be mono: {path}")
    if wavefile.getsampwidth() != 2:
      raise ValueError(f"custom sound must use 16-bit PCM: {path}")
    if wavefile.getframerate() != SAMPLE_RATE:
      raise ValueError(f"custom sound must be {SAMPLE_RATE} Hz: {path}")

    length = wavefile.getnframes()
    if length <= 0:
      raise ValueError(f"custom sound is empty: {path}")
    return np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16 / 2)


def _requested_sound_selection(params) -> CustomSoundSelection | None:
  """Return the selected override, or stock device behavior when disabled."""
  try:
    if params.get_bool(CUSTOM_SOUNDS_ENABLED_PARAM):
      return get_custom_sound_selection(params)
  except Exception:
    pass
  return None


class SounddBP(Soundd):
  """Use BluePilot sounds without changing upstream soundd behavior."""

  def __init__(self) -> None:
    self._test_control_socket: socket.socket | None = None
    self._test_control_path: str | None = None
    self._test_alert_active = False
    self._test_report_audio = False
    super().__init__()
    if os.getenv(SOUND_TEST_CONTROL_ENV) == "1":
      self._open_test_control()

  def load_sounds(self) -> None:
    # Load every upstream sound first. Any custom-sound failure therefore leaves
    # soundd fully functional with the correct device-default sounds.
    super().load_sounds()
    self._load_theme_pack_sounds()
    selection = _requested_sound_selection(self.params)
    if selection is None:
      return

    try:
      custom_sounds = {
        alert: _load_mono_sound(path)
        for alert, path in SOUND_PACK_FILES[selection].items()
      }
    except Exception:
      cloudlog.exception("BluePilot: failed to load selected sound pack; using device-default sounds")
      return

    self.loaded_sounds.update(custom_sounds)
    cloudlog.info(f"BluePilot: loaded {selection.name} sound pack")

  def get_stream(self, sd):
    """Allow local simulations to select an output without affecting devices."""
    output_device = _get_output_device()
    if output_device is None:
      return super().get_stream(sd)

    sd._terminate()
    sd._initialize()
    cloudlog.info(f"BluePilot: using sound output device {output_device!r}")
    return sd.OutputStream(
      device=output_device,
      channels=1,
      samplerate=SAMPLE_RATE,
      callback=self.callback,
      blocksize=SAMPLE_BUFFER,
    )

  def _open_test_control(self) -> None:
    """Open an opt-in local chime-control socket without touching car state."""
    path = get_test_control_socket_path()
    try:
      os.unlink(path)
    except FileNotFoundError:
      pass

    test_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
      test_socket.bind(path)
    except Exception:
      test_socket.close()
      raise
    test_socket.setblocking(False)
    self._test_control_socket = test_socket
    self._test_control_path = path
    cloudlog.info(f"BluePilot: local sound test control listening at {path}")

  def _poll_test_control(self) -> int | None:
    """Return the newest locally requested test chime, if any."""
    if self._test_control_socket is None:
      return None

    requested_alert = None
    while True:
      try:
        command = self._test_control_socket.recv(64).strip().lower()
      except BlockingIOError:
        break
      if command in TEST_ALERTS:
        requested_alert = TEST_ALERTS[command]
        print(f"BluePilot sound test: playing {command.decode()} chime", flush=True)
      else:
        cloudlog.warning(f"BluePilot: ignored unknown local sound test command {command!r}")
    return requested_alert

  def _load_theme_pack_sounds(self) -> None:
    # Seasonal theme packs: a pack wav replaces the stock sound of the same file name. A malformed pack wav
    # (stereo, 44.1 kHz, mp3 in disguise) keeps the stock sound.
    pack = theme_pack.get_active_pack(force=True)
    self._theme_pack_name = pack.name if pack else ""
    if pack is None:
      return
    for sound, (filename, _play_count, _volume) in sound_list.items():
      path = pack.sound_path(filename)
      if not path:
        continue
      try:
        with wave.open(path, 'r') as wavefile:
          assert wavefile.getnchannels() == 1
          assert wavefile.getsampwidth() == 2
          assert wavefile.getframerate() == SAMPLE_RATE
          length = wavefile.getnframes()
          self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)
      except (OSError, AssertionError, wave.Error) as e:
        cloudlog.warning(f"soundd: theme pack sound {path} unusable ({e}), using stock {filename}")

  def get_audible_alert(self, sm) -> None:
    # hot-reload sounds when the theme pack changes (only while no alert plays)
    if self.current_alert == AudibleAlert.none:
      pack = theme_pack.get_active_pack()
      if (pack.name if pack else "") != self._theme_pack_name:
        self.load_sounds()

    test_alert = self._poll_test_control()
    if test_alert is not None:
      # The manual simulator command is sound-only: it deliberately leaves the
      # replayed selfdriveState and SunnyPilot engagement state untouched.
      self.update_alert(test_alert)
      self._test_alert_active = True
      self._test_report_audio = True

    if self._test_alert_active:
      # Hold the preview at full volume and prevent replayed alerts from
      # replacing it before its single pass through the audio callback ends.
      self.current_volume = MAX_VOLUME
      if self.current_sound_frame >= len(self.loaded_sounds[self.current_alert]):
        self._test_alert_active = False
        self.current_alert = AudibleAlert.none
        self.current_sound_frame = 0
      return

    super().get_audible_alert(sm)

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    super().callback(data_out, frames, time, status)
    if self._test_alert_active and self._test_report_audio:
      peak = float(np.max(np.abs(data_out[:frames, 0])))
      if peak > 0:
        print(f"BluePilot sound test: sent audio to CoreAudio (peak={peak:.3f})", flush=True)
        self._test_report_audio = False

  def close(self) -> None:
    if self._test_control_socket is not None:
      self._test_control_socket.close()
      self._test_control_socket = None
    if self._test_control_path is not None:
      try:
        os.unlink(self._test_control_path)
      except FileNotFoundError:
        pass
      self._test_control_path = None


def main() -> None:
  soundd = SounddBP()
  try:
    soundd.soundd_thread()
  finally:
    soundd.close()


if __name__ == "__main__":
  main()
