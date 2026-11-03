import os
import socket
import time

import numpy as np

from opendbc.car.structs import car
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.events import EVENTS, EventName
from openpilot.sunnypilot.selfdrive.ui import quiet_mode
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET
from openpilot.selfdrive.ui.bp import soundd_bp
from openpilot.selfdrive.ui.bp.lib.custom_sound import (
  CUSTOM_SOUNDS_ENABLED_PARAM,
  CUSTOM_SOUNDS_SELECTION_PARAM,
  CustomSoundSelection,
)
from openpilot.selfdrive.ui.bp.soundd_bp import (
  SOUND_PACK_FILES,
  SounddBP,
  _get_output_device,
  _load_mono_sound,
  _requested_sound_selection,
  get_test_control_socket_path,
)


AudibleAlert = car.CarControl.HUDControl.AudibleAlert


def test_output_device_override(monkeypatch):
  monkeypatch.delenv(soundd_bp.SOUND_OUTPUT_DEVICE_ENV, raising=False)
  assert _get_output_device() is None
  monkeypatch.setenv(soundd_bp.SOUND_OUTPUT_DEVICE_ENV, "5")
  assert _get_output_device() == 5
  monkeypatch.setenv(soundd_bp.SOUND_OUTPUT_DEVICE_ENV, "MacBook Pro Speakers")
  assert _get_output_device() == "MacBook Pro Speakers"


def test_sound_only_control_channel(monkeypatch, tmp_path):
  params = Params(str(tmp_path / "params"))
  monkeypatch.setattr(quiet_mode, "Params", lambda: params)
  monkeypatch.setenv("OPENPILOT_PREFIX", f"bp_sound_test_{tmp_path.name}")
  monkeypatch.setenv(soundd_bp.SOUND_TEST_CONTROL_ENV, "1")

  daemon = SounddBP()
  path = get_test_control_socket_path()
  try:
    with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as test_socket:
      test_socket.sendto(b"engage", path)

    class ReplayState:
      updated = {"selfdriveState": False}
      recv_time = {"selfdriveState": time.monotonic()}

    daemon.get_audible_alert(ReplayState())
    assert daemon.current_alert == AudibleAlert.engage
    assert daemon.current_volume == soundd_bp.MAX_VOLUME
    assert daemon._test_alert_active

    daemon.current_volume = 0.1
    daemon.current_sound_frame = len(daemon.loaded_sounds[AudibleAlert.engage]) - 1
    daemon.get_audible_alert(ReplayState())
    assert daemon.current_alert == AudibleAlert.engage
    assert daemon.current_volume == soundd_bp.MAX_VOLUME

    daemon.current_sound_frame += 1
    daemon.get_audible_alert(ReplayState())
    assert daemon.current_alert == AudibleAlert.none
    assert not daemon._test_alert_active
  finally:
    daemon.close()
  assert not os.path.exists(path)


class FakeParams:
  def __init__(self, enabled: bool, selection: int):
    self.enabled = enabled
    self.selection = selection

  def get_bool(self, key: str) -> bool:
    assert key == CUSTOM_SOUNDS_ENABLED_PARAM
    return self.enabled

  def get(self, key: str):
    assert key == CUSTOM_SOUNDS_SELECTION_PARAM
    return self.selection


def test_custom_sound_param_gate():
  assert _requested_sound_selection(FakeParams(False, CustomSoundSelection.TESLA)) is None
  assert _requested_sound_selection(FakeParams(True, CustomSoundSelection.COMMA_4)) == CustomSoundSelection.COMMA_4
  assert _requested_sound_selection(FakeParams(True, CustomSoundSelection.COMMA_3X)) == CustomSoundSelection.COMMA_3X
  assert _requested_sound_selection(FakeParams(True, CustomSoundSelection.TESLA)) == CustomSoundSelection.TESLA
  assert _requested_sound_selection(FakeParams(True, 99)) == CustomSoundSelection.COMMA_4


def test_custom_sound_assets_are_soundd_compatible():
  for sound_files in SOUND_PACK_FILES.values():
    for path in sound_files.values():
      samples = _load_mono_sound(path)
      assert samples.dtype == np.float32
      assert samples.size > 0
      assert np.any(samples)


def test_tesla_sound_assets_have_device_speaker_frequency_content():
  # The original Tesla engagement captures are almost entirely below 500 Hz
  # and can be inaudible on the device speakers. Keep most of every prepared
  # Tesla asset above that range.
  for path in SOUND_PACK_FILES[CustomSoundSelection.TESLA].values():
    samples = _load_mono_sound(path)
    spectrum = np.abs(np.fft.rfft(samples * np.hanning(samples.size))) ** 2
    frequencies = np.fft.rfftfreq(samples.size, 1 / soundd_bp.SAMPLE_RATE)
    total_energy = float(np.sum(spectrum))
    device_audible_energy = float(np.sum(spectrum[frequencies >= 500]))

    assert total_energy > 0
    assert device_audible_energy / total_energy > 0.75


def test_parking_brake_and_gear_no_entry_events_use_refusal_alert():
  for event in (EventName.parkBrake, EventName.wrongGear, EventName.reverseGear):
    assert EVENTS[event][ET.NO_ENTRY].audible_alert == AudibleAlert.refuse


def _custom_params(tmp_path, selection: CustomSoundSelection = CustomSoundSelection.TESLA) -> Params:
  params = Params(str(tmp_path))
  params.put_bool(CUSTOM_SOUNDS_ENABLED_PARAM, True, block=True)
  params.put(CUSTOM_SOUNDS_SELECTION_PARAM, int(selection), block=True)
  return params


def test_soundd_bp_replaces_tesla_sound_pack(monkeypatch, tmp_path):
  params = _custom_params(tmp_path)
  monkeypatch.setattr(quiet_mode, "Params", lambda: params)

  daemon = SounddBP()
  for alert, path in SOUND_PACK_FILES[CustomSoundSelection.TESLA].items():
    expected = _load_mono_sound(path)
    np.testing.assert_array_equal(daemon.loaded_sounds[alert], expected)


def test_soundd_bp_loads_each_comma_sound_pack(monkeypatch, tmp_path):
  stock_refuse = _load_mono_sound(os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "sounds", "refuse.wav"))
  for selection in (CustomSoundSelection.COMMA_4, CustomSoundSelection.COMMA_3X):
    params = _custom_params(tmp_path / selection.name, selection)
    monkeypatch.setattr(quiet_mode, "Params", lambda params=params: params)
    daemon = SounddBP()
    for alert, path in SOUND_PACK_FILES[selection].items():
      np.testing.assert_array_equal(daemon.loaded_sounds[alert], _load_mono_sound(path))
    np.testing.assert_array_equal(daemon.loaded_sounds[AudibleAlert.refuse], stock_refuse)


def test_soundd_bp_falls_back_to_stock_on_asset_error(monkeypatch, tmp_path):
  params = _custom_params(tmp_path)
  monkeypatch.setattr(quiet_mode, "Params", lambda: params)
  missing_files = {
    alert: str(tmp_path / "missing" / os.path.basename(path))
    for alert, path in SOUND_PACK_FILES[CustomSoundSelection.TESLA].items()
  }
  monkeypatch.setitem(soundd_bp.SOUND_PACK_FILES, CustomSoundSelection.TESLA, missing_files)

  daemon = SounddBP()
  stock_files = {
    AudibleAlert.engage: "engage.wav",
    AudibleAlert.disengage: "disengage.wav",
    AudibleAlert.refuse: "refuse.wav",
  }
  for alert, filename in stock_files.items():
    stock_sound = _load_mono_sound(os.path.join(BASEDIR, "openpilot", "selfdrive", "assets", "sounds", filename))
    np.testing.assert_array_equal(daemon.loaded_sounds[alert], stock_sound)
