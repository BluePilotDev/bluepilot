import builtins
from io import BytesIO
import importlib
import sys

import pytest


def test_snapshot_import_does_not_load_camera_or_image_dependencies(monkeypatch):
  """Portal/manager startup must not require camera IPC, NumPy, or Pillow."""
  real_import = builtins.__import__

  def guarded_import(name, global_vars=None, local_vars=None, fromlist=(), level=0):
    if name.split(".", 1)[0] in {"msgq", "numpy", "PIL"}:
      raise ModuleNotFoundError(name)
    return real_import(name, global_vars, local_vars, fromlist, level)

  sys.modules.pop("openpilot.bluepilot.vasm.snapshot", None)
  monkeypatch.setattr(builtins, "__import__", guarded_import)
  importlib.import_module("openpilot.bluepilot.vasm.snapshot")


def test_invalid_timeout_does_not_change_driver_view():
  from openpilot.bluepilot.vasm.snapshot import capture_parked_driver_snapshot

  class FakeParams:
    def __init__(self):
      self.enabled = False

    def get_bool(self, _key):
      return self.enabled

    def put_bool(self, _key, value, block=False):
      self.enabled = value

  params = FakeParams()
  with pytest.raises(ValueError):
    capture_parked_driver_snapshot(params, timeout=0)
  assert not params.enabled


def test_nv12_frame_encodes_as_jpeg():
  from PIL import Image
  import numpy as np

  from openpilot.bluepilot.vasm.snapshot import _vision_buffer_to_jpeg

  class FakeBuffer:
    width = 4
    height = 4
    stride = 4
    uv_offset = 16
    # Four Y rows plus an aligned UV plane. Neutral chroma yields gray pixels.
    data = memoryview(bytes([128] * uv_offset + [128] * (stride * 16)))

  jpeg = _vision_buffer_to_jpeg(FakeBuffer())
  assert jpeg.startswith(b"\xff\xd8")
  with Image.open(BytesIO(jpeg)) as image:
    assert image.size == (4, 4)
    pixel = np.array(image)[0, 0]
    assert np.max(np.abs(pixel.astype(int) - 128)) <= 2


def test_capture_temporarily_enables_and_restores_driver_view(monkeypatch):
  from msgq.visionipc import VisionStreamType

  from openpilot.bluepilot.vasm import snapshot

  class FakeParams:
    def __init__(self):
      self.enabled = False
      self.writes = []

    def get_bool(self, _key):
      return self.enabled

    def put_bool(self, _key, value, block=False):
      self.enabled = value
      self.writes.append((value, block))

  class FakeBuffer:
    width = 4
    height = 4
    stride = 4
    uv_offset = 16
    data = memoryview(bytes([128] * uv_offset + [128] * (stride * 16)))

  class FakeClient:
    @staticmethod
    def available_streams(_name, block=False):
      return {VisionStreamType.VISION_STREAM_DRIVER}

    def __init__(self, _name, _stream, _conflate):
      self.connected = False

    def is_connected(self):
      return self.connected

    def connect(self, _blocking):
      self.connected = True
      return True

    def recv(self, timeout_ms=100):
      return FakeBuffer()

  import msgq.visionipc
  monkeypatch.setattr(msgq.visionipc, "VisionIpcClient", FakeClient)
  params = FakeParams()

  jpeg = snapshot.capture_parked_driver_snapshot(params, timeout=1.0, warmup_seconds=0.0)

  assert jpeg.startswith(b"\xff\xd8")
  assert params.writes == [(True, True), (False, True)]
  assert not params.enabled
