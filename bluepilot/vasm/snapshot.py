#!/usr/bin/env python3
from __future__ import annotations

from io import BytesIO
import threading
import time
from typing import Any


DEFAULT_CAPTURE_TIMEOUT = 15.0
DEFAULT_WARMUP_SECONDS = 1.5

_capture_lock = threading.Lock()


class SnapshotUnavailable(RuntimeError):
  """Raised when a parked driver-camera frame cannot be captured safely."""


def _put_bool(params: Any, key: str, value: bool) -> None:
  """Support both the native Params API and BluePilot's file fallback."""
  try:
    params.put_bool(key, value, block=True)
  except TypeError:
    params.put_bool(key, value)


def _yuv_to_rgb(y: Any, u: Any, v: Any) -> Any:
  # Keep optional/native dependencies out of module import and manager startup.
  import numpy as np

  ul = np.repeat(np.repeat(u, 2).reshape(u.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
  vl = np.repeat(np.repeat(v, 2).reshape(v.shape[0], y.shape[1]), 2, axis=0).reshape(y.shape)
  yuv = np.dstack((y, ul, vl)).astype(np.int16)
  yuv[:, :, 1:] -= 128
  matrix = np.array([
    [1.00000,  1.00000, 1.00000],
    [0.00000, -0.39465, 2.03211],
    [1.13983, -0.58060, 0.00000],
  ])
  return np.dot(yuv, matrix).clip(0, 255).astype(np.uint8)


def _vision_buffer_to_jpeg(buffer: Any) -> bytes:
  """Convert camerad's NV12 VisionBuf into a browser-ready JPEG."""
  try:
    import numpy as np
    from PIL import Image
  except (ImportError, OSError) as exc:
    raise SnapshotUnavailable("Driver-camera image support is unavailable on this build.") from exc

  if buffer is None or buffer.width <= 0 or buffer.height <= 0 or buffer.stride <= 0:
    raise SnapshotUnavailable("The driver camera returned an invalid frame.")

  uv_height = ((buffer.height // 2) + 15) // 16 * 16
  uv_plane_size = buffer.stride * uv_height
  y_data = buffer.data[:buffer.uv_offset]
  uv_data = buffer.data[buffer.uv_offset:buffer.uv_offset + uv_plane_size]

  try:
    y = np.array(y_data, dtype=np.uint8).reshape((-1, buffer.stride))[:buffer.height, :buffer.width]
    u = np.array(uv_data[::2], dtype=np.uint8).reshape((-1, buffer.stride // 2))[:buffer.height // 2, :buffer.width // 2]
    v = np.array(uv_data[1::2], dtype=np.uint8).reshape((-1, buffer.stride // 2))[:buffer.height // 2, :buffer.width // 2]
  except (TypeError, ValueError) as exc:
    raise SnapshotUnavailable("The driver camera returned an unreadable frame.") from exc

  output = BytesIO()
  try:
    Image.fromarray(_yuv_to_rgb(y, u, v)).save(output, "JPEG", quality=92)
  except (OSError, TypeError, ValueError) as exc:
    raise SnapshotUnavailable("The driver-camera frame could not be converted to an image.") from exc
  return output.getvalue()


def capture_parked_driver_snapshot(params: Any, timeout: float = DEFAULT_CAPTURE_TIMEOUT,
                                   warmup_seconds: float = DEFAULT_WARMUP_SECONDS) -> bytes:
  """Start parked driver-view mode temporarily and return a fresh driver-camera JPEG.

  The bounded polling loop deliberately avoids VisionIPC's blocking connect path so a
  camera or process failure cannot wedge a BluePilot Portal request indefinitely.
  """
  if timeout <= 0:
    raise ValueError("timeout must be positive")
  if not _capture_lock.acquire(blocking=False):
    raise SnapshotUnavailable("A driver-camera snapshot is already being captured. Please wait a moment.")

  enabled_by_us = False
  try:
    try:
      was_enabled = params.get_bool("IsDriverViewEnabled")
    except Exception as exc:
      raise SnapshotUnavailable("The parked driver-camera state could not be read.") from exc
    if not was_enabled:
      try:
        _put_bool(params, "IsDriverViewEnabled", True)
        enabled_by_us = True
      except Exception as exc:
        raise SnapshotUnavailable("The parked driver camera could not be started.") from exc

    try:
      from msgq.visionipc import VisionIpcClient, VisionStreamType
    except (ImportError, OSError) as exc:
      raise SnapshotUnavailable("Driver-camera capture support is unavailable on this build.") from exc

    stream_type = VisionStreamType.VISION_STREAM_DRIVER
    deadline = time.monotonic() + timeout
    client = None
    first_frame_at = None
    first_frame_jpeg = None

    while time.monotonic() < deadline:
      if client is None:
        try:
          available = VisionIpcClient.available_streams("camerad", block=False)
        except Exception:
          available = []
        if stream_type not in available:
          time.sleep(0.1)
          continue
        try:
          client = VisionIpcClient("camerad", stream_type, True)
        except Exception:
          client = None
          time.sleep(0.1)
          continue

      try:
        if not client.is_connected() and not client.connect(False):
          time.sleep(0.1)
          continue
        buffer = client.recv(timeout_ms=250)
      except Exception:
        client = None
        time.sleep(0.1)
        continue

      if buffer is None:
        continue

      now = time.monotonic()
      if first_frame_at is None:
        first_frame_at = now
        # Preserve a usable frame in case exposure warmup consumes the deadline.
        first_frame_jpeg = _vision_buffer_to_jpeg(buffer)
      elif now - first_frame_at >= max(0.0, warmup_seconds):
        return _vision_buffer_to_jpeg(buffer)

    if first_frame_jpeg is not None:
      return first_frame_jpeg
    raise SnapshotUnavailable(
      "The parked driver camera did not become ready. Wait a moment, then tap New snapshot."
    )
  finally:
    if enabled_by_us:
      try:
        _put_bool(params, "IsDriverViewEnabled", False)
      except Exception:
        # The manager clears this non-persistent key on restart; a cleanup failure
        # must not discard a frame that was captured successfully.
        pass
    _capture_lock.release()
