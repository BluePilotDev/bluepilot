"""One-second automatic-camera fade shared by C3x and C4, without extra textures."""
import math
from time import monotonic


class CameraFade:
  HALF_SECONDS = 0.5

  def __init__(self):
    self.reset()

  def reset(self):
    self.started = None
    self.committed = False
    self.frame = None
    self.frame_time = 0.0
    self.target = None

  def alpha(self, now):
    if self.started is None:
      return 0.0
    elapsed = max(0.0, now - self.started)
    if elapsed >= self.HALF_SECONDS * 2:
      self.reset()
      return 0.0
    phase = elapsed / self.HALF_SECONDS
    progress = phase if phase <= 1 else 2 - phase
    return (1 - math.cos(math.pi * progress)) / 2


class AutomaticCameraFadeMixin:
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._camera_fade = CameraFade()

  def _handle_switch(self):
    fade = self._camera_fade
    if not self._bp_camera_angle_auto or self.frame is None:
      fade.reset()
      return super()._handle_switch()
    if not self._switching or self._target_client is None:
      return
    if fade.target != self._target_stream_type:
      fade.reset()
      fade.target = self._target_stream_type
    client = self._target_client
    if not client.is_connected() and (not client.connect(False) or not client.num_buffers):
      fade.reset()
      return
    now = monotonic()
    frame = client.recv(timeout_ms=0)
    if frame is not None:
      fade.frame = frame
      fade.frame_time = now
      if fade.started is None:
        fade.started = now
    if fade.started is None:
      return
    # A failed destination never leaves the scene black or commits a stale buffer.
    if now - fade.frame_time > 0.25:
      fade.reset()
      return
    if now - fade.started >= fade.HALF_SECONDS:
      self.frame = fade.frame
      self._complete_switch()
      fade.frame = None
      fade.committed = True

  def _draw_camera_transition(self, rect):
    import pyray as rl

    fade = self._camera_fade
    if not self._bp_camera_angle_auto or (not self._switching and not fade.committed):
      fade.reset()
      return
    alpha = fade.alpha(monotonic())
    if alpha > 0:
      # Called after camera + geometry, before DCam, HUD and safety warnings.
      rl.draw_rectangle_rec(rect, rl.Color(0, 0, 0, round(alpha * 255)))

  def hide_event(self):
    self._camera_fade.reset()
    super().hide_event()
