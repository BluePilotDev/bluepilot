"""One-second snapshot crossfade for automatic camera changes on C3x and C4."""
import math
from time import monotonic


class CameraFade:
  DURATION = 1.0

  def __init__(self):
    self.reset()

  def reset(self):
    self.started = None
    self.committed = False
    self.frame = None
    self.frame_time = 0.0
    self.target = None
    self.captured = False

  def alpha(self, now):
    if self.started is None:
      return 0.0
    progress = max(0.0, min(1.0, (now - self.started) / self.DURATION))
    return (1 + math.cos(math.pi * progress)) / 2


class AutomaticCameraFadeMixin:
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._camera_fade = CameraFade()
    self._camera_snapshot = None
    self._camera_snapshot_size = None

  def _clear_camera_transition(self):
    import pyray as rl

    if self._camera_snapshot is not None:
      rl.rl_draw_render_batch_active()
      previous = rl.rl_get_active_framebuffer()
      rl.unload_render_texture(self._camera_snapshot)
      # Raylib texture cleanup may unbind the caller's offscreen framebuffer.
      rl.rl_bind_framebuffer(0x8CA8, previous)
      rl.rl_bind_framebuffer(0x8CA9, previous)
    self._camera_snapshot = None
    self._camera_snapshot_size = None
    self._camera_fade.reset()

  def _handle_switch(self):
    fade = self._camera_fade
    if not self._bp_camera_angle_auto or self.frame is None:
      self._clear_camera_transition()
      return super()._handle_switch()
    if not self._switching or self._target_client is None:
      return
    if fade.target != self._target_stream_type:
      self._clear_camera_transition()
      fade.target = self._target_stream_type
    client = self._target_client
    if not client.is_connected() and (not client.connect(False) or not client.num_buffers):
      self._clear_camera_transition()
      return
    now = monotonic()
    frame = client.recv(timeout_ms=0)
    if frame is not None:
      fade.frame = frame
      fade.frame_time = now
    if fade.frame is None:
      return
    if now - fade.frame_time > 0.25:
      self._clear_camera_transition()
      return
    # Capture the outgoing camera + geometry at the end of its render pass.
    # Commit next frame, so the snapshot never contains the incoming projection.
    if fade.captured:
      self.frame = fade.frame
      self._complete_switch()
      fade.frame = None
      fade.committed = True
      fade.started = now

  def _capture_camera_scene(self, rect):
    import pyray as rl
    from openpilot.system.ui.lib.application import gui_app

    width, height = rl.get_render_width(), rl.get_render_height()
    rl.rl_draw_render_batch_active()
    previous = rl.rl_get_active_framebuffer()
    snapshot = rl.load_render_texture(width, height)
    if not snapshot.id or not snapshot.texture.id:
      if snapshot.id:
        rl.unload_render_texture(snapshot)
      rl.rl_bind_framebuffer(0x8CA8, previous)
      rl.rl_bind_framebuffer(0x8CA9, previous)
      return False
    # Blit the current framebuffer directly to a GPU texture. There is no CPU
    # readback, second camera render, or framebuffer copy during normal driving.
    rl.end_scissor_mode()
    try:
      rl.rl_bind_framebuffer(0x8CA8, previous)  # GL_READ_FRAMEBUFFER
      rl.rl_bind_framebuffer(0x8CA9, snapshot.id)  # GL_DRAW_FRAMEBUFFER
      rl.rl_blit_framebuffer(0, 0, width, height, 0, 0, width, height, 0x4000)  # GL_COLOR_BUFFER_BIT
    finally:
      rl.rl_bind_framebuffer(0x8CA8, previous)
      rl.rl_bind_framebuffer(0x8CA9, previous)
      rl.begin_scissor_mode(int(rect.x), int(rect.y), int(rect.width), int(rect.height))
    self._camera_snapshot = snapshot
    self._camera_snapshot_size = (width, height, gui_app.width, gui_app.height)
    return True

  def _draw_camera_transition(self, rect):
    import pyray as rl
    from openpilot.system.ui.lib.application import gui_app

    fade = self._camera_fade
    if not self._bp_camera_angle_auto or (not self._switching and not fade.committed):
      self._clear_camera_transition()
      return
    if not fade.committed:
      if fade.frame is not None and not fade.captured:
        if self._capture_camera_scene(rect):
          fade.captured = True
        else:
          # Optional allocation failure falls back to the native immediate switch.
          self.frame = fade.frame
          self._complete_switch()
          self._clear_camera_transition()
      return
    alpha = fade.alpha(monotonic())
    size = (rl.get_render_width(), rl.get_render_height(), gui_app.width, gui_app.height)
    if alpha <= 0.0 or size != self._camera_snapshot_size:
      self._clear_camera_transition()
      return
    if self._camera_snapshot is not None:
      # Snapshot contains only the scene below HUD/DCam/warnings. Incoming camera
      # and geometry remain live beneath it, with no fade through black.
      width, height, logical_width, logical_height = size
      rl.draw_texture_pro(self._camera_snapshot.texture, rl.Rectangle(0, 0, width, -height),
                          rl.Rectangle(0, 0, logical_width, logical_height), rl.Vector2(0, 0),
                          0, rl.Color(255, 255, 255, round(alpha * 255)))

  def hide_event(self):
    self._clear_camera_transition()
    super().hide_event()
