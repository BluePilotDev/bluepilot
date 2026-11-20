"""BluePilot cropped driver-camera view for lane changes and blindspots."""

from time import monotonic

import pyray as rl

from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.cameraview import (
  CameraView as MiciCameraView,
  FRAME_FRAGMENT_SHADER as MICI_FRAME_FRAGMENT_SHADER,
  VERTEX_SHADER as MICI_VERTEX_SHADER,
)
from openpilot.selfdrive.ui.onroad.cameraview import (
  CameraView as TiciCameraView,
  FRAME_FRAGMENT_SHADER as TICI_FRAME_FRAGMENT_SHADER,
  VERTEX_SHADER as TICI_VERTEX_SHADER,
)
from openpilot.selfdrive.ui.bp.onroad.cropped_dcam_geometry import (
  DEFAULT_WINDOW_CENTER_Y,
  compact_panel_region,
  PostTriggerHold,
  Region,
  Side,
  ease_visibility_alpha,
  low_light_enhancement,
  source_crop,
  wedge_canvas_region,
  wedge_local_insets,
)
from openpilot.system.hardware import TICI
from openpilot.system.ui.lib.application import gui_app


FADE_IN_RC = 0.12
FADE_OUT_RC = 0.22
LOW_LIGHT_RC = 0.80
VISIBLE_ALPHA_THRESHOLD = 0.01
def _with_crop_effects(fragment_shader: str) -> str:
  """Add BP-only wedge masking, opacity, and shadow lift to a camera shader."""
  output_declaration = "out vec4 fragColor;"
  assert output_declaration in fragment_shader
  crop_uniforms = f"""{output_declaration}
uniform float cropAlpha;
uniform float cropLowLight;
uniform vec4 cropMaskViewport;
uniform vec2 cropWedgeInsets;
uniform float cropMaskSide;"""
  shader = fragment_shader.replace(
    output_declaration,
    crop_uniforms,
    1,
  )
  body, trailing = shader.rsplit("}", 1)
  effects = """
      float cropLuma = dot(fragColor.rgb, vec3(0.299, 0.587, 0.114));
      float cropShadow = cropLowLight * (1.0 - smoothstep(0.08, 0.55, cropLuma));
      vec3 cropLifted = pow(max(fragColor.rgb, vec3(0.0)), vec3(0.72)) + vec3(0.018);
      vec3 cropColor = mix(fragColor.rgb, cropLifted, 0.72 * cropShadow);
      float cropEnhancedLuma = dot(cropColor, vec3(0.299, 0.587, 0.114));
      cropColor = mix(vec3(cropEnhancedLuma), cropColor, 1.0 + 0.06 * cropLowLight);
      vec2 cropMaskUv = (gl_FragCoord.xy - cropMaskViewport.xy) / cropMaskViewport.zw;
      float cropMaskY = 1.0 - clamp(cropMaskUv.y, 0.0, 1.0);
      float cropMaskEdge = mix(cropWedgeInsets.x, cropWedgeInsets.y, cropMaskY);
      float cropMaskFeather = 2.0 / max(cropMaskViewport.z, 1.0);
      float cropMaskDistance = cropMaskSide > 0.0
        ? cropMaskUv.x - cropMaskEdge
        : (1.0 - cropMaskEdge) - cropMaskUv.x;
      float cropMask = smoothstep(-cropMaskFeather, cropMaskFeather, cropMaskDistance);
      fragColor = vec4(clamp(cropColor, 0.0, 1.0), fragColor.a * cropAlpha * cropMask);
  """
  return f"{body}{effects}}}{trailing}"


def wedge_canvas_rect(content_rect: rl.Rectangle, side: Side,
                      companion_alpha: float) -> rl.Rectangle:
  """Map the full source crop across the visible wedge's bounding rectangle."""
  region = wedge_canvas_region(
    Region(content_rect.x, content_rect.y, content_rect.width, content_rect.height),
    side,
    companion_alpha,
  )
  return rl.Rectangle(region.x, region.y, region.width, region.height)


class _CroppedDcamMixin:
  """Shared renderer layered onto the native TICI/MICI camera implementations."""

  CROP_VERTEX_SHADER: str
  CROP_FRAGMENT_SHADER: str

  def __init__(self):
    super().__init__("camerad", VisionStreamType.VISION_STREAM_DRIVER)
    self._last_frame_time = 0.0
    self._window_center_y = DEFAULT_WINDOW_CENTER_Y
    self._has_window_landmark = False
    self._side_alpha = {
      "left": FirstOrderFilter(0.0, FADE_IN_RC, 1.0 / gui_app.target_fps),
      "right": FirstOrderFilter(0.0, FADE_IN_RC, 1.0 / gui_app.target_fps),
    }
    self._post_trigger_hold = {
      "left": PostTriggerHold(),
      "right": PostTriggerHold(),
    }
    self._low_light_filter = FirstOrderFilter(0.0, LOW_LIGHT_RC, 1.0 / gui_app.target_fps)
    # The native shaders output opaque camera pixels. Replace only this child
    # view's shader with an alpha-aware equivalent so the road view underneath
    # remains visible throughout the transition.
    native_shader = self.shader
    crop_shader = rl.load_shader_from_memory(self.CROP_VERTEX_SHADER, self.CROP_FRAGMENT_SHADER)
    if rl.get_shader_location(crop_shader, "cropAlpha") < 0:
      rl.unload_shader(crop_shader)
      raise RuntimeError("DCam crop shader could not be compiled")
    self.shader = crop_shader
    rl.unload_shader(native_shader)
    self._texture1_loc = rl.get_shader_location(self.shader, "texture1") if not TICI else -1
    if hasattr(self, "_engaged_loc"):
      self._engaged_loc = rl.get_shader_location(self.shader, "engaged")
      self._enhance_driver_loc = rl.get_shader_location(self.shader, "enhance_driver")
    self._crop_alpha_loc = rl.get_shader_location(self.shader, "cropAlpha")
    self._crop_alpha_value = rl.ffi.new("float[1]", [0.0])
    self._crop_low_light_loc = rl.get_shader_location(self.shader, "cropLowLight")
    self._crop_low_light_value = rl.ffi.new("float[1]", [0.0])
    self._crop_mask_viewport_loc = rl.get_shader_location(self.shader, "cropMaskViewport")
    self._crop_mask_viewport_value = rl.ffi.new("float[4]", [0.0, 0.0, 1.0, 1.0])
    self._crop_wedge_insets_loc = rl.get_shader_location(self.shader, "cropWedgeInsets")
    self._crop_wedge_insets_value = rl.ffi.new("float[2]", [0.0, 1.0])
    self._crop_mask_side_loc = rl.get_shader_location(self.shader, "cropMaskSide")
    self._crop_mask_side_value = rl.ffi.new("float[1]", [1.0])

  def is_visible(self) -> bool:
    return any(alpha_filter.x > VISIBLE_ALPHA_THRESHOLD for alpha_filter in self._side_alpha.values())

  def _update_side_alpha(self, side: Side, active: bool) -> float:
    alpha_filter = self._side_alpha[side]
    alpha_filter.update_alpha(FADE_IN_RC if active else FADE_OUT_RC)
    return ease_visibility_alpha(alpha_filter.update(1.0 if active else 0.0))

  def update_frame(self) -> bool:
    """Keep the conflated driver stream warm for immediate signal activation."""
    if self.client is None:
      self.client = VisionIpcClient(self._name, self._stream_type, conflate=True)
    if not self._ensure_connection():
      self.frame = None
      self._reset_transition()
      return False

    buffer = self.client.recv(timeout_ms=0)
    if buffer:
      self._texture_needs_update = True
      self.frame = buffer
      self._last_frame_time = monotonic()
    elif not self.client.is_connected() or monotonic() - self._last_frame_time > 0.5:
      self.frame = None
      self._reset_transition()
    return self.frame is not None

  def _reset_transition(self) -> None:
    for alpha_filter in self._side_alpha.values():
      alpha_filter.x = 0.0
    for hold in self._post_trigger_hold.values():
      hold._input_active = False
      hold._hold_until = 0.0

  def suspend(self) -> None:
    """Release camera buffers while idle without reallocating shaders or callbacks."""
    self._clear_textures()
    self.frame = None
    self.client = None
    self._last_frame_time = 0.0
    self._reset_transition()

  def render_crops(self, content_rect: rl.Rectangle, left_active: bool, right_active: bool,
                   calibration_rpy: tuple[float, float, float], window_center_y: float | None,
                   focal_length: float, light_sensor: float = -1.0) -> None:
    # Keep each side fully targeted for one second after its turn-signal/BLIS
    # trigger drops, then hand off to the existing OEM-like fade-out.
    now = monotonic()
    held_active = {
      "left": self._post_trigger_hold["left"].update(left_active, now),
      "right": self._post_trigger_hold["right"].update(right_active, now),
    }

    # Do not advance the transition before the first usable frame; otherwise a
    # newly connected stream could pop in after the fade has already completed.
    if not self.update_frame() or self.frame is None:
      return

    side_alpha = {
      "left": self._update_side_alpha("left", held_active["left"]),
      "right": self._update_side_alpha("right", held_active["right"]),
    }
    if max(side_alpha.values()) <= VISIBLE_ALPHA_THRESHOLD:
      return

    # Smooth driver-model landmark changes so a marginal face detection cannot
    # make the safety view jump between frames.
    if window_center_y is not None:
      self._window_center_y = 0.95 * self._window_center_y + 0.05 * window_center_y
      self._has_window_landmark = True
    low_light = self._low_light_filter.update(low_light_enhancement(light_sensor))
    for side in ("left", "right"):
      alpha = side_alpha[side]
      if alpha <= VISIBLE_ALPHA_THRESHOLD:
        continue
      companion_side = "right" if side == "left" else "left"
      companion_alpha = side_alpha[companion_side]
      destination, mask_insets = self._crop_layout(content_rect, side, companion_alpha)
      crop = source_crop(self.frame.width, self.frame.height, destination.width, destination.height, side,
                         calibration_rpy, self._window_center_y if self._has_window_landmark else None, focal_length)
      self._draw_crop(content_rect, destination, crop, alpha, low_light, side, mask_insets)

  def _crop_layout(self, content_rect: rl.Rectangle, side: Side, companion_alpha: float):
    destination = wedge_canvas_rect(content_rect, side, companion_alpha)
    insets = wedge_local_insets(
      Region(content_rect.x, content_rect.y, content_rect.width, content_rect.height), companion_alpha,
    )
    return destination, insets

  def _draw_crop(self, content_rect: rl.Rectangle, destination: rl.Rectangle,
                 crop: Region, alpha: float, low_light: float, side: Side,
                 mask_insets: tuple[float, float]) -> None:
    rl.begin_scissor_mode(int(destination.x), int(destination.y),
                          int(destination.width), int(destination.height))
    # A negative source width mirrors only the selected side crop, matching the
    # stock full-screen driver-camera behavior.
    source = rl.Rectangle(crop.x, crop.y, -crop.width, crop.height)
    self._crop_alpha_value[0] = alpha
    self._crop_low_light_value[0] = low_light
    screen_width = max(float(rl.get_screen_width()), 1.0)
    screen_height = max(float(rl.get_screen_height()), 1.0)
    render_width = max(float(rl.get_render_width()), 1.0)
    render_height = max(float(rl.get_render_height()), 1.0)
    scale_x = render_width / screen_width
    scale_y = render_height / screen_height
    self._crop_mask_viewport_value[0] = destination.x * scale_x
    self._crop_mask_viewport_value[1] = render_height - (destination.y + destination.height) * scale_y
    self._crop_mask_viewport_value[2] = destination.width * scale_x
    self._crop_mask_viewport_value[3] = destination.height * scale_y
    top_inset, bottom_inset = mask_insets
    self._crop_wedge_insets_value[0] = top_inset
    self._crop_wedge_insets_value[1] = bottom_inset
    self._crop_mask_side_value[0] = -1.0 if side == "left" else 1.0
    rl.set_shader_value(
      self.shader,
      self._crop_alpha_loc,
      self._crop_alpha_value,
      rl.ShaderUniformDataType.SHADER_UNIFORM_FLOAT,
    )
    rl.set_shader_value(
      self.shader,
      self._crop_low_light_loc,
      self._crop_low_light_value,
      rl.ShaderUniformDataType.SHADER_UNIFORM_FLOAT,
    )
    rl.set_shader_value(
      self.shader,
      self._crop_mask_viewport_loc,
      self._crop_mask_viewport_value,
      rl.ShaderUniformDataType.SHADER_UNIFORM_VEC4,
    )
    rl.set_shader_value(
      self.shader,
      self._crop_wedge_insets_loc,
      self._crop_wedge_insets_value,
      rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2,
    )
    rl.set_shader_value(
      self.shader,
      self._crop_mask_side_loc,
      self._crop_mask_side_value,
      rl.ShaderUniformDataType.SHADER_UNIFORM_FLOAT,
    )
    if TICI:
      self._render_egl(source, destination)
    else:
      self._render_textures(source, destination)

    # Restore the parent's scissor rectangle for all subsequent UI renderers.
    rl.begin_scissor_mode(int(content_rect.x), int(content_rect.y),
                          int(content_rect.width), int(content_rect.height))


class CroppedDcamViewBP(_CroppedDcamMixin, TiciCameraView):
  CROP_VERTEX_SHADER = TICI_VERTEX_SHADER
  CROP_FRAGMENT_SHADER = _with_crop_effects(TICI_FRAME_FRAGMENT_SHADER)


class MiciCroppedDcamViewBP(_CroppedDcamMixin, MiciCameraView):
  CROP_VERTEX_SHADER = MICI_VERTEX_SHADER
  CROP_FRAGMENT_SHADER = _with_crop_effects(MICI_FRAME_FRAGMENT_SHADER)

  def _crop_layout(self, content_rect: rl.Rectangle, side: Side, companion_alpha: float):
    region = compact_panel_region(
      Region(content_rect.x, content_rect.y, content_rect.width, content_rect.height), side,
    )
    # Move the wedge boundary outside the viewport for a complete rectangular crop.
    return rl.Rectangle(region.x, region.y, region.width, region.height), (-0.02, -0.02)

  def _draw_crop(self, content_rect: rl.Rectangle, destination: rl.Rectangle,
                 crop: Region, alpha: float, low_light: float, side: Side,
                 mask_insets: tuple[float, float]) -> None:
    border = 1.5
    frame = rl.Rectangle(destination.x - border, destination.y - border,
                         destination.width + border * 2, destination.height + border * 2)
    rl.draw_rectangle_rec(frame, rl.Color(0, 0, 0, int(230 * alpha)))
    super()._draw_crop(content_rect, destination, crop, alpha, low_light, side, mask_insets)
    rl.draw_rectangle_lines_ex(frame, border, rl.Color(0, 52, 120, int(245 * alpha)))
