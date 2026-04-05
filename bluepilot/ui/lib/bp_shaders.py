"""
BluePilot: GPU shader utilities for rainbow path and gradient circle rendering.

These shaders were removed from upstream system/ui/lib/shader_polygon.py.
Preserved here as BP-only features (rainbow driving path, confidence ball).
"""

import pyray as rl
import numpy as np
from typing import Any, Optional

from openpilot.system.ui.lib.application import gui_app, GL_VERSION
from openpilot.system.ui.lib.shader_polygon import (
  VERTEX_SHADER, ShaderState, Gradient, triangulate,
  UNIFORM_INT, UNIFORM_FLOAT, UNIFORM_VEC2, UNIFORM_VEC4,
)

# ── Rainbow animated HSV shader ──────────────────────────────────────────────

RAINBOW_SHADER = GL_VERSION + """
out vec4 finalColor;

uniform vec2 squarePos;
uniform float squareSize;
uniform float offset;

vec3 hsv2rgb(vec3 c)
{
    vec4 K = vec4(1.0, 2.0/3.0, 1.0/3.0, 3.0);
    vec3 p = abs(fract(c.xxx + K.xyz) * 6.0 - K.www);
    return c.z * mix(K.xxx, clamp(p - K.xxx, 0.0, 1.0), c.y);
}

void main()
{
    vec2 p = gl_FragCoord.xy;

    // Clip to square
    if (p.x < squarePos.x || p.x > squarePos.x + squareSize ||
        p.y < squarePos.y || p.y > squarePos.y + squareSize)
        discard;

    float t = (p.y - squarePos.y) / squareSize;

    // Animate
    t = fract(t + offset * 0.2);

    vec3 col = hsv2rgb(vec3(t, 1.0, 1.0));
    //alpha 60%
    finalColor = vec4(col, 0.6);
}
"""


class RainbowShaderState:
  """Singleton managing the rainbow path shader."""

  _instance: Any = None

  @classmethod
  def get_instance(cls):
    if cls._instance is None:
      cls._instance = cls()
    return cls._instance

  def __init__(self):
    if RainbowShaderState._instance is not None:
      raise Exception("Singleton. Use get_instance().")

    self.initialized = False
    self.shader = None
    self.offset = 0.0
    self.last_time = 0.0

    self.locations = {
      'squarePos': None,
      'squareSize': None,
      'offset': None,
      'mvp': None,
    }

    self.square_pos = rl.ffi.new("float[2]", [0, 0])
    self.square_size = rl.ffi.new("float *", 0.0)
    self.offset_val = rl.ffi.new("float *", 0.0)

  def initialize(self):
    if self.initialized:
      return

    self.shader = rl.load_shader_from_memory(VERTEX_SHADER, RAINBOW_SHADER)
    self.last_time = rl.get_time()

    for uniform in self.locations:
      self.locations[uniform] = rl.get_shader_location(self.shader, uniform)

    proj = rl.matrix_ortho(0, gui_app.width, gui_app.height, 0, -1, 1)
    rl.set_shader_value_matrix(self.shader, self.locations['mvp'], proj)
    self.initialized = True

  def increment_offset(self, v: float = 1.0) -> float:
    time = rl.get_time()
    delta = time - self.last_time
    self.last_time = time
    self.offset += delta * v
    return self.offset

  def cleanup(self):
    if self.initialized and self.shader:
      rl.unload_shader(self.shader)
      self.shader = None
    self.initialized = False


def draw_rainbow_polygon(origin_rect: rl.Rectangle, points: np.ndarray, rainbow_v: float = 1.0) -> None:
  """Draw a ribbon polygon with animated rainbow HSV colors."""
  if len(points) < 3:
    return

  state = RainbowShaderState.get_instance()
  state.initialize()

  pts = np.ascontiguousarray(points, dtype=np.float32)
  assert pts.ndim == 2 and pts.shape[1] == 2, "points must be (N,2)"

  tri_strip = triangulate(pts)

  state.square_pos[0] = origin_rect.x
  state.square_pos[1] = origin_rect.y
  state.square_size[0] = float(origin_rect.width)
  state.offset_val[0] = state.increment_offset(rainbow_v)

  rl.set_shader_value(state.shader, state.locations['squarePos'], state.square_pos, UNIFORM_VEC2)
  rl.set_shader_value(state.shader, state.locations['squareSize'], state.square_size, UNIFORM_FLOAT)
  rl.set_shader_value(state.shader, state.locations['offset'], state.offset_val, UNIFORM_FLOAT)

  rl.begin_shader_mode(state.shader)
  rl.draw_triangle_strip(tri_strip, len(tri_strip), rl.WHITE)
  rl.end_shader_mode()


# ── GPU-accelerated gradient circle shader ───────────────────────────────────

class CircleShaderState:
  """Singleton for anti-aliased gradient circle rendering."""

  _instance: Any = None

  @classmethod
  def get_instance(cls):
    if cls._instance is None:
      cls._instance = cls()
    return cls._instance

  FRAGMENT_SHADER = GL_VERSION + """
out vec4 finalColor;

uniform vec2 center;
uniform float radius;
uniform vec4 topColor;
uniform vec4 bottomColor;

void main()
{
    vec2 uv = gl_FragCoord.xy;
    float dist = distance(uv, center);

    // Anti-aliased edge: smooth falloff over 1px
    float alpha = 1.0 - smoothstep(radius - 1.0, radius, dist);
    if (alpha <= 0.0) discard;

    // Vertical gradient: 0 at bottom of circle, 1 at top
    float t = clamp((uv.y - center.y + radius) / (2.0 * radius), 0.0, 1.0);
    vec4 col = mix(bottomColor, topColor, t);
    col.a *= alpha;
    finalColor = col;
}
"""

  def __init__(self):
    if CircleShaderState._instance is not None:
      raise Exception("Singleton. Use get_instance().")

    self.initialized = False
    self.shader = None

    self.locations = {
      'center': None,
      'radius': None,
      'topColor': None,
      'bottomColor': None,
      'mvp': None,
    }

    self.center_ptr = rl.ffi.new("float[2]", [0, 0])
    self.radius_ptr = rl.ffi.new("float *", 0.0)
    self.top_color_ptr = rl.ffi.new("float[4]", [0.0, 0.0, 0.0, 0.0])
    self.bottom_color_ptr = rl.ffi.new("float[4]", [0.0, 0.0, 0.0, 0.0])

  def initialize(self):
    if self.initialized:
      return

    self.shader = rl.load_shader_from_memory(VERTEX_SHADER, self.FRAGMENT_SHADER)

    for uniform in self.locations:
      self.locations[uniform] = rl.get_shader_location(self.shader, uniform)

    proj = rl.matrix_ortho(0, gui_app.width, gui_app.height, 0, -1, 1)
    rl.set_shader_value_matrix(self.shader, self.locations['mvp'], proj)
    self.initialized = True

  def cleanup(self):
    if self.initialized and self.shader:
      rl.unload_shader(self.shader)
      self.shader = None
    self.initialized = False


def draw_shader_circle_gradient(center_x: float, center_y: float, radius: float,
                                top_color: rl.Color, bottom_color: rl.Color) -> None:
  """Draw a gradient circle using a GPU shader. Produces a clean anti-aliased circle."""
  state = CircleShaderState.get_instance()
  state.initialize()

  # Shader uses bottom-left origin for gl_FragCoord, so flip Y
  state.center_ptr[0] = center_x
  state.center_ptr[1] = gui_app.height - center_y
  state.radius_ptr[0] = radius
  state.top_color_ptr[0:4] = [top_color.r / 255.0, top_color.g / 255.0, top_color.b / 255.0, top_color.a / 255.0]
  state.bottom_color_ptr[0:4] = [bottom_color.r / 255.0, bottom_color.g / 255.0, bottom_color.b / 255.0, bottom_color.a / 255.0]

  rl.set_shader_value(state.shader, state.locations['center'], state.center_ptr, UNIFORM_VEC2)
  rl.set_shader_value(state.shader, state.locations['radius'], state.radius_ptr, UNIFORM_FLOAT)
  rl.set_shader_value(state.shader, state.locations['topColor'], state.top_color_ptr, UNIFORM_VEC4)
  rl.set_shader_value(state.shader, state.locations['bottomColor'], state.bottom_color_ptr, UNIFORM_VEC4)

  # Draw a quad covering the circle bounding box; shader discards pixels outside radius
  pad = 2  # 1px extra for anti-aliasing
  rl.begin_shader_mode(state.shader)
  rl.draw_rectangle(int(center_x - radius - pad), int(center_y - radius - pad),
                    int(radius * 2 + pad * 2), int(radius * 2 + pad * 2), rl.WHITE)
  rl.end_shader_mode()
