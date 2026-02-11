import pyray as rl
from pyray import ffi
import numpy as np
from dataclasses import dataclass
from typing import Any, Optional, cast
from openpilot.system.ui.lib.application import gui_app, GL_VERSION

MAX_GRADIENT_COLORS = 20  # includes stops as well


@dataclass
class Gradient:
  start: tuple[float, float]
  end: tuple[float, float]
  colors: list[rl.Color]
  stops: list[float]

  def __post_init__(self):
    if len(self.colors) > MAX_GRADIENT_COLORS:
      self.colors = self.colors[:MAX_GRADIENT_COLORS]
      print(f"Warning: Gradient colors truncated to {MAX_GRADIENT_COLORS} entries")

    if len(self.stops) > MAX_GRADIENT_COLORS:
      self.stops = self.stops[:MAX_GRADIENT_COLORS]
      print(f"Warning: Gradient stops truncated to {MAX_GRADIENT_COLORS} entries")

    if not len(self.stops):
      color_count = min(len(self.colors), MAX_GRADIENT_COLORS)
      self.stops = [i / max(1, color_count - 1) for i in range(color_count)]


FRAGMENT_SHADER = GL_VERSION + """
in vec2 fragTexCoord;
out vec4 finalColor;

uniform vec4 fillColor;

// Gradient line defined in *screen pixels*
uniform int useGradient;
uniform vec2 gradientStart;  // e.g. vec2(0, 0)
uniform vec2 gradientEnd;    // e.g. vec2(0, screenHeight)
uniform vec4 gradientColors[20];
uniform float gradientStops[20];
uniform int gradientColorCount;

vec4 getGradientColor(vec2 p) {
  // Compute t from screen-space position
  vec2 d = gradientStart - gradientEnd;
  float len2 = max(dot(d, d), 1e-6);
  float t = clamp(dot(p - gradientEnd, d) / len2, 0.0, 1.0);

  // Clamp to range
  float t0 = gradientStops[0];
  float tn = gradientStops[gradientColorCount-1];
  if (t <= t0) return gradientColors[0];
  if (t >= tn) return gradientColors[gradientColorCount-1];

  for (int i = 0; i < gradientColorCount - 1; i++) {
    float a = gradientStops[i];
    float b = gradientStops[i+1];
    if (t >= a && t <= b) {
      float k = (t - a) / max(b - a, 1e-6);
      return mix(gradientColors[i], gradientColors[i+1], k);
    }
  }

  return gradientColors[gradientColorCount-1];
}

void main() {
  // TODO: do proper antialiasing
  finalColor = useGradient == 1 ? getGradientColor(gl_FragCoord.xy) : fillColor;
}
"""

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

CIRCLE_SHADER = GL_VERSION + """
out vec4 finalColor;

uniform vec2 center;
uniform float radius;
uniform vec4 topColor;
uniform vec4 bottomColor;

void main()
{
    // Pixel coordinates
    vec2 uv = gl_FragCoord.xy;

    // Distance in pixel space (no aspect distortion)
    float dist = distance(uv, center);

    // Mask outside the circle
    if (dist > radius) {
        finalColor = vec4(0.0, 0.0, 0.0, 0.0);
        return;
    }

    // Gradient factor
    float t = clamp(dist / radius, 0.0, 1.0);

    // Radial gradient
    finalColor = mix(topColor, bottomColor, t);
}
"""

# Default vertex shader
VERTEX_SHADER = GL_VERSION + """
in vec3 vertexPosition;
in vec2 vertexTexCoord;
out vec2 fragTexCoord;
uniform mat4 mvp;

void main() {
  fragTexCoord = vertexTexCoord;
  gl_Position = mvp * vec4(vertexPosition, 1.0);
}
"""

UNIFORM_INT = rl.ShaderUniformDataType.SHADER_UNIFORM_INT
UNIFORM_FLOAT = rl.ShaderUniformDataType.SHADER_UNIFORM_FLOAT
UNIFORM_VEC2 = rl.ShaderUniformDataType.SHADER_UNIFORM_VEC2
UNIFORM_VEC4 = rl.ShaderUniformDataType.SHADER_UNIFORM_VEC4

class ShaderState:
  _instance: Any = None

  @classmethod
  def get_instance(cls):
    if cls._instance is None:
      cls._instance = cls()
    return cls._instance

  def __init__(self):
    if ShaderState._instance is not None:
      raise Exception("This class is a singleton. Use get_instance() instead.")

    self.initialized = False
    self.shader = None
    self.exp_shader = None
    self.circle_shader = None
    self.rainbow_shader = None
    self.rainbow_shader_offset = 0.0
    self.last_time = rl.get_time()

    # Shader uniform locations
    self.locations = {
      'fillColor': None,
      'useGradient': None,
      'gradientStart': None,
      'gradientEnd': None,
      'gradientColors': None,
      'gradientStops': None,
      'gradientColorCount': None,
      'mvp': None,
    }

    self.rainbow_locations = {
      'squarePos': None,
      'squareSize': None,
      'offset': None,
    }

    self.circle_locations = {
      'center': None,
      'radius': None,
      'topColor': None,
      'bottomColor': None,
    }

    # Pre-allocated FFI objects
    self.fill_color_ptr = rl.ffi.new("float[]", [0.0, 0.0, 0.0, 0.0])
    self.use_gradient_ptr = rl.ffi.new("int[]", [0])
    self.color_count_ptr = rl.ffi.new("int[]", [0])
    self.gradient_colors_ptr = rl.ffi.new("float[]", MAX_GRADIENT_COLORS * 4)
    self.gradient_stops_ptr = rl.ffi.new("float[]", MAX_GRADIENT_COLORS)

    self.square_pos = ffi.new("float[2]", [0, 0])
    self.square_size = ffi.new("float *", 0.0)
    self.offset_val = ffi.new("float *", 0.0)

    self.circle_center = ffi.new("float[2]", [0, 0])
    self.circle_radius = ffi.new("float *", 0.0)
    self.circle_top_color = ffi.new("float[4]", [0.0, 0.0, 0.0, 0.0])
    self.circle_bottom_color = ffi.new("float[4]", [0.0, 0.0, 0.0, 0.0])

  def initialize(self):
    if self.initialized:
      return

    self.exp_shader = rl.load_shader_from_memory(VERTEX_SHADER, FRAGMENT_SHADER)
    self.rainbow_shader = rl.load_shader_from_memory(VERTEX_SHADER, RAINBOW_SHADER)
    self.circle_shader = rl.load_shader_from_memory(VERTEX_SHADER, CIRCLE_SHADER)

    # Cache all uniform locations
    for uniform in self.locations.keys():
      self.locations[uniform] = rl.get_shader_location(self.exp_shader, uniform)

    for uniform in self.rainbow_locations.keys():
      self.rainbow_locations[uniform] = rl.get_shader_location(self.rainbow_shader, uniform)

    for uniform in self.circle_locations.keys():
      self.circle_locations[uniform] = rl.get_shader_location(self.circle_shader, uniform)

    # Orthographic MVP (origin top-left)
    proj = rl.matrix_ortho(0, gui_app.width, gui_app.height, 0, -1, 1)
    rl.set_shader_value_matrix(self.exp_shader, self.locations['mvp'], proj)

    self.initialized = True

  def IncrementOffset(self, v: float = 1.0) -> float:
    time = rl.get_time()
    delta = time - self.last_time
    self.last_time = time
    self.rainbow_shader_offset += delta * v
    return self.rainbow_shader_offset

  def cleanup(self):
    if not self.initialized:
      return
    if self.shader:
      rl.unload_shader(self.shader)
      self.shader = None

    self.initialized = False


def _configure_shader_color(state: ShaderState, color: Optional[rl.Color],
                            gradient: Gradient | None, origin_rect: rl.Rectangle,
                            rainbow: bool = False, rainbow_v: float = 1.0):
  assert (color is not None) or (gradient is not None) or rainbow, "Either color, gradient, or rainbow must be provided"

  use_gradient = 1 if (gradient is not None and len(gradient.colors) >= 1) else 0
  state.use_gradient_ptr[0] = use_gradient
  rl.set_shader_value(state.exp_shader, state.locations['useGradient'], state.use_gradient_ptr, UNIFORM_INT)

  if use_gradient:
    gradient = cast(Gradient, gradient)
    state.color_count_ptr[0] = len(gradient.colors)
    for i in range(len(gradient.colors)):
      c = gradient.colors[i]
      base = i * 4
      state.gradient_colors_ptr[base:base + 4] = [c.r / 255.0, c.g / 255.0, c.b / 255.0, c.a / 255.0]
    rl.set_shader_value_v(state.exp_shader, state.locations['gradientColors'], state.gradient_colors_ptr, UNIFORM_VEC4, len(gradient.colors))

    for i in range(len(gradient.stops)):
      s = float(gradient.stops[i])
      state.gradient_stops_ptr[i] = 0.0 if s < 0.0 else 1.0 if s > 1.0 else s
    rl.set_shader_value_v(state.exp_shader, state.locations['gradientStops'], state.gradient_stops_ptr, UNIFORM_FLOAT, len(gradient.stops))
    rl.set_shader_value(state.exp_shader, state.locations['gradientColorCount'], state.color_count_ptr, UNIFORM_INT)

    # Map normalized start/end to screen pixels
    start_vec = rl.Vector2(origin_rect.x + gradient.start[0] * origin_rect.width, origin_rect.y + gradient.start[1] * origin_rect.height)
    end_vec = rl.Vector2(origin_rect.x + gradient.end[0] * origin_rect.width, origin_rect.y + gradient.end[1] * origin_rect.height)
    rl.set_shader_value(state.exp_shader, state.locations['gradientStart'], start_vec, UNIFORM_VEC2)
    rl.set_shader_value(state.exp_shader, state.locations['gradientEnd'], end_vec, UNIFORM_VEC2)
    state.shader = state.exp_shader
  elif rainbow:
    state.square_pos[0] = origin_rect.x
    state.square_pos[1] = origin_rect.y
    state.square_size[0] = float(origin_rect.width)
    state.offset_val[0] = state.IncrementOffset(rainbow_v)

    rl.set_shader_value(state.rainbow_shader, state.rainbow_locations['squarePos'], state.square_pos, rl.SHADER_UNIFORM_VEC2)
    rl.set_shader_value(state.rainbow_shader, state.rainbow_locations['squareSize'], state.square_size, rl.SHADER_UNIFORM_FLOAT)
    rl.set_shader_value(state.rainbow_shader, state.rainbow_locations['offset'], state.offset_val, rl.SHADER_UNIFORM_FLOAT)
    state.shader = state.rainbow_shader
  else:
    color = color or rl.WHITE
    state.fill_color_ptr[0:4] = [color.r / 255.0, color.g / 255.0, color.b / 255.0, color.a / 255.0]
    rl.set_shader_value(state.exp_shader, state.locations['fillColor'], state.fill_color_ptr, UNIFORM_VEC4)
    state.shader = state.exp_shader

def triangulate(pts: np.ndarray) -> list[tuple[float, float]]:
  """Only supports simple polygons with two chains (ribbon)."""

  # TODO: consider deduping close screenspace points
  # interleave points to produce a triangle strip
  # assert len(pts) % 2 == 0, "Interleaving expects even number of points"
  if len(pts) % 2 != 0:
    pts = pts[:-1]

  tri_strip = []
  for i in range(len(pts) // 2):
    tri_strip.append(pts[i])
    tri_strip.append(pts[-i - 1])

  return cast(list, np.array(tri_strip).tolist())


def draw_polygon(origin_rect: rl.Rectangle, points: np.ndarray,
                 color: Optional[rl.Color] = None,
                 gradient: Gradient | None = None,
                 rainbow: bool = False, rainbow_v: float = 1.0) -> None:

  """
  Draw a ribbon polygon (two chains) with a triangle strip and gradient.
  - Input must be [L0..Lk-1, Rk-1..R0], even count, no crossings/holes.
  """
  if len(points) < 3:
    return

  # Initialize shader on-demand
  state = ShaderState.get_instance()
  state.initialize()

  # Ensure (N,2) float32 contiguous array
  pts = np.ascontiguousarray(points, dtype=np.float32)
  assert pts.ndim == 2 and pts.shape[1] == 2, "points must be (N,2)"

  # Triangulate via interleaving
  tri_strip = triangulate(pts)

  # Configure gradient shader
  _configure_shader_color(state, color, gradient, origin_rect, rainbow, rainbow_v)

  # Draw strip, color here doesn't matter
  rl.begin_shader_mode(state.shader)
  rl.draw_triangle_strip(tri_strip, len(tri_strip), rl.WHITE)
  rl.end_shader_mode()

def draw_circle_gradient(rect: rl.Rectangle, center_x: float, center_y: float, radius: float, top_color: rl.Color, bottom_color: rl.Color) -> None:
  state = ShaderState.get_instance()
  state.initialize()

  state.circle_center[0:2] = [center_x, center_y]
  state.circle_radius[0] = radius
  state.circle_top_color[0:4] = [top_color.r / 255.0, top_color.g / 255.0, top_color.b / 255.0, top_color.a / 255.0]
  state.circle_bottom_color[0:4] = [bottom_color.r / 255.0, bottom_color.g / 255.0, bottom_color.b / 255.0, bottom_color.a / 255.0]

  rl.set_shader_value(state.circle_shader, state.circle_locations['center'], state.circle_center, rl.SHADER_UNIFORM_VEC2)
  rl.set_shader_value(state.circle_shader, state.circle_locations['radius'], state.circle_radius, rl.SHADER_UNIFORM_FLOAT)
  rl.set_shader_value(state.circle_shader, state.circle_locations['topColor'], state.circle_top_color, rl.SHADER_UNIFORM_VEC4)
  rl.set_shader_value(state.circle_shader, state.circle_locations['bottomColor'], state.circle_bottom_color, rl.SHADER_UNIFORM_VEC4)

  # Draw quad covering the circle area; shader will discard pixels outside the radius
  # square_pos = (center[0] - radius, center[1] - radius)
  # square_size = int(radius * 2)

  rl.begin_shader_mode(state.circle_shader)
  rl.draw_rectangle_rec(rect, rl.WHITE)
  rl.end_shader_mode()


def cleanup_shader_resources():
  state = ShaderState.get_instance()
  state.cleanup()
