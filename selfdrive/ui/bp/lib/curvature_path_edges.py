"""Small neutral highlights preserve the selected path hue and geometry."""
import math
import time

import numpy as np


def bend_strength(raw_points):
  points = np.asarray(raw_points)
  if points.ndim != 2 or points.shape[1] < 2 or len(points) < 5:
    return 0.0
  xy = points[:, :2]
  if not np.isfinite(xy).all() or np.any(np.diff(xy[:, 0]) <= 0):
    return 0.0
  # Fixed metric samples avoid sensitivity to model point spacing and far-horizon noise.
  if xy[-1, 0] < 20 or xy[0, 0] > 5:
    return 0.0
  x = np.arange(5.0, min(45.0, xy[-1, 0]) + 0.01, 5.0)
  y = np.interp(x, xy[:, 0], xy[:, 1])
  slope = np.gradient(y, x)
  curvature = np.abs(np.gradient(slope, x)) / (1 + slope * slope) ** 1.5
  return float(np.clip((np.median(curvature) - 0.001) / 0.009, 0, 1))


class CurvaturePathEdges:
  def __init__(self):
    self.value = 0.0
    self.last_time = time.monotonic()

  def update(self, enabled, points, fresh, now=None):
    now = time.monotonic() if now is None else now
    dt = max(0.0, min(now - self.last_time, 0.1))
    self.last_time = now
    if not enabled:
      self.value = 0.0
      return 0.0
    target = bend_strength(points) if fresh else 0.0
    self.value += (target - self.value) * (1 - math.exp(-dt / 0.45))
    return self.value


def edge_ribbons(points, half_width):
  points = np.asarray(points, dtype=np.float32)
  if points.ndim != 2 or points.shape[1] != 2 or len(points) < 6 or not np.isfinite(points).all():
    return []
  ribbons = []
  middle = len(points) // 2
  for edge in (points[:middle], points[middle:][::-1]):
    direction = np.gradient(edge, axis=0)
    length = np.linalg.norm(direction, axis=1)
    normal = np.stack((-direction[:, 1], direction[:, 0]), axis=1) / np.maximum(length[:, None], 1e-3)
    offset = normal * half_width
    ribbons.append(np.concatenate((edge + offset, (edge - offset)[::-1])).astype(np.float32))
  return ribbons


def draw_curvature_path_edges(renderer, sm, points, opacity=1.0):
  import pyray as rl
  from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state
  from openpilot.system.ui.lib.shader_polygon import Gradient, draw_polygon

  if not hasattr(renderer, '_curvature_path_edges'):
    renderer._curvature_path_edges = CurvaturePathEdges()
  now = time.monotonic()
  fresh = (sm.valid.get('modelV2', False) and sm.seen.get('modelV2', False) and
           0 <= now - sm.recv_time.get('modelV2', 0) <= 0.5)
  enabled = (renderer._curvature_path_edges_enabled and not renderer._rad_racer and
             ui_state.status != UIStatus.DISENGAGED)
  value = renderer._curvature_path_edges.update(enabled, renderer._path.raw_points, fresh, now)
  alpha = int(65 * value * max(0.0, min(1.0, opacity)))
  if alpha < 2:
    return
  # Two narrow batched ribbons, no cap or extra path geometry. Fade at the horizon.
  gradient = Gradient(start=(0.0, 1.0), end=(0.0, 0.0),
                      colors=[rl.Color(255, 255, 255, alpha), rl.Color(255, 255, 255, 0)], stops=[0.0, 1.0])
  for ribbon in edge_ribbons(points, max(0.6, renderer._rect.width / 1080)):
    draw_polygon(renderer._rect, ribbon, gradient=gradient)
