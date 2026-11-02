import math
from typing import Any


MAX_IMAGE_DIMENSION = 10_000
MAX_POLYGON_POINTS = 64


def normalize_annotation_config(payload: Any) -> dict[str, Any]:
  """Validate and normalize the driver-camera polygons used by V-ASM."""
  if not isinstance(payload, dict):
    raise ValueError("configuration must be an object")

  try:
    width = int(payload["width"])
    height = int(payload["height"])
  except (KeyError, TypeError, ValueError) as exc:
    raise ValueError("width and height are required") from exc

  if not (1 <= width <= MAX_IMAGE_DIMENSION and 1 <= height <= MAX_IMAGE_DIMENSION):
    raise ValueError("invalid image dimensions")

  normalized: dict[str, Any] = {"width": width, "height": height}
  for side in ("left", "right"):
    key = f"poly_{side}"
    points = payload.get(key, [])
    if not isinstance(points, list) or len(points) > MAX_POLYGON_POINTS:
      raise ValueError(f"invalid {side} polygon")
    if points and len(points) < 3:
      raise ValueError(f"{side} polygon needs at least three points")

    normalized_points: list[list[float]] = []
    for point in points:
      if not isinstance(point, (list, tuple)) or len(point) != 2:
        raise ValueError(f"invalid point in {side} polygon")
      try:
        x, y = float(point[0]), float(point[1])
      except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid point in {side} polygon") from exc
      if not math.isfinite(x) or not math.isfinite(y) or not (0 <= x <= width and 0 <= y <= height):
        raise ValueError(f"point outside image in {side} polygon")
      normalized_points.append([x, y])
    normalized[key] = normalized_points

  if not normalized["poly_left"] and not normalized["poly_right"]:
    raise ValueError("draw at least one monitoring region")
  return normalized
