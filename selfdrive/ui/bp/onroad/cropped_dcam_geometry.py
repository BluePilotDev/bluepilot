"""Pure geometry helpers for BluePilot's cropped driver-camera view."""

from dataclasses import dataclass
from math import isfinite, tan
from typing import Literal


Side = Literal["left", "right"]

# A single active camera follows the broad side-window silhouette from the
# original feature reference. When both sides are visible, the companion alpha
# smoothly narrows both wedges into non-overlapping bookends. The tighter MICI
# profile preserves a generous center corridor on comma 4's compact display.
SINGLE_WEDGE_TOP_INSET = 0.31
SINGLE_WEDGE_TOP_INSET_COMPACT = 0.40
SINGLE_WEDGE_BOTTOM_INSET = 0.80
DUAL_WEDGE_TOP_INSET = 0.52
DUAL_WEDGE_TOP_INSET_COMPACT = 0.58
DUAL_WEDGE_BOTTOM_INSET = 0.82
DUAL_WEDGE_BOTTOM_INSET_COMPACT = 0.86
COMPACT_CONTENT_HEIGHT = 300.0

# Zoom into the outer third of the mirrored dcam image. Keeping this narrow
# source region in the larger integrated view prioritizes the side window over
# the occupants and seats on the supported fisheye cameras.
CROP_WIDTH_RATIO = 0.34
LEFT_RAW_CENTER_X = 0.83
RIGHT_RAW_CENTER_X = 0.17
DEFAULT_WINDOW_CENTER_Y = 0.51
WINDOW_CENTER_FACE_OFFSET_Y = 0.13
WINDOW_CENTER_MIN_Y = 0.45
WINDOW_CENTER_MAX_Y = 0.63

# ui_state.light_sensor is 100 in bright light and approaches zero in darkness.
LOW_LIGHT_ENHANCEMENT_START = 70.0
LOW_LIGHT_ENHANCEMENT_FULL = 20.0
POST_TRIGGER_HOLD_SECONDS = 1.0

@dataclass(frozen=True)
class Region:
  x: float
  y: float
  width: float
  height: float


@dataclass
class PostTriggerHold:
  """Keep a trigger asserted briefly after its falling edge."""
  hold_seconds: float = POST_TRIGGER_HOLD_SECONDS
  _input_active: bool = False
  _hold_until: float = 0.0

  def update(self, active: bool, now: float) -> bool:
    if active:
      self._input_active = True
      return True

    if self._input_active:
      self._input_active = False
      self._hold_until = now + self.hold_seconds

    return now < self._hold_until


def active_dcam_sides(car_state, turn_signal: bool = True, bsm: bool = True) -> tuple[bool, bool]:
  """Return left/right activation without coupling to either warning toggle."""
  return (
    bool((turn_signal and car_state.leftBlinker) or (bsm and car_state.leftBlindspot)),
    bool((turn_signal and car_state.rightBlinker) or (bsm and car_state.rightBlindspot)),
  )


def ease_visibility_alpha(alpha: float) -> float:
  """Clamp and smooth visibility for an OEM-like camera-card transition."""
  alpha = min(1.0, max(0.0, alpha))
  return alpha * alpha * (3.0 - 2.0 * alpha)


def wedge_insets(content: Region, companion_alpha: float) -> tuple[float, float]:
  """Return top/bottom outer-edge insets for a side camera wedge.

  ``companion_alpha`` makes the active side contract continuously while the
  other camera fades in, rather than jumping between single and dual layouts.
  """
  compact = content.height <= COMPACT_CONTENT_HEIGHT
  single_top = SINGLE_WEDGE_TOP_INSET_COMPACT if compact else SINGLE_WEDGE_TOP_INSET
  dual_top = DUAL_WEDGE_TOP_INSET_COMPACT if compact else DUAL_WEDGE_TOP_INSET
  dual_bottom = DUAL_WEDGE_BOTTOM_INSET_COMPACT if compact else DUAL_WEDGE_BOTTOM_INSET
  mix = ease_visibility_alpha(companion_alpha)
  return (
    single_top + (dual_top - single_top) * mix,
    SINGLE_WEDGE_BOTTOM_INSET + (dual_bottom - SINGLE_WEDGE_BOTTOM_INSET) * mix,
  )


def wedge_edge_x(content: Region, y_normalized: float, companion_alpha: float) -> float:
  """Return the normalized inner edge of a right-side wedge at ``y``."""
  top, bottom = wedge_insets(content, companion_alpha)
  y = min(1.0, max(0.0, y_normalized))
  return top + (bottom - top) * y


def wedge_canvas_region(content: Region, side: Side, companion_alpha: float) -> Region:
  """Bound a wedge so its broad top edge receives the complete source crop."""
  top, _ = wedge_insets(content, companion_alpha)
  width = content.width * (1.0 - top)
  x = content.x if side == "left" else content.x + content.width - width
  return Region(x, content.y, width, content.height)


def wedge_local_insets(content: Region, companion_alpha: float) -> tuple[float, float]:
  """Convert full-content wedge insets into the wedge canvas coordinate space."""
  top, bottom = wedge_insets(content, companion_alpha)
  return 0.0, (bottom - top) / (1.0 - top)


def source_crop(frame_width: float, frame_height: float, destination_width: float,
                destination_height: float, side: Side,
                calibration_rpy: tuple[float, float, float],
                window_center_y: float | None = None,
                focal_length: float = 567.0) -> Region:
  """Calculate a calibration-aware raw dcam crop.

  The driver stream is mirrored when drawn. The physical left side is therefore
  sourced from the raw image's right edge and vice versa. liveCalibration keeps
  the crop vehicle-relative across device roll/pitch/yaw, while the driver-model
  window anchor supplied by the caller adapts vertically to the installation.
  """
  if frame_width <= 0 or frame_height <= 0 or destination_width <= 0 or destination_height <= 0:
    return Region(0.0, 0.0, max(frame_width, 0.0), max(frame_height, 0.0))

  roll, pitch, yaw = calibration_rpy
  roll = roll if isfinite(roll) else 0.0
  pitch = pitch if isfinite(pitch) else 0.0
  yaw = yaw if isfinite(yaw) else 0.0

  crop_width = frame_width * CROP_WIDTH_RATIO
  crop_height = crop_width / (destination_width / destination_height)
  crop_height = min(frame_height * 0.70, crop_height)

  raw_center_x = LEFT_RAW_CENTER_X if side == "left" else RIGHT_RAW_CENTER_X
  raw_center_x += focal_length * tan(yaw) / frame_width

  # A detected face is already expressed in raw dcam coordinates, so it
  # inherently accounts for device pitch. Only project liveCalibration pitch
  # when the landmark is unavailable; applying both shifted the crop into the
  # headliner on pitched installations. Roll still moves the two sides in
  # opposite vertical directions around the optical center.
  center_y = window_center_y
  if center_y is None:
    center_y = DEFAULT_WINDOW_CENTER_Y - focal_length * tan(pitch) / frame_height
  center_y += (raw_center_x - 0.5) * (frame_width / frame_height) * tan(roll)

  center_x_px = raw_center_x * frame_width
  center_y_px = center_y * frame_height
  x = min(max(center_x_px - crop_width / 2, 0.0), frame_width - crop_width)
  y = min(max(center_y_px - crop_height / 2, 0.0), frame_height - crop_height)
  return Region(x, y, crop_width, crop_height)


def adaptive_window_center_y(face_position: tuple[float, float] | list[float] | None,
                             face_prob: float) -> float | None:
  """Use the driver's face as a stable per-installation vertical landmark."""
  if face_prob <= 0.5 or face_position is None or len(face_position) != 2:
    return None

  face_x, face_y = face_position
  # Same inexpensive raw-dcam approximation used by DriverCameraDialog.
  face_y_px = -135.0 + (504.0 + abs(face_x) * 112.0) + (1205.0 - abs(face_x) * 724.0) * face_y
  face_y_norm = face_y_px / 1208.0
  return min(WINDOW_CENTER_MAX_Y, max(WINDOW_CENTER_MIN_Y, face_y_norm + WINDOW_CENTER_FACE_OFFSET_Y))


def low_light_enhancement(light_sensor: float) -> float:
  """Return a smooth 0..1 shadow-lift strength from the existing UI light estimate."""
  if not isfinite(light_sensor) or light_sensor < 0:
    return 0.0

  span = LOW_LIGHT_ENHANCEMENT_START - LOW_LIGHT_ENHANCEMENT_FULL
  strength = (LOW_LIGHT_ENHANCEMENT_START - light_sensor) / span
  return ease_visibility_alpha(strength)
