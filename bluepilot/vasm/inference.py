from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


VASM_MODEL_PATH = Path(__file__).resolve().parents[1] / "assets" / "vision_models" / "v_asm_model.onnx"
# StarPilot's integrated export keeps 299 final candidates so OpenCV's TopK
# importer can load it. The original 300-candidate PR export is incompatible.
VASM_MODEL_SHA256 = "00247ede5159dff9a0768c171095711d40f1ee109ac7b5e24adce344fe4ba6f9"

MODEL_INPUT_H = 256
MODEL_INPUT_W = 352
HYSTERESIS_ON = 0.65
HYSTERESIS_OFF = 0.25


def detection_confidence(output: np.ndarray) -> float:
  """Extract the strongest class-0 confidence from the exported YOLO output."""
  predictions = np.squeeze(output)
  if predictions.ndim == 2:
    if predictions.shape[0] < predictions.shape[1]:
      predictions = predictions.T
    if predictions.shape[1] >= 6:
      relevant = predictions[np.round(predictions[:, 5]).astype(int) == 0]
      return 0.0 if len(relevant) == 0 else float(np.max(relevant[:, 4]))
    if predictions.shape[1] >= 5:
      return float(np.max(predictions[:, 4]))
    return float(np.max(predictions[:, 0]))
  if predictions.ndim == 1 and predictions.size > 0:
    return float(np.max(predictions))
  return 0.0


class VASMInference:
  def __init__(self, model_path: Path = VASM_MODEL_PATH):
    self.model_path = model_path
    self.net = None
    self.valid = False
    self.last_error = ""

    self.frame_res = (0, 0)
    self.config_width = 0
    self.config_height = 0
    self.masks: dict[str, np.ndarray | None] = {"left": None, "right": None}
    self.bboxes: dict[str, tuple[int, int, int, int] | np.ndarray | None] = {
      "left": None, "right": None, "left_raw": None, "right_raw": None,
    }
    self.reset_state()

  def load(self) -> bool:
    if not self.model_path.is_file():
      self.last_error = f"Missing model: {self.model_path}"
      return False
    try:
      self.net = cv2.dnn.readNetFromONNX(str(self.model_path))
      self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
      self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
      self.valid = True
      self.last_error = ""
    except Exception as exc:
      self.net = None
      self.valid = False
      self.last_error = f"Failed to load model: {exc}"
    return self.valid

  def reset_state(self) -> None:
    self._scores = {"left": 0.0, "right": 0.0}
    self.left_active = False
    self.right_active = False
    self.left_confidence = 0.0
    self.right_confidence = 0.0

  def load_config(self, config: dict) -> bool:
    self.frame_res = (0, 0)
    self.config_width = int(config.get("width", 0))
    self.config_height = int(config.get("height", 0))
    if self.config_width <= 0 or self.config_height <= 0:
      return False

    valid_side = False
    for side in ("left", "right"):
      polygon = config.get(f"poly_{side}", [])
      if len(polygon) >= 3:
        points = np.asarray(polygon, dtype=np.float32)
        if points.ndim == 2 and points.shape[1] == 2 and np.isfinite(points).all():
          self.bboxes[f"{side}_raw"] = points
          valid_side = True
          continue
      self.bboxes[f"{side}_raw"] = None
      self.bboxes[side] = None
      self.masks[side] = None
    return valid_side

  def _prepare_geometry(self, height: int, width: int) -> None:
    if (height, width) == self.frame_res:
      return
    self.frame_res = (height, width)

    scale_x = width / float(self.config_width)
    scale_y = height / float(self.config_height)
    for side in ("left", "right"):
      raw_points = self.bboxes.get(f"{side}_raw")
      if raw_points is None:
        self.bboxes[side] = None
        self.masks[side] = None
        continue

      points = raw_points.copy()
      points[:, 0] *= scale_x
      points[:, 1] *= scale_y
      bx, by, box_width, box_height = cv2.boundingRect(points.astype(np.int32))

      bx, by = (bx // 2) * 2, (by // 2) * 2
      box_width, box_height = ((box_width + 1) // 2) * 2, ((box_height + 1) // 2) * 2
      bx, by = max(0, min(bx, width - 2)), max(0, min(by, height - 2))
      box_width = (max(2, min(box_width, width - bx)) // 2) * 2
      box_height = (max(2, min(box_height, height - by)) // 2) * 2
      self.bboxes[side] = (bx, by, box_width, box_height)

      mask = np.zeros((box_height, box_width), dtype=np.uint8)
      cv2.fillPoly(mask, [points.astype(np.int32) - [bx, by]], 255)
      self.masks[side] = mask

  def _run_inference(self, raw_image: np.ndarray, height: int, side: str) -> float:
    bbox = self.bboxes[side]
    if bbox is None or self.net is None:
      return 0.0
    x, y, width, box_height = bbox

    y_crop = raw_image[y:y + box_height, x:x + width]
    uv_crop = raw_image[height + y // 2:height + (y + box_height) // 2, x:x + width]
    crop_rgb = cv2.cvtColor(np.vstack([y_crop, uv_crop]), cv2.COLOR_YUV2RGB_NV12)
    if self.masks[side] is not None:
      crop_rgb = cv2.bitwise_and(crop_rgb, crop_rgb, mask=self.masks[side])

    resized = cv2.resize(crop_rgb, (MODEL_INPUT_W, MODEL_INPUT_H), interpolation=cv2.INTER_LINEAR)
    model_input = np.expand_dims(np.transpose(resized.astype(np.float32) / 255.0, (2, 0, 1)), axis=0)
    self.net.setInput(model_input)
    output = self.net.forward()
    return detection_confidence(output)

  def update(self, raw_image: np.ndarray, width: int, height: int, dt: float,
             confidence_threshold: float, smooth_seconds: float, side: str) -> tuple[bool, bool]:
    if not self.valid:
      return False, False

    self._prepare_geometry(height, width)
    alpha = min(1.0, dt / max(smooth_seconds, 0.001))
    confidence = self._run_inference(raw_image, height, side)
    self._scores[side] = min(1.0, self._scores[side] + alpha) if confidence >= confidence_threshold \
      else max(0.0, self._scores[side] - alpha)

    setattr(self, f"{side}_confidence", confidence)
    active_name = f"{side}_active"
    if self._scores[side] >= HYSTERESIS_ON:
      setattr(self, active_name, True)
    elif self._scores[side] <= HYSTERESIS_OFF:
      setattr(self, active_name, False)
    return self.left_active, self.right_active
