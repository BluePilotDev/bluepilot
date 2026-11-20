"""Runtime input queues for v22 chunked tinygrad models.

These helpers mirror the queue layout used when the v22 QCOM catalog artifacts
were compiled. They intentionally live outside compile_modeld so legacy model
pickles can continue using the older queue layout during migration.
"""

import io
import math
import pickle
import struct

import numpy as np
from tinygrad.device import Device
from tinygrad.tensor import Tensor

WARP_INPUTS = ('tfm', 'big_tfm')
POLICY_INPUTS = ('img_q', 'big_img_q', 'feat_q', 'desire_q', 'packed_npy_inputs')


def load_oob(model_file):
  """Load a protocol-5 pickle whose large buffers follow the opcode stream."""
  header = model_file.read(8)
  if len(header) != 8:
    raise pickle.UnpicklingError("Truncated model header")
  opcode_size = struct.unpack('<q', header)[0]
  if opcode_size <= 0:
    raise pickle.UnpicklingError("Invalid model opcode size")
  opcodes = model_file.read(opcode_size)
  if len(opcodes) != opcode_size:
    raise pickle.UnpicklingError("Truncated model opcode stream")

  def buffers():
    while header := model_file.read(8):
      if len(header) != 8:
        raise pickle.UnpicklingError("Truncated model buffer header")
      buffer_size = struct.unpack('<q', header)[0]
      if buffer_size < 0:
        raise pickle.UnpicklingError("Invalid model buffer size")
      buffer = pickle.PickleBuffer(bytearray(buffer_size))
      if model_file.readinto(buffer) != buffer_size:
        raise pickle.UnpicklingError("Truncated model buffer")
      yield buffer

  return pickle.load(io.BytesIO(opcodes), buffers=buffers())


def _detect_desire_key(shapes: dict) -> str | None:
  return next((key for key in shapes if key.startswith('desire')), None)


def _detect_vision_keys(shapes: dict) -> tuple[str | None, str | None]:
  image_keys = sorted(key for key in shapes if 'img' in key)
  return (
    next((key for key in image_keys if 'big' not in key), None),
    next((key for key in image_keys if 'big' in key), None),
  )


def get_policy_npy_shapes(input_shapes: dict, is_supercombo: bool = False) -> tuple[dict, list[int]]:
  desire_key = _detect_desire_key(input_shapes)
  shapes = {}
  if desire_key:
    shapes['desire'] = (input_shapes[desire_key][2],)

  for key, shape in input_shapes.items():
    if key not in (desire_key, 'features_buffer') and 'img' not in key:
      shapes[key] = tuple(shape)

  if is_supercombo and 'features_buffer' in input_shapes:
    features_buffer = input_shapes['features_buffer']
    shapes['prev_feat'] = (features_buffer[0], math.prod(features_buffer[2:]))

  return shapes, [int(np.prod(shape)) for shape in shapes.values()]


def generate_queues_and_npy(input_shapes: dict, frame_skip: int, device: str = Device.DEFAULT,
                            is_supercombo: bool = False) -> tuple[dict, dict]:
  road_key, _ = _detect_vision_keys(input_shapes)
  if road_key is None:
    raise ValueError("Vision road key missing from input shapes")

  image_shape = input_shapes[road_key]
  image_buffer_shape = (frame_skip * (image_shape[1] // 6 - 1) + 1, 6, image_shape[2], image_shape[3])

  desire_key = _detect_desire_key(input_shapes)
  if desire_key is None:
    raise ValueError("Desire key missing from input shapes")

  desire_shape = input_shapes[desire_key]
  features_buffer = input_shapes.get('features_buffer')
  numpy_inputs = {
    'tfm': np.zeros((3, 3), dtype=np.float32),
    'big_tfm': np.zeros((3, 3), dtype=np.float32),
  }

  shapes, sizes = get_policy_npy_shapes(input_shapes, is_supercombo=is_supercombo)
  packed_inputs = np.zeros(sum(sizes), dtype=np.float32)
  split_indices = np.cumsum(sizes[:-1]) if len(sizes) > 1 else []
  split_views = np.split(packed_inputs, split_indices) if sizes else []
  for (key, shape), view in zip(shapes.items(), split_views, strict=True):
    numpy_inputs[key] = view.reshape(shape)

  queues = {
    'img_q': Tensor(np.zeros(image_buffer_shape, dtype=np.uint8), device=device).contiguous().realize(),
    'big_img_q': Tensor(np.zeros(image_buffer_shape, dtype=np.uint8), device=device).contiguous().realize(),
    'desire_q': Tensor(np.zeros((frame_skip * desire_shape[1], desire_shape[0], desire_shape[2]), dtype=np.float32),
                       device=device).contiguous().realize(),
    'packed_npy_inputs': Tensor(packed_inputs, device='NPY').realize(),
  }

  if features_buffer:
    feature_dimension = math.prod(features_buffer[2:])
    queue_length = frame_skip * features_buffer[1] if is_supercombo else frame_skip * (features_buffer[1] - 1) + 1
    queues['feat_q'] = Tensor(np.zeros((queue_length, features_buffer[0], feature_dimension), dtype=np.float32),
                              device=device).contiguous().realize()

  queues.update({key: Tensor(value, device='NPY').realize()
                 for key, value in numpy_inputs.items() if key in WARP_INPUTS})
  return queues, numpy_inputs


def make_split_input_queues(vision_input_shapes: dict, policy_input_shapes: dict, frame_skip: int,
                            device: str = Device.DEFAULT) -> tuple[dict, dict]:
  return generate_queues_and_npy({**vision_input_shapes, **policy_input_shapes}, frame_skip, device, is_supercombo=False)


def make_supercombo_input_queues(input_shapes: dict, frame_skip: int,
                                 device: str = Device.DEFAULT) -> tuple[dict, dict]:
  return generate_queues_and_npy(input_shapes, frame_skip, device, is_supercombo=True)
