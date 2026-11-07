"""Camera preparation and recurrent input handling for portable QCOM models."""

import base64
import io
import math

import numpy as np
from tinygrad import Tensor, TinyJit
from tinygrad.uop.ops import UOp

from openpilot.sunnypilot.modeld_v2.plain_artifact import _MetadataUnpickler
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info


def make_camera_warp(cam_w: int, cam_h: int, model_w: int, model_h: int):
  """Use the NV12 conversion shipped with the catalog's pinned tinygrad."""
  from examples.openpilot.compile_warp import NV12Frame, make_frame_prepare
  frame = NV12Frame(cam_w, cam_h, *get_nv12_info(cam_w, cam_h))
  prepare = make_frame_prepare(frame, model_w, model_h)

  @TinyJit
  def warp(tfm, big_tfm, frame, big_frame):
    return Tensor.stack(prepare(frame, tfm), prepare(big_frame, big_tfm)).realize()

  return warp


def output_slices_from_metadata(metadata: dict) -> dict:
  encoded = metadata['metadata']['output_slices']
  return _MetadataUnpickler(io.BytesIO(base64.b64decode(encoded))).load()


class NativePlainModel:
  """Keep native recurrent state on the model device, with aliased next-state outputs."""

  def __init__(self, jits: dict, cam_w: int, cam_h: int):
    self.run_model = jits['run']
    self.input_specs = jits['input_specs']
    self.state_pairs = {name: f'next_{name}' for name in self.input_specs
                        if f'next_{name}' in jits['output_specs']}
    self.input_queues = {
      name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
      for name, (shape, dtype, device) in self.input_specs.items()
    }
    self.outputs = {
      name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
      for name, (shape, dtype, device) in jits['output_specs'].items()
    }
    for name, next_name in self.state_pairs.items():
      state = self.input_queues[name]
      view = state._buffer().view(math.prod(state.shape), state.dtype, 0).ensure_allocated()
      self.outputs[next_name] = Tensor(UOp.from_buffer(view)).reshape(state.shape)

    self.numpy_inputs = {
      name: np.zeros(shape, dtype=dtype)
      for name, (shape, dtype, _) in self.input_specs.items()
      if name not in self.state_pairs and name != 'new_img'
    }
    self.numpy_inputs |= {name: np.eye(3, dtype=np.float32) for name in ('tfm', 'big_tfm')}
    self.transforms = {name: Tensor(value, device='NPY').realize()
                       for name, value in self.numpy_inputs.items() if name in ('tfm', 'big_tfm')}
    shape = self.input_specs['new_img'][0]
    if len(shape) != 4 or shape[:2] != (2, 6):
      raise ValueError(f'Unsupported native camera input shape: {shape}')
    self.warp = make_camera_warp(cam_w, cam_h, shape[3] * 2, shape[2] * 2)
    self.vision_output_slices = output_slices_from_metadata(jits['metadata'])
    # Compile/capture the warp during model loading, before processing real frames.
    frame_size = get_nv12_info(cam_w, cam_h)[3]
    device = self.input_queues['new_img'].device
    frame = Tensor(np.zeros(frame_size, dtype=np.uint8), device=device).realize()
    big_frame = Tensor(np.zeros(frame_size, dtype=np.uint8), device=device).realize()
    for _ in range(3):
      self.warp(**self.transforms, frame=frame, big_frame=big_frame)

  def run(self, frames: dict, prepare_only: bool):
    warped = self.warp(**self.transforms, frame=frames['img'], big_frame=frames['big_img'])
    if prepare_only:
      return None
    self.input_queues['new_img'].assign(warped).realize()
    for name, value in self.numpy_inputs.items():
      if name not in self.transforms:
        queue = self.input_queues[name]
        queue.assign(Tensor(value, device='NPY').to(queue.device)).realize()
    self.run_model(**self.input_queues, output_buffers=self.outputs)
    return self.outputs['outputs']
