"""Run in a fresh process with the v26 tinygrad directory first on PYTHONPATH."""

import base64
import pickle
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from tinygrad import Tensor, TinyJit

from openpilot.sunnypilot.modeld_v2.plain_artifact import _dtype, cache_is_fresh, load_model, write_bundle
from openpilot.sunnypilot.modeld_v2.plain_runtime import NativePlainModel
from openpilot.sunnypilot.modeld_v2.tests.plain_fixture_compiler import capture_frame, record_kernel_linear, serialize_jit
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info


def compile_part(function, inputs, output_container=None):
  run = TinyJit(function, prune=True)
  with record_kernel_linear() as recorded:
    run(**inputs)
    run(**inputs)
    check, _ = capture_frame(run, inputs, output_container)
  assert len(recorded) == 1
  return serialize_jit(run, recorded[0], check)


def main():
  from tinygrad.dtype import dtypes
  # Cheddar's actual QCOM artifact uses short dtype names, unlike our CPU fixture.
  for short, long in (('f16', 'float16'), ('f32', 'float32'), ('f64', 'float64'), ('bf16', 'bfloat16'),
                      *((f'{prefix}{bits}', f'{kind}{bits}') for prefix, kind in (('i', 'int'), ('u', 'uint'))
                        for bits in (8, 16, 32, 64)), ('bool', 'bool')):
    assert _dtype(f'dtypes.{short}') == getattr(dtypes, long)
  with tempfile.TemporaryDirectory() as temp:
    path = str(Path(temp) / 'native.pkl')
    specs = {
      'new_img': ((2, 6, 8, 16), '|u1', 'CPU'),
      'desire': ((8,), '<f4', 'CPU'),
      'traffic_convention': ((1, 2), '<f4', 'CPU'),
      'action_t': ((1, 2), '<f4', 'CPU'),
      'state_feat_q': ((4, 16), '<f4', 'CPU'),
    }
    out_specs = {'outputs': ((1, 16), '<f4', 'CPU'), 'next_state_feat_q': ((4, 16), '<f4', 'CPU')}
    weights_array = np.arange(16, dtype=np.float32).reshape(1, 16)
    weights = Tensor(weights_array).realize()

    def native(new_img, desire, traffic_convention, action_t, state_feat_q, output_buffers):
      result = (state_feat_q.mean(axis=0) + new_img.float().mean() + desire.sum() + traffic_convention.sum() + action_t.sum())
      output_buffers['outputs'].assign(result.reshape(1, 16) + weights).realize()
      output_buffers['next_state_feat_q'].assign(state_feat_q + 1).realize()

    def allocate(specifications):
      return {name: Tensor(np.zeros(shape, dtype=dtype), device=device).realize()
              for name, (shape, dtype, device) in specifications.items()}

    inputs = allocate(specs) | {'output_buffers': allocate(out_specs)}
    metadata = {
      'metadata': {'output_slices': base64.b64encode(pickle.dumps({'hidden_state': slice(0, 16)})).decode()},
      'output_shapes': {name: shape for name, (shape, _, _) in out_specs.items()},
    }
    write_bundle(path, {'run': compile_part(native, inputs, 'output_buffers')},
                 {'metadata': metadata, 'input_specs': specs, 'output_specs': out_specs})
    jits = load_model(path)
    assert cache_is_fresh(path)
    jits = load_model(path)
    model = NativePlainModel(jits, 32, 16)
    size = get_nv12_info(32, 16)[3]
    frames = {key: Tensor(np.full(size, 16, dtype=np.uint8)).realize() for key in ('img', 'big_img')}
    model.numpy_inputs['traffic_convention'][0, 0] = 1
    model.numpy_inputs['desire'][2] = 1
    model.numpy_inputs['action_t'][0, 0] = 2
    assert model.run(frames, prepare_only=True) is None
    np.testing.assert_array_equal(model.input_queues['state_feat_q'].numpy(), 0)
    for index in range(4):
      result = model.run(frames, prepare_only=False).numpy()
      np.testing.assert_allclose(result, weights_array + 20 + index)
      np.testing.assert_allclose(model.input_queues['state_feat_q'].numpy(), index + 1)
    model.numpy_inputs['desire'][:] = 0
    np.testing.assert_allclose(model.run(frames, prepare_only=False).numpy(), weights_array + 23)

    # Exercise the actual modeld dispatch, including desire-edge handling and slicing.
    from openpilot.sunnypilot.modeld_v2.modeld import ModelState
    from openpilot.sunnypilot.modeld_v2.parse_model_outputs import Parser
    state = ModelState.__new__(ModelState)
    state._init_combined(path, 32, 16, None)
    assert state.is_native_model and not state.is_legacy_model
    state.parser = Parser(ignore_missing=True)
    buffers = {name: SimpleNamespace(data=np.full(size, 16, dtype=np.uint8)) for name in ('img', 'big_img')}
    transforms = {name: np.eye(3, dtype=np.float32) for name in buffers}
    inputs = {'desire': np.eye(8, dtype=np.float32)[2], 'traffic_convention': np.array([[1, 0]], dtype=np.float32)}
    assert state.run(buffers, transforms, inputs, prepare_only=True) is None
    inputs['desire'][:] = 0
    np.testing.assert_allclose(state.run(buffers, transforms, inputs, prepare_only=False)['hidden_state'], weights_array + 17)
    print('PASS: portable reference/cache checks, native NV12 warp, prepare-only, changing inputs, recurrent state, and modeld dispatch')


if __name__ == '__main__':
  main()
