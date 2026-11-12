import io
import pickle
import struct

import numpy as np

from openpilot.sunnypilot.modeld_v2.runtime_helpers import (POLICY_INPUTS, WARP_INPUTS, load_oob,
                                                            make_supercombo_input_queues)
from openpilot.sunnypilot.models.fetcher import ModelParser
from openpilot.sunnypilot.models.helpers import _bundle_artifacts


def _chunked_bundle():
  return {
    'short_name': 'RL',
    'display_name': 'RL Model',
    'is_20hz': True,
    'ref': 'abc123',
    'environment': 'development',
    'runner': 'tinygrad',
    'index': 1,
    'minimum_selector_version': '19',
    'generation': '12',
    'models': [{
      'type': 'chunked',
      'artifact': {
        'file_name': 'driving_rl_tinygrad.pkl',
        'download_uri': {'url': 'https://example.test/driving_rl_tinygrad.pkl', 'sha256': 'ff' * 32},
        'chunks': [
          {'file_name': 'driving_rl_tinygrad.pkl.chunk01of02', 'sha256': '11' * 32},
          {'file_name': 'driving_rl_tinygrad.pkl.chunk02of02', 'sha256': '22' * 32},
        ],
      },
    }],
  }


def test_parse_v22_chunked_bundle():
  bundles = ModelParser.parse_models({'bundles': [_chunked_bundle()]})
  assert len(bundles) == 1
  model = bundles[0].models[0]
  assert str(model.type) == 'chunked'
  assert [chunk.fileName for chunk in model.artifact.chunks] == [
    'driving_rl_tinygrad.pkl.chunk01of02',
    'driving_rl_tinygrad.pkl.chunk02of02',
  ]


def test_chunk_hashes_are_validated_individually():
  bundle = ModelParser.parse_models({'bundles': [_chunked_bundle()]})[0]
  assert _bundle_artifacts(bundle) == [
    ('driving_rl_tinygrad.pkl.chunk01of02', '11' * 32),
    ('driving_rl_tinygrad.pkl.chunk02of02', '22' * 32),
  ]


def test_load_oob_round_trip():
  source = {'metadata': {'model': {'input_shapes': {'action_t': (1, 2)}}},
            'weights': np.arange(16, dtype=np.float32)}
  buffers = io.BytesIO()

  def buffer_callback(pickle_buffer):
    raw = pickle_buffer.raw()
    buffers.write(struct.pack('<q', raw.nbytes))
    buffers.write(raw)

  opcodes = io.BytesIO()
  pickle.Pickler(opcodes, protocol=5, buffer_callback=buffer_callback).dump(source)
  encoded = io.BytesIO(struct.pack('<q', len(opcodes.getvalue())) + opcodes.getvalue() + buffers.getvalue())
  decoded = load_oob(encoded)

  assert decoded['metadata'] == source['metadata']
  np.testing.assert_array_equal(decoded['weights'], source['weights'])


def test_v22_rl_queue_layout():
  input_shapes = {
    'img': (1, 12, 128, 256),
    'big_img': (1, 12, 128, 256),
    'desire_pulse': (1, 25, 8),
    'traffic_convention': (1, 2),
    'action_t': (1, 2),
    'features_buffer': (1, 24, 512),
  }
  queues, numpy_inputs = make_supercombo_input_queues(input_shapes, frame_skip=4, device='NPY')

  assert set(POLICY_INPUTS).issubset(queues)
  assert set(WARP_INPUTS).issubset(queues)
  assert queues['packed_npy_inputs'].shape == (524,)
  assert queues['feat_q'].shape == (96, 1, 512)
  assert numpy_inputs['action_t'].shape == (1, 2)
  assert numpy_inputs['prev_feat'].shape == (1, 512)
