import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import cereal.messaging as messaging

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params, UnknownKeyName
from openpilot.selfdrive.ui.bp.lib.camera_angle_auto import speed_camera_stream


@pytest.mark.parametrize('low,high', [(10.0, 15.0), (5.0, 10.0)])
def test_speed_hysteresis_matches_stock_devices(low, high):
  current = 'road'
  expected = ['wide', 'wide', 'wide', 'wide', 'road', 'road', 'road', 'road', 'wide']
  speeds = [low - 1, low, (low + high) / 2, high, high + 1, high, (low + high) / 2, low, low - 1]
  for speed, target in zip(speeds, expected, strict=True):
    current = speed_camera_stream(speed, current, 'wide', 'road', True, low, high)
    assert current == target


@pytest.mark.parametrize('speed', [float('nan'), float('inf'), -float('inf')])
def test_invalid_speed_uses_normal_camera(speed):
  assert speed_camera_stream(speed, 'wide', 'wide', 'road', True, 10, 15) == 'road'


def test_missing_wide_camera_and_invalid_initial_stream():
  assert speed_camera_stream(0, 'wide', 'wide', 'road', False, 10, 15) == 'road'
  assert speed_camera_stream(12, 'driver', 'wide', 'road', True, 10, 15) == 'road'


@pytest.mark.parametrize('device', ['onroad', 'mici/onroad'])
def test_real_stream_override_preserves_experimental_fallback_without_gui(device):
  # Execute only the real selection method, avoiding WindowServer/renderer setup.
  source = Path(BASEDIR) / f'selfdrive/ui/bp/{device}/augmented_road_view_bp.py'
  tree = ast.parse(source.read_text())
  method = next(n for c in tree.body if isinstance(c, ast.ClassDef) for n in c.body
                if isinstance(n, ast.FunctionDef) and n.name == '_switch_stream_if_needed')
  stock = ast.parse((Path(BASEDIR) / f'selfdrive/ui/{device}/augmented_road_view.py').read_text())
  constants = {n.targets[0].id: ast.literal_eval(n.value) for n in stock.body
               if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name)
               and n.targets[0].id in ('WIDE_CAM_MAX_SPEED', 'ROAD_CAM_MIN_SPEED')}
  delegated = []
  namespace = dict(constants, WIDE_CAM='wide', ROAD_CAM='road', speed_camera_stream=speed_camera_stream,
                   AugmentedRoadView=NS(_switch_stream_if_needed=lambda view, sm: delegated.append(sm)))
  exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
  switch = namespace['_switch_stream_if_needed']
  view = NS(_bp_camera_angle_auto=True, available_streams={'wide', 'road'}, stream_type='road')
  view.switch_stream = lambda target: setattr(view, 'stream_type', target)
  sm = {'carState': NS(vEgo=0), 'selfdriveState': NS(experimentalMode=False)}
  switch(view, sm)
  assert view.stream_type == 'wide' and not delegated
  sm['carState'].vEgo = constants['ROAD_CAM_MIN_SPEED'] + 1
  switch(view, sm)
  assert view.stream_type == 'road'
  view._bp_camera_angle_auto = False
  for experimental in (False, True):
    sm['selfdriveState'].experimentalMode = experimental
    switch(view, sm)
  assert len(delegated) == 2


def test_param_registry_and_append_only_telemetry():
  params = Params()
  assert not params.get_bool('BPCameraAngleAuto')
  params.put_bool('BPCameraAngleAuto', True, block=True)
  assert params.get_bool('BPCameraAngleAuto')
  with pytest.raises(UnknownKeyName):
    params.check_key('BPWideAngleCamera')
  msg = messaging.new_message('controllerStateBP')
  msg.controllerStateBP.bmsCameraAngleAuto = True
  assert msg.controllerStateBP.bmsCameraAngleAuto
  assert not msg.controllerStateBP.bmsUseWideCamera
