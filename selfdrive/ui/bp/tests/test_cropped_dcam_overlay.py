import sys
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.bp.onroad import cropped_dcam_overlay as overlay


@pytest.fixture
def setup_overlay(monkeypatch):
  clock = [10.0]
  monkeypatch.setattr(overlay, 'monotonic', lambda: clock[0])
  constructed = []

  class Camera:
    def __init__(self):
      self.calls = []
      self.visible = True
      self.suspended = 0
      constructed.append(self)

    def is_visible(self):
      return self.visible

    def suspend(self):
      self.suspended += 1
      self.visible = False

    def render_crops(self, *args, **kwargs):
      self.calls.append((args, kwargs))

  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.ui.bp.onroad.cropped_dcam_view',
                      NS(CroppedDcamViewBP=Camera, MiciCroppedDcamViewBP=Camera))

  class Messages(dict):
    valid = {'carState': True, 'liveCalibration': False, 'driverStateV2': False}
    seen = {'carState': True}
    recv_time = {'carState': clock[0]}

  state = NS(leftBlinker=True, rightBlinker=False, leftBlindspot=False, rightBlindspot=True)
  sm = Messages(carState=state)
  ui = NS(sm=sm, started=True, light_sensor=80)
  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.ui.ui_state', NS(ui_state=ui))
  values = {'BPCroppedDcam': False, 'BPCroppedDcamTurnSignal': True, 'BPCroppedDcamBSM': True}
  reads = []

  class Host(overlay.CroppedDcamOverlayMixin):
    device_camera = None

    def __init__(self):
      self._bp_params = NS(get_bool=lambda key: reads.append(key) or values[key])
      self._init_cropped_dcam()

    def _child(self, child):
      return child

  return NS(host=Host(), ui=ui, values=values, clock=clock, reads=reads, constructed=constructed)


def test_disabled_feature_never_allocates_driver_camera(setup_overlay):
  s = setup_overlay
  for _ in range(20):
    s.host._render_cropped_dcam('rect')
  assert not s.constructed
  assert len(s.reads) == 3  # Cached settings, no per-frame Params reads.


@pytest.mark.parametrize('compact', [False, True])
def test_selected_triggers_render_without_changing_forward_camera(setup_overlay, compact):
  s = setup_overlay
  s.host._dcam_compact = compact
  s.values.update(BPCroppedDcam=True, BPCroppedDcamTurnSignal=False)
  s.host._render_cropped_dcam('rect')
  assert len(s.constructed) == 1
  assert s.constructed[0].calls[-1][0][1:3] == (False, True)
  s.values['BPCroppedDcam'] = False
  s.clock[0] += 1.1
  s.host._render_cropped_dcam('rect')
  assert s.constructed[0].suspended == 1
  s.host._render_cropped_dcam('rect')
  assert s.constructed[0].suspended == 1


def test_no_trigger_no_client_and_stale_vehicle_state_suspends(setup_overlay):
  s = setup_overlay
  s.values.update(BPCroppedDcam=True, BPCroppedDcamTurnSignal=False, BPCroppedDcamBSM=False)
  s.host._render_cropped_dcam('rect')
  assert not s.constructed
  s.values['BPCroppedDcamBSM'] = True
  s.clock[0] += 1.1
  s.ui.sm.recv_time['carState'] = s.clock[0]
  s.host._render_cropped_dcam('rect')
  assert len(s.constructed) == 1
  s.clock[0] += .6
  s.host._render_cropped_dcam('rect')
  assert s.constructed[0].suspended == 1


def test_optional_shader_failure_does_not_break_road_view(setup_overlay, monkeypatch):
  s = setup_overlay
  attempts = []

  def unavailable():
    attempts.append(True)
    raise RuntimeError('shader unavailable')

  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.ui.bp.onroad.cropped_dcam_view',
                      NS(CroppedDcamViewBP=unavailable, MiciCroppedDcamViewBP=unavailable))
  s.values['BPCroppedDcam'] = True
  s.host._render_cropped_dcam('rect')
  s.host._render_cropped_dcam('rect')
  assert attempts == [True]
  assert s.host._cropped_dcam is None


def test_registered_defaults_and_telemetry():
  from openpilot.common.params import Params
  from opendbc.car import structs
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  params = Params()
  assert params.get('BPCroppedDcam', return_default=True) is False
  assert params.get('BPCroppedDcamTurnSignal', return_default=True) is True
  assert params.get('BPCroppedDcamBSM', return_default=True) is True
  state = structs.ControllerStateBP(bmsExperimentalDcam=True, bmsDcamTurnSignal=False, bmsDcamBsm=True)
  message = convert_to_capnp(state)
  assert message.bmsExperimentalDcam and not message.bmsDcamTurnSignal and message.bmsDcamBsm
