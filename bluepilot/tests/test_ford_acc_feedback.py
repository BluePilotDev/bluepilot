from types import SimpleNamespace as NS

import pytest
from opendbc.car import Bus
import cereal.messaging as messaging

from openpilot.bluepilot.selfdrive.car.ford_acc_feedback import ford_acc_feedback
from openpilot.bluepilot.selfdrive.car.bp_card_publisher import publish_car_state_bp
from openpilot.selfdrive.ui.bp.lib.ford_acc_path import FordAccPathFeedback


NOW = 10_000_000_000


def interface(mode=1, braking=False):
  cp = NS(vl={'EngBrakeData': {'CcMde_D_Actl': mode, 'CcOvrrdActv_B_Actl': 0},
              'DesiredTorqBrk': {'AccBrkActv_B_Actl': braking}},
          ts_nanos={'EngBrakeData': {'CcMde_D_Actl': NOW}, 'DesiredTorqBrk': {'AccBrkActv_B_Actl': NOW}})
  camera = NS(vl={'ACCDATA': {'AccPrpl_A_Rq': 1.5 if mode == 2 else -5,
                            'AccBrkTot_A_Rq': -2.0 if braking or mode == 3 else 0,
                            'AccBrkDecel_B_Rq': braking}},
              ts_nanos={'ACCDATA': {'AccPrpl_A_Rq': NOW}})
  cp.ts_nanos['EngBrakeData']['CcOvrrdActv_B_Actl'] = NOW
  cs = NS(cruiseState=NS(enabled=True), gasPressed=False, brakePressed=False)
  return NS(CP=NS(brand='ford', openpilotLongitudinalControl=False),
            CS=NS(out=cs, car_state_bp_msg=messaging.new_message('carStateBP')), can_parsers={Bus.pt: cp, Bus.cam: camera})


@pytest.mark.parametrize('mode,braking,expected', [(1, False, 0), (2, False, 1), (3, False, -1), (2, True, -1)])
def test_ford_reported_modes_and_brake_priority(mode, braking, expected):
  assert ford_acc_feedback(interface(mode, braking), True, NOW) == (True, expected)


@pytest.mark.parametrize('condition', ['op_long', 'other_car', 'invalid', 'gas', 'pedal', 'disabled', 'override', 'stale', 'missing'])
def test_no_feedback_without_fresh_unoverridden_stock_acc(condition):
  ci = interface(2)
  if condition == 'op_long':
    ci.CP.openpilotLongitudinalControl = True
  if condition == 'other_car':
    ci.CP.brand = 'toyota'
  if condition == 'gas':
    ci.CS.out.gasPressed = True
  if condition == 'pedal':
    ci.CS.out.brakePressed = True
  if condition == 'disabled':
    ci.CS.out.cruiseState.enabled = False
  if condition == 'override':
    ci.can_parsers[Bus.pt].vl['EngBrakeData']['CcOvrrdActv_B_Actl'] = 1
  if condition == 'stale':
    ci.can_parsers[Bus.pt].ts_nanos['EngBrakeData']['CcOvrrdActv_B_Actl'] -= 600_000_000
  if condition == 'missing':
    ci.can_parsers.clear()
  assert ford_acc_feedback(ci, condition != 'invalid', NOW) == (False, 0)


def test_real_message_publication_preserves_other_fields(monkeypatch):
  ci = interface(2)
  ci.CS.car_state_bp_msg.carStateBP.brakeLightStatus.dataAvailable = True
  monkeypatch.setattr('openpilot.bluepilot.selfdrive.car.bp_card_publisher.time.monotonic_ns', lambda: NOW)
  sent = []
  publish_car_state_bp(ci, NS(send=lambda name, msg: sent.append((name, msg.to_bytes()))), True)
  with messaging.log.Event.from_bytes(sent[0][1]) as msg:
    assert msg.valid and msg.carStateBP.fordAcc.available
    assert msg.carStateBP.fordAcc.feedback == 1
    assert msg.carStateBP.brakeLightStatus.dataAvailable


class Messages(dict):
  healthy = True

  def all_checks(self, services):
    return self.healthy


def messages(value=1):
  return Messages(carState=interface().CS.out, carStateBP=NS(fordAcc=NS(available=True, feedback=value)), carControl=NS(longActive=False))


def test_smooth_transition_and_immediate_safety_gating(monkeypatch):
  clock = [0.0]
  monkeypatch.setattr('openpilot.selfdrive.ui.bp.lib.ford_acc_path.time.monotonic', lambda: clock[0])
  feedback = FordAccPathFeedback()
  sm = messages()
  clock[0] += .1
  first = feedback.update(True, sm, False)
  assert 0 < first < 1
  sm['carStateBP'].fordAcc.feedback = -1
  clock[0] += .1
  assert -1 < feedback.update(True, sm, False) < first
  sm['carControl'].longActive = True
  assert feedback.update(True, sm, False) == 0
  sm['carControl'].longActive = False
  assert feedback.update(True, sm, True) == 0
  assert feedback.update(False, sm, False) == 0
  sm.healthy = False
  assert feedback.update(True, sm, False) == 0


def test_nonfinite_data_never_reaches_shader():
  assert FordAccPathFeedback().update(True, messages(float('nan')), False) == 0


def test_real_ford_dbc_decode():
  from opendbc.can import CANPacker, CANParser
  packer = CANPacker('ford_lincoln_base_pt')
  parser = CANParser('ford_lincoln_base_pt', [('EngBrakeData', 10), ('DesiredTorqBrk', 50)], 0)
  parser.update([[NOW, [packer.make_can_msg('EngBrakeData', 0, {'CcMde_D_Actl': 2}),
                       packer.make_can_msg('DesiredTorqBrk', 0, {'AccBrkActv_B_Actl': 1})]]])
  ci = interface()
  ci.can_parsers[Bus.pt] = parser
  camera = CANParser('ford_lincoln_base_pt', [('ACCDATA', 50)], 2)
  camera.update([[NOW, [packer.make_can_msg('ACCDATA', 2,
    {'AccPrpl_A_Rq': 1.5, 'AccBrkTot_A_Rq': -2.0, 'AccBrkDecel_B_Rq': 1})]]])
  ci.can_parsers[Bus.cam] = camera
  assert ford_acc_feedback(ci, True, NOW) == (True, -1.0)


def test_overlay_preserves_base_and_handles_startup_without_carparams(monkeypatch):
  import sys
  import numpy as np
  from openpilot.selfdrive.ui.bp.lib.ford_acc_path import draw_ford_acc_feedback
  calls = []
  ui = NS(CP=None)
  monkeypatch.setitem(sys.modules, 'pyray', NS(Color=lambda *rgba: rgba))
  monkeypatch.setitem(sys.modules, 'openpilot.selfdrive.ui.ui_state', NS(ui_state=ui))
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.shader_polygon',
                      NS(Gradient=lambda **kwargs: kwargs, draw_polygon=lambda *args, **kwargs: calls.append((args, kwargs))))
  renderer = NS(_ford_acc_path_enabled=True, _rad_racer=False, _rect=object())
  points = np.array([[0, 0], [1, 1], [2, 0]], dtype=np.float32)
  original = points.copy()
  draw_ford_acc_feedback(renderer, messages(), points)
  assert not calls  # No CarParams at startup must be harmless.
  ui.CP = NS(openpilotLongitudinalControl=False)
  renderer._ford_acc_feedback.last_time -= .1
  draw_ford_acc_feedback(renderer, messages(-1), points)
  assert len(calls) == 1
  assert calls[0][1]['gradient']['colors'][0][:3] == (235, 85, 40)
  assert 0 < calls[0][1]['gradient']['colors'][0][3] <= 155
  np.testing.assert_array_equal(points, original)


def test_registered_param_defaults_off_and_can_be_enabled():
  from openpilot.common.params import Params
  params = Params()
  assert not params.get_bool('BPFordACCPathFeedback')
  params.put_bool('BPFordACCPathFeedback', True, block=True)
  assert params.get_bool('BPFordACCPathFeedback')


@pytest.mark.parametrize('propulsion,braking,brake_request,expected', [
  (-5.0, 0.0, False, 0.0),  # inactive propulsion sentinel is not braking
  (0.05, -0.05, False, 0.0),
  (0.75, 0.0, False, 0.5),
  (1.5, -1.0, True, -0.5),
  (-5.0, 0.0, True, -0.35),
])
def test_actual_requests_work_even_when_generic_cruise_mode_is_keeping_speed(propulsion, braking, brake_request, expected):
  ci = interface(mode=1)
  ci.can_parsers[Bus.cam].vl['ACCDATA'].update(
    AccPrpl_A_Rq=propulsion, AccBrkTot_A_Rq=braking, AccBrkDecel_B_Rq=brake_request,
  )
  assert ford_acc_feedback(ci, True, NOW) == (True, expected)


def test_stale_camera_request_is_rejected():
  ci = interface(2)
  ci.can_parsers[Bus.cam].ts_nanos['ACCDATA']['AccPrpl_A_Rq'] -= 600_000_000
  assert ford_acc_feedback(ci, True, NOW) == (False, 0)
