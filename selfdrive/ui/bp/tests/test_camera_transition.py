from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.bp.lib import camera_transition as transition


def test_one_second_fade_has_opaque_midpoint_and_clear_end():
  fade = transition.CameraFade()
  fade.started = 10
  assert fade.alpha(10) == 0
  assert fade.alpha(10.25) == pytest.approx(.5)
  assert fade.alpha(10.5) == 1
  assert fade.alpha(10.75) == pytest.approx(.5)
  assert fade.alpha(11) == 0
  assert fade.started is None


@pytest.fixture
def view(monkeypatch):
  clock = [10.0]
  monkeypatch.setattr(transition, 'monotonic', lambda: clock[0])

  class Native:
    def __init__(self):
      self.frame = 'old'
      self._bp_camera_angle_auto = True
      self._switching = True
      self._target_stream_type = 'wide'
      self._target_client = NS(is_connected=lambda: True, recv=lambda **kwargs: None)
      self.commits = 0
      self.delegated = 0

    def _complete_switch(self):
      self.commits += 1
      self._switching = False
      self._target_client = None

    def _handle_switch(self):
      self.delegated += 1

  class View(transition.AutomaticCameraFadeMixin, Native):
    pass

  return View(), clock


def test_wait_for_destination_then_switch_only_at_opaque_midpoint(view):
  v, clock = view
  v._handle_switch()
  assert v._camera_fade.started is None and v.frame == 'old'
  v._target_client.recv = lambda **kwargs: 'new'
  v._handle_switch()
  clock[0] += .25
  v._handle_switch()
  assert v.frame == 'old' and v.commits == 0
  clock[0] += .25
  v._handle_switch()
  assert v.frame == 'new' and v.commits == 1
  assert v._camera_fade.committed
  assert v._camera_fade.alpha(clock[0]) == 1
  assert v._camera_fade.alpha(clock[0]+.5) == 0


def test_stalled_camera_aborts_without_switching(view):
  v, clock = view
  v._target_client.recv = lambda **kwargs: 'new'
  v._handle_switch()
  v._target_client.recv = lambda **kwargs: None
  clock[0] += .3
  v._handle_switch()
  assert v.frame == 'old' and v.commits == 0
  assert v._camera_fade.started is None


def test_disabled_auto_and_startup_preserve_native_switch(view):
  v, _ = view
  v._bp_camera_angle_auto = False
  v._handle_switch()
  assert v.delegated == 1
  v._bp_camera_angle_auto = True
  v.frame = None
  v._handle_switch()
  assert v.delegated == 2


def test_retarget_restarts_fade(view):
  v, clock = view
  v._target_client.recv = lambda **kwargs: 'new'
  v._handle_switch()
  clock[0] += .4
  v._target_stream_type = 'road'
  v._handle_switch()
  assert v.commits == 0 and v._camera_fade.started == clock[0]


@pytest.mark.parametrize('device', ['onroad', 'mici/onroad'])
def test_fade_is_after_scene_geometry_and_before_dcam_and_hud(device):
  import ast
  from pathlib import Path
  from openpilot.common.basedir import BASEDIR

  source = (Path(BASEDIR) / f'selfdrive/ui/bp/{device}/augmented_road_view_bp.py').read_text()
  tree = ast.parse(source)
  render = next(n for cls in tree.body if isinstance(cls, ast.ClassDef) and cls.name.endswith("RoadViewBP") for n in cls.body
                if isinstance(n, ast.FunctionDef) and n.name == '_render')
  calls = [n for n in ast.walk(render) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
  fade = next(n.lineno for n in calls if n.func.attr == '_draw_camera_transition')
  dcam = next(n.lineno for n in calls if n.func.attr == '_render_cropped_dcam')
  model = next(n.lineno for n in calls if n.func.attr == 'render' and
               isinstance(n.func.value, ast.Attribute) and n.func.value.attr in ('model_renderer', '_model_renderer'))
  hud = next(n.lineno for n in calls if n.func.attr == 'render' and
             isinstance(n.func.value, ast.Attribute) and n.func.value.attr == '_hud_renderer')
  assert model < fade < dcam < hud
