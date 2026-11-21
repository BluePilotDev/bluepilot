from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.ui.bp.lib import camera_transition as transition


def test_one_second_snapshot_fade_starts_opaque_and_ends_clear():
  fade = transition.CameraFade()
  fade.started = 10
  assert fade.alpha(10) == 1
  assert fade.alpha(10.25) > .5
  assert fade.alpha(10.5) == pytest.approx(.5)
  assert fade.alpha(10.75) < .5
  assert fade.alpha(11) == 0


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
    def _clear_camera_transition(self):
      self._camera_snapshot = None
      self._camera_snapshot_size = None
      self._camera_fade.reset()

  return View(), clock


def test_wait_for_destination_and_outgoing_snapshot_before_switch(view):
  v, clock = view
  v._handle_switch()
  assert v._camera_fade.frame is None and v.frame == 'old'
  v._target_client.recv = lambda **kwargs: 'new'
  v._handle_switch()
  assert v._camera_fade.frame == 'new' and v.commits == 0
  assert v._camera_fade.started is None
  # End of the outgoing scene's render pass captures it before the next switch.
  v._camera_fade.captured = True
  clock[0] += 1/60
  v._handle_switch()
  assert v.frame == 'new' and v.commits == 1
  assert v._camera_fade.committed
  assert v._camera_fade.alpha(clock[0]) == 1
  assert v._camera_fade.alpha(clock[0]+1) == 0


def test_stalled_camera_discards_snapshot_without_switching(view):
  v, clock = view
  v._target_client.recv = lambda **kwargs: 'new'
  v._handle_switch()
  v._camera_fade.captured = True
  v._target_client.recv = lambda **kwargs: None
  clock[0] += .3
  v._handle_switch()
  assert v.frame == 'old' and v.commits == 0
  assert not v._camera_fade.captured


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
  assert v.commits == 0 and v._camera_fade.target == 'road'
  assert not v._camera_fade.captured and v._camera_fade.started is None


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


def test_gpu_capture_preserves_source_framebuffer_across_allocation(view, monkeypatch):
  import sys

  v, _ = view
  state = {'read': 7, 'draw': 7}
  copies = []

  def allocate(width, height):
    assert (width, height) == (536, 240)
    # Raylib allocation binds/unbinds framebuffers internally.
    state.update(read=0, draw=0)
    return NS(id=9, texture=NS(id=12))

  def bind(target, value):
    state['read' if target == 0x8CA8 else 'draw'] = value

  gpu = NS(get_render_width=lambda: 536, get_render_height=lambda: 240,
           rl_draw_render_batch_active=lambda: None,
           rl_get_active_framebuffer=lambda: state['draw'],
           load_render_texture=allocate, end_scissor_mode=lambda: None,
           rl_bind_framebuffer=bind,
           rl_blit_framebuffer=lambda *args: copies.append((state.copy(), args)),
           begin_scissor_mode=lambda *args: None)
  monkeypatch.setitem(sys.modules, 'pyray', gpu)
  monkeypatch.setitem(sys.modules, 'openpilot.system.ui.lib.application', NS(gui_app=NS(width=536, height=240)))
  assert v._capture_camera_scene(NS(x=0, y=0, width=476, height=240))
  assert copies[0][0] == {'read': 7, 'draw': 9}
  assert state == {'read': 7, 'draw': 7}
  assert v._camera_snapshot.texture.id == 12


def test_snapshot_texture_released_when_transition_clears(monkeypatch):
  import sys

  unloaded = []
  restored = []
  monkeypatch.setitem(sys.modules, 'pyray', NS(
    unload_render_texture=unloaded.append, rl_draw_render_batch_active=lambda: None,
    rl_get_active_framebuffer=lambda: 7, rl_bind_framebuffer=lambda target, value: restored.append((target, value)),
  ))
  host = NS(_camera_snapshot='texture', _camera_snapshot_size=(536, 240), _camera_fade=transition.CameraFade())
  host._camera_fade.started = 1
  transition.AutomaticCameraFadeMixin._clear_camera_transition(host)
  assert unloaded == ['texture']
  assert restored == [(0x8CA8, 7), (0x8CA9, 7)]
  assert host._camera_snapshot is None and host._camera_fade.started is None
  transition.AutomaticCameraFadeMixin._clear_camera_transition(host)
  assert unloaded == ['texture']
