"""
JetLink on BluePilot: JetLink's fork kit (jetlink.openpilot.testing) over the adapter and both modelds, wrapped for
BluePilot's unittest runner (tools/test_runner.py), and BluePilot's own pieces.

Runs with jetlink_repo present (the kit's checks) and absent (the fallback): the kit's tests skip without a checkout,
and the guard's tests below hide or break JetLink in-process, so both cases run either way. CI runs the module twice,
with the submodule and without it (.github/workflows/bp-jetlink.yml).

Every test runs under a params store of its own (PARAMS_ROOT), and with JetLink present JetLink's guard records any
reach for the machine's own params, /dev/shm records, sysfs or sudo.
"""
import importlib
import math
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

from openpilot.common.basedir import BASEDIR

# on a PC nothing links jetlink in (launch_chffrplus.sh does on a comma): the submodule's checkout, when it has one
_JETLINK_REPO = Path(BASEDIR) / 'jetlink_repo'
if (_JETLINK_REPO / 'jetlink' / '__init__.py').is_file() and str(_JETLINK_REPO) not in sys.path:
  sys.path.append(str(_JETLINK_REPO))

from openpilot.bluepilot import jetlink_adapter

try:
  import jetlink.openpilot.testing as kit
  HAVE_JETLINK = getattr(sys.modules.get('jetlink'), '__file__', None) is not None
except ImportError:
  kit = None
  HAVE_JETLINK = False

GUARD = kit.install_guard() if HAVE_JETLINK else None

MODELDS = ('openpilot/selfdrive/modeld/modeld.py', 'openpilot/sunnypilot/modeld_v2/modeld.py')
# every hook BluePilot's lines call (modelds, manager, selfdrived, hardwared)
HOOKS_CALLED = ('should_run', 'reason', 'prepare', 'attach', 'handed_over', 'state', 'runtime', 'shutdown_step',
                'swap_events')
# what each answers with the link off, for the cases without JetLink's own Off to compare to
_MODEL = object()
LINK_OFF = {'should_run': ((False, None, None), False), 'status': ((), None), 'prepare': ((False,), False),
            'handed_over': ((_MODEL,), False), 'state': ((_MODEL,), 'none'), 'runtime': ((), None),
            'model_state': (('f' * 40,), None), 'lists_big_models': ((), False), 'big_catalog': ((21,), None),
            'listed_ref': (('f' * 40,), None), 'swap_events': ((0.01,), None)}


class ParamsStoreTest(unittest.TestCase):
  """A params store of the test's own, so link_on() and Params never reach the machine's."""

  def setUp(self):
    self._tmp = tempfile.TemporaryDirectory()
    self._env = mock.patch.dict(os.environ, {'PARAMS_ROOT': self._tmp.name, 'OPENPILOT_PREFIX': 'd'})
    self._env.start()
    self.store = Path(self._tmp.name) / 'd'
    self.store.mkdir(parents=True, exist_ok=True)
    if GUARD is not None:
      GUARD.start()

  def tearDown(self):
    touched = GUARD.stop() if GUARD is not None else []
    self._env.stop()
    self._tmp.cleanup()
    self.assertEqual(touched, [])

  def set_link(self, value):
    path = self.store / jetlink_adapter.KEYS.link
    if value is None:
      path.unlink(missing_ok=True)
    else:
      path.write_text(str(value))


# -- JetLink's kit ---------------------------------------------------------------

@unittest.skipUnless(HAVE_JETLINK, "no jetlink_repo checkout")
class TestJetlinkKit(ParamsStoreTest):
  def test_adapter(self):
    heavy = ('openpilot.common.params', 'openpilot.common.swaglog', 'openpilot.cereal')
    self.assertEqual(kit.check_adapter(jetlink_adapter, heavy=heavy), [])

  def test_modeld_seams(self):
    for modeld in MODELDS:
      with self.subTest(modeld=modeld):
        self.assertEqual(kit.check_seam(Path(BASEDIR) / modeld), [])

  def test_fallback_link_off(self):
    self.set_link(0)
    self.assertEqual(kit.check_fallback(jetlink_adapter._Off('jetlink is missing'), HOOKS_CALLED,
                                        link_on=jetlink_adapter.link_on), [])

  def test_fallback_link_on(self):
    self.set_link(1)
    self.assertEqual(kit.check_fallback(jetlink_adapter._Off('jetlink is missing'), HOOKS_CALLED,
                                        link_on=jetlink_adapter.link_on), [])

  def test_hooks_bind_this_api(self):
    hooks = jetlink_adapter._guard()
    self.assertEqual(type(hooks).__name__, 'Hooks')
    self.assertEqual(jetlink_adapter.API, importlib.import_module('jetlink.openpilot').API)

  def test_hooks_link_off(self):
    # with the setting off every hook answers as the link off, and nothing tells the driver why
    self.set_link(0)
    hooks = jetlink_adapter._guard()
    self.assertFalse(hooks.should_run(False, None, None))
    self.assertFalse(hooks.prepare(False))
    self.assertIs(hooks.attach(_MODEL, object(), 1928, 1208), _MODEL)
    self.assertIsNone(hooks.reason())
    # no power-off unasked; a wanted one goes ahead at once with no host to ask, and stays a yes
    self.assertFalse(hooks.shutdown_step(False))
    self.assertTrue(hooks.shutdown_step(True, 'test'))
    self.assertTrue(hooks.shutdown_step(False))

  def test_chestnut_never_prepares(self):
    self.set_link(1)
    self.assertFalse(jetlink_adapter._guard().prepare(True))

  def test_adapter_keys(self):
    keys = jetlink_adapter.adapter().keys
    self.assertEqual(keys.link, 'JetlinkLink')
    self.assertEqual(keys.progress, 'JetlinkProgress')
    self.assertEqual(keys.big_model, 'ModelManager_ActiveBundleChestnut')
    self.assertEqual(keys.catalog, 'ModelManager_ModelsCache_Chestnut')
    self.assertEqual(keys.select, 'ModelManager_DownloadRef')

  def test_engaged_mads_paused_counts(self):
    op = jetlink_adapter.adapter()
    sm = {'carControl': types.SimpleNamespace(enabled=False), 'carControlSP': types.SimpleNamespace(mads=types.SimpleNamespace(enabled=True))}
    self.assertTrue(op.engaged(sm))
    sm['carControlSP'].mads.enabled = False
    self.assertFalse(op.engaged(sm))

  def test_unusable_jetlink_tells_who_turned_it_on(self):
    # another API, and a package too old to have the hooks: the link off, with the reason as the offroad alert
    for patch in (mock.patch.object(jetlink_adapter, 'API', 99),
                  mock.patch.dict(sys.modules, {'jetlink.openpilot.hooks': None})):
      with self.subTest(patch=patch), patch:
        off = jetlink_adapter._guard()
        self.assertIsInstance(off, jetlink_adapter._Off)
        self.set_link(0)
        self.assertIsNone(off.reason())
        self.set_link(1)
        self.assertIsInstance(off.reason(), str)


# -- the guard without a usable JetLink ----------------------------------------------

class TestJetlinkAbsent(ParamsStoreTest):
  def _hidden(self):
    """jetlink as missing: no checkout on the path, whatever this process loaded."""
    gone = {name: None for name in list(sys.modules) if name == 'jetlink' or name.startswith('jetlink.')}
    return mock.patch.dict(sys.modules, {**gone, 'jetlink': None})

  def _check_off(self, off, why: bool):
    for name, (args, answer) in LINK_OFF.items():
      with self.subTest(hook=name):
        self.assertEqual(getattr(off, name)(*args), answer)
    self.assertIs(off.attach(_MODEL, object(), 1928, 1208), _MODEL)
    self.assertTrue(off.shutdown_step(True, 'test'))
    self.assertFalse(off.shutdown_step(False))
    self.set_link(0)
    self.assertIsNone(off.reason())
    self.set_link(1)
    self.assertEqual(off.reason() is not None, why)

  def test_missing_checkout(self):
    with self._hidden():
      off = jetlink_adapter._guard()
    self.assertIsInstance(off, jetlink_adapter._Off)
    # no checkout: the link does not exist on this device, and nobody is told so
    self._check_off(off, why=False)

  def test_empty_submodule_dir(self):
    # an uninitialized jetlink_repo leaves an empty jetlink/ link target, which imports as a namespace package
    with tempfile.TemporaryDirectory() as tmp, self._hidden():
      (Path(tmp) / 'jetlink').mkdir()
      del sys.modules['jetlink']
      with mock.patch.object(sys, 'path', [tmp]):
        off = jetlink_adapter._guard()
    self.assertIsInstance(off, jetlink_adapter._Off)
    self._check_off(off, why=False)

  def test_hooks_by_name_follow_the_guard(self):
    with mock.patch.object(jetlink_adapter, '_hooks', jetlink_adapter._Off()):
      self.assertFalse(jetlink_adapter.should_run(False, None, None))
      self.assertFalse(jetlink_adapter.prepare(False))
      self.assertIsNone(jetlink_adapter.swap_events(0.01))
      self.assertTrue(jetlink_adapter.shutdown_step(True, 'test'))

  def test_link_on_reads_the_file(self):
    for value, on in ((None, False), ('', False), ('0', False), ('1', True), ('2', True), ('3', False), ('x', False)):
      with self.subTest(value=value):
        self.set_link(value)
        self.assertEqual(jetlink_adapter.link_on(), on)

  def test_chestnut_cable_only_with_the_link_off(self):
    self.set_link(0)
    self.assertTrue(jetlink_adapter.chestnut_cable(True))
    self.assertFalse(jetlink_adapter.chestnut_cable(False))
    self.set_link(1)
    self.assertFalse(jetlink_adapter.chestnut_cable(True))

  def test_top_level_is_stdlib(self):
    # manager and the resident owner import the adapter: its top level imports the standard library alone
    import ast
    tree = ast.parse(Path(jetlink_adapter.__file__).read_text())
    for node in tree.body:
      if isinstance(node, (ast.Import, ast.ImportFrom)):
        names = [a.name for a in node.names] if isinstance(node, ast.Import) else [node.module]
        for name in names:
          self.assertIn(name.split('.')[0], sys.stdlib_module_names | {'__future__'}, name)


# -- the big model's action path ------------------------------------------------------

class TestWithBlendful(unittest.TestCase):
  """with_blendful() must give the big model the curvature law modeld_v2 gives the small model: Blendful's
  correction on the raw curvature, before smoothing, above the minimum speed."""
  DT, MIN_SPEED = 0.05, 0.3

  def _stock(self, tau):
    # stock modeld's shape: a raw curvature from the output, smoothed toward the previous one
    def action(model_output, prev_action, lat_action_t, long_action_t, v_ego):
      raw = model_output['raw']
      if v_ego > self.MIN_SPEED:
        alpha = 1 - math.exp(-self.DT / tau) if tau > 0 else 1.
        curvature = alpha * raw + (1 - alpha) * prev_action.desiredCurvature
      else:
        curvature = prev_action.desiredCurvature
      return types.SimpleNamespace(desiredCurvature=curvature, desiredAcceleration=0., shouldStop=False)
    return action

  def _reference(self, tau, raw, prev, v_ego, blendful):
    # modeld_v2's ModelState.get_action_from_model: the correction first, then the smoothing
    raw = raw + blendful(raw)
    if v_ego > self.MIN_SPEED:
      alpha = 1 - math.exp(-self.DT / tau) if tau > 0 else 1.
      return alpha * raw + (1 - alpha) * prev
    return prev

  def test_matches_modeld_v2(self):
    for tau in (0., 0.1, 0.3):
      for v_ego in (0., 0.2, 5., 30.):
        with self.subTest(tau=tau, v_ego=v_ego):
          seen = []

          def blendful(curvature, seen=seen):
            seen.append(curvature)
            return 0.002 - 0.5 * curvature
          action = jetlink_adapter.with_blendful(self._stock(tau), tau, self.MIN_SPEED, self.DT)
          prev = types.SimpleNamespace(desiredCurvature=0.004, desiredAcceleration=0.)
          got = action({'raw': 0.01}, prev, 0.2, 0.3, v_ego, blendful)
          want = self._reference(tau, 0.01, 0.004, v_ego, lambda c: 0.002 - 0.5 * c)
          self.assertAlmostEqual(got.desiredCurvature, want, places=9)
          # Blendful runs every frame, as on the small model, and above the speed sees the raw curvature
          self.assertEqual(len(seen), 1)
          if v_ego > self.MIN_SPEED:
            self.assertAlmostEqual(seen[0], 0.01, places=9)

  def test_without_callback_is_stock(self):
    stock = self._stock(0.)
    prev = types.SimpleNamespace(desiredCurvature=0.004, desiredAcceleration=0.)
    action = jetlink_adapter.with_blendful(stock, 0., self.MIN_SPEED, self.DT)
    self.assertEqual(action({'raw': 0.01}, prev, 0.2, 0.3, 10.).desiredCurvature,
                     stock({'raw': 0.01}, prev, 0.2, 0.3, 10.).desiredCurvature)

  def test_bluepilot_stock_modeld_constants(self):
    # the adapter wraps stock modeld's own function and constants; LAT_SMOOTH_SECONDS 0 today
    from openpilot.selfdrive.modeld import modeld
    action = jetlink_adapter.with_blendful(modeld.get_action_from_model, modeld.LAT_SMOOTH_SECONDS,
                                           modeld.MIN_LAT_CONTROL_SPEED, 0.05)
    self.assertTrue(callable(action))


# -- manager, selfdrived, hardwared, settings ---------------------------------------------

class TestRestartingProcess(unittest.TestCase):
  def _proc(self):
    from openpilot.bluepilot.system.manager.restarting_process import RestartingPythonProcess
    proc = RestartingPythonProcess('jetlinkd', 'openpilot.bluepilot.jetlink_adapter', lambda *a: True)
    proc.clock = 100.
    proc.now = lambda: proc.clock
    started = []

    def start():
      proc.proc = types.SimpleNamespace(exitcode=None)
      started.append(proc.clock)
    proc._started = started
    proc.stop = lambda *a, **k: setattr(proc, 'proc', None)
    return proc, start

  def test_restarts_and_backs_off(self):
    proc, fake_start = self._proc()
    with mock.patch('openpilot.system.manager.process.PythonProcess.start', lambda self: fake_start()):
      proc.start()
      self.assertEqual(proc._started, [100.])
      # dies within QUICK_DEATH: waits BACKOFF before the next start
      proc.proc.exitcode = 1
      proc.clock = 102.
      proc.start()
      self.assertEqual(proc._started, [100.])
      proc.clock = 102. + proc.BACKOFF
      proc.start()
      self.assertEqual(proc._started, [100., 102. + proc.BACKOFF])
      # ran long: started again at once
      proc.proc.exitcode = 1
      proc.clock += proc.QUICK_DEATH + 1
      proc.start()
      self.assertEqual(len(proc._started), 3)


class TestJetlinkEvents(unittest.TestCase):
  def test_flags_to_events(self):
    from openpilot.cereal import log, custom
    from openpilot.selfdrive.selfdrived.events import Events
    from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
    from openpilot.bluepilot.selfdrive.selfdrived.events_bp import EventsBP, EventNameBP
    from openpilot.bluepilot.selfdrive.selfdrived import jetlink_events

    flags = types.SimpleNamespace(offer=False, link_lost=False, no_entry=False, chime=False, settling=False)
    tracker = types.SimpleNamespace(update=lambda fresh, big, in_control: flags, settling=False)
    with mock.patch.object(jetlink_events.jetlink_adapter, 'swap_events', lambda tick: tracker):
      je = jetlink_events.JetlinkEvents()
    model = types.SimpleNamespace(big=False)
    sm = mock.MagicMock()
    sm.alive = {'modelV2': True}
    sm.valid = {'modelV2': True}
    sm.__getitem__.return_value = model

    def run(**set_flags):
      vars(flags).update(dict.fromkeys(vars(flags), False), **set_flags)
      events, events_sp, events_bp = Events(), EventsSP(), EventsBP()
      events_sp.add(custom.OnroadEventSP.EventName.bigModelReady)
      je.update(sm, True, events, events_sp, events_bp)
      return events, events_sp, events_bp

    events, events_sp, events_bp = run(offer=True)
    self.assertTrue(events_bp.has(EventNameBP.bigModelAvailable))
    events, events_sp, events_bp = run(link_lost=True)
    self.assertTrue(events_bp.has(EventNameBP.bigModelLinkLost))
    events, events_sp, events_bp = run(no_entry=True)
    self.assertTrue(events.has(log.OnroadEvent.EventName.bigModelLoading))
    self.assertFalse(events_sp.has(custom.OnroadEventSP.EventName.bigModelReady))
    self.assertTrue(je.no_entry)
    events, events_sp, events_bp = run(chime=True)
    self.assertTrue(events_sp.has(custom.OnroadEventSP.EventName.bigModelReady))
    self.assertFalse(events.has(log.OnroadEvent.EventName.bigModelLoading))
    self.assertFalse(je.no_entry)

  def test_without_jetlink_no_events(self):
    from openpilot.bluepilot.selfdrive.selfdrived import jetlink_events
    with mock.patch.object(jetlink_events.jetlink_adapter, 'swap_events', lambda tick: None):
      je = jetlink_events.JetlinkEvents()
    self.assertFalse(je.settling)
    je.update(None, True, None, None, None)
    self.assertFalse(je.no_entry)
    self.assertIn('jetlinkd', je.OPTIONAL_PROCESSES)

  def test_alerts_defined(self):
    from openpilot.cereal import custom
    from openpilot.bluepilot.selfdrive.selfdrived.events_bp import EVENTS_BP
    self.assertEqual(set(EVENTS_BP), set(custom.OnroadEventBP.EventName.schema.enumerants.values()))


class TestJetlinkShutdown(unittest.TestCase):
  def test_cap(self):
    from openpilot.bluepilot.system.hardware import jetlink_shutdown as js
    clock = [0.]
    shutdown = js.JetlinkShutdown(clock=lambda: clock[0])
    with mock.patch.object(js.jetlink_adapter, 'shutdown_step', lambda wanted, reason='': False):
      self.assertFalse(shutdown.step(False))
      self.assertFalse(shutdown.waiting)
      self.assertFalse(shutdown.step(True))
      self.assertTrue(shutdown.waiting)
      clock[0] = js.JETLINK_SHUTDOWN_CAP_S
      self.assertFalse(shutdown.step(False))
      clock[0] = js.JETLINK_SHUTDOWN_CAP_S + 0.5
      self.assertTrue(shutdown.step(False))
    self.assertEqual(js.JETLINK_SHUTDOWN_CAP_S, 30.)

  def test_link_off_powers_off_at_once(self):
    from openpilot.bluepilot.system.hardware import jetlink_shutdown as js
    with mock.patch.object(js.jetlink_adapter, '_hooks', jetlink_adapter._Off()):
      self.assertTrue(js.JetlinkShutdown().step(True, 'test'))

  def test_alert(self):
    from openpilot.bluepilot.system.hardware import jetlink_shutdown as js
    calls = []
    for reason in (None, 'no gadget'):
      with mock.patch.object(js.jetlink_adapter, 'reason', lambda reason=reason: reason):
        js.JetlinkShutdown.update_alert(lambda *a, **k: calls.append((a, k)))
    self.assertEqual(calls, [(('Offroad_JetlinkUnavailable', False), {'extra_text': None}),
                             (('Offroad_JetlinkUnavailable', True), {'extra_text': 'no gadget'})])


class TestJetlinkSettings(ParamsStoreTest):
  def test_offroad_only_and_usb_only(self):
    from openpilot.common.params import Params
    from openpilot.selfdrive.ui.bp.lib import jetlink_settings as js

    class SyncParams(Params):
      # the UI's put is asynchronous; the test reads straight back
      def put(self, key, dat, block=False):
        super().put(key, dat, block=True)
    params = SyncParams()
    self.assertEqual(js.selected_index(params), 0)
    self.assertFalse(js.write_link(params, 1, offroad=False))
    self.assertEqual(js.read_link(params), 0)
    self.assertTrue(js.write_link(params, 1, offroad=True))
    self.assertEqual(js.read_link(params), 1)
    self.assertTrue(jetlink_adapter.link_on())
    # no iOS button in phase 1
    self.assertFalse(js.write_link(params, 2, offroad=True))
    params.put(js.LINK_PARAM, 2)
    self.assertEqual(js.selected_index(params), -1)
    self.assertTrue(js.write_link(params, 0, offroad=True))
    self.assertFalse(jetlink_adapter.link_on())


if __name__ == '__main__':
  unittest.main()
