"""
JetLink phase 2 on BluePilot: the big-model list in sunnypilot's model manager (catalog.py, R9), the status both
settings menus show and the pickers' default big model (selfdrive/ui/bp/lib/jetlink_status.py, R8.2 and R8.3).

JetLink's hooks are patched where a test needs an answer, so the module runs with jetlink_repo present or absent;
the one test that reads JetLink's own PHASES skips without it. Each test has a params store of its own.
"""
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

from openpilot.common.basedir import BASEDIR

_JETLINK_REPO = Path(BASEDIR) / 'jetlink_repo'
if (_JETLINK_REPO / 'jetlink' / '__init__.py').is_file() and str(_JETLINK_REPO) not in sys.path:
  sys.path.append(str(_JETLINK_REPO))

from openpilot.bluepilot import jetlink_adapter
from openpilot.bluepilot.jetlink_adapter import catalog
from openpilot.selfdrive.ui.bp.lib import jetlink_status
from openpilot.sunnypilot.models.fetcher import ModelFetcher, ModelParser
from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS, REQUIRED_JSON_VERSION

try:
  from jetlink.openpilot.status import PHASES
except ImportError:
  PHASES = None

CACHE_KEY = "ModelManager_ModelsCache_Chestnut"


def bundle(ref: str, name: str, index: int = 1) -> dict:
  """A big-model bundle in the shape JetLink's big_catalog() lists one."""
  return {'ref': ref, 'display_name': name, 'short_name': name.lower(), 'index': index, 'is_big': True,
          'is_20hz': True, 'generation': 13, 'environment': 'release', 'runner': 1,
          'minimum_selector_version': str(REQUIRED_JSON_VERSION), 'build_time': '2026-11-01', 'overrides': {'folder': 'x'},
          'models': []}


def hooks(lists=True, link=True, big_catalog=None, listed=None):
  """jetlink_adapter's hooks the catalog asks, patched."""
  return mock.patch.multiple(jetlink_adapter, lists_big_models=lambda: lists, link_on=lambda: link,
                             big_catalog=lambda selector: big_catalog, listed_ref=lambda ref: listed)


class ParamsTest(unittest.TestCase):
  def setUp(self):
    self._tmp = tempfile.TemporaryDirectory()
    self._env = mock.patch.dict(os.environ, {'PARAMS_ROOT': self._tmp.name, 'OPENPILOT_PREFIX': 'd'})
    self._env.start()
    from openpilot.common.params import Params

    class SyncParams(Params):
      def put(self, key, dat, block=False):
        super().put(key, dat, block=True)
    self.params = SyncParams()

  def tearDown(self):
    self._env.stop()
    self._tmp.cleanup()


# -- the big-model list (R9) -----------------------------------------------------------

class TestCatalogGate(unittest.TestCase):
  def test_lists_jetlink(self):
    stamped = {'bundles': [], catalog.JETLINK_KEY: True}
    for lists, link, cached, want in ((False, True, stamped, False),   # a chestnut is fitted
                                      (True, False, {}, False),         # never turned on: no index fetched
                                      (True, False, None, False),
                                      (True, True, {}, True),           # turned on
                                      (True, False, stamped, True)):    # turned off again: the list stays
      with self.subTest(lists=lists, link=link, cached=cached), hooks(lists, link):
        self.assertEqual(catalog.lists_jetlink(cached), want)


class TestJetlinkModelFetcher(ParamsTest):
  def test_jetlinks_list_with_the_link_on(self):
    listed = {'bundles': [bundle('a' * 40, 'Big One')]}
    with hooks(big_catalog=listed):
      fetcher = catalog.JetlinkModelFetcher(self.params)
      bundles = fetcher.get_bundles_for_source('chestnut')
    self.assertEqual([b.ref for b in bundles], ['a' * 40])
    self.assertTrue(catalog.is_jetlinks(self.params.get(CACHE_KEY)))
    # sunnylink lists the comma's copy, not sunnypilot's chestnut catalog
    self.assertEqual(self.params.get("ModelManager_ActiveJson"), {"qcom": ModelFetcher.MODEL_URL})

  def test_sunnypilots_list_without_the_link(self):
    with hooks(link=False), mock.patch.object(ModelFetcher, '_fetch_and_cache_models', return_value=['sp']) as fetch:
      fetcher = catalog.JetlinkModelFetcher(self.params)
      self.assertEqual(fetcher.get_bundles_for_source('chestnut'), ['sp'])
    fetch.assert_called_once_with('chestnut')
    self.assertEqual(self.params.get("ModelManager_ActiveJson"),
                     {"qcom": ModelFetcher.MODEL_URL, "chestnut": ModelFetcher.MODEL_URL_CHESTNUT})

  def test_small_models_stay_sunnypilots(self):
    with hooks(), mock.patch.object(ModelFetcher, '_fetch_and_cache_models', return_value=['qcom']) as fetch:
      self.assertEqual(catalog.JetlinkModelFetcher(self.params).get_bundles_for_source('qcom'), ['qcom'])
    fetch.assert_called_once_with('qcom')

  def test_offline_keeps_the_cache(self):
    old = {'bundles': [bundle('b' * 40, 'Cached')], catalog.JETLINK_KEY: True}
    self.params.put(CACHE_KEY, old)
    with hooks(big_catalog=None):
      bundles = catalog.JetlinkModelFetcher(self.params).get_bundles_for_source('chestnut')
    self.assertEqual([b.ref for b in bundles], ['b' * 40])
    self.assertEqual(self.params.get(CACHE_KEY), old)

  def test_refetched_once_for_other_hardware(self):
    # a list of JetLink's cached, then a chestnut fitted: sunnypilot's list is fetched at once, not in an hour
    self.params.put(CACHE_KEY, {'bundles': [bundle('c' * 40, 'Jet')], catalog.JETLINK_KEY: True})
    self.params.put("ModelManager_LastSyncTime_Chestnut", 2 ** 62)  # not expired
    with hooks(lists=False), mock.patch.object(ModelFetcher, '_fetch_and_cache_models', return_value=None) as fetch:
      fetcher = catalog.JetlinkModelFetcher(self.params)
      fetcher.get_bundles_for_source('chestnut')
      fetcher.get_bundles_for_source('chestnut')
    self.assertEqual(fetch.call_count, 1)


class TestPickCarryOver(ParamsTest):
  def _pick(self, ref):
    picked = ModelParser.parse_models({'bundles': [bundle(ref, 'Picked')]})[0]
    self.params.put(ACTIVE_BUNDLE_KEYS['chestnut'], picked.to_dict())

  def _listed(self, *refs, jetlinks=True):
    data = {'bundles': [bundle(r, f'M{i}', i + 1) for i, r in enumerate(refs)]}
    if jetlinks:
      data[catalog.JETLINK_KEY] = True
    self.params.put(CACHE_KEY, data)
    return {'chestnut': ModelParser.parse_models(data), 'qcom': []}

  def test_carries_a_recompile_over(self):
    self._pick('a' * 40)
    models = self._listed('b' * 40)
    carry = catalog.PickCarryOver()
    with hooks(listed='b' * 40):
      self.assertEqual(carry.update(self.params, models), 'b' * 40)
      self.assertEqual(self.params.get(catalog.DOWNLOAD_REF), 'b' * 40)
      # once per ref
      self.params.remove(catalog.DOWNLOAD_REF)
      self.assertIsNone(carry.update(self.params, models))

  def test_leaves_it(self):
    cases = {
      'still listed': (lambda: self._listed('a' * 40), 'a' * 40),
      'not listed by JetLink': (lambda: self._listed('b' * 40), None),
      'listed ref not in the list': (lambda: self._listed('b' * 40), 'c' * 40),
      "sunnypilot's list": (lambda: self._listed('b' * 40, jetlinks=False), 'b' * 40),
    }
    for name, (models, listed) in cases.items():
      with self.subTest(name):
        self._pick('a' * 40)
        source_models = models()
        with hooks(listed=listed):
          self.assertIsNone(catalog.PickCarryOver().update(self.params, source_models))
        self.assertIsNone(self.params.get(catalog.DOWNLOAD_REF))

  def test_never_over_a_queued_download(self):
    self._pick('a' * 40)
    models = self._listed('b' * 40)
    self.params.put(catalog.DOWNLOAD_REF, 'q' * 40)
    with hooks(listed='b' * 40):
      self.assertIsNone(catalog.PickCarryOver().update(self.params, models))
    self.assertEqual(self.params.get(catalog.DOWNLOAD_REF), 'q' * 40)

  def test_no_pick(self):
    with hooks(listed='b' * 40):
      self.assertIsNone(catalog.PickCarryOver().update(self.params, self._listed('b' * 40)))


# -- the settings' status (R8.2) and the pickers' default (R8.3) ---------------------------

def status(**fields):
  base = {'phase': 'off', 'progress': None, 'mode': 'usb', 'port_state': None, 'transport': 'USB', 'reason': None,
          'enabled': False, 'active_model': None, 'model': None, 'holds_port': False, 'shown': False, 'default_model': None}
  return types.SimpleNamespace(**{**base, **fields})


class TestStatusReader(unittest.TestCase):
  def test_at_most_five_hz(self):
    clock = [0.]
    calls = []
    reader = jetlink_status.StatusReader(clock=lambda: clock[0])
    with mock.patch.object(jetlink_adapter, 'status', lambda: calls.append(1) or len(calls)):
      self.assertEqual(reader.get(), 1)
      clock[0] = 0.1
      self.assertEqual(reader.get(), 1)
      clock[0] = 0.2
      self.assertEqual(reader.get(), 2)

  def test_a_failure_is_nothing_to_show(self):
    def boom():
      raise RuntimeError("x")
    with mock.patch.object(jetlink_adapter, 'status', boom):
      self.assertIsNone(jetlink_status.StatusReader().get())

  def test_without_jetlink(self):
    with mock.patch.object(jetlink_adapter, '_hooks', jetlink_adapter._Off()):
      self.assertIsNone(jetlink_status.StatusReader().get())
    self.assertEqual(jetlink_status.chip(None), jetlink_status.NO_JETLINK)
    self.assertEqual(jetlink_status.detail_lines(None), [])
    self.assertFalse(jetlink_status.shown(None))


class TestStatusLines(unittest.TestCase):
  def test_chip(self):
    self.assertEqual(jetlink_status.chip(status(phase='active')), "Big model driving")
    self.assertEqual(jetlink_status.chip(status(phase='a phase from a newer jetlink')), jetlink_status.UNKNOWN_PHASE)
    preparing = status(phase='preparing', progress={'stage': 'build', 'frac': 0.42, 'msg': 'about 2 min left'})
    self.assertEqual(jetlink_status.chip(preparing), "Preparing 42%")

  @unittest.skipUnless(PHASES, "no jetlink_repo checkout")
  def test_every_phase_has_words(self):
    self.assertEqual(set(PHASES) - set(jetlink_status.PHASE_LABELS), set())

  def test_progress(self):
    self.assertIsNone(jetlink_status.progress(status(progress={'stage': 'ready', 'frac': 1.0})))
    self.assertIsNone(jetlink_status.progress(status(progress=None)))
    self.assertEqual(jetlink_status.progress(status(progress={'stage': 'upload', 'frac': 'x', 'msg': ''})), ('upload', 0.0, 'upload'))
    drops = {'stage': 'connect', 'frac': 0.0, 'msg': 'waiting for jetlink', 'drops': 4}
    self.assertEqual(jetlink_status.progress(status(progress=drops))[2], "waiting for jetlink, check cable (4 drops)")
    self.assertIn("check cable or app", jetlink_status.progress(status(progress=drops, mode='ios'))[2])

  def test_port(self):
    self.assertEqual(jetlink_status.port_line(status(port_state='connected', transport='USB')), "Connected: USB.")
    self.assertEqual(jetlink_status.port_line(status(port_state='empty')), "Nothing on the USB port.")
    self.assertEqual(jetlink_status.port_line(status(port_state='device')), "A device is on the USB port.")
    self.assertEqual(jetlink_status.port_line(status(port_state=None)), "")

  def test_details(self):
    on = status(phase='prepared', enabled=True, holds_port=True, shown=True, port_state='connected',
                active_model='Big One')
    self.assertEqual(jetlink_status.detail_lines(on), ["Connected: USB.", "Big model: Big One", jetlink_status.ADB_NOTE])
    pending = status(enabled=True, model='Next One')
    self.assertEqual(jetlink_status.detail_lines(pending), ["Big model: Next One (not built yet)"])
    down = status(phase='unavailable', reason='no warp built for this camera')
    self.assertEqual(jetlink_status.detail_lines(down), ["Unavailable: no warp built for this camera"])
    self.assertTrue(jetlink_status.shown(down))
    self.assertFalse(jetlink_status.shown(status()))


class TestPickerDefault(ParamsTest):
  def test_jetlinks_default_only_with_its_list(self):
    with mock.patch.object(jetlink_adapter, 'status', lambda: status(default_model='JetLink Default')):
      self.assertIsNone(jetlink_status.picker_default_big_model(self.params))
      self.params.put(CACHE_KEY, {'bundles': [], catalog.JETLINK_KEY: True})
      self.assertEqual(jetlink_status.picker_default_big_model(self.params), 'JetLink Default')

  def test_stamp_matches_the_catalog(self):
    self.assertEqual(jetlink_status._BIG_CATALOG_CACHE, CACHE_KEY)
    self.assertEqual(catalog.JETLINK_KEY, 'jetlink')


if __name__ == '__main__':
  unittest.main()
