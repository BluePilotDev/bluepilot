"""
JetLink's big-model list in sunnypilot's model manager (JetLink's docs/porting.md, "model manager"; R9).

With no chestnut fitted, and once the link has been turned on, the big-model (chestnut) catalog is JetLink's model
index (big_catalog), the models a host can run, in place of sunnypilot's chestnut list; with a chestnut fitted it is
sunnypilot's own, as fetched. BluePilot's gate on top of JetLink's (whose catalog hooks answer with the link off): a
device that never turned the link on never fetches JetLink's index, and one that did keeps JetLink's list with the
link turned off again, so a pick is not reset by turning the link off for a drive. The model
manager's downloads, its active-bundle slot (ModelManager_ActiveBundleChestnut, the pick JetLink runs) and the
pickers' cache (ModelManager_ModelsCache_Chestnut) all stay sunnypilot's: only where the list comes from changes.
A pick the new list no longer names (a recompile listed under a newer ref) is carried over to listed_ref(), the ref
the list names the same ONNX under, before the manager's validation would reset it.

Adapted from zoompilot's openpilot/sunnypilot/models/fetcher_zp.py and default_bootstrap.py
(https://github.com/zoompilot/zoompilot, MIT, Copyright (c) 2026- Zeph Leggett). See THIRD_PARTY_NOTICES.md.
"""
from __future__ import annotations

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.bluepilot import jetlink_adapter
from openpilot.sunnypilot.models.fetcher import ModelCache, ModelFetcher
from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS, REQUIRED_JSON_VERSION, get_selected_bundle

SOURCE = "chestnut"
# stamped on a catalog JetLink listed, so the cache says whose list it holds
JETLINK_KEY = "jetlink"
DOWNLOAD_REF = "ModelManager_DownloadRef"


def is_jetlinks(catalog) -> bool:
  return isinstance(catalog, dict) and catalog.get(JETLINK_KEY) is True


def lists_jetlink(cached) -> bool:
  """Is the big-model list JetLink's: no chestnut fitted (JetLink's lists_big_models), and the link on now or a
  list of JetLink's already cached (the link was on)."""
  return bool(jetlink_adapter.lists_big_models()) and (jetlink_adapter.link_on() or is_jetlinks(cached))


class BigCatalogCache(ModelCache):
  """The big-model catalog's cache, expired once when it was fetched for the other hardware: a chestnut fitted or
  removed changes whose list it is, and the hour-long cache would hide that. Once per change, so offline the failed
  refetch leaves the cache standing until it expires."""

  def __init__(self, params: Params, suffix: str, fetcher: JetlinkModelFetcher):
    super().__init__(params, suffix=suffix)
    self._fetcher = fetcher
    self._refetched_for: bool | None = None

  def get(self) -> tuple[dict, bool]:
    cached, expired = super().get()
    jetlink = lists_jetlink(cached)
    self._fetcher.show_jetlink_list(jetlink)
    if cached and not expired and is_jetlinks(cached) != jetlink and self._refetched_for != jetlink:
      self._refetched_for = jetlink
      cloudlog.warning(f"big-model catalog was fetched for {'a chestnut' if jetlink else 'jetlink'}; refetching")
      expired = True
    return cached, expired


class JetlinkModelFetcher(ModelFetcher):
  """sunnypilot's fetcher, with the big-model list JetLink's while no chestnut is fitted. Without JetLink, or with
  the link's hooks off, lists_big_models() is False and this is sunnypilot's fetcher."""

  def __init__(self, params: Params):
    super().__init__(params)
    # what the base just wrote
    self._active_json: dict[str, str] | None = {"qcom": self.MODEL_URL, SOURCE: self.MODEL_URL_CHESTNUT}
    cache = self.model_caches[SOURCE] = BigCatalogCache(params, self.MODEL_SOURCES[SOURCE][1], self)
    self.show_jetlink_list(is_jetlinks(params.get(cache._CACHE_KEY)))

  def show_jetlink_list(self, jetlink: bool) -> None:
    """ModelManager_ActiveJson without the chestnut URL while JetLink's list stands in for it, so sunnylink lists
    the comma's cached copy rather than sunnypilot's chestnut catalog."""
    active = {"qcom": self.MODEL_URL} if jetlink else {"qcom": self.MODEL_URL, SOURCE: self.MODEL_URL_CHESTNUT}
    if active != self._active_json:
      self.params.put("ModelManager_ActiveJson", active, block=True)
      self._active_json = active

  def _fetch_and_cache_models(self, source: str):
    if source != SOURCE or not lists_jetlink(self.params.get(self.model_caches[SOURCE]._CACHE_KEY)):
      return super()._fetch_and_cache_models(source)
    catalog = jetlink_adapter.big_catalog(REQUIRED_JSON_VERSION)
    if catalog is None:  # offline, or no JetLink: the cache stands
      return None
    catalog = {**catalog, JETLINK_KEY: True}
    try:
      parsed = self.model_parser.parse_models(catalog)
    except Exception:
      cloudlog.exception("jetlink: the big-model list does not parse")
      return None
    if parsed:
      self.model_caches[source].set(catalog)
    return parsed


class PickCarryOver:
  """Carries the big-model pick over to the ref JetLink's list names its ONNX under, when the list no longer names
  the pick's own: queued as a download of the listed ref, which writes the pick's slot when it lands. Once per ref,
  and never over a download already queued."""

  def __init__(self):
    self._queued: set[str] = set()

  def update(self, params: Params, source_models: dict) -> str | None:
    """Before the manager validates its slots. The ref queued, or None."""
    bundles = source_models.get(SOURCE) or []
    pick = get_selected_bundle(params, SOURCE)
    if not bundles or pick is None or any(b.ref == pick.ref for b in bundles):
      return None
    if not is_jetlinks(params.get(f"ModelManager_ModelsCache{ModelFetcher.MODEL_SOURCES[SOURCE][1]}")):
      return None  # sunnypilot's list: its own validation decides
    listed = jetlink_adapter.listed_ref(pick.ref)
    if not listed or listed == pick.ref or listed in self._queued or params.get(DOWNLOAD_REF) is not None:
      return None
    if not any(b.ref == listed for b in bundles):
      return None
    self._queued.add(listed)
    params.put(DOWNLOAD_REF, listed)
    cloudlog.warning(f"jetlink: carrying the big-model pick {pick.displayName} over to {listed} ({ACTIVE_BUNDLE_KEYS[SOURCE]})")
    return listed
