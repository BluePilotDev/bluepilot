"""
JetLink on BluePilot: the adapter JetLink runs over, and the hooks manager, both modelds, selfdrived, hardwared and
the BluePilot settings call.

JetLink (the jetlink_repo submodule, https://github.com/zoompilot/jetlink, MIT, Copyright (c) 2026- Zeph Leggett)
runs openpilot's large driving model on a computer on the comma's USB-C port: a Jetson, a Mac, a PC or a phone. Its
stock adapter (jetlink.openpilot.stock) knows stock openpilot's paths; Adapter below is that with sunnypilot's model
manager slots, BluePilot's modeld_v2 action path (Blendful) and MADS. The contract is JetLink's docs/porting.md.

Adapted from zoompilot's openpilot/sunnypilot/jetlink_adapter/__init__.py (https://github.com/zoompilot/zoompilot,
MIT, Copyright (c) 2026- Zeph Leggett): the guard (_guard, _Off), the hooks by name, link_on() and the Adapter
overrides. See THIRD_PARTY_NOTICES.md beside this file.

manager imports this module to build the process list, and runs it as jetlinkd, the resident gadget owner. So the
top level is the standard library only; everything else is imported where it is used (tests/test_jetlink.py holds
the line with JetLink's check_adapter).

Every hook answers as the link off when JetLink is not checked out, is too old, or speaks another API: a JetLink
failure never takes manager, modeld, selfdrived, hardwared or the UI down. reason() says why, as an offroad alert,
only to someone who turned the link on.
"""
from __future__ import annotations

import functools
import math
import os
import threading
from collections import namedtuple
from pathlib import Path

# the version of jetlink.openpilot's API this adapter is written to; any other is treated as JetLink being absent,
# with the reason as the offroad alert
API = 4

# the gadget owner, as manager names the process and selfdrived leaves it out of processNotRunning
OWNER = 'jetlinkd'

# the JetLink setting (JetlinkLink), stored as an index: jetlink.openpilot.MODES, written out so the settings build
# the selector without a JetLink checkout
MODES = ('off', 'usb', 'ios')
# the modes BluePilot's selector offers in phase 1: off and USB (a Jetson, Mac or PC). iOS is experimental upstream
SELECTABLE_MODES = (0, 1)

# the params JetLink reads and writes, all declared in params_keys.h. JetLink's canonical names, plus sunnypilot's
# model manager slots: big_model and catalog are its big-model pick and catalog (models.helpers'
# ModelManager_ActiveBundleChestnut and ModelFetcher's cache), and select is where a pick is asked for by its ref,
# which JetLink writes for a newer pick made on the host. The owner cannot import the model manager, so they are
# written out here
_Keys = namedtuple('_Keys', 'link offroad progress spec big_model catalog charge_phone select')
KEYS = _Keys(link='JetlinkLink', offroad='IsOffroad', progress='JetlinkProgress', spec='JetlinkSpec',
             big_model='ModelManager_ActiveBundleChestnut', catalog='ModelManager_ModelsCache_Chestnut',
             charge_phone='JetlinkChargePhone', select='ModelManager_DownloadRef')

_AGNOS = os.path.isfile('/AGNOS')


def _params_dir() -> Path:
  """The params store's directory, by params.cc and hw.h's rule: PARAMS_ROOT, else /data/params on a device and
  $HOME/.comma<OPENPILOT_PREFIX>/params elsewhere, then /<OPENPILOT_PREFIX, or d>. Stock.params_dir() without
  JetLink. Per call, so it follows a test's prefix as Params does; the environment only, so it never raises."""
  prefix = os.environ.get('OPENPILOT_PREFIX', '')
  root = os.environ.get('PARAMS_ROOT')
  if root is None:
    root = '/data/params' if _AGNOS else os.environ.get('HOME', '') + '/.comma' + prefix + '/params'
  return Path(root) / os.environ.get('OPENPILOT_PREFIX', 'd')


def link_on() -> bool:
  """Is the JetLink setting on? Read as JetLink reads it, a file, so a JetLink this build cannot use is told only
  to someone who turned the link on."""
  try:
    return 0 < int((_params_dir() / KEYS.link).read_bytes()) < len(MODES)
  except (OSError, ValueError):
    return False


def chestnut_cable(connected: bool | None = None) -> bool:
  """A cable on the port, for modeld's chestnut guess: `connected`, or the port read now. A Jetson or a phone on
  the port is a cable too, so it counts only while JetLink is off (JetLink's docs/porting.md). zoompilot also
  excludes a USB host on the port (usb_host_connected(), a dev or CI case BluePilot does not have): left out."""
  if connected is None:
    from openpilot.common.hardware.usb import cable_connected
    connected = cable_connected()
  return bool(connected) and not link_on()


def with_blendful(get_action_from_model, lat_smooth_seconds: float, min_lat_control_speed: float, dt: float):
  """Stock modeld's action function, which comma's large model drives on, taking Blendful's callback as BluePilot's
  modeld_v2 passes it to the small model's ModelState.get_action_from_model(..., blendful=): the correction is added
  to the raw curvature, before the curvature smoothing, and only above min_lat_control_speed, where stock holds the
  previous curvature instead.

  Stock smooths as smooth_value(raw, prev, tau) = a * raw + (1 - a) * prev, with a = 1 - exp(-dt / tau) (1 at tau
  0, BluePilot's LAT_SMOOTH_SECONDS today). So the raw curvature is (smoothed - (1 - a) * prev) / a and the
  correction added before smoothing is a * correction after it: exact for any tau, not only 0. Below the speed the
  callback still runs every frame, as on the small model, on the held curvature: Blendful fades there anyway."""
  alpha = 1 - math.exp(-dt / lat_smooth_seconds) if lat_smooth_seconds > 0 else 1.

  def action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego, blendful=None):
    action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
    if blendful is not None:
      if v_ego > min_lat_control_speed:
        raw = (action.desiredCurvature - (1 - alpha) * prev_action.desiredCurvature) / alpha
        action.desiredCurvature = float(action.desiredCurvature + alpha * blendful(raw))
      else:
        blendful(action.desiredCurvature)
    return action
  return action_from_model


@functools.cache
def _adapter_class():
  from jetlink.openpilot.interface import Keys
  from jetlink.openpilot.stock import Stock

  class Adapter(Stock):
    """jetlink.openpilot.interface.Openpilot over BluePilot: stock openpilot but for what follows."""
    KEYS = Keys(**KEYS._asdict())
    # MADS's state rides on carControlSP
    IN_CONTROL_SERVICES = ('carState', 'carControl', 'carControlSP')

    def chestnut_present(self) -> bool:
      # modeld's own test, the one it runs a native big model on: a chestnut whose firmware names itself. The
      # owner's port walk is wider (CHESTNUT_IDS, running or in its ROM), so it never holds a chestnut as a host
      from openpilot.selfdrive.modeld.helpers import chestnut_present
      return chestnut_present()

    def model_constants(self):
      # modeld_tinygrad reads modeld_v2's off the model
      from openpilot.sunnypilot.modeld_v2.constants import ModelConstants
      return ModelConstants

    def action_function(self):
      # comma's large model on stock modeld's action function, with Blendful's correction as modeld_v2 gives the
      # small model. The lateral delay (livedelay) is not the function's: modeld writes model.lat_delay every
      # 60 frames and the joined model passes it to whichever model drives
      from openpilot.common.realtime import DT_MDL
      from openpilot.selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS, MIN_LAT_CONTROL_SPEED, get_action_from_model
      return with_blendful(get_action_from_model, LAT_SMOOTH_SECONDS, MIN_LAT_CONTROL_SPEED, DT_MDL)

    def engaged(self, sm) -> bool:
      # selfdrived's enabled or mads.enabled, as controlsd republishes them. MADS counts with its lateral paused
      # (a stop, a blinker, the brake): it steers again on its own
      return bool(sm['carControl'].enabled or sm['carControlSP'].mads.enabled)

  return Adapter


def adapter():
  """The adapter, for JetLink's entry points that run as their own process: the provisioning run."""
  return _adapter_class()()


def owner_config():
  """What jetlinkd needs: data only, so the owner imports nothing heavy."""
  return _adapter_class().owner_config()


def main() -> None:
  """jetlinkd: hold the USB gadget until manager stops this process."""
  _adapter_class().main()


# -- the hooks ----------------------------------------------------------------

# this process's hooks, made on first use and kept, the link off included: Python does not cache a failed import.
# One per process, so prepare() and attach() reach the same JetLink
_hooks = None
_making = threading.Lock()


def hooks():
  """jetlink.openpilot.hooks.Hooks over this adapter, or the link off. Never raises."""
  global _hooks
  with _making:
    if _hooks is None:
      _hooks = _guard()
    return _hooks


def _guard():
  # inside a function, so a JetLink that fails to import lands here too
  try:
    import jetlink
    if getattr(jetlink, '__file__', None) is None:
      return _Off()   # an empty jetlink_repo, which Python takes for a namespace package
    from jetlink.openpilot.hooks import Hooks
  except ModuleNotFoundError as e:
    if e.name == 'jetlink':
      return _Off()   # no checkout: the link does not exist on this device
    if (e.name or '').startswith('jetlink.'):
      return _unusable("JetLink package too old for this build", e)
    return _unusable(f"JetLink failed to load: {e}", e)
  except Exception as e:
    return _unusable(f"JetLink failed to load: {type(e).__name__}: {e}", e)
  try:
    return Hooks(API, adapter, link_on=link_on, log=_cloudlog)
  except Exception as e:
    return _unusable(str(e), e)   # another API, which it names


def _cloudlog():
  from openpilot.common.swaglog import cloudlog
  return cloudlog


def _unusable(why: str, error: Exception) -> _Off:
  try:
    _cloudlog().error("jetlink: %s", why, exc_info=error)
  except Exception:
    pass
  return _Off(why)


class _Off:
  """Every hook with the link off; reason() says why, to someone who turned the link on. BluePilot's own: JetLink's
  Off cannot answer for a JetLink that is missing or too old to have it, and one answer for every case keeps them
  alike."""
  # by hook; a callable is handed the hook's arguments. A wanted power-off goes ahead: no JetLink must keep a comma
  # on to drain the car's battery
  ANSWERS = {'should_run': False, 'status': None, 'prepare': False, 'attach': lambda model, *_: model,
             'handed_over': False, 'state': 'none', 'runtime': None, 'shutdown_step': lambda wanted, reason='': wanted,
             'model_state': None, 'lists_big_models': False, 'big_catalog': None, 'listed_ref': None,
             'swap_events': None}

  def __init__(self, why: str = ''):
    self.why = why

  def reason(self) -> str | None:
    return self.why if self.why and link_on() else None

  def __getattr__(self, name: str):
    if name not in self.ANSWERS:
      raise AttributeError(name)
    answer = self.ANSWERS[name]
    return answer if callable(answer) else lambda *args: answer


def _hook(name: str):
  def call(*args):
    return getattr(hooks(), name)(*args)
  call.__name__ = call.__qualname__ = name
  return call


# the hooks by the names the hook lines call, each this process's hooks() at the call: jetlink.openpilot.hooks says
# what each does and answers
HOOK_NAMES = (*_Off.ANSWERS, 'reason')
(should_run, status, prepare, attach, handed_over, state, runtime, shutdown_step, model_state, lists_big_models,
 big_catalog, listed_ref, swap_events, reason) = map(_hook, HOOK_NAMES)
