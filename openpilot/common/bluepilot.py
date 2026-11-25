"""BluePilot detection. Safe to import on any fork — returns False when not BluePilot."""
import importlib
import os
from functools import cache

NAME = "BluePilot"  # shown where sunnypilot shows its name (screensaver, #255)


@cache
def is_bluepilot() -> bool:
  return os.path.isfile(os.path.join(os.path.dirname(__file__), '..', '..', 'BPVERSION'))  # BPVERSION is at the repo root


def _log_failure(message: str) -> None:
  try:
    from openpilot.common.swaglog import cloudlog
    cloudlog.exception(message)
  except Exception:
    pass


class CrashReporting:
  """BluePilot crash reporting (bluepilot/system/sentry.py) as upstream files call it: `sentry.<function>(...)`.

  It can never stop the car software. A call runs only on BluePilot, once the module and its SDK have loaded, and
  never raises; if `init` fails, every later call does nothing, and so does a call in a process where reporting never
  started. Each call returns True when it ran, so a hook can fall back to upstream's code when it didn't.
  """
  def __init__(self) -> None:
    self._module = None
    self._off = False

  def _load(self):
    if self._module is None and not self._off:
      self._off = not is_bluepilot()
      if not self._off:
        try:
          self._module = importlib.import_module("openpilot.bluepilot.system.sentry")
        except Exception:
          self._off = True
          _log_failure("BluePilot crash reporting failed to load")
    return self._module

  def init(self, project: str) -> bool:
    """Starts reporting for a SentryProject, by name. False when it didn't start (e.g. on a PC): later calls then do
    nothing, as there's no active client, but the guard stays on."""
    if (module := self._load()) is None:
      return False
    try:
      return bool(module.init(module.SentryProject[project]))
    except Exception:
      self._module, self._off = None, True
      _log_failure("BluePilot crash reporting failed to start")
      return False

  def __getattr__(self, name: str):
    if name.startswith("_"):  # not a call: lets hasattr, copy and pickle probe the object normally
      raise AttributeError(name)

    def call(*args, **kwargs) -> bool:
      if (module := self._load()) is None:
        return False
      try:
        if not module.is_active():  # loaded, but reporting never started in this process
          return False
        getattr(module, name)(*args, **kwargs)
        return True
      except Exception:
        _log_failure(f"BluePilot crash reporting: {name} failed")
        return False
    return call


sentry = CrashReporting()
