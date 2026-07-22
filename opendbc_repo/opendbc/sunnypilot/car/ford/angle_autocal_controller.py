"""BluePilot: lifecycle controller for the Ford angle-mode auto-calibration.

AutoCalPipeline (angle_autocal.py) is pure math with no I/O. This controller owns
everything between that math and the car: the params handle, arm/disarm from the
FordAngleAutoCal toggle, evidence (de)serialization to FordAngleAutoCalState with
its save cadence, user-edit debounce, nudge application to the factor params,
error reporting to FordAngleAutoCalError, and the ground-truth telemetry status
string. FordLateralAngleExt holds one instance and calls exactly three things:
poll_params() at its ~1 Hz param cadence, feed() once per 20 Hz lateral frame,
and idle() on frames where lateral is inactive.

The controller never touches the strategy's in-memory factors — feed() returns a
nudged (low, high) pair for the strategy to adopt, keeping the write path to the
live steering values in exactly one place (the caller).
"""
import json

from opendbc.sunnypilot.car.ford.angle_autocal import AutoCalPipeline

SAVE_PERIOD_S = 30.0


def _state_locked(state: str) -> bool:
  """True when the persisted state says the calibration is finished.
  Legacy pre-JSON states ("done low=... high=... verified") stay honored."""
  if state.startswith("done"):
    return True
  if state.startswith("{"):
    try:
      return json.loads(state).get("phase") == "locked"
    except (ValueError, AttributeError):
      return False
  return False


def _restore(pipeline, state: str):
  """Load serialized evidence into a fresh pipeline; anything unparseable (legacy round
  strings, garbage, empty) simply starts a fresh collection."""
  if not state.startswith("{"):
    return
  try:
    d = json.loads(state)
    pipe = d.get("pipe")
    if isinstance(pipe, dict) and int(d.get("v", 0)) == 1:
      pipeline.from_dict(pipe)
  except (ValueError, KeyError, TypeError):
    pass


class AutoCalController:
  def __init__(self, dt: float):
    self.dt = dt
    self.enabled = False
    self.done = True            # conservative until params are read
    self.pipeline = None        # AutoCalPipeline while collecting
    self.status = ""            # live ground-truth status, published in telemetry
    self._params = None
    self._last_written = None   # ("x.xx", "x.xx") the nudger last wrote; edits differ
    self._edit_pending = False  # user edit needs 2 consecutive ticks (async put lag)
    self._save_s = 0.0
    self._dirty = False

  # -- ~1 Hz: toggle, restore, user edits, status ------------------------------------------
  def poll_params(self, params, low_factor: float, high_factor: float, platform_gain_high: float):
    """Arm/disarm from the toggle, restore evidence on arm, detect user hand-edits of the
    factor params, refresh the status string. low/high are the currently applied values."""
    try:
      enabled = bool(params.get_bool("FordAngleAutoCal"))
      state = params.get("FordAngleAutoCalState", return_default=True) or ""
      if isinstance(state, bytes):
        state = state.decode("utf-8", errors="replace")
      if self.pipeline is None:
        self.done = _state_locked(state)
      else:
        self.done = self.pipeline.locked
      self.enabled = enabled and not self.done
      if self.enabled and self.pipeline is None:
        # Arm: build the pipeline and restore serialized evidence from a prior drive.
        # The currently applied factors are the nudge baseline.
        self.pipeline = AutoCalPipeline(platform_gain_high, dt=self.dt)
        _restore(self.pipeline, state)
        self._last_written = (f"{low_factor:.2f}", f"{high_factor:.2f}")
        self._edit_pending = False
      elif not self.enabled:
        self.pipeline = None
      else:
        # User-edit detection: the factor params moved without the nudger writing them.
        # Confirmed on two consecutive ticks — an async put of our own nudge may not be
        # readable yet on the first tick after it. The driver's judgment is adopted
        # (values already live in the strategy); evidence is soft-reset, not wiped.
        cur = (f"{low_factor:.2f}", f"{high_factor:.2f}")
        if self._last_written is not None and cur != self._last_written:
          if self._edit_pending:
            self.pipeline.user_edit()
            self._last_written = cur
            self._edit_pending = False
            self._dirty = True
          else:
            self._edit_pending = True
        else:
          self._edit_pending = False
      self._params = params
      # Live status for telemetry: published from actual controller state (ground truth),
      # never from a param re-read — a param/telemetry mismatch is exactly the failure
      # mode that made earlier on-device issues undiagnosable.
      if self.done:
        self.status = "locked"
      elif not self.enabled:
        self.status = "off"
      else:
        est = self.pipeline.est
        self.status = (f"armed n={est.n} w={est.weight_low:.0f}/{est.weight_high:.0f}"
                       f" applied={low_factor:.2f}/{high_factor:.2f}"
                       f" nudges={self.pipeline.nudges}")
    except Exception as e:
      self.enabled = False
      self.status = f"tick error: {type(e).__name__}: {e}"[:200]
      self._error(self.status)

  # -- 20 Hz frames ------------------------------------------------------------------------
  def idle(self):
    """Frames where lateral is inactive (disengaged / human turn / stall blip)."""
    if self.pipeline is not None:
      self.pipeline.idle()

  def feed(self, v_ego: float, kappa_cmd: float, kappa_meas: float,
           steering_pressed: bool, angle_rate_limited: bool, deviation_limited: bool,
           saturated: bool, driver_torque: float, a_ego: float, ws_spread: float,
           low_factor: float, high_factor: float, delay_estimated: bool):
    """One active lateral frame. Returns a nudged (low, high) pair the strategy should
    adopt, or None. Save cadence and the lock -> disarm transition happen here."""
    if not self.enabled or self.pipeline is None:
      return None
    if not delay_estimated:
      # Measurement-chain warmup: kappa_meas flows through liveParameters from the same
      # locationd stack that estimates the actuation delay — until lagd reports
      # 'estimated' those inputs are defaults/converging. Idle (not pause): staged
      # samples and peak windows must not straddle the unestimated period.
      self.idle()
      return None
    committed = self.pipeline.update(v_ego, kappa_cmd, kappa_meas,
                                     steering_pressed, angle_rate_limited, deviation_limited,
                                     saturated=saturated, driver_torque=driver_torque,
                                     a_ego=a_ego, ws_spread=ws_spread,
                                     low_factor=low_factor, high_factor=high_factor)
    if committed:
      self._dirty = True
    applied = (low_factor, high_factor)
    out = None
    rec = self.pipeline.recommend(low_factor, high_factor)
    if rec is not None and self._apply_nudge(rec, applied):
      applied = rec
      out = rec
    if self.pipeline.locked:
      self._save("locked", applied)
      self.done = True
      self.enabled = False
      self.pipeline = None
    else:
      self._save_s += self.dt
      if self._dirty and self._save_s >= SAVE_PERIOD_S:
        self._save("collecting", applied)
    return out

  # -- params I/O --------------------------------------------------------------------------
  def _apply_nudge(self, rec, applied) -> bool:
    """Write a nudged factor pair to the params (the lateral tuning menu shows them move).
    Returns True when the write landed — only then does the caller adopt the values.

    The params are TYPED (FLOAT) in this fork: writes must be python floats — a string
    raises TypeError. That failure mode was invisible once (swallowed except -> nudges
    silently never landed); now any write error is recorded in FordAngleAutoCalError so
    it shows up in the next drive's logs instead of vanishing."""
    low_new, high_new = rec
    if self._params is None:
      return False
    try:
      self._params.put("FordLowSpeedFactor_ang", float(low_new))
      self._params.put("FordHighSpeedFactor_ang", float(high_new))
    except Exception as e:
      self._error(f"nudge write failed: {type(e).__name__}: {e}")
      return False
    self._last_written = (f"{low_new:.2f}", f"{high_new:.2f}")
    self._edit_pending = False
    self._save("collecting", rec)
    return True

  def _error(self, msg: str):
    """Self-reporting diagnostics: park the error in its OWN param so it is visible in
    qlogs/initData without ever touching FordAngleAutoCalState — an error written just
    before ignition-off must not be able to replace (and thereby erase) the serialized
    evidence from the last good save. Never raises."""
    try:
      if self._params is not None:
        self._params.put("FordAngleAutoCalError", f"{msg[:300]}")
    except Exception:
      pass

  def _save(self, phase: str, applied):
    """Serialize the pipeline into FordAngleAutoCalState (JSON). Async put is fine:
    a lost final write costs at most SAVE_PERIOD_S of evidence."""
    if self._params is None or self.pipeline is None:
      return
    d = {
      "v": 1,
      "phase": phase,
      "pipe": self.pipeline.to_dict(),
      "applied": {"low": round(applied[0], 2), "high": round(applied[1], 2)},
    }
    sol = self.pipeline.est.solve()
    if sol is not None:
      low_t, high_t, st = sol
      d["target"] = {"low": round(low_t, 2), "high": round(high_t, 2)}
      d["weight"] = {"low": round(st["weight_low"], 1), "high": round(st["weight_high"], 1)}
      d["stderr"] = {"low": round(st["stderr_eff_low"], 3), "high": round(st["stderr_eff_high"], 3)}
      d["stable_s"] = round(self.pipeline.stable_s, 1)
    try:
      self._params.put("FordAngleAutoCalState", json.dumps(d, separators=(",", ":")))
    except Exception as e:
      self._error(f"state save failed: {type(e).__name__}: {e}")
      return
    self._save_s = 0.0
    self._dirty = False
