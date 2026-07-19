#!/usr/bin/env python3
"""Frame-exact replay of ford.h's steering TX-hook against real rlogs.

Attributes real panda `safetyTxBlocked` increments to specific safety checks, with the
reset-bypass latch modeled. (An earlier shadow-check-only analysis omitted the latch and
mis-attributed re-engage-edge frames as blocked; this tool exists so firmware semantics
can't silently drift from the analysis again.)

Ported line-for-line from opendbc/safety/modes/ford.h (CAN path). Models the
configuration the analyzed road-test routes were recorded with: angle_meas sourced from
SteeringPinion_Data (Explorer geometry) and the 0.003 error band of the fork's opt-in
steering-angle measurement. Stock yaw-sourced firmware differs only in the angle_meas
source and a 0.002 band -- swap rx_steering_pinion and MAX_ANGLE_ERROR to replay stock
routes.
  rx state:  vehicle_speed (BrakeSysFeatures, QF==3), angle_meas (SteeringPinion_Data,
             QF==3, Explorer geometry)
  LKA hook (0x3CA):  action!=0 block; latches angle_mode_engaged + shadow_curvature
  LMC hook (0x3D3):  value limits, steer_angle_cmd_checks (curvature mode) + explicit
             controls gate at curvature==0, shadow-curvature check (angle mode),
             path_angle/path_offset/curvature_rate ROC checks, reset-bypass latch

Ground truth is independent of the sim: a `sendcan` frame with no matching TX loopback
echo in `can` (src >= 128) was actually blocked by panda; pandaStates.safetyTxBlocked
counter deltas cross-check the totals. The sim then explains each real block (which
check fired) and a no-latch counterfactual shows what the latch masked.

Usage:
  FORD_REPLAY_DONGLE_ID=<dongleid> tools/ford_lmc_safety_replay.py <route> [<route> ...]
  e.g. tools/ford_lmc_safety_replay.py 00000002--71f65bbf45
Optional: --json <path-prefix> to dump per-frame records for further analysis.
"""
import argparse
import json
import math
import os
from collections import defaultdict, deque

from openpilot.tools.lib.logreader import LogReader

DONGLE = os.environ.get('FORD_REPLAY_DONGLE_ID', '')

# ---- ford.h constants (CAN path, FORD_LIMITS / FORD_*_LIMITS) ----
VEHICLE_SPEED_FACTOR = 1000.0
MAX_SAMPLE_VALS = 6

FORD_INACTIVE_CURVATURE = 1000
FORD_INACTIVE_CURVATURE_RATE = 4096
FORD_INACTIVE_PATH_OFFSET = 512
FORD_INACTIVE_PATH_ANGLE = 1000

STEERING = dict(  # FORD_LIMITS values, with the 150-unit (0.003) pinion band; stock band is 100
  max_angle=1000, deg_to_can=50000, max_angle_error=150,
  rate_up=([5., 16., 25.], [0.0025, 0.0014, 0.00018]),
  rate_down=([5., 16., 25.], [0.0025, 0.0014, 0.00018]),
  angle_error_min_speed=10.0,
)
PATH_ANGLE = dict(  # FORD_PATH_ANGLE_LIMITS
  deg_to_can=2000, rate_up=([10., 15., 25.], [0.0561, 0.04335, 0.00918]),
)
PATH_OFFSET = dict(  # FORD_PATH_OFFSET_LIMITS
  deg_to_can=100, rate_up=([5., 15., 25.], [0.05, 0.025, 0.01]),
)
CURV_RATE = dict(  # FORD_CURVATURE_RATE_LIMITS_CAN
  deg_to_can=4000000, rate_up=([5., 15., 25.], [0.05, 0.025, 0.01]),
)

FORD_CURVATURE_MIN, FORD_CURVATURE_MAX = -0.02, 0.02
FORD_CURVATURE_RATE_MIN, FORD_CURVATURE_RATE_MAX = -0.001024, 0.00102375
FORD_PATH_OFFSET_MIN, FORD_PATH_OFFSET_MAX = -1.0, 1.0
FORD_PATH_ANGLE_MIN, FORD_PATH_ANGLE_MAX = -0.25, 0.25
FORD_DBC_PATH_ANGLE_MIN, FORD_DBC_PATH_ANGLE_MAX = -0.5, 0.5235

RESET_BYPASS_LATCH_DURATION = 60

# Explorer pinion->curvature geometry (slip factor / steer ratio / wheelbase)
SLIP, SR, WB = -0.00055447339, 16.8, 3.025

ADDR_LKA, ADDR_LMC = 0x3CA, 0x3D3
ADDR_PINION, ADDR_BRAKE_SYS = 0x7E, 0x415
ECHO_TIMEOUT_S = 0.5

CHECK_KEYS = ['v_curv_val', 'v_curv_rate_val', 'v_po_val', 'v_pa_val', 'v_curv_check',
              'v_controls_gate', 'v_shadow', 'v_pa_roc', 'v_po_roc', 'v_curv_rate_roc']


def interp_hold(xy, x):
  xs, ys = xy
  if x <= xs[0]:
    return ys[0]
  for i in range(len(xs) - 1):
    if x < xs[i + 1]:
      dx = max(xs[i + 1] - xs[i], 0.0001)
      return ys[i] + (ys[i + 1] - ys[i]) * (x - xs[i]) / dx
  return ys[-1]


def limit_check(val, max_val, min_val):
  return (val > max_val) or (val < min_val)


class Sample:
  def __init__(self):
    self.values = deque([0] * MAX_SAMPLE_VALS, maxlen=MAX_SAMPLE_VALS)

  def update(self, v):
    self.values.appendleft(int(v))

  @property
  def min(self):
    return min(self.values)

  @property
  def max(self):
    return max(self.values)

  @property
  def latest(self):
    return self.values[0]


class FordLmcSafetySim:
  """Firmware state machine for the CAN LMC/LKA tx hooks + relevant rx state."""

  def __init__(self):
    self.vehicle_speed = Sample()
    self.angle_meas = Sample()
    self.desired_angle_last = 0
    self.desired_path_angle_last = 0
    self.desired_path_offset_last = 0
    self.desired_curvature_rate_last = 0
    self.reset_bypass_latch_counter = 0
    self.angle_mode_engaged = False
    self.shadow_curvature_raw = 0
    # externally-fed panda state (from pandaStates log)
    self.controls_allowed = False
    self.controls_allowed_lateral = False

  # ---- rx side ----
  def rx_brake_sys_features(self, d):
    if (d[2] >> 6) == 0x3:  # VehVActlBrk_D_Qf
      speed_ms = ((d[0] << 8) | d[1]) * 0.01 / 3.6
      self.vehicle_speed.update(round(speed_ms * VEHICLE_SPEED_FACTOR))

  def rx_steering_pinion(self, d):
    if ((d[5] >> 2) & 0x3) != 0x3:  # StePinCompAnEst_D_Qf
      return
    angle_raw = ((d[2] & 0x7F) << 8) | d[3]
    pinion_angle_rad = math.radians((angle_raw * 0.1) - 1600.0)
    speed = max(self.vehicle_speed.latest / VEHICLE_SPEED_FACTOR, 0.1)
    curvature_factor = 1. / (1. - (SLIP * speed * speed)) / WB
    current_curvature = pinion_angle_rad * curvature_factor / SR
    self.angle_meas.update(round(current_curvature * STEERING['deg_to_can']))

  # ---- tx side ----
  def tx_lka(self, d, pressed=False, truthful_shadow=False):
    """Returns True if blocked. Latches angle-mode statics regardless (as firmware does).

    truthful_shadow: counterfactual for the truthful-shadow control fix -- on frames the
    fix would publish the shadow from measured curvature (driver pressing, or the old
    code's zeroed override/inactive frames), latch the measured value instead, at real
    LKA cadence so latch-age timing skew is modeled faithfully.
    """
    action = d[0] >> 5
    self.angle_mode_engaged = (d[4] & 0x1) != 0
    raw = (d[5] << 8) | d[6]
    raw = raw - 0x10000 if raw >= 0x8000 else raw  # int16
    if truthful_shadow and (pressed or raw == 0):
      raw = int(self.angle_meas.latest * 20)  # CAN units (2e-5) -> shadow raw units (1e-6)
    self.shadow_curvature_raw = raw
    return action != 0

  def _steer_angle_cmd_checks(self, desired_angle, en, lim):
    """lateral.h steer_angle_cmd_checks, angle_is_curvature=false, inactive_angle_is_zero=true."""
    violation = False
    if (self.controls_allowed or self.controls_allowed_lateral) and en:
      fudged_speed = (self.vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.
      delta_up = int(interp_hold(lim['rate_up'], fudged_speed) * lim['deg_to_can'] + 1.)
      delta_down = int(interp_hold(lim['rate_down'], fudged_speed) * lim['deg_to_can'] + 1.)
      last = self.desired_angle_last
      highest = last + (delta_up if last > 0 else delta_down)
      lowest = last - (delta_down if last >= 0 else delta_up)
      if (self.vehicle_speed.latest / VEHICLE_SPEED_FACTOR) > lim['angle_error_min_speed']:
        fudged_speed_error = (self.vehicle_speed.max / VEHICLE_SPEED_FACTOR) + 1.
        delta_up_rlx = int(interp_hold(lim['rate_up'], fudged_speed_error) * lim['deg_to_can'] - 1.)
        delta_down_rlx = int(interp_hold(lim['rate_down'], fudged_speed_error) * lim['deg_to_can'] - 1.)
        lowest_err = self.angle_meas.min - lim['max_angle_error'] - 1
        highest_err = self.angle_meas.max + lim['max_angle_error'] + 1
        if last > highest_err:
          delta = delta_down_rlx if last >= 0 else delta_up_rlx
          highest = max(last - delta, highest_err)
        elif last < lowest_err:
          delta = delta_down_rlx if last <= 0 else delta_up_rlx
          lowest = min(last + delta, lowest_err)
        else:
          highest = min(highest, highest_err)
          lowest = max(lowest, lowest_err)
        lowest = min(max(lowest, -lim['max_angle']), lim['max_angle'])
        highest = min(max(highest, -lim['max_angle']), lim['max_angle'])
      violation |= limit_check(desired_angle, highest, lowest)
    self.desired_angle_last = desired_angle
    if not en:
      violation |= desired_angle != 0
    # No angle control allowed when controls are not allowed (lateral.h:267-269)
    if not (self.controls_allowed or self.controls_allowed_lateral):
      violation |= en
    # reset on violation or controls-not-allowed (lateral.h:271-277, inactive_angle_is_zero);
    # firmware does this BEFORE the reset-bypass latch can clear the violation
    if violation or not (self.controls_allowed or self.controls_allowed_lateral):
      self.desired_angle_last = 0
    return violation

  def _roc_check(self, desired, last_attr, en, lim):
    violation = False
    if en:
      speed = (self.vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.
      delta = int(interp_hold(lim['rate_up'], speed) * lim['deg_to_can'] + 1.)
      last = getattr(self, last_attr)
      violation |= limit_check(desired, last + delta, last - delta)
    setattr(self, last_attr, desired)
    if not en:
      violation |= desired != 0
    return violation

  def _shadow_check(self, shadow_can, en, lim):
    if en and (self.vehicle_speed.latest / VEHICLE_SPEED_FACTOR) > lim['angle_error_min_speed']:
      return limit_check(shadow_can, self.angle_meas.max + lim['max_angle_error'] + 1,
                         self.angle_meas.min - lim['max_angle_error'] - 1)
    return False

  def tx_lmc(self, d):
    """Full LMC tx-hook. Returns a dict of per-check verdicts + final pre/post-latch."""
    en = ((d[4] >> 2) & 0x7) != 0
    raw_curvature = (d[0] << 3) | (d[1] >> 5)
    raw_curvature_rate = ((d[1] & 0x1F) << 8) | d[2]
    raw_path_angle = (d[3] << 3) | (d[4] >> 5)
    raw_path_offset = (d[5] << 2) | (d[6] >> 6)

    curv = raw_curvature - FORD_INACTIVE_CURVATURE
    curv_rate = raw_curvature_rate - FORD_INACTIVE_CURVATURE_RATE
    pa = raw_path_angle - FORD_INACTIVE_PATH_ANGLE
    po = raw_path_offset - FORD_INACTIVE_PATH_OFFSET

    r = {'en': en, 'curv': curv, 'pa': pa, 'po': po, 'curv_rate': curv_rate,
         'engaged': self.angle_mode_engaged, 'shadow_raw': self.shadow_curvature_raw,
         'shadow_can': None, 'meas_min': self.angle_meas.min, 'meas_max': self.angle_meas.max,
         'latch_pre': self.reset_bypass_latch_counter}

    # value limits
    r['v_curv_val'] = limit_check(curv, int(FORD_CURVATURE_MAX * STEERING['deg_to_can']),
                                  int(FORD_CURVATURE_MIN * STEERING['deg_to_can']))
    r['v_curv_rate_val'] = limit_check(curv_rate, int(FORD_CURVATURE_RATE_MAX * CURV_RATE['deg_to_can']),
                                       int(FORD_CURVATURE_RATE_MIN * CURV_RATE['deg_to_can']))
    r['v_po_val'] = limit_check(po, int(FORD_PATH_OFFSET_MAX * PATH_OFFSET['deg_to_can']),
                                int(FORD_PATH_OFFSET_MIN * PATH_OFFSET['deg_to_can']))
    pa_min = FORD_DBC_PATH_ANGLE_MIN if self.angle_mode_engaged else FORD_PATH_ANGLE_MIN
    pa_max = FORD_DBC_PATH_ANGLE_MAX if self.angle_mode_engaged else FORD_PATH_ANGLE_MAX
    r['v_pa_val'] = limit_check(pa, int(pa_max * PATH_ANGLE['deg_to_can']),
                                int(pa_min * PATH_ANGLE['deg_to_can']))

    # curvature checks: always call (keeps desired_angle_last in sync), apply if curv != 0
    curv_violation = self._steer_angle_cmd_checks(curv, en, STEERING)
    if curv != 0:
      r['v_curv_check'] = curv_violation
      r['v_controls_gate'] = False
    else:
      r['v_curv_check'] = False
      r['v_controls_gate'] = en and not (self.controls_allowed or self.controls_allowed_lateral)

    # angle mode's shadow-curvature deviation check
    r['v_shadow'] = False
    if curv == 0 and self.angle_mode_engaged:
      shadow_can = int(float(self.shadow_curvature_raw) * 0.05)
      r['shadow_can'] = shadow_can
      r['v_shadow'] = self._shadow_check(shadow_can, en, STEERING)

    # ROC checks
    r['v_pa_roc'] = self._roc_check(pa, 'desired_path_angle_last', en, PATH_ANGLE)
    r['v_po_roc'] = self._roc_check(po, 'desired_path_offset_last', en, PATH_OFFSET)
    r['v_curv_rate_roc'] = self._roc_check(curv_rate, 'desired_curvature_rate_last', en, CURV_RATE)

    violation = any(r[k] for k in CHECK_KEYS)
    r['pre_latch_violation'] = violation

    # reset-bypass latch
    if curv == 0 and pa == 0:
      self.reset_bypass_latch_counter = RESET_BYPASS_LATCH_DURATION
      violation = False
    elif self.reset_bypass_latch_counter > 0:
      self.reset_bypass_latch_counter -= 1
      violation = False
    r['blocked'] = violation
    return r


class EchoMatcher:
  """Ground truth: sent frames that never echo back (src >= 128) were blocked by panda."""

  def __init__(self):
    self.pending = defaultdict(deque)        # addr -> deque of (t, dat, seq)
    self.recent_echoes = defaultdict(deque)  # addr -> deque of (t, dat), reorder tolerance
    self.blocked = []                        # (t, addr, dat, seq)
    self.sent = defaultdict(int)
    self.echoed = defaultdict(int)

  def on_send(self, t, addr, dat, seq):
    self.sent[addr] += 1
    dat = bytes(dat)
    # tolerate log reordering: echo may have been logged just before the sendcan event
    for i, (te, de) in enumerate(self.recent_echoes[addr]):
      if de == dat and (t - te) < 0.2:
        del self.recent_echoes[addr][i]
        return
    self.pending[addr].append((t, dat, seq))

  def on_echo(self, t, addr, dat):
    self.echoed[addr] += 1
    dat = bytes(dat)
    q = self.pending[addr]
    for i, (_ts, d, _seq) in enumerate(q):
      if d == dat:
        for _ in range(i):  # frames sent before this one and never echoed -> blocked
          tb, db, sb = q.popleft()
          self.blocked.append((tb, addr, db, sb))
        q.popleft()
        return
    re = self.recent_echoes[addr]
    re.append((t, dat))
    while len(re) > 8:
      re.popleft()

  def expire(self, now):
    for addr, q in self.pending.items():
      while q and (now - q[0][0]) > ECHO_TIMEOUT_S:
        tb, db, sb = q.popleft()
        self.blocked.append((tb, addr, db, sb))

  def finish(self):
    for addr, q in self.pending.items():
      while q:
        tb, db, sb = q.popleft()
        self.blocked.append((tb, addr, db, sb))


def iter_route(route):
  """Yield log events segment by segment, skipping segments that were never uploaded."""
  misses = 0
  seg = 0
  while misses < 3 and seg < 100:
    try:
      lr = LogReader(f'{DONGLE}|{route}/{seg}')
      yield from lr
      misses = 0
    except Exception as e:
      print(f'  (seg {seg} unavailable: {type(e).__name__})')
      misses += 1
    seg += 1


def run_route(route, json_path=None, truthful_shadow=False):
  assert DONGLE, 'set FORD_REPLAY_DONGLE_ID'
  sim = FordLmcSafetySim()
  echo = EchoMatcher()
  ctx = {'v_ego': 0.0, 'pressed': False, 'mads': False, 'safety_model': '',
         'controls_allowed': False, 'controls_allowed_lat': False}
  t0 = None
  seq = 0
  lmc_records = []          # (t, seq, record, ctx snapshot)
  lka_blocks = []
  tx_blocked_counter = []   # (t, value)
  latch_empty_frames = 0

  for m in iter_route(route):
    w = m.which()
    t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t

    if w == 'can':
      for c in m.can:
        if c.src == 0:
          if c.address == ADDR_BRAKE_SYS:
            sim.rx_brake_sys_features(c.dat)
          elif c.address == ADDR_PINION:
            sim.rx_steering_pinion(c.dat)
        elif c.src >= 128:
          echo.on_echo(t, c.address, c.dat)
      echo.expire(t)
    elif w == 'sendcan':
      for c in m.sendcan:
        seq += 1
        echo.on_send(t, c.address, c.dat, seq)
        if c.address == ADDR_LKA:
          if sim.tx_lka(c.dat, pressed=ctx['pressed'], truthful_shadow=truthful_shadow):
            lka_blocks.append((t, seq))
        elif c.address == ADDR_LMC:
          if sim.reset_bypass_latch_counter == 0:
            latch_empty_frames += 1
          rec = sim.tx_lmc(c.dat)
          lmc_records.append((t, seq, rec, dict(ctx)))
    elif w == 'carState':
      ctx['v_ego'] = m.carState.vEgo
      ctx['pressed'] = m.carState.steeringPressed
    elif w == 'selfdriveStateSP':
      ctx['mads'] = m.selfdriveStateSP.mads.active
    elif w == 'pandaStates':
      if len(m.pandaStates) > 0:
        ps = m.pandaStates[0]
        ctx['safety_model'] = str(ps.safetyModel)
        ctx['controls_allowed'] = bool(ps.controlsAllowed)
        ctx['controls_allowed_lat'] = bool(ps.controlsAllowedLateral)
        sim.controls_allowed = ctx['controls_allowed']
        sim.controls_allowed_lateral = ctx['controls_allowed_lat']
        if not tx_blocked_counter or tx_blocked_counter[-1][1] != ps.safetyTxBlocked:
          tx_blocked_counter.append((t, int(ps.safetyTxBlocked)))
  echo.finish()

  # ---- report ----
  mode = ' [truthful-shadow counterfactual]' if truthful_shadow else ''
  print(f'\n===== {route} ====={mode}  (t0 mono = {t0:.1f}s)')
  def rt(t):
    return t - t0
  n = len(lmc_records)
  pre = [x for x in lmc_records if x[2]['pre_latch_violation']]
  post = [x for x in lmc_records if x[2]['blocked']]
  print(f'LMC frames: {n}; latch empty at frame: {latch_empty_frames} ({100.0 * latch_empty_frames / max(n, 1):.1f}%)')
  print(f'sim violations pre-latch (no-latch counterfactual): {len(pre)}; post-latch (predicted real blocks): {len(post)}')
  for name, group in [('pre-latch', pre), ('post-latch', post)]:
    if group:
      counts = {k: sum(1 for _, _, r, _ in group if r[k]) for k in CHECK_KEYS}
      print(f'  {name} by check: ' + ', '.join(f'{k}={v}' for k, v in counts.items() if v))
  print(f'LKA action blocks (sim): {len(lka_blocks)}')

  print('echo ground truth per addr (sent / echoed / no-echo):')
  blocked_by_addr = defaultdict(list)
  for tb, addr, db, sb in echo.blocked:
    blocked_by_addr[addr].append((tb, db, sb))
  for addr in sorted(echo.sent):
    print(f'  0x{addr:X}: sent={echo.sent[addr]} echoed={echo.echoed[addr]} no-echo={len(blocked_by_addr.get(addr, []))}')

  if tx_blocked_counter:
    print(f'safetyTxBlocked counter: start={tx_blocked_counter[0][1]}, end={tx_blocked_counter[-1][1]}')
    for i in range(1, len(tx_blocked_counter)):
      tprev, vprev = tx_blocked_counter[i - 1]
      tcur, vcur = tx_blocked_counter[i]
      print(f'  t_route={rt(tcur):8.1f}s (mono {tcur:.1f}) counter {vprev} -> {vcur}')

  # join: actually-blocked LMC frames vs sim verdicts
  blocked_seqs = {sb for _, addr, _, sb in echo.blocked if addr == ADDR_LMC}
  print(f'actually-blocked LMC frames (no echo): {len(blocked_seqs)}')
  for t, s, r, c in lmc_records:
    if s in blocked_seqs or r['blocked'] or r['pre_latch_violation']:
      fired = [k for k in CHECK_KEYS if r[k]] or ['NONE(unexplained)']
      tag = ('REAL+SIM' if (s in blocked_seqs and r['blocked']) else
             'REAL only' if s in blocked_seqs else
             'SIM block' if r['blocked'] else 'SIM pre-latch only')
      line = (f'  [{tag}] t_route={rt(t):8.1f}s en={int(r["en"])} curv={r["curv"]} pa={r["pa"]}'
              + f' shadow={r["shadow_can"]} meas=[{r["meas_min"]},{r["meas_max"]}] latch={r["latch_pre"]}'
              + f' checks={fired} v={c["v_ego"]:.1f} pressed={int(c["pressed"])} mads={int(c["mads"])}'
              + f' ctl={int(c["controls_allowed"])}/{int(c["controls_allowed_lat"])} sm={c["safety_model"]}')
      print(line)
  # non-LMC real blocks, grouped
  for addr, items in sorted(blocked_by_addr.items()):
    if addr == ADDR_LMC:
      continue
    times = ', '.join(f'{rt(tb):.1f}' for tb, _, _ in items[:20])
    print(f'  non-LMC no-echo 0x{addr:X}: n={len(items)} t_route=[{times}{", ..." if len(items) > 20 else ""}]')

  if json_path:
    with open(json_path, 'w') as f:
      json.dump({'route': route, 't0': t0,
                 'lmc': [{'t': t, 'seq': s, **r, 'ctx': c} for t, s, r, c in lmc_records],
                 'blocked': [[tb, addr, sb] for tb, addr, _, sb in echo.blocked],
                 'tx_blocked_counter': tx_blocked_counter}, f)
    print(f'wrote {json_path}')
  return lmc_records, echo, tx_blocked_counter, t0


if __name__ == '__main__':
  ap = argparse.ArgumentParser()
  ap.add_argument('routes', nargs='+')
  ap.add_argument('--json', help='dump per-frame records (one file per route, suffixed)')
  ap.add_argument('--truthful-shadow', action='store_true',
                  help=('counterfactual: latch the shadow from measured curvature on frames the '
                        + 'truthful-shadow fix would republish (pressed / previously-zeroed)'))
  args = ap.parse_args()
  for route in args.routes:
    jp = f'{args.json}.{route}.json' if args.json else None
    run_route(route, jp, truthful_shadow=args.truthful_shadow)
