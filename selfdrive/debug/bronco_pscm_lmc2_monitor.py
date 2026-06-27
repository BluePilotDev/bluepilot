#!/usr/bin/env python3
"""
Bronco 6G PSCM read-only test monitor.

Purpose: with the red panda passively monitoring the PSCM CAN-FD connector
(FORD_BRONCO_MK6 = CANFD_PSCM | CANFD_PSCM_RP_MON), answer ONE question:

    Does LateralMotionControl2 (0x3D6) that the C3X sends on the CAN/IPMA bus
    actually arrive at the PSCM CAN-FD bus via the gateway?

If YES  -> the gateway forwards LMC2; the injection harness is unnecessary.
If NO   -> the gateway does not forward it; direct injection is needed.

It also guards against the FALSE NEGATIVE: if the C3X never transmits LMC2
(openpilot not engaged / not commanding steer), "no LMC2 at PSCM" is meaningless.

Run on device:  PYTHONPATH=/data/openpilot python3 -m selfdrive.debug.bronco_pscm_lmc2_monitor [seconds]
Default window: 15 s.  Drive / engage openpilot during the window so it commands steering.
"""
import sys
import time
from collections import defaultdict

import cereal.messaging as messaging

# Ford message addresses (same on all current Ford ports)
LMC2 = 0x3D6   # LateralMotionControl2  (what the PSCM needs)
LMC = 0x3CA    # LateralMotionControl   (legacy / CAN camera)
EPAS_INFO = 0x82
STEERING_PINION = 0x7E
LANE_ASSIST_3 = 0x3CC

PSCM_SIGNATURE = {EPAS_INFO, STEERING_PINION, LANE_ASSIST_3}

# bus_offset = 4 per panda. internal C3X = primary -> buses 0-2; red panda = secondary -> buses 4-6.
C3X_BUSES = {0, 1, 2}
RP_BUSES = {4, 5, 6}

NAMES = {LMC2: "LMC2(0x3D6)", LMC: "LMC(0x3CA)", EPAS_INFO: "EPAS_INFO(0x82)",
         STEERING_PINION: "SteeringPinion(0x7E)", LANE_ASSIST_3: "LaneAssist3(0x3CC)"}


def main(dur: float) -> None:
  can = messaging.sub_sock("can", timeout=100)
  sendcan = messaging.sub_sock("sendcan", timeout=100)

  rx = defaultdict(int)            # (bus, addr) -> count   (frames seen on the wire)
  tx = defaultdict(int)            # (bus, addr) -> count   (openpilot sendcan)
  bus_total = defaultdict(int)

  print(f"Listening {dur:.0f}s ... drive / engage openpilot so it commands steering.\n")
  t0 = time.monotonic()
  while time.monotonic() - t0 < dur:
    for m in messaging.drain_sock(can):
      for c in m.can:
        if c.src >= 128:  # skip TX-echo / rejected pseudo-buses
          continue
        bus_total[c.src] += 1
        if c.address in NAMES:
          rx[(c.src, c.address)] += 1
    for m in messaging.drain_sock(sendcan):
      for c in m.sendcan:
        if c.address in NAMES:
          tx[(c.src, c.address)] += 1

  elapsed = time.monotonic() - t0

  def rate(n):
    return n / elapsed if elapsed else 0.0

  # --- which red-panda bus is the PSCM bus? (the one carrying PSCM signature msgs) ---
  pscm_bus = None
  best = 0
  for b in sorted(RP_BUSES):
    sig = sum(rx[(b, a)] for a in PSCM_SIGNATURE)
    if sig > best:
      best, pscm_bus = sig, b

  print("=== buses seen (frames/s) ===")
  for b in sorted(bus_total):
    side = "C3X/IPMA" if b in C3X_BUSES else ("red-panda" if b in RP_BUSES else "?")
    print(f"  bus {b:>2} [{side}]: {rate(bus_total[b]):7.1f} fps")

  print("\n=== key messages (frames/s) ===")
  print(f"  {'msg':<22}{'bus':>5}{'src':>11}{'fps':>9}")
  for (b, a), n in sorted(rx.items()):
    side = "C3X/IPMA" if b in C3X_BUSES else ("red-panda" if b in RP_BUSES else "?")
    print(f"  {NAMES[a]:<22}{b:>5}{side:>11}{rate(n):>9.1f}")
  for (b, a), n in sorted(tx.items()):
    print(f"  {NAMES[a]:<22}{b:>5}{'TX(sendcan)':>11}{rate(n):>9.1f}")

  # --- C3X actually sending LMC2? (sendcan, or seen on a C3X bus) ---
  c3x_lmc2 = sum(tx[(b, LMC2)] for b in C3X_BUSES) + sum(rx[(b, LMC2)] for b in C3X_BUSES)
  # --- LMC2 present on the PSCM bus? ---
  pscm_lmc2 = rx[(pscm_bus, LMC2)] if pscm_bus is not None else 0

  print("\n" + "=" * 60)
  if pscm_bus is None:
    print("WARNING: no PSCM signature messages on any red-panda bus (4/5/6).")
    print("         The red panda is not seeing the PSCM CAN-FD bus.")
    print("         Check the harness tap / connection before trusting any result.")
    return
  print(f"PSCM bus identified: bus {pscm_bus} "
        f"(EPAS/SteeringPinion present at {rate(best):.0f} fps total)")

  if c3x_lmc2 == 0:
    print("\nVERDICT: INCONCLUSIVE — the C3X is NOT transmitting LMC2 (0x3D6).")
    print("         openpilot must be engaged and commanding steering for this test.")
    print("         Engage, ensure the wheel is being commanded, and re-run.")
  elif pscm_lmc2 > 0:
    print(f"\nVERDICT: LMC2 IS reaching the PSCM  ({rate(pscm_lmc2):.1f} fps on bus {pscm_bus}).")
    print("         The gateway forwards LMC2 -> the injection harness is NOT needed.")
    print("         Stay on CANFD_PSCM (C3X sends LMC2 via gateway).")
  else:
    print("\nVERDICT: LMC2 is NOT reaching the PSCM.")
    print(f"         C3X is sending LMC2 (~{rate(c3x_lmc2):.1f} fps) but none appears on bus {pscm_bus}.")
    print("         The gateway does not forward it -> direct injection at the PSCM is required.")
  print("=" * 60)


if __name__ == "__main__":
  duration = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
  main(duration)
