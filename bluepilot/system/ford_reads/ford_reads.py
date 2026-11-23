#!/usr/bin/env python3
"""ford_reads.py: read-only diagnostic reads of a Ford's modules, run on a comma device over SSH.

Truck parked, ignition on, engine off. The script stops openpilot first (the tmux session "comma", then
pandad); openpilot stays stopped until you reboot.

  python3 ford_reads.py --save-as before                    read, save /data/ford-reads/before.json, print a summary
  python3 ford_reads.py --compare before --save-as after    the same, and list the fault codes that are new
  python3 ford_reads.py --dry-run                           print every request it would send; sends nothing
  python3 ford_reads.py --summary before.json [--compare other.json]    print a saved read again; sends nothing
  sudo reboot                                               start openpilot again

What it sends, per module: 10 03 (extended session), 22 <identifier> (read data by identifier),
19 02 FF (read fault codes). Every request goes through guard(), which raises before anything reaches the bus
unless the service is on ALLOWED. There is no write (2E), security access (27), routine (31), reset (11),
fault-code clear (14), programming session (10 02) or memory read (23).

Modules: steering (PSCM), brakes (ABS), camera (IPMA), engine (PCM), body (BCM), cluster (IPC), gateway (GWM),
steering column (SCCM). Identifiers: part and software numbers and the module serial (F1xx), and the As-Built
configuration blocks (DE00-DE0F). The VIN is not requested.

Each module is asked on bus 0 (the harness). One that does not answer there is asked once more on bus 1, which
the panda's diagnostic mode connects to the OBD-II port.
"""
import argparse
import json
import os
import subprocess
import sys
import time

OUT_DIR = "/data/ford-reads"

ALLOWED = {0x10: {0x01, 0x03}, 0x3E: None, 0x22: None, 0x19: None}
MODULES = {  # tx address: name (Ford HS-CAN diagnostic addresses)
  0x730: "PSCM", 0x760: "ABS", 0x706: "IPMA", 0x7E0: "PCM", 0x726: "BCM", 0x720: "IPC",
}
ID_DIDS = [0xF10A, 0xF110, 0xF111, 0xF113, 0xF124, 0xF125, 0xF162, 0xF163, 0xF188, 0xF18C]
ASBUILT_DIDS = list(range(0xDE00, 0xDE10))


def guard(client, allowed=ALLOWED, dids=None):
  """dids: when given, service 0x22 is allowed for these identifiers only"""
  orig = client._uds_request
  def checked(service_type, subfunction=None, data=None):
    s = int(service_type)
    if s not in allowed or (allowed[s] is not None and subfunction not in allowed[s]):
      raise RuntimeError(f"blocked: UDS service 0x{s:02X} sub {subfunction} is not on the read-only allowlist")
    if dids is not None and (s != 0x22 or data is None or len(data) != 2 or int.from_bytes(data, "big") not in dids):
      raise RuntimeError(f"blocked: only 22 <DID> for the read check's DIDs may be sent, not {s:02X} {data!r}")
    return orig(service_type, subfunction, data)
  client._uds_request = checked
  return client


def read_modules(panda, bus, scan=False):
  """The six modules' reads (identifiers, As-Built, DTCs), through the guard. panda: anything with a panda's
  can_send / can_recv. Returns the result dict that main() writes."""
  from opendbc.car.uds import (UdsClient, SESSION_TYPE, DTC_REPORT_TYPE, DTC_STATUS_MASK_TYPE,
                               NegativeResponseError, MessageTimeoutError)
  out = {"time": time.strftime("%Y-%m-%dT%H:%M:%S"), "bus": bus, "modules": {}}
  for addr, name in MODULES.items():
    c = guard(UdsClient(panda, addr, bus=bus, timeout=0.3))
    m = out["modules"][name] = {"addr": hex(addr), "dids": {}, "errors": {}}
    try:
      c.diagnostic_session_control(SESSION_TYPE.EXTENDED_DIAGNOSTIC)
    except (NegativeResponseError, MessageTimeoutError) as e:
      m["errors"]["session"] = repr(e)
      if isinstance(e, MessageTimeoutError):
        continue  # module not answering on this bus
    dids = ID_DIDS + ASBUILT_DIDS + (PSCM_EXTRA_DIDS if addr == 0x730 else []) + (SCAN_DIDS if (scan and addr == 0x730) else [])
    for d in dids:
      try:
        m["dids"][f"{d:04X}"] = c.read_data_by_identifier(d).hex()
      except NegativeResponseError as e:
        m["errors"][f"{d:04X}"] = str(e)
      except MessageTimeoutError:
        m["errors"][f"{d:04X}"] = "timeout"
    try:
      m["dtc_raw"] = c.read_dtc_information(DTC_REPORT_TYPE.DTC_BY_STATUS_MASK, DTC_STATUS_MASK_TYPE.ALL).hex()
    except (NegativeResponseError, MessageTimeoutError) as e:
      m["errors"]["dtc"] = repr(e)
  return out


MODULES.update({0x716: "GWM", 0x724: "SCCM"})
PSCM_EXTRA_DIDS = []  # nothing beyond ID_DIDS and ASBUILT_DIDS is read
SCAN_DIDS = []


def plan() -> list[str]:
  lines = []
  for addr, name in MODULES.items():
    lines.append(f"{name:5s} 0x{addr:03X}  10 03 (extended session)")
    lines += [f"{name:5s} 0x{addr:03X}  22 {d:04X}" for d in ID_DIDS + ASBUILT_DIDS]
    lines.append(f"{name:5s} 0x{addr:03X}  19 02 FF (fault codes)")
  return lines


def stop_openpilot(run=subprocess.run, sleep=time.sleep) -> None:
  """openpilot runs in the tmux session "comma"; AGNOS's own service stops it by killing that session."""
  quiet = {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL}
  print("stopping openpilot (it stays stopped until you reboot)")
  run(["tmux", "kill-session", "-t", "comma"], **quiet)
  sleep(1)
  run(["pkill", "-f", "pandad"], **quiet)
  sleep(7)  # the panda leaves the car's safety mode 5 s after pandad's last heartbeat
  if run(["pgrep", "-f", "pandad"], **quiet).returncode == 0:
    sys.exit("pandad is still running: run the same command again")


def no_answer(m: dict) -> bool:
  return not m["dids"] and "dtc_raw" not in m and set(m["errors"]) == {"session"}


def read_all(panda) -> dict:
  """read_modules() on bus 0, then once more on bus 1 for the modules that stayed silent."""
  out = read_modules(panda, 0)
  for m in out["modules"].values():
    m["bus"] = 0
  silent = {addr: name for addr, name in MODULES.items() if no_answer(out["modules"][name])}
  if silent:
    everything = dict(MODULES)
    MODULES.clear()
    MODULES.update(silent)
    try:
      second = read_modules(panda, 1)
    finally:
      MODULES.clear()
      MODULES.update(everything)
    for name, m in second["modules"].items():
      if not no_answer(m):
        m["bus"] = 1
        out["modules"][name] = m
  del out["bus"]
  out["tool"] = "ford_reads 1"
  return out


def text(hexstr: str) -> str:
  raw = bytes.fromhex(hexstr).rstrip(b"\x00")
  return raw.decode() if raw and all(32 <= b < 127 for b in raw) else hexstr.upper()


def codes(m: dict) -> dict[str, str] | None:
  """The 19 02 answer is the availability mask, then 4 bytes per code: 3 code bytes and the status byte."""
  if "dtc_raw" not in m:
    return None
  b = bytes.fromhex(m["dtc_raw"])[1:]
  out = {}
  for i in range(0, len(b) - 3, 4):
    b0, b1, b2, status = b[i:i + 4]
    out[f"{'PCBU'[b0 >> 6]}{(b0 >> 4) & 3}{b0 & 0xF:X}{b1:02X}:{b2:02X}"] = f"{status:02X}"
  return out


def summary(result: dict, label: str, base: dict | None = None, base_label: str = "") -> None:
  print(f"\nread {result['time']} ({label})")
  new = []
  for name, m in result["modules"].items():
    if no_answer(m):
      print(f"\n{name} {m['addr']}: NO ANSWER")
      continue
    print(f"\n{name} {m['addr']} (bus {m.get('bus', '?')})")
    for did, value in m["dids"].items():
      if not did.startswith("DE"):
        print(f"  {did} {text(value)}")
    for did, value in m["dids"].items():
      if did.startswith("DE"):
        print(f"  As-Built {did} {value.upper()}")
    found = codes(m)
    if found is None:
      print(f"  codes: not read ({m['errors'].get('dtc', '?')[:60]})")
      continue
    old = codes(base["modules"].get(name, {})) if base else None
    unread = base is not None and old is None  # nothing to compare with: none of its codes is called new
    print(f"  codes: {len(found)}" + ("" if found else " (none)") + (f" (not read in {base_label})" if unread else ""))
    for code, status in found.items():
      is_new = old is not None and code not in old
      new += [f"{name} {code}"] if is_new else []
      print(f"    {code} status {status}" + ("  NEW" if is_new else ""))
  if base is not None:
    print(f"\nnew codes since {base_label}: {len(new)}" + "".join(f"\n  {c}" for c in new))


def main() -> int:
  ap = argparse.ArgumentParser(description="Read-only reads of a Ford's modules from a comma device.")
  ap.add_argument("--save-as", metavar="NAME", help="save the read as NAME.json in the output folder")
  ap.add_argument("--compare", metavar="NAME", help="an earlier read (a name or a .json path): list the codes new since it")
  ap.add_argument("--summary", metavar="FILE", help="print a saved read again; sends nothing")
  ap.add_argument("--dry-run", action="store_true", help="print the requests; sends nothing")
  ap.add_argument("--out", default=OUT_DIR, help=f"output folder (default {OUT_DIR})")
  a = ap.parse_args()
  named = lambda v: v if os.path.sep in v or v.endswith(".json") else os.path.join(a.out, f"{v}.json")

  if a.dry_run:
    lines = plan()
    print("\n".join(lines))
    print(f"{len(lines)} requests on bus 0, all read-only; a module silent there is asked once more on bus 1")
    return 0

  base = base_label = None
  if a.compare:
    with open(named(a.compare)) as f:
      base, base_label = json.load(f), os.path.basename(named(a.compare))
  if a.summary:
    with open(named(a.summary)) as f:
      summary(json.load(f), os.path.basename(named(a.summary)), base, base_label)
    return 0

  fn = named(a.save_as or time.strftime("read-%Y%m%dT%H%M%S"))
  if os.path.exists(fn):
    sys.exit(f"{fn} exists already: pick another name (nothing was sent)")
  os.makedirs(os.path.dirname(fn) or ".", exist_ok=True)

  stop_openpilot()
  for p in ("/data/pythonpath", "/data/openpilot"):  # where the openpilot tree puts opendbc and panda
    if os.path.isdir(p) and p not in sys.path:
      sys.path.append(p)
  from opendbc.car.structs import CarParams
  from panda import Panda
  panda = Panda()
  panda.set_safety_mode(CarParams.SafetyModel.elm327)  # the panda's diagnostic mode
  panda.set_heartbeat_disabled()  # or the panda goes silent 5 s later, with no pandad to send a heartbeat
  result = read_all(panda)
  if all(no_answer(m) for m in result["modules"].values()):
    sys.exit("no module answered and nothing was saved: is the ignition on? Run the same command again.")

  with open(fn, "w") as f:
    json.dump(result, f, indent=1)
  summary(result, os.path.basename(fn), base, base_label)
  print(f"\nsaved {fn}")
  print("openpilot is stopped: sudo reboot to start it again")
  return 0


if __name__ == "__main__":
  sys.exit(main())
