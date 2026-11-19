"""
BluePilot: how this Ford is steered, kept for the settings pages and for the next start.

The lateral library decides at car start how it steers a Ford, from the part numbers its steering
module answers (opendbc/bluepilot_lateral/core/strategy.py). This module keeps that decision in one
param, FordLateralSupport, so that:
  - the settings pages can show it, parked or driving (support_row);
  - the next start can tell that the module's software changed since this one: Ford changes it over
    the air and at the dealer. The old and new part numbers are logged;
  - the driver is told once, in an offroad alert, when the module's software changed or when the car
    is on the fallback, and is asked for a route when the software isn't in the table yet.

card calls record_start once per start and the publisher calls record_degraded when the library's
health check changes its mind during a drive.
"""
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from opendbc.bluepilot_lateral.core.strategy import PscmIdentity, UNKNOWN_DATA_FILE, UNKNOWN_STRATEGY
from opendbc.bluepilot_lateral.host_types import SupportLevel

SUPPORT_PARAM = "FordLateralSupport"
NOTICE_ALERT = "Offroad_FordLateralSupport"  # cleared by the next onroad transition (params_keys.h)

PART_NUMBERS = ("strategy", "dataFile", "calibration")
LEVEL_NAMES = {SupportLevel.fallback: "Fallback", SupportLevel.curvature: "Curvature", SupportLevel.full: "Full",
               SupportLevel.not_supported: "Not supported"}
MATCHED_BY = {"can": "CAN messages", "fw": "module software", "fixed": "your selection"}

_stored: dict | None = None  # this start's record as record_start stored it, for record_degraded


def _parts(record: dict) -> str:
  """The part numbers of a record, for people."""
  if not record.get("strategy"):
    return "no answer"
  named = [record["strategy"], record.get("dataFile") or "no data file"]
  if record.get("calibration"):
    named.append(record["calibration"])
  return " / ".join(named)


def start_record(CP, status) -> dict:
  """This start, from CarParams and the library's status (BluePilotLateral.status())."""
  identity = PscmIdentity.from_car_fw(CP.carFw)
  return {
    "vin": str(CP.carVin),
    "fingerprint": str(CP.carFingerprint),
    "matchedBy": str(CP.fingerprintSource),
    "fuzzy": bool(CP.fuzzyFingerprint),
    "active": bool(status.active),
    "level": int(status.supportLevel),
    "reason": str(status.supportReason),
    "strategy": identity.strategy,
    "dataFile": identity.data_file,
    "calibration": identity.calibration,
    "otaCounter": identity.ota_counter,
    "degradedReason": "",
  }


def compare_with_last(record: dict, last: dict | None) -> tuple[dict, str]:
  """What to store for this start, and the notice to show ("" for none).

  The part numbers kept are the last ones the module answered: a start without an answer isn't a
  change of software. A change is told on the start that finds it. The fallback is told once: again
  only when what there is to say about it changes."""
  last = last if last and last.get("vin") == record["vin"] else {}
  answered = bool(record["strategy"] and record["dataFile"])
  known = {k: last.get(k, "") for k in (*PART_NUMBERS, "otaCounter")}
  changed = answered and bool(known["strategy"]) and any(known[k] != record[k] for k in PART_NUMBERS)

  fallback = ""
  if record["active"] and record["level"] == SupportLevel.fallback:
    fallback = f"BluePilot is steering this car with its fallback: {record['reason']} ({_parts(record)})."
    if record["reason"] in (UNKNOWN_STRATEGY, UNKNOWN_DATA_FILE):
      fallback += " Please upload a route from this drive so this software can be added."

  lines = []
  if changed:
    lines.append(f"The steering module's software changed since the last drive: {_parts(known)} before, {_parts(record)} now.")
  if fallback and fallback != last.get("fallbackNotice", ""):
    lines.append(fallback)

  stored = dict(record)
  if not answered:
    stored.update(known)
  stored["fallbackNotice"] = fallback
  stored["changedFrom"] = {k: known[k] for k in PART_NUMBERS} if changed else None
  return stored, " ".join(lines)


def record_start(CI, params: Params) -> None:
  """At every car start, for a car with the lateral library: store this start, log a change of part
  numbers with the old and the new, and show the notice if there is a new one."""
  global _stored
  bp = getattr(getattr(CI, "CC", None), "bp", None)
  if bp is None or not hasattr(bp, "status"):
    return
  record = start_record(CI.CP, bp.status())
  stored, notice = compare_with_last(record, params.get(SUPPORT_PARAM))
  if stored["changedFrom"] is not None:
    cloudlog.event("BluePilot steering module changed", old=stored["changedFrom"],
                   new={k: record[k] for k in PART_NUMBERS}, otaCounter=record["otaCounter"])
  cloudlog.event("BluePilot steering support", **{k: v for k, v in stored.items() if k != "vin"})
  params.put(SUPPORT_PARAM, stored, block=True)
  _stored = stored
  if notice:
    from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert  # card only: the settings pages import this module too
    set_offroad_alert(NOTICE_ALERT, True, notice)


def record_degraded(params: Params, reason: str) -> dict | None:
  """During a drive: the health check's reason, when it changes ("" when it clears). Returns the
  record as stored, or None before a start was recorded. The write doesn't wait for the disk: this
  runs in card's loop."""
  if _stored is not None and _stored["degradedReason"] != reason:
    _stored["degradedReason"] = reason
    params.put(SUPPORT_PARAM, _stored)
  return _stored


def support_row(record: dict | None) -> tuple[str, str]:
  """The settings row: (value, description)."""
  if not record:
    return "Not read yet", "Shown after the car has been started once with this software."
  if not record["active"]:
    value = "Off"
    lines = ["BluePilot lateral control is disabled: openpilot's stock Ford command steers."]
  else:
    value = LEVEL_NAMES.get(record["level"], "Fallback")
    lines = [record["reason"] + "."]
  if record.get("degradedReason"):
    value += ", degraded"
    lines.append(f"Degraded on the last drive: {record['degradedReason']}.")
  lines.append(f"Steering module: {_parts(record)}.")
  matched = MATCHED_BY.get(record["matchedBy"], record["matchedBy"])
  lines.append(f"Car: {record['fingerprint']}, matched by {matched}{' (closest match)' if record['fuzzy'] else ''}.")
  return value, " ".join(lines)
