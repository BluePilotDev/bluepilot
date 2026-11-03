#!/usr/bin/env python3
import os
import time
import json
import jwt
from typing import cast
from pathlib import Path

from datetime import datetime, timedelta, UTC
from openpilot.common.api import api_get, get_key_pair
from openpilot.common.params import Params
from openpilot.common.spinner import Spinner
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.common.hardware import HARDWARE, PC
from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog

# BluePilot: comma / Konik / offline dongle ID switching
from bluepilot.backend_switch import BACKEND_COMMA, BACKEND_OFFLINE, reconcile_backend
# End BluePilot


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

def is_registered_device() -> bool:
  dongle = Params().get("DongleId")
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def register(show_spinner=False) -> str | None:
  """
  All devices built since March 2024 come with all
  info stored in /persist/. This is kept around
  only for devices built before then.

  With a backend update to take serial number instead
  of dongle ID to some endpoints, this can be removed
  entirely.
  """
  params = Params()
  register_start = time.monotonic()  # BluePilot: diagnostic timing for bp_register_* events

  # BluePilot: swap/clear DongleId when BPConnectBackend changed. Non-comma backends skip the
  # /persist comma dongle ID restore below — it would short-circuit Konik registration on
  # devices built since 2/28/24. Offline never attempts network registration.
  backend = reconcile_backend(params)
  cloudlog.event("bp_register_start", backend=backend, api_host=os.environ.get("API_HOST"),
                 athena_host=os.environ.get("ATHENA_HOST"), dongle_id_on_disk=params.get("DongleId"))
  # End BluePilot

  dongle_id: str | None = params.get("DongleId")
  if dongle_id is None and backend == BACKEND_COMMA and Path(Paths.persist_root()+"/comma/dongle_id").is_file():  # BluePilot: comma only
    # not all devices will have this; added early in comma 3X production (2/28/24)
    with open(Paths.persist_root()+"/comma/dongle_id") as f:
      dongle_id = f.read().strip()
    cloudlog.event("bp_register_persist_restore", dongle_id=dongle_id)  # BluePilot: diagnostic
  elif dongle_id is None and backend == BACKEND_OFFLINE:  # BluePilot: no network against bogus hosts
    dongle_id = UNREGISTERED_DONGLE_ID

  # Create registration token, in the future, this key will make JWTs directly
  jwt_algo, private_key, public_key = get_key_pair()

  if not public_key:
    dongle_id = UNREGISTERED_DONGLE_ID
    cloudlog.warning("missing public key")
  elif dongle_id is None:
    cloudlog.event("bp_register_network_attempt", backend=backend)  # BluePilot: diagnostic
    if show_spinner:
      spinner = Spinner()
      spinner.update("registering device")

    # Block until we get the imei
    serial = HARDWARE.get_serial()
    start_time = time.monotonic()
    imei: str | None = None
    while imei is None:
      try:
        imei = HARDWARE.get_imei()
      except Exception:
        cloudlog.exception("Error getting imei, trying again...")
        time.sleep(1)

      if time.monotonic() - start_time > 60 and show_spinner:
        spinner.update(f"registering device - serial: {serial}, IMEI: {imei}")

    backoff = 0
    attempt = 0  # BluePilot: diagnostic
    start_time = time.monotonic()
    while True:
      attempt += 1  # BluePilot: diagnostic
      try:
        register_token = jwt.encode({'register': True, 'exp': datetime.now(UTC).replace(tzinfo=None) + timedelta(hours=1)},
                                    cast(str, private_key), algorithm=jwt_algo)
        cloudlog.info("getting pilotauth")
        cloudlog.info("getting pilotauth")
        resp = api_get("v2/pilotauth/", method='POST', timeout=15,
                       imei=imei, imei2="", serial=serial, public_key=public_key, register_token=register_token)

        if resp.status_code in (402, 403):
          cloudlog.info(f"Unable to register device, got {resp.status_code}")
          dongle_id = UNREGISTERED_DONGLE_ID
        else:
          dongleauth = json.loads(resp.text)
          dongle_id = dongleauth["dongle_id"]
        break
      except NotImplementedError:
        # dependency issues with PyJWT will hang the registration test in backoff loop otherwise
        raise
      except Exception as e:
        # BluePilot: diagnostic -- attempt count/backoff/elapsed alongside the existing traceback
        cloudlog.event("bp_register_attempt_failed", backend=backend, attempt=attempt, backoff=backoff,
                       elapsed=time.monotonic() - start_time, error=str(e))
        cloudlog.exception("failed to authenticate")
        backoff = min(backoff + 1, 15)
        time.sleep(backoff)

      if time.monotonic() - start_time > 60 and show_spinner:
        spinner.update(f"registering device - serial: {serial}, IMEI: {imei}")
        # BluePilot: diagnostic -- this early return never persists DongleId; the device stays
        # unregistered for the rest of this boot with no further retry until the next register() call.
        cloudlog.event("bp_register_timeout", backend=backend, attempts=attempt,
                       elapsed=time.monotonic() - start_time)
        return UNREGISTERED_DONGLE_ID  # hotfix to prevent an infinite wait for registration

    if show_spinner:
      spinner.close()

  # BluePilot: diagnostic -- final outcome for this register() call
  cloudlog.event("bp_register_complete", backend=backend, dongle_id=dongle_id,
                 elapsed=time.monotonic() - register_start)

  if dongle_id:
    params.put("DongleId", dongle_id, block=True)
    set_offroad_alert("Offroad_UnregisteredHardware", (dongle_id == UNREGISTERED_DONGLE_ID) and not PC)
  return dongle_id


if __name__ == "__main__":
  print(register())
