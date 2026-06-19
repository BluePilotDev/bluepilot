#!/usr/bin/env python3
# simple pandad wrapper that updates the panda first
import os
import usb1
import time
import signal
import subprocess

from panda import Panda, PandaDFU, PandaProtocolMismatch, McuType, FW_PATH
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.system.hardware import HARDWARE
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.pandad.rivian_long_flasher import flash_rivian_long


def get_expected_signature() -> bytes:
  try:
    fn = os.path.join(FW_PATH, McuType.H7.config.app_fn)
    return Panda.get_signature_from_firmware(fn)
  except Exception:
    cloudlog.exception("Error computing expected signature")
    return b""

def flash_panda(panda_serial: str) -> Panda:
  try:
    panda = Panda(panda_serial)
  except PandaProtocolMismatch:
    cloudlog.warning("detected protocol mismatch, reflashing panda")
    HARDWARE.recover_internal_panda()
    raise

  # skip flashing if the detected panda is not supported
  supported_panda = check_panda_support(panda)
  if not supported_panda:
    cloudlog.warning(f"Panda {panda_serial} is not supported (hw_type: {panda.get_type()}), skipping flash...")
    return panda

  fw_signature = get_expected_signature()
  internal_panda = panda.is_internal()

  panda_version = "bootstub" if panda.bootstub else panda.get_version()
  panda_signature = b"" if panda.bootstub else panda.get_signature()
  cloudlog.warning(f"Panda {panda_serial} connected, version: {panda_version}, signature {panda_signature.hex()[:16]}, expected {fw_signature.hex()[:16]}")

  if panda.bootstub or panda_signature != fw_signature:
    cloudlog.info("Panda firmware out of date, update required")
    panda.flash()
    cloudlog.info("Done flashing")

  if panda.bootstub:
    bootstub_version = panda.get_version()
    cloudlog.info(f"Flashed firmware not booting, flashing development bootloader. {bootstub_version=}, {internal_panda=}")
    if internal_panda:
      HARDWARE.recover_internal_panda()
    panda.recover(reset=(not internal_panda))
    cloudlog.info("Done flashing bootstub")

  if panda.bootstub:
    cloudlog.info("Panda still not booting, exiting")
    raise AssertionError

  panda_signature = panda.get_signature()
  if panda_signature != fw_signature:
    cloudlog.info("Version mismatch after flashing, exiting")
    raise AssertionError

  return panda


def check_panda_support(panda) -> bool:
  hw_type = panda.get_type()
  if hw_type in Panda.SUPPORTED_DEVICES:
    return True

  return False


def main() -> None:
  # signal pandad to close the relay and exit
  processes: list[subprocess.Popen] = []

  def signal_handler(signum, frame):
    cloudlog.info(f"Caught signal {signum}, exiting")
    nonlocal do_exit
    do_exit = True
    for p in processes:
      try:
        p.send_signal(signal.SIGINT)
      except Exception:
        pass

  do_exit = False
  signal.signal(signal.SIGINT, signal_handler)

  count = 0
  first_run = True
  params = Params()
  no_internal_panda_count = 0

  while not do_exit:
    try:
      count += 1
      cloudlog.event("pandad.flash_and_connect", count=count)
      params.remove("PandaSignatures")
      processes.clear()

      # Handle missing internal panda
      if no_internal_panda_count > 0:
        if no_internal_panda_count == 3:
          cloudlog.info("No pandas found, putting internal panda into DFU")
          HARDWARE.recover_internal_panda()
        else:
          cloudlog.info("No pandas found, resetting internal panda")
          HARDWARE.reset_internal_panda()
        time.sleep(3)  # wait to come back up

      # Flash all Pandas in DFU mode
      dfu_serials = PandaDFU.list()
      if len(dfu_serials) > 0:
        for serial in dfu_serials:
          cloudlog.info(f"Panda in DFU mode found, flashing recovery {serial}")
          PandaDFU(serial).recover()
        time.sleep(1)

      panda_serials = Panda.list()
      if len(panda_serials) == 0:
        no_internal_panda_count += 1
        continue

      cloudlog.info(f"{len(panda_serials)} panda(s) found, connecting - {panda_serials}")

      # When multiple pandas are present (e.g. red panda attached), USB enumeration
      # order is non-deterministic — the internal tres can appear at any position.
      # Explicitly identify and move it to index 0 before flashing so the is_internal()
      # guard below doesn't spin-loop for minutes waiting for list order to flip.
      if HARDWARE.has_internal_panda() and len(panda_serials) > 1:
        for s in panda_serials:
          try:
            p = Panda(s)
            if p.is_internal():
              p.close()
              panda_serials = [s] + [x for x in panda_serials if x != s]
              cloudlog.info(f"Internal panda {s} identified, reordered list: {panda_serials}")
              break
            p.close()
          except Exception:
            cloudlog.exception(f"Error checking panda {s} for internal status")

      # Flash the primary panda (tres / internal)
      panda_serial = panda_serials[0]
      panda = flash_panda(panda_serial)

      # flash Rivian longitudinal upgrade panda
      flash_rivian_long(panda)

      # Ensure internal panda is present if expected
      if HARDWARE.has_internal_panda() and not panda.is_internal():
        cloudlog.error("Internal panda is missing, trying again")
        no_internal_panda_count += 1
        continue
      no_internal_panda_count = 0

      # Collect firmware signatures — primary panda first
      all_signatures = panda.get_signature()

      # skip health check if the detected panda is not supported
      supported_panda = check_panda_support(panda)
      if not supported_panda:
        cloudlog.warning(f"Panda {panda.get_usb_serial()} is not supported (hw_type: {panda.get_type()}), skipping health check...")
        continue

      # check health for lost heartbeat
      health = panda.health()
      if health["heartbeat_lost"]:
        params.put_bool("PandaHeartbeatLost", True)
        cloudlog.event("heartbeat lost", deviceState=health, serial=panda.get_usb_serial())
      if health["som_reset_triggered"]:
        params.put_bool("PandaSomResetTriggered", True)
        cloudlog.event("panda.som_reset_triggered", health=health, serial=panda.get_usb_serial())

      if first_run:
        # reset panda to ensure we're in a good state
        cloudlog.info(f"Resetting panda {panda.get_usb_serial()}")
        panda.reset(reconnect=True)

      panda.close()

      # Flash and prepare any secondary pandas (e.g. external red panda for CANFD_PSCM_RP)
      secondary_serials: list[tuple[str, int]] = []
      for idx, serial in enumerate(panda_serials[1:], start=1):
        try:
          secondary = flash_panda(serial)
          all_signatures += secondary.get_signature()
          if first_run:
            cloudlog.info(f"Resetting secondary panda {secondary.get_usb_serial()} (index {idx})")
            secondary.reset(reconnect=True)
          secondary.close()
          secondary_serials.append((serial, idx))
        except Exception:
          cloudlog.exception(f"Failed to set up secondary panda {serial} (index {idx}), skipping")

      # log all panda fw versions
      params.put("PandaSignatures", all_signatures)

    # TODO: wrap all panda exceptions in a base panda exception
    except (usb1.USBErrorNoDevice, usb1.USBErrorPipe):
      # a panda was disconnected while setting everything up. let's try again
      cloudlog.exception("Panda USB exception while setting up")
      continue
    except PandaProtocolMismatch:
      cloudlog.exception("pandad.protocol_mismatch")
      continue
    except Exception:
      cloudlog.exception("pandad.uncaught_exception")
      continue

    first_run = False

    # Launch a single pandad C++ process for the primary panda.
    # Secondary panda serials are passed as extra args (serial idx pairs) so
    # the primary pandad manages all CAN recv through a single PubMaster,
    # avoiding the msgq single-writer constraint on the "can" topic.
    os.environ['MANAGER_DAEMON'] = 'pandad'
    pandad_path = os.path.join(BASEDIR, "selfdrive/pandad")
    cmd = ["./pandad", panda_serial, "0"]
    for serial, idx in secondary_serials:
      cloudlog.info(f"Passing secondary panda {serial} (index {idx}) to primary pandad")
      cmd += [serial, str(idx)]
    processes = [subprocess.Popen(cmd, cwd=pandad_path)]

    # Wait for the single pandad process to exit
    processes[0].wait()
    processes = []


if __name__ == "__main__":
  main()
