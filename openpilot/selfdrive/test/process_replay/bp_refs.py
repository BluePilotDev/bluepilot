"""BluePilot: which repo's process-replay branch holds the references for test_processes.py's BP_REF_CARS.

The tree decides, not the repo CI runs in: full BluePilot (BPVERSION at the repo root) reads the references that
BluePilotDev/bluepilot publishes, bp-sunnypilot reads ford-op/bp-sunnypilot's. That holds for a developer's local run
and for CI in a fork too. BP_REF_REPO (owner/repo) overrides it.
"""
import os

from openpilot.common.basedir import BASEDIR


def bp_ref_repo(environ=os.environ, root=BASEDIR) -> str:
  if repo := environ.get("BP_REF_REPO"):
    return repo
  return "BluePilotDev/bluepilot" if os.path.isfile(os.path.join(root, "BPVERSION")) else "ford-op/bp-sunnypilot"
