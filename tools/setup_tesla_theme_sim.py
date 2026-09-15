#!/usr/bin/env python3
"""Create an isolated desktop-UI profile for the automatic Tesla theme."""

from __future__ import annotations

import argparse
import os
import re

from openpilot.common.params import Params
from openpilot.system.version import (
  get_version,
  sunnylink_consent_version,
  terms_version,
  terms_version_sp,
  training_version,
)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("profile", help="Isolated OPENPILOT_PREFIX used by replay and ui.py")
  parser.add_argument("theme", nargs="?", choices=("auto", "light", "dark"), default="auto",
                      help="legacy light/dark arguments are accepted; Tesla now switches automatically")
  parser.add_argument("--metric", action="store_true", help="Render km/h instead of mph")
  args = parser.parse_args()

  if re.fullmatch(r"[A-Za-z0-9_-]+", args.profile) is None:
    parser.error("profile may contain only letters, numbers, underscores, and hyphens")

  os.environ["OPENPILOT_PREFIX"] = args.profile
  params = Params()
  theme_value = "tesla"
  for key, value in {
    "HasAcceptedTerms": terms_version,
    "CompletedTrainingVersion": training_version,
    "HasAcceptedTermsSP": terms_version_sp,
    "CompletedSunnylinkConsentVersion": sunnylink_consent_version,
    "DongleId": "bdbbc283faf3afd4",
    "BPLastSeenVersion": get_version(),
    "BPThemePack": theme_value,
  }.items():
    params.put(key, value, block=True)

  for key, value in {
    "IsMetric": args.metric,
    "BlindSpot": True,
    "AlwaysOnDM": True,
  }.items():
    params.put_bool(key, value, block=True)

  if params.get("BPThemePack") != theme_value:
    raise RuntimeError("failed to persist BPThemePack in the simulator profile")
  units = "metric" if args.metric else "imperial"
  print(f"{args.profile}: Tesla Auto ({units}) simulator profile ready")


if __name__ == "__main__":
  main()
