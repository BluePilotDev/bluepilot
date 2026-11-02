#!/usr/bin/env python3
"""Summarize the fused primary lead consumed by BluePilot's Tesla-style UI."""

from __future__ import annotations

import argparse
import json
import math

from openpilot.tools.lib.logreader import LogReader


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("logs", nargs="+", help="rlog/qlog path(s) or a route accepted by LogReader")
  args = parser.parse_args()

  primary_track_ids: set[tuple[str, int]] = set()
  primary_distances: list[float] = []
  counts = {
    "radarState_frames": 0,
    "valid_radarState_frames": 0,
    "primary_lead_frames": 0,
    "primary_radar_frames": 0,
    "primary_vision_only_frames": 0,
    "rejected_primary_frames": 0,
    "duplicate_secondary_frames": 0,
    "left_blindspot_frames": 0,
    "right_blindspot_frames": 0,
  }

  for source in args.logs:
    for msg in LogReader(source, sort_by_time=True):
      which = msg.which()

      if which == "radarState":
        counts["radarState_frames"] += 1
        if not msg.valid:
          continue
        counts["valid_radarState_frames"] += 1
        lead_one = msg.radarState.leadOne
        lead_two = msg.radarState.leadTwo
        if lead_one.status:
          values = (float(lead_one.dRel), float(lead_one.yRel), float(lead_one.vRel))
          if not all(math.isfinite(value) for value in values) or not 0.0 < values[0] <= 140.0:
            counts["rejected_primary_frames"] += 1
            continue
          counts["primary_lead_frames"] += 1
          counts["primary_radar_frames" if getattr(lead_one, "radar", False)
                 else "primary_vision_only_frames"] += 1
          primary_distances.append(float(lead_one.dRel))
          track_id = int(getattr(lead_one, "radarTrackId", -1))
          if track_id >= 0:
            primary_track_ids.add((source, track_id))
          counts["duplicate_secondary_frames"] += bool(
            lead_two.status and track_id >= 0 and int(getattr(lead_two, "radarTrackId", -1)) == track_id
          )

      elif which == "carState":
        counts["left_blindspot_frames"] += bool(msg.carState.leftBlindspot)
        counts["right_blindspot_frames"] += bool(msg.carState.rightBlindspot)

  counts.update({
    "primary_min_distance_m": round(min(primary_distances), 3) if primary_distances else None,
    "primary_max_distance_m": round(max(primary_distances), 3) if primary_distances else None,
    "primary_unique_radar_track_ids": len(primary_track_ids),
  })
  print(json.dumps(counts, indent=2, sort_keys=True))


if __name__ == "__main__":
  main()
