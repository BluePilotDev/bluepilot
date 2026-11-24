#!/usr/bin/env python3
"""BluePilot: run upstream's mutation.py on the BluePilot safety headers (#53).

Upstream's run only mutates code from files under <pkg>/safety/, and the BluePilot headers live in
the library (<pkg>/bluepilot_lateral/safety/). This points the same run at them; the killer tests
are the ones mutation.py already discovers in <pkg>/safety/tests/, the BluePilot ones through the
loader there (test_bluepilot_lateral.py). Same options as mutation.py. <pkg> is the host's opendbc
package, whatever it's named (opendbc, iqdbc): the one the library is mounted in."""
import importlib
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[3].name
mutation = importlib.import_module(f"{PKG}.safety.tests.mutation")

mutation.SAFETY_DIR = mutation.ROOT / PKG / "bluepilot_lateral" / "safety"

# Equivalent mutants: no test can kill them, so they are skipped (mutation.py keeps its own list inside
# main()). ford.h's +-0.02 range check in ford_lmc_checks is the same +-1000 CAN cap as lateral.h's
# max_curvature check, which blocks every frame it would (its steering-off and controls-not-allowed
# checks cover the rest), and the range check writes no state (only ford_bp_curvature_cmd_checks
# resets desired_last). It stays as the backstop if a platform's max_curvature ever differs, and a
# test can reach it then.
KNOWN_SURVIVORS = {
  (f"{PKG}/bluepilot_lateral/safety/ford.h", 197, "|=", "&="),
}

_enumerate_sites = mutation.enumerate_sites


def key(site):
  return str(site.origin_file.relative_to(mutation.ROOT)), site.origin_line, site.original_op, site.mutated_op


def enumerate_sites(input_source, preprocessed_file):
  sites, counts, build_incompatible_ids, source = _enumerate_sites(input_source, preprocessed_file)
  skipped = [s for s in sites if key(s) in KNOWN_SURVIVORS]
  assert len(skipped) == len(KNOWN_SURVIVORS), f"an equivalent mutant moved or is gone: {sorted(KNOWN_SURVIVORS - {key(s) for s in skipped})}"
  for s in skipped:
    print(f"skipping equivalent mutant: {key(s)[0]}:{s.origin_line} [{s.mutator}] {s.original_op}->{s.mutated_op}", flush=True)
  return [s for s in sites if key(s) not in KNOWN_SURVIVORS], counts, build_incompatible_ids, source


mutation.enumerate_sites = enumerate_sites  # main() looks it up by name

if __name__ == "__main__":
  sys.exit(mutation.main())
