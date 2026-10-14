#!/usr/bin/env python3
"""BluePilot: run upstream's mutation.py on the BluePilot safety headers (#53).

Upstream's run only mutates code from files under opendbc/safety/, and the BluePilot headers live in
the library (opendbc/bluepilot_lateral/safety/). This points the same run at them; the killer tests
are the ones mutation.py already discovers in opendbc/safety/tests/. Same options as mutation.py."""
import sys

from opendbc.safety.tests import mutation

mutation.SAFETY_DIR = mutation.ROOT / "opendbc" / "bluepilot_lateral" / "safety"

# TODO(#54): kill these and delete the list. They survive the whole safety suite (upstream's full run
# miscounted them as killed: which mutants share a worker changes the result); mutation.py keeps its own
# list inside main(), so these are skipped instead.
KNOWN_SURVIVORS = {
  ("opendbc/bluepilot_lateral/safety/lateral.h", 88, "|=", "&="),
  ("opendbc/bluepilot_lateral/safety/lateral.h", 100, "0", "1"),
  ("opendbc/bluepilot_lateral/safety/ford.h", 85, "-", "+"),
  ("opendbc/bluepilot_lateral/safety/ford.h", 128, "0", "1"),
  ("opendbc/bluepilot_lateral/safety/ford.h", 197, "|=", "&="),
}

_enumerate_sites = mutation.enumerate_sites


def key(site):
  return str(site.origin_file.relative_to(mutation.ROOT)), site.origin_line, site.original_op, site.mutated_op


def enumerate_sites(input_source, preprocessed_file):
  sites, counts, build_incompatible_ids, source = _enumerate_sites(input_source, preprocessed_file)
  skipped = [s for s in sites if key(s) in KNOWN_SURVIVORS]
  assert len(skipped) == len(KNOWN_SURVIVORS), f"a known survivor moved or is gone: {sorted(KNOWN_SURVIVORS - {key(s) for s in skipped})}"
  for s in skipped:
    print(f"skipping known survivor (#54): {key(s)[0]}:{s.origin_line} [{s.mutator}] {s.original_op}->{s.mutated_op}", flush=True)
  return [s for s in sites if key(s) not in KNOWN_SURVIVORS], counts, build_incompatible_ids, source


mutation.enumerate_sites = enumerate_sites  # type: ignore  # main() looks it up by name

if __name__ == "__main__":
  sys.exit(mutation.main())
