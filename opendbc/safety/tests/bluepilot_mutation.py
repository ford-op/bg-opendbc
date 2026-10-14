#!/usr/bin/env python3
"""BluePilot: run upstream's mutation.py on the BluePilot safety headers (#53).

Upstream's run only mutates code from files under opendbc/safety/, and the BluePilot headers live in
the library (opendbc/bluepilot_lateral/safety/). This points the same run at them; the killer tests
are the ones mutation.py already discovers in opendbc/safety/tests/. Same options as mutation.py."""
import sys

from opendbc.safety.tests import mutation

mutation.SAFETY_DIR = mutation.ROOT / "opendbc" / "bluepilot_lateral" / "safety"

if __name__ == "__main__":
  sys.exit(mutation.main())
