#!/usr/bin/env bash
# BluePilot: 100% line coverage of the BluePilot safety headers (#53). Upstream's test.sh only measures
# files under opendbc/safety/, and the headers live in opendbc/bluepilot_lateral/safety/.
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
ROOT="$(cd "$DIR/../../../.." && pwd)"
cd "$ROOT/opendbc/safety/tests"

source "$ROOT/setup.sh"

# reset coverage data, then run upstream's safety tests (the BluePilot ones come in through
# test_bluepilot_lateral.py, the loader)
rm -f ./libsafety/*.gcda ./libsafety/*.gcno
SAFETY_COVERAGE=1 python -m unittest discover -s .

if [ "$(uname)" = "Darwin" ]; then
  GCOV_EXEC="llvm-cov gcov"
else
  GCOV_EXEC="gcov"
fi

if ! OUT="$(gcovr -r "$ROOT" --gcov-executable "$GCOV_EXEC" -d --fail-under-line=100 \
             -f "$ROOT/opendbc/bluepilot_lateral/safety/" -f "$ROOT/opendbc/safety/modes/ford.h")"; then
  echo -e "FAILED:\n$OUT"
  exit 1
fi
echo "$OUT"
echo "SUCCESS: the BluePilot safety headers and modes/ford.h have 100% coverage!"
