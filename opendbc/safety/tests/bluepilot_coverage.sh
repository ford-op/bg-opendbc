#!/usr/bin/env bash
# BluePilot: 100% line coverage of the BluePilot safety headers (#53). Upstream's test.sh only measures
# files under opendbc/safety/, and the headers live in opendbc/bluepilot_lateral/safety/.
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

source ../../../setup.sh

# reset coverage data, then run the safety tests (the BluePilot ones live here too)
rm -f ./libsafety/*.gcda
python -m unittest discover -s .

if [ "$(uname)" = "Darwin" ]; then
  GCOV_EXEC="llvm-cov gcov"
else
  GCOV_EXEC="gcov"
fi

ROOT="$(cd ../../.. && pwd)"
if ! OUT="$(gcovr -r "$ROOT" --gcov-executable "$GCOV_EXEC" -d --fail-under-line=100 \
             -f "$ROOT/opendbc/bluepilot_lateral/safety/" -f "$ROOT/opendbc/safety/modes/ford.h")"; then
  echo -e "FAILED:\n$OUT"
  exit 1
fi
echo "$OUT"
echo "SUCCESS: the BluePilot safety headers and modes/ford.h have 100% coverage!"
