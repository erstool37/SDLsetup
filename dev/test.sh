#!/usr/bin/env bash
# Run every test suite. No hardware, no motion, no network.
#
#   dev/test.sh
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
PY="${PYTHON:-$HOME/.pyenv/versions/main/bin/python}"
fail=0
for t in dev/tests/test_*.py; do
  printf "  %-34s " "$(basename "$t")"
  if out=$("$PY" "$t" 2>&1); then
    case "$out" in *"ALL PASS"*|*"OK"*) echo "PASS" ;; *) echo "PASS" ;; esac
  else
    echo "FAIL"; echo "$out" | tail -12 | sed 's/^/      /'; fail=1
  fi
done
exit $fail
