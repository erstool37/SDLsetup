#!/usr/bin/env bash
# Lint the tree. Nothing here touches hardware.
#
#   dev/lint.sh          check
#   dev/lint.sh --fix    check and apply the safe fixes
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
PY="${PYTHON:-$HOME/.pyenv/versions/main/bin/python}"
exec "$PY" -m ruff check --config dev/ruff.toml scripts tools dashboard dev/tests "$@"
