#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export IDF_CCACHE_ENABLE="${IDF_CCACHE_ENABLE:-1}"
export CCACHE_BASEDIR="${CCACHE_BASEDIR:-$PROJECT_ROOT}"

if ! command -v idf.py >/dev/null 2>&1; then
  if [ -f "$HOME/esp-idf/export.sh" ]; then
    # shellcheck disable=SC1090
    source "$HOME/esp-idf/export.sh" >/dev/null
  fi
fi

cd "$PROJECT_ROOT"
exec idf.py "$@"
