#!/usr/bin/env bash
# Show exactly what BlueGill changes in the firmware vs the upstream Bluejay base it forked from.
# BlueGill is a hard fork of Bluejay v0.21.0; the base commit is tagged `v0.21.0-base`, so the
# firmware delta is a plain git diff. (Replaces the old vendor-submodule overlay diff.)
#
# Exit 0 if src/ is byte-identical to the base, 1 if it differs (usable as a CI gate).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
BASE="${1:-v0.21.0-base}"

git -C "${ROOT}" rev-parse -q --verify "${BASE}^{commit}" >/dev/null 2>&1 \
  || { echo "ERROR: base ref '${BASE}' not found (expected the fork base tag)." >&2; exit 2; }

git -C "${ROOT}" --no-pager diff "${BASE}" -- src/
if git -C "${ROOT}" diff --quiet "${BASE}" -- src/; then
  echo "# src/ is byte-identical to ${BASE} (BlueGill == upstream Bluejay base)."
  exit 0
fi
exit 1
