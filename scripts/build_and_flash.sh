#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

"${SCRIPT_DIR}/build_only.sh" "$@"
"${SCRIPT_DIR}/flash_only.sh" "$@"

echo "==> Done"
