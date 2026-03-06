#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$REPO_DIR"

echo "==> Cleaning build artifacts"
make clean

echo "==> Building firmware"
make -l

echo "==> Build complete"
