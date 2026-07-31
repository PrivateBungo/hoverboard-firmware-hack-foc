#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
clean=false

usage() {
  cat <<'EOF'
Usage: ./scripts/build.sh [--clean] [make arguments...]

Build the firmware from anywhere in the repository.
  --clean  Remove existing build artifacts before compiling.
  -h, --help  Show this help.

Examples:
  ./scripts/build.sh --clean
  ./scripts/build.sh VARIANT=VARIANT_USART
EOF
}

case "${1:-}" in
  --clean)
    clean=true
    shift
    ;;
  -h|--help)
    usage
    exit 0
    ;;
esac

cd "$REPO_DIR"

if [[ "$clean" == true ]]; then
  echo "==> Cleaning build artifacts"
  make clean
fi

echo "==> Building firmware"
make -j"$(nproc)" "$@"
echo "==> Firmware is in ${REPO_DIR}/build"
