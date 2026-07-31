#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIRMWARE_BIN="${FIRMWARE_BIN:-${REPO_DIR}/build/hover.bin}"
FLASH_ADDRESS="${FLASH_ADDRESS:-0x08000000}"

usage() {
  cat <<'EOF'
Usage: ./scripts/flash.sh

Flash build/hover.bin over an ST-Link SWD connection. The script uses st-flash
by default. Set PROGRAMMER_CLI to use STM32CubeProgrammer instead.

Environment variables:
  FIRMWARE_BIN   Firmware binary to flash (default: build/hover.bin)
  FLASH_ADDRESS  MCU flash address (default: 0x08000000)
  PROGRAMMER_CLI Path or command name for STM32_Programmer_CLI
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
elif [[ $# -gt 0 ]]; then
  usage >&2
  exit 2
fi

if [[ ! -f "$FIRMWARE_BIN" ]]; then
  echo "Error: firmware file not found: $FIRMWARE_BIN" >&2
  echo "Run ./scripts/build.sh first, or set FIRMWARE_BIN." >&2
  exit 1
fi

if [[ -n "${PROGRAMMER_CLI:-}" ]]; then
  if ! command -v "$PROGRAMMER_CLI" >/dev/null 2>&1 && [[ ! -x "$PROGRAMMER_CLI" ]]; then
    echo "Error: STM32CubeProgrammer CLI not found: $PROGRAMMER_CLI" >&2
    exit 1
  fi

  echo "==> Flashing with STM32CubeProgrammer"
  "$PROGRAMMER_CLI" -c port=SWD -e all -w "$FIRMWARE_BIN" "$FLASH_ADDRESS" -v -rst
elif command -v st-flash >/dev/null 2>&1; then
  echo "==> Flashing with st-flash"
  st-flash --reset write "$FIRMWARE_BIN" "$FLASH_ADDRESS"
else
  echo "Error: st-flash is not installed." >&2
  echo "Install it with: sudo apt install stlink-tools" >&2
  echo "Alternatively, set PROGRAMMER_CLI to STM32_Programmer_CLI." >&2
  exit 1
fi

echo "==> Flash complete"
