#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

PROGRAMMER_CLI="${PROGRAMMER_CLI:-/home/gijs/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI}"
FIRMWARE_ELF="${FIRMWARE_ELF:-/home/gijs/Documents/hoverboard-firmware-hack-foc/build/hover.elf}"

cd "$REPO_DIR"

if [[ ! -x "$PROGRAMMER_CLI" ]]; then
  echo "Error: programmer CLI not executable: $PROGRAMMER_CLI" >&2
  exit 1
fi

if [[ ! -f "$FIRMWARE_ELF" ]]; then
  echo "Error: firmware file not found: $FIRMWARE_ELF" >&2
  exit 1
fi

echo "==> Flashing firmware: $FIRMWARE_ELF"
"$PROGRAMMER_CLI" \
  -c port=SWD \
  -e all \
  -w "$FIRMWARE_ELF" \
  -v \
  -rst

echo "==> Flash complete"
