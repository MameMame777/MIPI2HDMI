#!/usr/bin/env bash
# Flash BOOT_CAM.bin to QSPI over USB-JTAG.
# BEFORE running: set JP5 = JTAG and power-cycle the board.
# AFTER it succeeds: set JP5 = QSPI and power-cycle -> the camera app boots standalone.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VITIS="${VITIS_ROOT:?set VITIS_ROOT to your Vitis 2024.2 install, e.g. C:/Xilinx/Vitis/2024.2}"

[ -f "$ROOT/BOOT_CAM.bin" ] || { echo "ERROR: BOOT_CAM.bin missing — run ./build.sh first"; exit 1; }
[ -f "$ROOT/gen2/fsbl/executable.elf" ] || { echo "ERROR: gen2/fsbl/executable.elf missing — run ./build.sh"; exit 1; }

# hw_server (JTAG, channel A) — start if not already listening on 3121.
if ! netstat -ano 2>/dev/null | grep -qE "3121.*LISTEN"; then
  echo "starting hw_server ..."
  "$VITIS/bin/hw_server.bat" >/dev/null 2>&1 &
  for i in $(seq 1 15); do sleep 1; netstat -ano 2>/dev/null | grep -qE "3121.*LISTEN" && break; done
fi

echo "flashing (qspi-x4-single, our FSBL as flash-writer, with verify) ..."
"$VITIS/bin/program_flash.bat" \
  -f "$ROOT/BOOT_CAM.bin" \
  -fsbl "$ROOT/gen2/fsbl/executable.elf" \
  -flash_type qspi-x4-single -offset 0 -verify \
  -url tcp:127.0.0.1:3121

echo "Flash done. Set JP5 = QSPI and power-cycle. Console on the USB-UART @115200 8N1."
