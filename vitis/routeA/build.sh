#!/usr/bin/env bash
# Reproducible Route A build: FSBL + standalone BSP (from the XSA) -> compile the bare-metal
# camera app -> BOOT_CAM.bin. Run from a shell with the Vitis 2024.2 tools available.
#
# Prerequisite (one-time, in Vivado): generate vloop_probes2.xsa with UART1 + QSPI enabled:
#   vivado -mode batch -source ../../vivado/rebuild_ps7_uart_qspi.tcl
# That re-exports vitis/routeA/vloop_probes2.xsa (PYNQ-safe: FCLK0/DDR/PL unchanged).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VITIS="${VITIS_ROOT:?set VITIS_ROOT to your Vitis 2024.2 install, e.g. C:/Xilinx/Vitis/2024.2}"
XSCT="$VITIS/bin/xsct.bat"
BOOTGEN="$VITIS/bin/bootgen.bat"
GCCBIN="$VITIS/gnu/aarch32/nt/gcc-arm-none-eabi/bin"
BSP="$ROOT/gen2/fsbl/zynq_fsbl_bsp/ps7_cortexa9_0"
CFLAGS="-mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard"

[ -f "$ROOT/vloop_probes2.xsa" ] || { echo "ERROR: $ROOT/vloop_probes2.xsa missing."; \
  echo "  Generate it first: vivado -mode batch -source $ROOT/../../vivado/rebuild_ps7_uart_qspi.tcl"; exit 1; }

# 1) FSBL + standalone BSP from the XSA (lean hsi flow; the Vitis platform flow hangs on qemu).
#    Skipped if already built (delete gen2/ to force a rebuild).
if [ ! -f "$ROOT/gen2/fsbl/executable.elf" ] || [ ! -f "$BSP/lib/libxil.a" ]; then
  echo "[1/3] generating FSBL + BSP from XSA ..."
  "$XSCT" "$ROOT/build_m1_v2.tcl"
fi
[ -f "$BSP/lib/libxil.a" ] || { echo "ERROR: BSP build failed (no libxil.a)"; exit 1; }

# 2) compile the camera app directly against the BSP (no make needed).
#    lscript.ld (DDR app layout) and Xilinx.spec are committed in src_cam/ so the build does
#    not depend on the empty_application scaffold (whose generate_app can abort).
echo "[2/3] compiling camera app ..."
export PATH="$GCCBIN:$PATH"
( cd "$ROOT/src_cam"
  arm-none-eabi-gcc -c main.c -o main.o $CFLAGS -O2 -I"$BSP/include" -I.
  arm-none-eabi-gcc -o mipi_cam.elf main.o $CFLAGS -Wl,-build-id=none -specs=Xilinx.spec \
    -Wl,--start-group,-lxil,-lgcc,-lc,--end-group -Wl,--gc-sections \
    -L"$BSP/lib" -T"$ROOT/src_cam/lscript.ld" )

# 3) boot image (FSBL + bitstream + app), per boot_cam.bif.
echo "[3/3] bootgen ..."
( cd "$ROOT"; "$BOOTGEN" -arch zynq -image boot_cam.bif -o BOOT_CAM.bin -w on >/dev/null )
echo "DONE -> $ROOT/BOOT_CAM.bin   ($(stat -c%s "$ROOT/BOOT_CAM.bin") bytes)"
echo "Flash it with:  ./flash.sh   (set JP5=JTAG + power-cycle first)"
