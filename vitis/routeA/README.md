# Route A — bare-metal standalone boot (no PYNQ/Linux)

The custom MIPI CSI-2 → HDMI pipeline running **standalone from QSPI**: FSBL → bitstream →
bare-metal C app, booting on power-on with no SD card / PYNQ / Linux. Confirmed live camera
image on HDMI (2026-06-27).

## Milestones

- **M1** — boot chain: FSBL + bitstream + a UART-heartbeat app from QSPI (`src_hello/main.c`).
- **M2** — `src_cam/main.c`: SCCB engine + 254-step OV5640 init (chip-ID 0x5640, 0 NACK).
- **M3** — software 8×8 bitslip D-PHY lock (the HW-lock FSM bogus-locks at 96 MHz byte_clk) +
  `settle_blank=14` band fix.
- **M4** — VDMA S2MM (camera→DDR) + MM2S (DDR→HDMI) → live image on the monitor.

The C control flow is a faithful port of `scripts/v65_capture.py` (`make_helpers`) +
`scripts/bitslip_lock.py` (`lock_mode`) + `scripts/camera_hdmi_demo.py` (VDMA), all with the
PYNQ/MMIO abstraction replaced by direct `volatile` AXI register access.

## Prerequisites (one-time)

PS7 must have **UART1 (MIO 48/49)** + **Quad SPI (MIO 1-6, x4)** enabled — the stock BD had
them off. Re-enabled PYNQ-safe (FCLK0=100/DDR/PL unchanged) by
[`../../vivado/rebuild_ps7_uart_qspi.tcl`](../../vivado/rebuild_ps7_uart_qspi.tcl), which also exports
`vloop_probes2.xsa`.

## Reproduce in one command

After the XSA exists (the Vivado prerequisite above), the whole flow is scripted:

```sh
./build.sh     # FSBL+BSP from XSA -> compile app -> BOOT_CAM.bin   (gen2/ cached; rm to force)
./flash.sh     # set JP5=JTAG + power-cycle first; flashes QSPI with verify
# then set JP5=QSPI + power-cycle -> camera->HDMI + filter console on the USB-UART
```

`src_cam/lscript.ld` (DDR app layout) and `src_cam/Xilinx.spec` are committed so the app build
is self-contained. The manual steps below are what the scripts run.

## Build (Vitis/Vivado 2024.2)

```sh
# 1. FSBL + standalone BSP from the XSA (lean hsi flow; the Vitis platform flow hangs on qemu)
xsct build_m1_v2.tcl                     # -> gen2/fsbl/executable.elf + zynq_fsbl_bsp (UART1 STDOUT, QSPI)

# 2. camera app (direct gcc against the BSP; no make needed)
cd src_cam
arm-none-eabi-gcc -c main.c -o main.o -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard -O2 \
    -I../gen2/fsbl/zynq_fsbl_bsp/ps7_cortexa9_0/include -I.
arm-none-eabi-gcc -o mipi_cam.elf main.o -mcpu=cortex-a9 -mfpu=vfpv3 -mfloat-abi=hard \
    -Wl,-build-id=none -specs=Xilinx.spec \
    -Wl,--start-group,-lxil,-lgcc,-lc,--end-group -Wl,--gc-sections \
    -L../gen2/fsbl/zynq_fsbl_bsp/ps7_cortexa9_0/lib -T../gen2/lscript.ld
cd ..

# 3. boot image
bootgen -arch zynq -image boot_cam.bif -o BOOT_CAM.bin -w on
```

## Flash to QSPI (USB-JTAG)

Set **JP5 = JTAG**, power on, then:

```sh
hw_server &
program_flash -f BOOT_CAM.bin -fsbl gen2/fsbl/executable.elf \
    -flash_type qspi-x4-single -offset 0 -verify -url tcp:127.0.0.1:3121
```

Then set **JP5 = QSPI**, power-cycle → the camera app runs automatically. UART on the on-board
USB-UART (PS7 UART1), 115200 8N1.

## Dev loop (no reflash)

The board boots whatever is in QSPI; iterate the app over JTAG without reflashing:

```sh
xsct jtag_dev.tcl        # connect, halt, dow src_cam/mipi_cam.elf, con
```

NB: a 2nd SCCB re-init on an already-streaming chip degrades the link — **power-cycle between
JTAG runs** (one clean run per power cycle). QSPI boots are always fresh.

Live-tune a PL register over JTAG (e.g. `settle_blank` via the idelay GPIO) — must `memmap` the
PL AXI range first (see `jtag_ktune.tcl`).

## Filter console (interactive, over UART)

Once live, the app runs a single-key filter console on the UART (COM4, 115200 8N1) — a
REPL-equivalent for the image-processing slots, applied live via the 0xFE-page SCCB intercept
+ `set_proc_op`. Open any serial terminal and press:

```
 MID point: 0 pass 1 invert 2 gray 3 BGRswap 4 thresh 5 R 6 G 7 B
 MID conv : g gauss  s sobelX  h sharpen  l laplacian  o outline  e emboss
 PRE      : v median  b blur   V off
 POST     : t threshold  T off
 dither   : d off  n halftone(1b)  p poster(2b)  r random(2b)
 x = reset all to passthrough     ? = menu
```

Numeric parameters (type the command + Enter):

```
 k c0 c1 .. c8 [shift]   custom 3x3 kernel (signed coeffs) + enable conv
 op N                    proc_op N (0-7 point, 8 conv, 12 DoG, 13-15 cascade)
 pt N / qt N             PRE / POST threshold level (0-255) + enable op-4
 pre N / post N          set pre_op / post_op
 fe LO VAL               raw 0xFE-page write (hex/dec) = ANY config reg
                         e.g. fe 47 80 (PRE thresh), fe 4A 05 (dither halftone)
```

(`?` reprints the menu — confirms both UART TX and RX are working.)

0xFE-page writes are intercepted by the PL and latched on the apply edge — the app fires
apply + 2 ms (no chip-write status poll), so filter changes apply instantly.

## Notes / gotchas

- D-PHY lock is a **/4-phase lottery**: good phases lock on ph0 (bitslip (3,3), strong); weak
  phases are marginal. Power-cycle to re-roll.
- The PL `long_pkt` debug counter read is **unreliable** in the bare-metal heartbeat — tune by
  the HDMI image, not the counter.
- Do **not** move IDELAY after the lock (loses byte alignment); only set `settle_blank`.
