## rebuild_ps7_uart_qspi.tcl (2026-06-27)
## Route A: enable PS7 UART1 (MIO 48/49) + Quad SPI Flash (MIO 1-6, x4 single) so the
## standalone FSBL has a console + the flash-writer/app can drive QSPI. PYNQ-SAFE:
##   - FCLK0 stays 100 MHz, DDR (MT41K256M16) unchanged, M_AXI_GP0/S_AXI_HP0 unchanged.
##   - UART1/QSPI are PS-dedicated MIO (1-6, 48/49) -> no PL pin / PL logic change.
##   - PYNQ boots from the SD's own FSBL (its PS7) + overlay applies FCLK only -> unaffected.
## Property values are taken verbatim from the Digilent Zybo Z7-20 Pcam reference BD
## (same board, known-good MIO mapping).
##
## PL is unchanged from the dither build (core0 + VDMA OOC netlists preserved): we only
## regenerate the BD output products (ps7_init/hwh/wrapper), re-stitch synth_1, re-impl.

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set xpr      [file join $repo_dir vloop_probes2 vloop.xpr]

puts "Opening $xpr"
open_project $xpr

# --- Apply PS7 UART1 + QSPI config (preserve everything else) ------------------
set bd [get_files -quiet *bd.bd]
puts "INFO: opening BD $bd"
open_bd_design $bd

set_property -dict [list \
  CONFIG.PCW_UART1_PERIPHERAL_ENABLE {1} \
  CONFIG.PCW_UART1_UART1_IO {MIO 48 .. 49} \
  CONFIG.PCW_UART1_BAUD_RATE {115200} \
  CONFIG.PCW_EN_UART1 {1} \
  CONFIG.PCW_UART_PERIPHERAL_FREQMHZ {100} \
  CONFIG.PCW_QSPI_PERIPHERAL_ENABLE {1} \
  CONFIG.PCW_QSPI_QSPI_IO {MIO 1 .. 6} \
  CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE {1} \
  CONFIG.PCW_QSPI_GRP_SINGLE_SS_IO {MIO 1 .. 6} \
  CONFIG.PCW_QSPI_GRP_FBCLK_ENABLE {1} \
  CONFIG.PCW_QSPI_PERIPHERAL_FREQMHZ {200} \
  CONFIG.PCW_QSPI_PERIPHERAL_CLKSRC {IO PLL} \
  CONFIG.PCW_SINGLE_QSPI_DATA_MODE {x4} \
  CONFIG.PCW_EN_QSPI {1} \
  CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100.000000} \
] [get_bd_cells ps7]

puts "INFO: validating BD ..."
validate_bd_design
save_bd_design

# Verify the PYNQ-critical invariants did not drift.
set fclk0 [get_property CONFIG.PCW_ACT_FPGA0_PERIPHERAL_FREQMHZ [get_bd_cells ps7]]
set uart1 [get_property CONFIG.PCW_UART1_PERIPHERAL_ENABLE [get_bd_cells ps7]]
set qspi  [get_property CONFIG.PCW_QSPI_PERIPHERAL_ENABLE [get_bd_cells ps7]]
puts "CHECK: ACT_FPGA0=$fclk0 (must be 100.x)  UART1=$uart1  QSPI=$qspi"
if {![string match 100.* $fclk0]} { puts "FCLK0_DRIFT_ABORT $fclk0"; exit 2 }

# Regenerate BD output products (ps7_init.c, wrapper, hwh) with the new PS7 config.
puts "INFO: regenerating BD targets ..."
generate_target all [get_files $bd]

update_compile_order -fileset sources_1

# PL is unchanged: keep bd_core0_0_synth_1 + bd_axi_vdma_0_0_synth_1 OOC netlists
# (the dither build). Only the top synth must re-stitch the regenerated BD, then re-impl.
puts "Resetting synth_1 + impl_1 (core0/VDMA OOC preserved) ..."
reset_run synth_1
reset_run impl_1

set_property strategy "Performance_ExplorePostRoutePhysOpt" [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE ExtraNetDelay_high [get_runs impl_1]
set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

puts "Launching impl_1 -to_step write_bitstream -jobs 6 ..."
launch_runs impl_1 -to_step write_bitstream -jobs 6
wait_on_run impl_1

set prog [get_property PROGRESS [get_runs impl_1]]
set wns  [get_property STATS.WNS [get_runs impl_1]]
puts "impl_1 progress=$prog  WNS=$wns"
if {$prog ne "100%"} { puts "IMPL_FAILED prog=$prog"; exit 1 }

# Export the fixed XSA (hwh with new PS7 + bit) for the Route A FSBL flow.
open_run impl_1
set xsa [file join $repo_dir vitis routeA vloop_probes2.xsa]
write_hw_platform -fixed -include_bit -force $xsa
puts "XSA_OK $xsa"

set bit [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.bit]
puts "BITSTREAM_OK $bit  WNS=$wns"
close_project
puts "REBUILD_PS7_DONE"
