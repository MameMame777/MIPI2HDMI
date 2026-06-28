## Reconfigure the AXI-VDMA S2MM to frame-sync on the AXIS TUSER (SOF) instead of
## free-running, matching the working Digilent Pcam reference (2026-06-06).
## Root cause: vloop VDMA had C_USE_S2MM_FSYNC=0 (free-run) -> ignored TUSER/SOF
## and wrote VSIZE lines continuously, tiling the buffer when the chip frame
## length != VSIZE. Digilent uses C_USE_S2MM_FSYNC=2 (s2mm_tuser) +
## C_S2MM_GENLOCK_MODE=2. The bridge already drives m_axis_capture_tuser[0]=SOF.
set repo_dir [file normalize [file dirname [info script]]/..]
set xpr [file join $repo_dir vloop_probes2 vloop.xpr]
puts "Opening $xpr"
open_project $xpr

set bdf [get_files -quiet *bd.bd]
puts "Opening BD: $bdf"
open_bd_design $bdf

# find the axi_vdma cell (by VLNV)
set vdma [get_bd_cells -quiet -filter {VLNV =~ "*:axi_vdma:*"}]
if {[llength $vdma] == 0} { puts "ERROR: no axi_vdma cell found"; exit 1 }
puts "VDMA cell: $vdma"
puts "  before: USE_S2MM_FSYNC=[get_property CONFIG.c_use_s2mm_fsync $vdma] GENLOCK=[get_property CONFIG.c_s2mm_genlock_mode $vdma]"

set_property -dict [list \
    CONFIG.c_use_s2mm_fsync {2} \
    CONFIG.c_s2mm_genlock_mode {2} \
] $vdma
puts "  after:  USE_S2MM_FSYNC=[get_property CONFIG.c_use_s2mm_fsync $vdma] GENLOCK=[get_property CONFIG.c_s2mm_genlock_mode $vdma]"

validate_bd_design
save_bd_design
generate_target all $bdf

puts "Resetting runs (VDMA IP OOC + core0 OOC + synth + impl) ..."
foreach r {bd_axi_vdma_0_0_synth_1 bd_core0_0_synth_1 synth_1 impl_1} {
    if {[llength [get_runs -quiet $r]]} { reset_run $r }
}
set_property strategy "Congestion_SpreadLogic_high" [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 6
wait_on_run impl_1
set prog [get_property PROGRESS [get_runs impl_1]]
set wns  [get_property STATS.WNS [get_runs impl_1]]
puts "impl_1 progress=$prog WNS=$wns"
if {$prog != "100%"} { puts "IMPL_FAILED prog=$prog"; exit 1 }
set bit [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.bit]
set hwh [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.hwh]
puts "BITSTREAM_OK $bit"
puts "HWH_OK $hwh"
