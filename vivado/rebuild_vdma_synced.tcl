## VDMA S2MM tear-free config (2026-06-21): c_use_s2mm_fsync=2 + c_flush_on_fsync=0.
## Empirically (vts_hdmi_ceiling.py, runtime DMAIntErr probe):
##   fsync=2/flush=1 -> clean but S2MM DMAIntErr-halts above ~19fps,
##   fsync=0 free-run -> no halt but TEARS (frame mixing, no read/write sync),
##   fsync=2/flush=0 -> tear-free AND halt-free; the highest clean (DMAIntErr-free)
##     live-HDMI frame rate the current VDMA sustains is ~25.5fps.
## This is the correct synced setting (matches the Digilent Pcam reference) and
## also fixes the pre-existing free-run tiling. core0/GPIO inherited from saved BD;
## RTL (incl. the chip-init ROM / fps) is whatever is currently in the source tree.
set repo_dir [file normalize [file dirname [info script]]/..]
set xpr [file join $repo_dir vloop_probes2 vloop.xpr]
puts "Opening $xpr"
open_project $xpr

set bdf [get_files -quiet *bd.bd]
open_bd_design $bdf
set vdma [get_bd_cells -quiet -filter {VLNV =~ "*:axi_vdma:*"}]
if {[llength $vdma] == 0} { puts "ERROR: no axi_vdma cell found"; exit 1 }
puts "  before: FSYNC=[get_property CONFIG.c_use_s2mm_fsync $vdma] FLUSH=[get_property CONFIG.c_flush_on_fsync $vdma] GENLOCK=[get_property CONFIG.c_s2mm_genlock_mode $vdma]"

set_property -dict [list \
    CONFIG.c_use_s2mm_fsync {2} \
    CONFIG.c_flush_on_fsync {0} \
] $vdma
puts "  after:  FSYNC=[get_property CONFIG.c_use_s2mm_fsync $vdma] FLUSH=[get_property CONFIG.c_flush_on_fsync $vdma] GENLOCK=[get_property CONFIG.c_s2mm_genlock_mode $vdma]"

validate_bd_design
save_bd_design
generate_target all $bdf

puts "Resetting runs ..."
foreach r {bd_axi_vdma_0_0_synth_1 bd_core0_0_synth_1 synth_1 impl_1} {
    if {[llength [get_runs -quiet $r]]} { reset_run $r }
}
set_property strategy "Performance_ExplorePostRoutePhysOpt" [get_runs impl_1]
launch_runs impl_1 -to_step write_bitstream -jobs 6
wait_on_run impl_1
set prog [get_property PROGRESS [get_runs impl_1]]
set wns  [get_property STATS.WNS [get_runs impl_1]]
puts "impl_1 progress=$prog WNS=$wns"
if {$prog != "100%"} { puts "IMPL_FAILED prog=$prog"; exit 1 }
puts "BITSTREAM_OK [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.bit]"
