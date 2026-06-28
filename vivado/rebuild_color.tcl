## Rebuild after the color BD recreate. The recreate left the BD in global
## synthesis mode (only synth_1 + impl_1 exist; the per-IP OOC runs e.g.
## bd_core0_0_synth_1 are gone), so rebuild_fe_min.tcl (which references the OOC
## run by name) fails. Here synth_1 synthesises the whole BD (core0 = the 24-bit
## color wrapper) from scratch, so the color datapath is guaranteed in. VDMA
## fsync=2 + GPIO C_DOUT_DEFAULT + core0 CONFIG are all baked in the recreated BD.
set repo_dir [file normalize [file join [file dirname [info script]] ..]]
open_project [file join $repo_dir vloop_probes2 vloop.xpr]
update_compile_order -fileset sources_1

puts "Resetting synth_1 + impl_1 ..."
reset_run synth_1
reset_run impl_1
set_property strategy "Performance_ExplorePostRoutePhysOpt" [get_runs impl_1]

puts "Launching impl_1 -to_step write_bitstream -jobs 6 ..."
launch_runs impl_1 -to_step write_bitstream -jobs 6
wait_on_run impl_1

set prog [get_property PROGRESS [get_runs impl_1]]
set wns  [get_property STATS.WNS [get_runs impl_1]]
puts "impl_1 progress=$prog  WNS=$wns"
if {$prog ne "100%"} { puts "IMPL_FAILED prog=$prog"; exit 1 }

set bit [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.bit]
set hwh [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.hwh]
puts "BITSTREAM_OK $bit"
puts "HWH_OK $hwh"
