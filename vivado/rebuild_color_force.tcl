## Forced clean rebuild for the color path. The prior rebuild reused a CACHED core0
## synthesis product (the IPI synthesis/IP cache is separate from the mref interface
## cache), so the gray rgb565_gray_unpack stayed in the bitstream even after the
## RGB_OUT fix (confirmed: DDR had byte[0]==byte[1]==byte[2] exactly). Disable the IP
## cache + reset/regenerate the BD targets so core0 is re-synthesised from the current
## RTL (true RGB888), then synth_1 + impl_1.
set repo_dir [file normalize [file join [file dirname [info script]] ..]]
open_project [file join $repo_dir vloop_probes2 vloop.xpr]

# Ensure new RTL sources (Phase 2 processing slot + DoG dual-kernel) are in the synth fileset.
foreach rel {rtl/img_proc/axis_rgb_proc_slot.sv rtl/img_proc/axis_rgb_conv3x3.sv \
             rtl/img_proc/axis_rgb_conv5x5.sv rtl/img_proc/axis_rgb_dog_combine.sv \
             rtl/img_proc/axis_rgb_conv5x5_sep.sv} {
    set f [file normalize [file join $repo_dir $rel]]
    set base [file tail $f]
    if {[get_files -quiet $f] eq "" && [get_files -quiet *$base] eq ""} {
        puts "INFO: adding source $f"
        add_files -norecurse -fileset sources_1 $f
    }
}
update_compile_order -fileset sources_1

puts "Disabling IP cache + regenerating BD targets (force core0 re-synth from RTL) ..."
catch { config_ip_cache -disable_cache }
set bdf [get_files -quiet *bd.bd]
reset_target -quiet all [get_files $bdf]
generate_target all [get_files $bdf]

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
puts "BITSTREAM_OK [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.bit]"
