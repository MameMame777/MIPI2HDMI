set script_dir [file normalize [file dirname [info script]]]
set repo_dir   [file normalize [file join $script_dir ../..]]
set proj_dir   [file join $repo_dir vloop]
set bit_file   [file join $proj_dir vloop.runs impl_1 bd_wrapper.bit]

if {![file exists $bit_file]} {
    puts "ERROR: Bitstream not found at $bit_file"
    exit 1
}

open_hw_manager
connect_hw_server
open_hw_target
set pl_dev [lindex [get_hw_devices xc7z020_1] 0]
if {$pl_dev eq ""} {
    puts "ERROR: No Zynq PL device found."
    close_hw_target
    disconnect_hw_server
    exit 1
}
current_hw_device $pl_dev
set_property PROGRAM.FILE $bit_file $pl_dev
program_hw_devices $pl_dev
puts "Programming complete: $pl_dev"
close_hw_target
disconnect_hw_server
