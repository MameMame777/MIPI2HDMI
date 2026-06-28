## Diagnose whether the BD core0 module-reference cell picks up the widened
## (24-bit, COLOR_CAPTURE default=1) m_axis_capture after a fresh open +
## update_compile_order. Decides the fix path (re-validate vs recreate core0).
set repo_dir [file normalize [file join [file dirname [info script]] ..]]
open_project [file join $repo_dir vloop_probes2 vloop.xpr]
update_compile_order -fileset sources_1
set bd [get_files -quiet *bd.bd]
open_bd_design $bd

set props [list_property [get_bd_cells core0]]
set has_param [expr {[lsearch $props CONFIG.COLOR_CAPTURE] >= 0}]
puts "DIAG CORE0_HAS_COLOR_PARAM=$has_param"
catch {puts "DIAG CORE0_COLOR_VALUE=[get_property CONFIG.COLOR_CAPTURE [get_bd_cells core0]]"}
catch {puts "DIAG CAP_TDATA_BYTES=[get_property CONFIG.TDATA_NUM_BYTES [get_bd_intf_pins core0/m_axis_capture]]"}

# If the cell now exposes the 24-bit port, the saved RGB connections just need a
# re-validate to rewire all 24 bits; commit it.
set capb 0
catch { set capb [get_property CONFIG.TDATA_NUM_BYTES [get_bd_intf_pins core0/m_axis_capture]] }
if {$capb == 3} {
    puts "DIAG -> core0 is 24-bit; re-validating + saving"
    validate_bd_design
    save_bd_design
    generate_target all [get_files $bd]
    puts "REFRESH_FIXED_OK"
} else {
    puts "DIAG -> core0 still [expr {$capb*8}]-bit; needs cell recreate (REFRESH_NEEDS_RECREATE)"
}
