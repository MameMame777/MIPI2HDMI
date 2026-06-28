## Recreate the BD from a fresh write_bd_tcl export so the core0 module-reference
## is re-read from the current 24-bit wrapper RTL (the .bd caches the old 8-bit
## interface; update_compile_order does NOT refresh it -- confirmed by
## bd_color_refresh_check.tcl). The export captures the full current (surgered)
## state -- RGB pack/unpack cells, cc_mm2s 24-bit, removed sub_y_to_rgb, VDMA
## fsync=2, GPIO C_DOUT_DEFAULT, core0 CONFIG -- so nothing is lost; only core0's
## interface is refreshed to 24-bit on re-source. Staged: recreate + validate +
## report the capture width (must be 3 bytes = 24-bit), NO rebuild yet.
set repo_dir [file normalize [file join [file dirname [info script]] ..]]
open_project [file join $repo_dir vloop_probes2 vloop.xpr]
update_compile_order -fileset sources_1

set bdf [get_files -quiet *bd.bd]
puts "Current BD: $bdf"
open_bd_design $bdf

set exp [file join $repo_dir vivado vloop_probes2_bd_color.tcl]
puts "Exporting current (surgered) BD -> $exp"
write_bd_tcl -force $exp

# Remove the existing BD so the recreate reads core0 fresh from RTL.
close_bd_design [current_bd_design]
catch { export_ip_user_files -of_objects [get_files $bdf] -no_script -reset -force -quiet }
remove_files $bdf
file delete -force [file dirname $bdf]
puts "Removed old BD; recreating from export ..."

source $exp

set bdf2 [get_files -quiet *bd.bd]
open_bd_design $bdf2
validate_bd_design
set capb [get_property CONFIG.TDATA_NUM_BYTES [get_bd_intf_pins core0/m_axis_capture]]
puts "RECREATE CAP_TDATA_BYTES=$capb  (3 = 24-bit RGB = success)"
save_bd_design
generate_target all [get_files $bdf2]
make_wrapper -files [get_files $bdf2] -top -force
if {$capb == 3} { puts "RECREATE_OK_24BIT" } else { puts "RECREATE_STILL_[expr {$capb*8}]BIT" }
