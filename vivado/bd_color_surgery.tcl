## BD surgery: Y8 grayscale capture -> 24-bit RGB888 / RGBA32 color path
## (2026-06-23, image-processing research base, Phase 1).
##   capture:  core0/m_axis_capture(24) -> pack_s2mm(axis_rgb24_to_vdma32, ->32 RGBA) -> VDMA S2MM
##   display:  VDMA MM2S(32) -> unpack_mm2s(axis_vdma32_to_rgb24, ->24) -> cc_mm2s(24b clkconv) -> core0/s_axis_hdmi(24)
## Staged: validate_bd_design + save + generate only (NO impl) so connection/width
## errors surface in ~5 min before the 25-min rebuild. Run rebuild_fe_min.tcl after.
set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set xpr [file join $repo_dir vloop_probes2 vloop.xpr]
puts "Opening $xpr"
open_project $xpr

# Add the new BD module-reference sources (idempotent)
foreach f {axis_rgb24_to_vdma32_ref.v axis_vdma32_to_rgb24_ref.v} {
    set fp [file normalize [file join $repo_dir rtl img_proc $f]]
    if {[get_files -quiet $fp] eq "" && [get_files -quiet *$f] eq ""} {
        puts "INFO: adding $fp"
        add_files -norecurse -fileset sources_1 $fp
    }
}
update_compile_order -fileset sources_1

set bd [get_files -quiet *bd.bd]
puts "Opening BD $bd"
open_bd_design $bd

# Capture the shared S2MM/MM2S clock + reset SOURCE pins from the existing pack_s2mm
# (robust: no hard-coded net names) BEFORE deleting any cell.
set clk_src [get_bd_pins -quiet -filter {DIR==O} -of_objects [get_bd_nets -of_objects [get_bd_pins pack_s2mm/aclk]]]
set rst_src [get_bd_pins -quiet -filter {DIR==O} -of_objects [get_bd_nets -of_objects [get_bd_pins pack_s2mm/aresetn]]]
puts "INFO: clk_src=$clk_src  rst_src=$rst_src"
if {$clk_src eq "" || $rst_src eq ""} { puts "ERROR: could not resolve clk/reset source"; exit 1 }

# --- capture side -----------------------------------------------------------
delete_bd_objs [get_bd_cells pack_s2mm]
# core0 m_axis_capture -> 24-bit (now unconnected after pack_s2mm delete)
set_property -dict [list CONFIG.COLOR_CAPTURE {1}] [get_bd_cells core0]
create_bd_cell -type module -reference axis_rgb24_to_vdma32_ref pack_s2mm
connect_bd_net $clk_src [get_bd_pins pack_s2mm/aclk]
connect_bd_net $rst_src [get_bd_pins pack_s2mm/aresetn]
connect_bd_intf_net [get_bd_intf_pins core0/m_axis_capture] [get_bd_intf_pins pack_s2mm/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins pack_s2mm/M_AXIS] [get_bd_intf_pins axi_vdma_0/S_AXIS_S2MM]

# --- display side -----------------------------------------------------------
delete_bd_objs [get_bd_cells unpack_mm2s]
create_bd_cell -type module -reference axis_vdma32_to_rgb24_ref unpack_mm2s
connect_bd_net $clk_src [get_bd_pins unpack_mm2s/aclk]
connect_bd_net $rst_src [get_bd_pins unpack_mm2s/aresetn]
connect_bd_intf_net [get_bd_intf_pins axi_vdma_0/M_AXIS_MM2S] [get_bd_intf_pins unpack_mm2s/S_AXIS]
connect_bd_intf_net [get_bd_intf_pins unpack_mm2s/M_AXIS] [get_bd_intf_pins cc_mm2s/S_AXIS]

# cc_mm2s clock converter: 8-bit (1 byte) -> 24-bit (3 bytes)
set_property -dict [list CONFIG.TDATA_NUM_BYTES {3}] [get_bd_cells cc_mm2s]

# sub_y_to_rgb (Y8->RGB replicate) no longer needed: cc_mm2s already carries RGB888
delete_bd_objs [get_bd_cells sub_y_to_rgb]
connect_bd_intf_net [get_bd_intf_pins cc_mm2s/M_AXIS] [get_bd_intf_pins core0/s_axis_hdmi]

puts "Validating ..."
validate_bd_design
save_bd_design
generate_target all [get_files $bd]
puts "BD_SURGERY_OK"
