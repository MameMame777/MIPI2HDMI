## rebuild_zeropynq.tcl (2026-06-19)
## Bake the full verified continuous + RGB565 RX config so the camera + FPGA RX
## pipeline self-configures and the HW-lock FSM auto-locks at POWER-UP (zero PYNQ).
## Builds on the C_DOUT_DEFAULT RESETB fix (rebuild_cdout_default.tcl) + the baked
## HW-lock FSM (HWLOCK_DEFAULT_ON=1, RTL default).
##
## NB: HDMI *display* still needs PYNQ to start the VDMA (S2MM/MM2S) -- this bake
## delivers the zero-PYNQ RX pipeline (chip configured + locked + 480-line RGB565
## frames assembled), verifiable via zero_pynq_test counters, NOT a monitor image.
##
## RTL defaults (mipi_to_hdmi_probe_top, this build's source) -- the binding path in
## this BD flow (fileset generics are "Unused"; verified 19b):
##   OV5640_MIPI_CTRL_4800 = 0x14 (continuous), OV5640_FORMAT_CTRL_4300 = 0x6F
##   (RGB565), OV5640_ISP_FORMAT_501F = 0x01 (RGB565 mux), PROBE_IDELAY_TAP = 16,
##   init FSM 0x300E idle = 0x40 (proven stream cycle 0x40->0x45). OV5640 values
##   cross-checked vs docs/doc/ov5640_linux_mainline_reference.md (see RTL comment).
##
## GPIO C_DOUT_DEFAULTs baked here (level fields read directly by the RTL at boot;
## the apply-gated frame height already resets to 480 in RTL):
##   frame_lines_gpio = 0xC24501E0 : bit31 force_expected, bit30 sof_synth,
##     bit25 RESETB, bits[23:17]=0x22 expected_dt, bit16 use_lsle, [15:0]=480.
##   idelay_gpio      = 0x40000000 : bits[30:27]=8 settle-blank K=8 (band fix); the
##     IDELAY taps come from PROBE_IDELAY_TAP=16 (apply-gated, RTL default).
##
## Resets the two GPIO OOCs + core0 OOC + synth + impl. VDMA fsync=2 OOC preserved.

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set xpr      [file join $repo_dir vloop_probes2 vloop.xpr]
set hook_tcl [file normalize [file join [file dirname [info script]] pre_synth_tpg.tcl]]
set bd_file  [file join $repo_dir vloop_probes2 vloop.srcs sources_1 bd bd bd.bd]

puts "Opening $xpr"
open_project $xpr

proc ensure_source {repo_dir rel} {
    set f [file normalize [file join $repo_dir {*}$rel]]
    set base [file tail $f]
    if {[get_files -quiet $f] eq "" && [get_files -quiet *$base] eq ""} {
        puts "INFO: Adding $f"
        add_files -norecurse -fileset [get_filesets sources_1] $f
        set_property file_type SystemVerilog [get_files $f]
    } else {
        puts "INFO: $base already in project"
    }
}
ensure_source $repo_dir {rtl mipi_rx dphy_cdc_prims.sv}
ensure_source $repo_dir {rtl mipi_rx dphy_lane_supervisor.sv}
ensure_source $repo_dir {rtl mipi_rx dphy_hwlock_fsm.sv}
ensure_source $repo_dir {rtl prototype csi2_tpg.sv}
ensure_source $repo_dir {rtl img_proc video_frame_normalizer.sv}
ensure_source $repo_dir {rtl mipi_rx csi2_frame_state.sv}
ensure_source $repo_dir {rtl prototype mipi_to_hdmi_probe_top.sv}
ensure_source $repo_dir {rtl prototype ov5640_sccb_init_probe.sv}
update_compile_order -fileset sources_1

# === Bake the GPIO C_DOUT_DEFAULTs ===
puts "INFO: Opening BD $bd_file"
open_bd_design $bd_file
proc set_gpio_default {name val} {
    set c [get_bd_cells -quiet $name]
    if {$c eq ""} { puts "ERROR: $name cell not found"; exit 1 }
    puts "INFO: $name C_DOUT_DEFAULT [get_property CONFIG.C_DOUT_DEFAULT $c] -> $val"
    set_property -dict [list CONFIG.C_DOUT_DEFAULT $val] $c
}
set_gpio_default frame_lines_gpio {0xC24501E0}
set_gpio_default idelay_gpio      {0x40000000}

# === Bake the core0 BD cell CONFIG (the controlling values; verified 2026-06-19) ===
# core0 = mipi_to_hdmi_vdma_loop_ref (wraps the probe). The BD captured these params,
# so the cell CONFIG overrides both the RTL default and any fileset generic. 8-bit
# params are binary strings (as the BD stores them). Fail-fast readback so a wrong
# format aborts BEFORE the ~2h impl. OV5640_ISP_FORMAT_501F is NOT captured by the BD
# -> set here too (overrides the wrapper default 0x00 -> 0x01 RGB565 mux).
set core0 [get_bd_cells -quiet core0]
if {$core0 eq ""} { puts "ERROR: core0 cell not found"; exit 1 }
proc set_core0_bin {core name binval} {
    set_property CONFIG.$name $binval $core
    set rb [get_property CONFIG.$name $core]
    set rbbin [regsub -all {[^01]} $rb {}]
    puts "INFO: core0 $name set=$binval readback=$rb (bin=$rbbin)"
    if {$rbbin ne $binval} { puts "ERROR: core0 $name not set as expected ($rb)"; exit 1 }
}
set_core0_bin $core0 OV5640_MIPI_CTRL_4800   {00010100}   ;# 0x14 continuous
set_core0_bin $core0 OV5640_FORMAT_CTRL_4300 {01101111}   ;# 0x6F RGB565
# OV5640_ISP_FORMAT_501F is NOT a CONFIG param on this cell (readback empty -- added
# to the wrapper after the BD cell was created). It is set via the wrapper RTL
# default (mipi_to_hdmi_vdma_loop_ref OV5640_ISP_FORMAT_501F = 8'h01) instead.
set_property CONFIG.PROBE_IDELAY_TAP {16} $core0
set _it [get_property CONFIG.PROBE_IDELAY_TAP $core0]
puts "INFO: core0 PROBE_IDELAY_TAP readback=$_it"
if {$_it ne "16"} { puts "ERROR: core0 PROBE_IDELAY_TAP not 16 ($_it)"; exit 1 }

validate_bd_design
save_bd_design
generate_target all [get_files $bd_file]
close_bd_design [current_bd_design]

# Pre-synth hook on core0 (IMAGE_FORMAT=1; HWLOCK_DEFAULT_ON + the OV5640/idelay
# values are RTL defaults).
set_property -name {STEPS.SYNTH_DESIGN.TCL.PRE} -value $hook_tcl \
             -objects [get_runs bd_core0_0_synth_1]
set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]
set_property INCREMENTAL_CHECKPOINT       "" [get_runs synth_1]

puts "Resetting frame_lines + idelay GPIO OOC + core0 OOC + synth_1 + impl_1 ..."
reset_run bd_frame_lines_gpio_0_synth_1
reset_run bd_idelay_gpio_0_synth_1
reset_run bd_core0_0_synth_1
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
puts "BITSTREAM_OK $bit"
puts "HWH_OK [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.hwh]"
