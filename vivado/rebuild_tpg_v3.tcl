## rebuild_tpg_v3.tcl
## Build the BD-based MIPI pipeline with CSI-2 TPG enabled.
##
## Uses vloop_probes2 as the source project because:
##   - vloop project has TopModule=hdmi_tpg_top (standalone HDMI TPG, no BD OOC)
##   - vloop_probes2 has the correct BD-based structure with bd_core0_0_synth_1
##
## RTL changes in this build (relative to vloop_probes2 May-17 baseline):
##   - csi2_tpg.sv: new module, OUTPUT_INTERVAL=2 (prevents parser FIFO overflow)
##   - mipi_to_hdmi_probe_top.sv: DONT_TOUCH on u_csi2_tpg + OUTPUT_INTERVAL(2)
##   - mipi_to_hdmi_vdma_loop_ref.v: IMAGE_FORMAT=1 default
##   - csi2_frame_state.sv, video_frame_normalizer.sv: recent fixes

set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set xpr      [file join $repo_dir vloop_probes2 vloop.xpr]
set hook_tcl [file normalize [file join [file dirname [info script]] pre_synth_tpg.tcl]]

puts "Opening $xpr"
open_project $xpr

# ---------------------------------------------------------------------------
# Add csi2_tpg.sv to sources_1 if not already present
# ---------------------------------------------------------------------------
set tpg_sv [file normalize [file join $repo_dir rtl prototype csi2_tpg.sv]]
if {[get_files -quiet $tpg_sv] eq "" && [get_files -quiet *csi2_tpg.sv] eq ""} {
    puts "INFO: Adding $tpg_sv"
    add_files -norecurse -fileset [get_filesets sources_1] $tpg_sv
    set_property file_type SystemVerilog [get_files $tpg_sv]
} else {
    puts "INFO: csi2_tpg.sv already in project"
}

# ---------------------------------------------------------------------------
# Add video_frame_normalizer.sv if not present (added in recent commits)
# ---------------------------------------------------------------------------
set nf [file normalize [file join $repo_dir rtl img_proc video_frame_normalizer.sv]]
if {[get_files -quiet $nf] eq "" && [get_files -quiet *video_frame_normalizer.sv] eq ""} {
    puts "INFO: Adding $nf"
    add_files -norecurse -fileset [get_filesets sources_1] $nf
    set_property file_type SystemVerilog [get_files $nf]
} else {
    puts "INFO: video_frame_normalizer.sv already in project"
}

update_compile_order -fileset sources_1

# ---------------------------------------------------------------------------
# Install pre-synth hook on bd_core0_0_synth_1 to ensure IMAGE_FORMAT=1
# ---------------------------------------------------------------------------
puts "INFO: Installing pre-synth hook on bd_core0_0_synth_1: $hook_tcl"
set_property -name {STEPS.SYNTH_DESIGN.TCL.PRE} \
             -value $hook_tcl \
             -objects [get_runs bd_core0_0_synth_1]

# Disable incremental synthesis to ensure all RTL changes enter the bitstream
set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]
set_property INCREMENTAL_CHECKPOINT       "" [get_runs synth_1]

# ---------------------------------------------------------------------------
# Verify bd_core0_0.tcl has the correct parameters
# ---------------------------------------------------------------------------
set core0_tcl [file join $repo_dir vloop_probes2 vloop.runs bd_core0_0_synth_1 bd_core0_0.tcl]
if {[file exists $core0_tcl]} {
    set fh [open $core0_tcl r]
    set content [read $fh]
    close $fh
    if {![string match "*IMAGE_FORMAT=1*" $content]} {
        puts "WARNING: bd_core0_0.tcl does NOT have IMAGE_FORMAT=1 — pre-synth hook will patch it"
    } else {
        puts "INFO: bd_core0_0.tcl already has IMAGE_FORMAT=1"
    }
} else {
    puts "INFO: bd_core0_0.tcl not found (first run — will be created by reset_run)"
}

# ---------------------------------------------------------------------------
# Reset and launch
# ---------------------------------------------------------------------------
puts "Resetting OOC core0 + synth_1 + impl_1 ..."
reset_run bd_core0_0_synth_1
reset_run synth_1
reset_run impl_1
set_property strategy "Congestion_SpreadLogic_high" [get_runs impl_1]

puts "Launching impl_1 -to_step write_bitstream -jobs 6 ..."
launch_runs impl_1 -to_step write_bitstream -jobs 6
wait_on_run impl_1

set prog [get_property PROGRESS [get_runs impl_1]]
set wns  [get_property STATS.WNS [get_runs impl_1]]
puts "impl_1 progress=$prog  WNS=$wns"

if {$prog ne "100%"} {
    puts "IMPL_FAILED prog=$prog"
    exit 1
}

set bit [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.bit]
set hwh [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper.hwh]
puts "BITSTREAM_OK $bit"
puts "HWH_OK $hwh"
