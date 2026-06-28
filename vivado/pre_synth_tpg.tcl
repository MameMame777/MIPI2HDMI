# Pre-synthesis hook for bd_core0_0_synth_1.
# Vivado sources this file; top-level code executes immediately.
# Ensures IMAGE_FORMAT=1 is set in fileset generics.
# NB: these fileset generics are reported "Unused top level parameter" in this BD
# flow -- the effective param values come from the RTL module DEFAULTS (the hook is
# belt-and-suspenders). HWLOCK_DEFAULT_ON is therefore baked via its RTL default
# (=1'b1) in mipi_to_hdmi_probe_top, not here.
# NB (verified 2026-06-19): the fileset generics are COSMETIC in this BD flow --
# the controlling values are the core0 BD cell CONFIG (set in rebuild_zeropynq.tcl)
# for params the BD captured (OV5640_MIPI_CTRL_4800/_4300, PROBE_IDELAY_TAP), and
# the RTL module default for params the BD did NOT capture (OV5640_ISP_FORMAT_501F,
# IMAGE_FORMAT, HWLOCK_DEFAULT_ON). A bitstream with generic=20 still read back 0x24
# = the BD CONFIG. This hook is kept only for IMAGE_FORMAT belt-and-suspenders.
set _cur [get_property generic [current_fileset]]
puts "INFO pre_synth_tpg: generics before patch: $_cur"
set _patched [regsub -all {IMAGE_FORMAT=\S+} $_cur {}]
append _patched " IMAGE_FORMAT=1"
set _patched [string trim [regsub -all {\s+} $_patched { }]]
puts "INFO pre_synth_tpg: generics after patch:  $_patched"
set_property generic $_patched [current_fileset]
