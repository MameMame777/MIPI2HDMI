## export_vloop_probes2_tcl.tcl (2026-06-20)
## Export the deployed BD project as version-controllable regeneration TCLs so the
## design (BD: C_DOUT_DEFAULT, core0 CONFIG, VDMA fsync=2, connections) is in git
## instead of only in the gitignored vloop_probes2/ build dir.
set repo_dir [file normalize [file join [file dirname [info script]] ..]]
open_project [file join $repo_dir vloop_probes2 vloop.xpr]
# 1. BD design as a recreatable TCL (the critical, hand-modified design source).
set bd [get_files -quiet *.bd]
puts "INFO: BD file = $bd"
open_bd_design $bd
write_bd_tcl -force -include_layout [file join $repo_dir vivado vloop_probes2_bd.tcl]
puts "BD_TCL_OK [file join $repo_dir vivado vloop_probes2_bd.tcl]"
# 2. Full project recreation TCL (create_project + add sources + runs config).
write_project_tcl -force -no_ip_version -paths_relative_to [file join $repo_dir vivado] \
    [file join $repo_dir vivado vloop_probes2_recreate.tcl]
puts "PROJ_TCL_OK [file join $repo_dir vivado vloop_probes2_recreate.tcl]"
close_project
