set repo_dir [file normalize [file join [file dirname [info script]] ../..]]
set proj_xpr [file join $repo_dir mipi2hdml_lane1_sweep vloop.xpr]
open_project $proj_xpr
open_bd_design [get_files bd.bd]
set out_tcl [file join [file dirname [info script]] bd_design.tcl]
write_bd_tcl -force -no_ip_version -hier_blks {} $out_tcl
puts "BD TCL exported to: $out_tcl"
close_project
