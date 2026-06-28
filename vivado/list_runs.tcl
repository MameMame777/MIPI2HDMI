set repo_dir [file normalize [file join [file dirname [info script]] ..]]
open_project [file join $repo_dir vloop_probes2 vloop.xpr]
puts "RUNS_BEGIN"
foreach r [get_runs] { puts "RUN: $r STATUS=[get_property STATUS $r] CONSTRSET=[get_property -quiet CONSTRSET $r]" }
puts "RUNS_END"
puts "BD_FILE: [get_files -quiet *bd.bd]"
puts "TOP: [get_property top [current_fileset]]"
