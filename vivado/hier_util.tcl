set ROOT [file dirname [file dirname [file normalize [info script]]]]
open_checkpoint $ROOT/vloop_probes2/vloop.runs/impl_1/bd_wrapper_routed.dcp
report_utilization -hierarchical -hierarchical_depth 25 -file $ROOT/vitis/routeA/hier_util.rpt
puts "HIER_UTIL_DONE"
