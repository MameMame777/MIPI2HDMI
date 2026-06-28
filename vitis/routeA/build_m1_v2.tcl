# Route A M1 v2: FSBL + app BSP from the NEW XSA (PS7 now has UART1 + QSPI).
set ROOT [file dirname [file normalize [info script]]]
set XSA  $ROOT/vloop_probes2.xsa
hsi::open_hw_design $XSA
set hw [hsi::current_hw_design]
file delete -force $ROOT/gen2
# 1) FSBL (now with UART1 console + QSPI) + compile
if {[catch {hsi::generate_app -hw $hw -os standalone -proc ps7_cortexa9_0 -app zynq_fsbl -compile -dir $ROOT/gen2/fsbl} e]} { puts "FSBL_CAUGHT=$e" }
puts "FSBL_DONE_V2"
# 2) empty_application scaffold (gives a UART1-STDOUT BSP + lscript) — compiled later with our main.c
if {[catch {hsi::generate_app -hw $hw -os standalone -proc ps7_cortexa9_0 -app empty_application -dir $ROOT/gen2/app} e2]} { puts "APP_CAUGHT=$e2" }
puts "APP_SCAFFOLD_DONE_V2"
