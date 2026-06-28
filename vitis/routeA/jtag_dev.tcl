connect
after 1000
jtag targets
after 300
targets
after 300
targets -set -filter {name =~ "*Cortex-A9*#0"}
stop
after 300
dow [file join [file dirname [file normalize [info script]]] src_cam mipi_cam.elf]
after 300
con
puts "JTAG_DEV_RUN_STARTED"
