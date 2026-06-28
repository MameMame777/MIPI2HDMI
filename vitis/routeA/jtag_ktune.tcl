connect
after 1000
jtag targets
after 300
targets
after 300
targets -set -filter {name =~ "*Cortex-A9*#0"}
memmap -addr 0x41220000 -size 0x10000 -flags rw
after 300
puts "STEP1_K8"
mwr 0x41220000 0x40000808
after 12000
puts "STEP2_K12"
mwr 0x41220000 0x60000808
after 12000
puts "STEP3_K15"
mwr 0x41220000 0x78000808
after 12000
mwr 0x41220000 0x60000808
puts "KTUNE_DONE_left_K12"
