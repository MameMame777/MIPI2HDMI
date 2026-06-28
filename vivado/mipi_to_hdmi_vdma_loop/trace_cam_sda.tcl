set repo_dir [file normalize [file dirname [info script]]/../..]
set dcp [file join $repo_dir vloop vloop.runs impl_1 bd_wrapper_placed.dcp]
puts "Opening: $dcp"
open_checkpoint $dcp

puts ""
puts "=== ALL cells driving/sensing cam_sda ==="
set cells [get_cells -hierarchical -filter {NAME =~ *cam_sda* || REF_NAME == IOBUF}]
foreach c $cells {
    puts "Cell: [get_property NAME $c]  REF_NAME=[get_property REF_NAME $c]  LOC=[get_property LOC $c]"
}

puts ""
puts "=== IOBUF connected to cam_sda port ==="
set port_net [get_nets -of_objects [get_ports cam_sda] -segments]
puts "Port net: $port_net"
foreach pin [get_pins -of_objects $port_net] {
    puts "  Pin: [get_property NAME $pin]  REF_NAME=[get_property REF_NAME $pin]  DIR=[get_property DIRECTION $pin]"
}

puts ""
puts "=== Trace IOBUF.O -> downstream ==="
set iobufs [get_cells -hierarchical -filter {REF_NAME == IOBUF}]
foreach iobuf $iobufs {
    set name [get_property NAME $iobuf]
    if {[string match "*cam_sda*" $name]} {
        puts "IOBUF: $name"
        set io_pin   [get_pins "$iobuf/IO"]
        set i_pin    [get_pins "$iobuf/I"]
        set t_pin    [get_pins "$iobuf/T"]
        set o_pin    [get_pins "$iobuf/O"]
        set io_net   [get_nets -of_objects $io_pin]
        set i_net    [get_nets -of_objects $i_pin]
        set t_net    [get_nets -of_objects $t_pin]
        set o_net    [get_nets -of_objects $o_pin]
        puts "  IO net: $io_net  ports=[get_ports -of_objects $io_net]"
        puts "  I  net: $i_net (drives I)"
        if {$i_net ne ""} {
            foreach p [get_pins -of_objects $i_net] { puts "    pin: $p (DIR=[get_property DIRECTION $p])" }
        }
        puts "  T  net: $t_net"
        if {$t_net ne ""} {
            foreach p [get_pins -of_objects $t_net] { puts "    pin: $p (DIR=[get_property DIRECTION $p])" }
        }
        puts "  O  net: $o_net (drives downstream)"
        if {$o_net ne ""} {
            foreach p [get_pins -of_objects $o_net] { puts "    pin: $p (DIR=[get_property DIRECTION $p])" }
        }
    }
}

puts ""
puts "=== Same dump for cam_scl ==="
foreach iobuf $iobufs {
    set name [get_property NAME $iobuf]
    if {[string match "*cam_scl*" $name]} {
        puts "IOBUF: $name"
        set i_pin    [get_pins "$iobuf/I"]
        set t_pin    [get_pins "$iobuf/T"]
        set o_pin    [get_pins "$iobuf/O"]
        set i_net    [get_nets -of_objects $i_pin]
        set t_net    [get_nets -of_objects $t_pin]
        set o_net    [get_nets -of_objects $o_pin]
        puts "  I  net: $i_net"
        puts "  T  net: $t_net"
        puts "  O  net: $o_net"
        if {$o_net ne ""} {
            foreach p [get_pins -of_objects $o_net] { puts "    pin: $p (DIR=[get_property DIRECTION $p])" }
        }
    }
}

close_design
exit
