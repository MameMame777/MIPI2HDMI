## Extract the known-good placement (LOC/BEL) of the D-PHY frontend primitives
## from the WNS=+0.112 routed checkpoint, so they can be pinned in the XDC for
## reproducible builds (2026-06-22, 30fps hardening).
set repo_dir [file normalize [file join [file dirname [info script]] ..]]
set dcp [file join $repo_dir vloop_probes2 vloop.runs impl_1 bd_wrapper_routed.dcp]
puts "Opening $dcp"
open_checkpoint $dcp

puts "==DPHY_LOC_BEGIN=="
foreach t {BUFIO BUFR IDELAYCTRL ISERDESE2 IDELAYE2 BUFGCTRL MMCME2_ADV PLLE2_ADV} {
    foreach c [get_cells -hier -filter "REF_NAME == $t"] {
        set loc [get_property LOC $c]
        set bel [get_property BEL $c]
        puts "DPHYLOC|$t|$loc|$bel|$c"
    }
}
puts "==DPHY_LOC_END=="
close_project
