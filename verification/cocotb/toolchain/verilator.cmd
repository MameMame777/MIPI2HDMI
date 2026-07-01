#!perl
# WA#2 (Windows-native cocotb+Verilator). cocotb's runner resolves the simulator with
# shutil.which("verilator") and then runs  `perl <that path>`.  On MSYS ucrt64 the only
# PATHEXT match is verilator.bat -- a cmd batch that perl cannot parse.  cocotb_site
# prepends this toolchain dir to PATH, so shutil.which returns THIS file (verilator.cmd)
# instead.  Run as `perl verilator.cmd`, it execs the real verilator_bin.exe (found on
# PATH in ucrt64/bin) with VERILATOR_ROOT already exported by cocotb_site.prepend_path.
#
# NB: the .cmd extension exists only so shutil.which matches it; the body is perl, not
# batch.  It is never meant to be run directly by cmd.exe -- only via `perl`.
exec("verilator_bin.exe", @ARGV);
die "cocotb verilator wrapper: failed to exec verilator_bin.exe (is ucrt64/bin on PATH?): $!\n";
