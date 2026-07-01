# cocotb + Verilator test runner -- the native-Windows successor to run_dsim.ps1.
#
#   .\scripts\run_cocotb.ps1 csi2_packet_parser         # one block
#   .\scripts\run_cocotb.ps1 csi2_packet_parser -Waves  # + dump.vcd
#   .\scripts\run_cocotb.ps1 -Suite smoke               # a whole suite
#   .\scripts\run_cocotb.ps1 -List                      # list blocks
#
# Resolves the MSYS2 ucrt64 python (env MSYS2_ROOT -> verilator-on-PATH -> probe) and drives
# verification/cocotb/runner.py. Logs/reports go to gitignored verification/cocotb/_exec/.
param(
    [Parameter(Position = 0)]
    [string]$Block,
    [string]$Suite,
    [switch]$Waves,
    [switch]$List
)

$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$Workspace = Split-Path -Parent $ScriptDir
. (Join-Path $Workspace 'verification\cocotb\toolchain\resolve_msys2.ps1')

$root = Resolve-Msys2Root
$env:MSYS2_ROOT = $root
$ucrtPy = Get-CocotbPython -Workspace $Workspace -Root $root   # project venv if present, else ucrt64
$runner = Join-Path $Workspace 'verification\cocotb\runner.py'

$runnerArgs = @()
if ($List) { $runnerArgs += '--list' }
if ($Suite) { $runnerArgs += @('--suite', $Suite) }
if ($Block) { $runnerArgs += $Block }
if ($Waves) { $runnerArgs += '--waves' }

& $ucrtPy $runner @runnerArgs
exit $LASTEXITCODE
