# Run a cocotb test file/dir directly under the ucrt64 python, without a manifest entry.
# Useful while porting a new block before it is registered in manifest.toml.
#
#   .\scripts\pytest_cocotb.ps1 verification/cocotb/csi2_header_ecc
param(
    [Parameter(Position = 0, Mandatory = $true)]
    [string]$Path
)
$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$Workspace = Split-Path -Parent $ScriptDir
. (Join-Path $Workspace 'verification\cocotb\toolchain\resolve_msys2.ps1')
$root = Resolve-Msys2Root
$env:MSYS2_ROOT = $root
$py = Get-CocotbPython -Workspace $Workspace -Root $root   # project venv if present, else ucrt64
Push-Location $Workspace
try {
    # -s (no capture) streams the full cocotb sim output live (regression table, per-test
    # SIM TIME, dut._log, assertions), not just pytest's pass/fail summary.
    & $py -m pytest $Path -s -v -p no:cacheprovider
    $rc = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $rc
