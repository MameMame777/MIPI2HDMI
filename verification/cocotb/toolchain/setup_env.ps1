# Prepare the current PowerShell session for the cocotb + Verilator toolchain.
# Dot-source to affect THIS shell:
#     . .\verification\cocotb\toolchain\setup_env.ps1
# Running it normally only prints the resolved paths (child-process env is discarded).

$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'resolve_msys2.ps1')

$root = Resolve-Msys2Root
$ucrt = Join-Path $root 'ucrt64\bin'
$usr = Join-Path $root 'usr\bin'
$env:MSYS2_ROOT = $root
$env:VERILATOR_ROOT = (Join-Path $root 'ucrt64\share\verilator').Replace('\', '/')
# toolchain shims first (perl verilator wrapper WA#2, make shim WA#3), then ucrt64 + usr bin
$toolchain = $PSScriptRoot
$makeShim = Join-Path $PSScriptRoot 'make_shim'
$env:PATH = "$toolchain;$makeShim;$ucrt;$usr;" + $env:PATH

Write-Host "MSYS2_ROOT     : $root"
Write-Host "ucrt64 python  : $(Join-Path $ucrt 'python.exe')"
Write-Host "VERILATOR_ROOT : $($env:VERILATOR_ROOT)"
& (Join-Path $ucrt 'verilator_bin.exe') --version
