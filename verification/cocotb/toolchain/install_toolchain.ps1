# One-shot installer for the cocotb + Verilator toolchain on native Windows (MSYS2 ucrt64).
# Prerequisite: MSYS2 itself must already be installed (https://www.msys2.org). This script
# installs the ucrt64 packages via pacman and the pinned Python deps via pip, then builds the
# static VPI lib. Idempotent (pacman --needed, pip re-install is cheap).
#
#     .\verification\cocotb\toolchain\install_toolchain.ps1

$ErrorActionPreference = 'Stop'

function Resolve-Msys2Base {
    if ($env:MSYS2_ROOT -and (Test-Path (Join-Path $env:MSYS2_ROOT 'usr\bin\pacman.exe'))) {
        return (Resolve-Path $env:MSYS2_ROOT).Path
    }
    $p = Get-Command pacman -ErrorAction SilentlyContinue
    if ($p -and $p.Source) {
        $r = Split-Path -Parent (Split-Path -Parent $p.Source)
        if (Test-Path (Join-Path $r 'usr\bin\pacman.exe')) { return $r }
    }
    $cands = @('C:\msys64', 'C:\msys2', 'C:\tools\msys64')
    if ($env:LOCALAPPDATA) { $cands += (Join-Path $env:LOCALAPPDATA 'msys64') }
    foreach ($c in $cands) {
        if (Test-Path (Join-Path $c 'usr\bin\pacman.exe')) { return $c }
    }
    throw "MSYS2 not found. Install MSYS2 (https://www.msys2.org) first, then set MSYS2_ROOT."
}

$root = Resolve-Msys2Base
$ucrt = Join-Path $root 'ucrt64\bin'
$pacman = Join-Path $root 'usr\bin\pacman.exe'
$ucrtPy = Join-Path $ucrt 'python.exe'
$env:MSYS2_ROOT = $root
$env:PATH = "$ucrt;$(Join-Path $root 'usr\bin');" + $env:PATH

Write-Host "== MSYS2_ROOT: $root =="
Write-Host "== [1/4] pacman ucrt64 packages =="
& $pacman -S --needed --noconfirm `
    mingw-w64-ucrt-x86_64-verilator `
    mingw-w64-ucrt-x86_64-gcc `
    mingw-w64-ucrt-x86_64-python `
    mingw-w64-ucrt-x86_64-python-pip `
    mingw-w64-ucrt-x86_64-make `
    mingw-w64-ucrt-x86_64-iverilog `
    perl
if ($LASTEXITCODE -ne 0) { throw "pacman failed ($LASTEXITCODE)" }

$cocotbDir = Split-Path -Parent $PSScriptRoot
$venv = Join-Path $cocotbDir '.venv'
$venvPy = Join-Path $venv 'bin\python.exe'

Write-Host "== [2/4] create the cocotb venv from the ucrt64 python (isolated deps) =="
if (-not (Test-Path $venvPy)) {
    & $ucrtPy -m venv $venv
    if ($LASTEXITCODE -ne 0) { throw "venv creation failed ($LASTEXITCODE)" }
}

Write-Host "== [3/4] pip deps into the venv (cocotb 2.0.1, pyuvm, pytest, ...) =="
$req = Join-Path $cocotbDir 'requirements.lock'
$env:COCOTB_IGNORE_PYTHON_REQUIRES = '1'
& $venvPy -m pip install --no-input -r $req
if ($LASTEXITCODE -ne 0) { throw "pip install failed ($LASTEXITCODE)" }

Write-Host "== [4/4] build static cocotb VPI lib for Verilator (into the venv's cocotb) =="
& $venvPy (Join-Path $cocotbDir 'bootstrap_vpi.py')
if ($LASTEXITCODE -ne 0) { throw "bootstrap_vpi failed ($LASTEXITCODE)" }

Write-Host "== toolchain ready (venv: $venv). Try: .\scripts\run_cocotb.ps1 -Suite smoke =="
