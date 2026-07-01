# Shared MSYS2 ucrt64 resolver for the cocotb+Verilator PowerShell entry points.
# Dot-source this file, then call Resolve-Msys2Root. Mirrors the env->probe->fail-loud
# pattern of scripts/run_dsim.ps1 (DSIM_HOME) and cocotb_site.py. Commits no absolute paths.

function Resolve-Msys2Root {
    # 1. explicit override
    if ($env:MSYS2_ROOT -and (Test-Path (Join-Path $env:MSYS2_ROOT 'ucrt64\bin\verilator_bin.exe'))) {
        return (Resolve-Path $env:MSYS2_ROOT).Path
    }
    # 2. derive from a verilator on PATH (works even at a non-standard install dir)
    $v = Get-Command verilator -ErrorAction SilentlyContinue
    if ($v -and $v.Source) {
        $root = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $v.Source))
        if (Test-Path (Join-Path $root 'ucrt64\bin\verilator_bin.exe')) { return $root }
    }
    # 3. probe well-known roots
    $cands = @('C:\msys64', 'C:\msys2', 'C:\tools\msys64')
    if ($env:LOCALAPPDATA) { $cands += (Join-Path $env:LOCALAPPDATA 'msys64') }
    foreach ($c in $cands) {
        if (Test-Path (Join-Path $c 'ucrt64\bin\verilator_bin.exe')) { return $c }
    }
    throw "MSYS2 ucrt64 not found. Set MSYS2_ROOT to the install dir (it must contain ucrt64\bin\verilator_bin.exe)."
}

function Get-Ucrt64Python {
    param([string]$Root = (Resolve-Msys2Root))
    return (Join-Path $Root 'ucrt64\bin\python.exe')
}

# The python the cocotb runner/tests use: the project's ucrt64-based venv if present
# (isolated deps), else the raw ucrt64 python. The venv is created from the ucrt64 python so
# it shares the same libpython -> same VPI ABI (conftest accepts it via sys.base_prefix).
function Get-CocotbPython {
    param([Parameter(Mandatory = $true)][string]$Workspace, [string]$Root = (Resolve-Msys2Root))
    $venvPy = Join-Path $Workspace 'verification\cocotb\.venv\bin\python.exe'
    if (Test-Path $venvPy) { return $venvPy }
    return (Join-Path $Root 'ucrt64\bin\python.exe')
}
