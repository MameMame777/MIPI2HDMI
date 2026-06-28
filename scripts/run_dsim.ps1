# Unified DSim testbench runner. Replaces the 39 per-block run_<block>_test.ps1.
#
#   .\scripts\run_dsim.ps1 csi2_packet_parser            # -f csi2_packet_parser.f -top tb_csi2_packet_parser
#   .\scripts\run_dsim.ps1 dphy_lane_supervisor -Waves
#   .\scripts\run_dsim.ps1 ov5640_sccb_runtime -Filelist ov5640_sccb_init_probe_runtime.f -Top tb_ov5640_sccb_init_probe_runtime
#
# Convention: -Filelist defaults to "<Block>.f" and -Top to "tb_<Block>"; override either
# for blocks whose filelist/top do not follow the name (e.g. ov5640_sccb_runtime).
# Logs/waves go to the gitignored _dsim/ directory.
param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string]$Block,
    [string]$Filelist,
    [string]$Top,
    [switch]$Waves,
    [int]$Seed = 1
)

$ErrorActionPreference = "Stop"
if (-not $Filelist) { $Filelist = "$Block.f" }
if (-not $Top)      { $Top = "tb_$Block" }

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$Workspace = Split-Path -Parent $ScriptDir

function Resolve-DsimHome {
    if ($env:DSIM_HOME -and (Test-Path (Join-Path $env:DSIM_HOME "bin\dsim.exe"))) {
        return $env:DSIM_HOME
    }
    $preferred = "C:\Program Files\Altair\DSim\2026"
    if (Test-Path (Join-Path $preferred "bin\dsim.exe")) {
        return $preferred
    }
    $installRoot = "C:\Program Files\Altair\DSim"
    if (Test-Path $installRoot) {
        $candidate = Get-ChildItem -Path $installRoot -Directory |
            Where-Object { Test-Path (Join-Path $_.FullName "bin\dsim.exe") } |
            Sort-Object Name -Descending |
            Select-Object -First 1
        if ($candidate) { return $candidate.FullName }
    }
    throw "DSim installation not found. Set DSIM_HOME to the DSim install directory."
}

$DsimHome = Resolve-DsimHome
$TbDir   = Join-Path $Workspace "verification\tb"
$LogDir  = Join-Path $Workspace "_dsim\logs"
$WaveDir = Join-Path $Workspace "_dsim\wave"
New-Item -ItemType Directory -Force -Path $LogDir  | Out-Null
New-Item -ItemType Directory -Force -Path $WaveDir | Out-Null

$env:DSIM_HOME = $DsimHome
$env:DSIM_ROOT = $DsimHome
$env:DSIM_LIB_PATH = Join-Path $DsimHome "lib"
$env:UVM_HOME = Join-Path $DsimHome "uvm\1.2"
$env:STD_LIBS = Join-Path $DsimHome "std_pkgs\lib"
$env:RADFLEX_PATH = Join-Path $DsimHome "radflex"

# License resolution order:
# 1. ALTAIR_LICENSE_PATH set → use Altair License Manager (unset DSIM_LICENSE to avoid
#    UsageMeter picking up the dead cloud json).
# 2. DSIM_LICENSE already set → honour it.
# 3. Fallback: search for local dsim-license.json files.
if (-not $env:ALTAIR_LICENSE_PATH) {
    if (-not $env:DSIM_LICENSE) {
        $licenseCandidates = @(
            (Join-Path $DsimHome "dsim-license.json"),
            (Join-Path $env:LOCALAPPDATA "metrics-ca\dsim-license.json")
        )
        foreach ($licensePath in $licenseCandidates) {
            if (Test-Path $licensePath) { $env:DSIM_LICENSE = $licensePath; break }
        }
    }
} else {
    # Altair LM mode: DSIM_LICENSE must be unset or UsageMeter ignores ALTAIR_LICENSE_PATH
    Remove-Item Env:\DSIM_LICENSE -ErrorAction SilentlyContinue
}

$env:PATH = (Join-Path $DsimHome "bin") + ";" + (Join-Path $DsimHome "mingw\bin") + ";" + (Join-Path $DsimHome "dsim_deps\bin") + ";" + (Join-Path $DsimHome "lib") + ";" + $env:PATH

$timestamp = Get-Date -Format "yyyyMMdd_HHmmss"
$logFile  = Join-Path $LogDir  "${Block}_${timestamp}.log"
$waveFile = Join-Path $WaveDir "${Block}_${timestamp}.mxd"
$dsimExe  = Join-Path $DsimHome "bin\dsim.exe"

$dsimArgs = @(
    "-timescale", "1ns/1ps",
    "-f", $Filelist,
    "-top", $Top,
    "-sv_seed", $Seed,
    "-l", $logFile
)
if ($Waves) { $dsimArgs += @("+acc+b", "-waves", $waveFile) }

Write-Host "============================================================"
Write-Host "DSIM Test: $Block"
Write-Host "Working Dir: $TbDir"
Write-Host "DSIM_HOME: $DsimHome"
Write-Host "Log File: $logFile"
Write-Host "Command: $dsimExe $($dsimArgs -join ' ')"
Write-Host "============================================================"

Push-Location $TbDir
try {
    $process = Start-Process -FilePath $dsimExe -ArgumentList $dsimArgs -Wait -PassThru -NoNewWindow
    $exitCode = $process.ExitCode
} finally {
    Pop-Location
}

$failed = $exitCode -ne 0
if (Test-Path $logFile) {
    foreach ($line in (Get-Content $logFile)) {
        if ($line -match '\$fatal|CHECK FAILED|TEST FAILED|^=E:|^=F:') { $failed = $true }
    }
}

Write-Host "Exit Code: $exitCode"
Write-Host "Status: $(if ($failed) { 'FAIL' } else { 'PASS' })"
Write-Host "Log: $logFile"
if ($Waves) { Write-Host "Wave: $waveFile" }

exit $(if ($failed) { 1 } else { 0 })
