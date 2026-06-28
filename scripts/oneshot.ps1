# One-shot camera capture -> picture\ directory.
#   .\scripts\oneshot.ps1                # capture one frame, save to picture\
#   .\scripts\oneshot.ps1 -Vcm 280       # set a VCM focus code first
#   .\scripts\oneshot.ps1 -Reboot        # power-state reset the chip first (if degraded)
param(
    [string]$Host_ = '192.168.2.99',
    [int]$Vcm = -1,
    [switch]$Reboot
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$py = Join-Path $repo '.venv\Scripts\python.exe'

$argList = @(
    (Join-Path $repo 'scripts\deploy_banding_test.py'),
    '--host', $Host_,
    '--script', 'oneshot_capture.py',
    '--download', '1', '--full-init', '1',
    '--pull-dir', 'picture',
    '--timeout', '400'
)
if ($Reboot) { $argList += '--reboot' }
if ($Vcm -ge 0) { $argList += @('--extra-args', "--vcm $Vcm") }

& $py @argList
Write-Host "`nLatest picture(s):"
Get-ChildItem (Join-Path $repo 'picture\pic_*.png') -ErrorAction SilentlyContinue |
    Sort-Object LastWriteTime -Descending | Select-Object -First 1 FullName
