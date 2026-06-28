# Live camera -> HDMI output (verified band-free mode, 2026-06-17):
#   continuous-legacy 0x14 + sup=0, SOF-synth + force-480 (constant height, no
#   roll), and the byte-domain settle-blank K=8 (the bottom-band fix). This is
#   the config confirmed band-free on the monitor.
#   .\scripts\hdmi.ps1                 # 180s live output to the HDMI monitor
#   .\scripts\hdmi.ps1 -Seconds 60     # shorter run
#   .\scripts\hdmi.ps1 -Vcm 280        # fix the VCM focus code
#   .\scripts\hdmi.ps1 -Sweep          # step the VCM focus during output (find sharp)
#   .\scripts\hdmi.ps1 -GainCeiling 0x40  # cap AGC gain (4x) to cut low-light column FPN
#   .\scripts\hdmi.ps1 -Reboot         # power-state reset the chip first (if degraded)
#
# NOTE: this drives S2MM (camera->DDR) + MM2S (DDR->HDMI). It runs for -Seconds
# then cleans up the VDMA itself. DO NOT Ctrl+C / kill it mid-run -- that leaves
# the VDMA writing DDR and hangs sshd (physical power cycle needed). Just wait.
param(
    [string]$Host_ = '192.168.2.99',
    [int]$Seconds = 180,
    [int]$Vcm = -1,
    [string]$GainCeiling = '',
    [switch]$Sweep,
    [switch]$Reboot
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$py = Join-Path $repo '.venv\Scripts\python.exe'

$sweepFlag = if ($Sweep) { 1 } else { 0 }
# Explicit verified band-fix mode (don't rely on script defaults drifting):
# 0x14 continuous-legacy, sup off, SOF-synth + force-480 (no roll), settle-blank K=8.
$extra = "--total $Seconds --vcm-sweep $sweepFlag " +
         "--val4800 0x14 --sup 0 --synth 1 --force-expected 1 --settle-blank 8 --lock-rerolls 8"
if ($Vcm -ge 0) { $extra += " --vcm $Vcm" }
if ($GainCeiling -ne '') { $extra += " --gain-ceiling $GainCeiling" }

$argList = @(
    (Join-Path $repo 'scripts\deploy_banding_test.py'),
    '--host', $Host_,
    '--script', 'camera_hdmi_demo.py',
    '--download', '1', '--full-init', '1',
    '--extra-args', $extra,
    '--timeout', ($Seconds + 120)
)
if ($Reboot) { $argList += '--reboot' }

Write-Host "HDMI live output for $Seconds s. Watch the monitor. Do NOT interrupt." -ForegroundColor Cyan
& $py @argList
