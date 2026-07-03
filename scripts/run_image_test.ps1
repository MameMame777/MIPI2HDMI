# Run the image-file pyuvm verification (verification/cocotb/img_file_uvm) with a real
# image and per-DUT filter settings. Output images land in
# verification/cocotb/_exec/img_file_uvm/<dut>_<timestamp>/ (paths printed in the log).
#
#   .\scripts\run_image_test.ps1 -Image photo.png -Dut conv3x3 -Kernel sobel_x
#   .\scripts\run_image_test.ps1 -Image photo.jpg -Dut prefilter -Op median
#   .\scripts\run_image_test.ps1 -Image photo.png -Dut dither -DitherMode random -DitherBits 2
#   .\scripts\run_image_test.ps1                  # builtin pattern, all five DUTs
param(
    [string]$Image,
    [ValidateSet('conv3x3', 'conv5x5', 'prefilter', 'proc_slot', 'dither')]
    [string]$Dut,
    [string]$Kernel,       # conv3x3: identity|gaussian|sharpen|sobel_x|sobel_y|emboss|laplacian
                           # conv5x5: identity|gaussian5|log5
    [string]$Op,           # prefilter: pass|invert|gray|swap|threshold|r_only|g_only|b_only|gaussian|median
                           # proc_slot: same minus gaussian/median
    [string]$Coeffs,       # raw comma list of 9/25 signed ints (overrides -Kernel)
    [ValidateSet('ordered', 'random')][string]$DitherMode,
    [int]$DitherBits = -1, # 1..6
    [int]$Shift = -1,      # conv normalisation shift override
    [switch]$Abs,          # conv |result| (gradient magnitude)
    [int]$Thresh = -1,     # threshold-op level
    [string]$OutDir,
    [int]$MaxWidth = 0,    # converter downscale bound (default 640)
    [int]$MaxHeight = 0,   # (default 480)
    [int]$Frames = 0,      # frames streamed back-to-back (default 1)
    [switch]$Waves
)
$ErrorActionPreference = 'Stop'
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Definition
$Workspace = Split-Path -Parent $ScriptDir
. (Join-Path $Workspace 'verification\cocotb\toolchain\resolve_msys2.ps1')
$root = Resolve-Msys2Root
$env:MSYS2_ROOT = $root
$py = Get-CocotbPython -Workspace $Workspace -Root $root

# start from a clean IMG_* slate so stale session vars cannot leak into the run
Get-ChildItem Env: | Where-Object { $_.Name -like 'IMG_*' } |
    ForEach-Object { Remove-Item "Env:$($_.Name)" }

if ($Image) { $env:IMG_FILE = (Resolve-Path $Image).Path }
if ($Dut) { $env:IMG_DUT = $Dut }
if ($Kernel) { $env:IMG_KERNEL = $Kernel }
if ($Op) { $env:IMG_OP = $Op }
if ($Coeffs) { $env:IMG_COEFFS = $Coeffs }
if ($DitherMode) { $env:IMG_DITHER_MODE = $DitherMode }
if ($DitherBits -ge 0) { $env:IMG_DITHER_BITS = "$DitherBits" }
if ($Shift -ge 0) { $env:IMG_SHIFT = "$Shift" }
if ($Abs) { $env:IMG_ABS = '1' }
if ($Thresh -ge 0) { $env:IMG_THRESH = "$Thresh" }
# GetFullPath (not Resolve-Path): the dir may not exist yet, and the pytest cwd is the
# workspace root, so a relative -OutDir must be anchored to the CALLER's cwd here.
if ($OutDir) { $env:IMG_OUT_DIR = [IO.Path]::GetFullPath($OutDir, (Get-Location).Path) }
if ($MaxWidth -gt 0) { $env:IMG_MAX_W = "$MaxWidth" }
if ($MaxHeight -gt 0) { $env:IMG_MAX_H = "$MaxHeight" }
if ($Frames -gt 0) { $env:IMG_FRAMES = "$Frames" }
if ($Waves) { $env:COCOTB_WAVES = '1' }

Push-Location $Workspace
try {
    & $py -m pytest verification/cocotb/img_file_uvm -s -v -p no:cacheprovider
    $rc = $LASTEXITCODE
} finally {
    Pop-Location
    Get-ChildItem Env: | Where-Object { $_.Name -like 'IMG_*' } |
        ForEach-Object { Remove-Item "Env:$($_.Name)" }
    if ($Waves) { Remove-Item Env:COCOTB_WAVES -ErrorAction SilentlyContinue }
}
exit $rc
