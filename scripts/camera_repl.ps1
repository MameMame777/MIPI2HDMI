# Launch the interactive camera-control REPL on the PYNQ board.
#   .\scripts\camera_repl.ps1                 # upload + open the REPL (>>> with `cam`)
#   .\scripts\camera_repl.ps1 -Go             # also run cam.go() (full bring-up) on start
#   .\scripts\camera_repl.ps1 -Host_ 192.168.2.99
#
# Uploads camera_repl.py + the shared modules (paramiko, no prompt), then opens an
# interactive `ssh -t` session running `python3 -i camera_repl.py`. Type the SSH
# password (xilinx) once. At the >>> prompt: cam.go() / cam.hdmi(60) / cam.status().
# Image processing: menu 2 (Live HDMI / processing) -> "Edge demo: cycle all + capture",
# "Binarize then Sobel edges", "Sobel edges then binarize"; or at >>> after cam.go():
# cam.edge_demo(testpattern=False) / cam.bin_edges(128) / cam.edge_binary(64).
#
# SAFETY: cam.hdmi()/cam.capture() auto-stop the VDMA on return/exit. Use cam.stop()
# if unsure. Quitting the REPL (exit() / Ctrl-D) stops the VDMA via atexit.
param(
    [string]$Host_ = '192.168.2.99',
    [switch]$Go
)
$ErrorActionPreference = 'Stop'
$repo    = Split-Path -Parent $PSScriptRoot
$py      = Join-Path $repo '.venv\Scripts\python.exe'
$scripts = Join-Path $repo 'scripts'
$pynqDir = '/home/xilinx/mipi2hdml'
$boardPy = '/usr/local/share/pynq-venv/bin/python3'

$files = @(
    'camera_repl.py', 'pynq_bringup.py', 'v65_capture.py', 'full_init_steps.py',
    'frame_height_stability.py', 'bitslip_lock.py', 'flicker_exposure_sweep.py'
)

# 1) Upload via paramiko (uses xilinx/xilinx; no interactive prompt).
$uploader = @'
import os, sys, paramiko
host, scripts_dir, pynq_dir = sys.argv[1], sys.argv[2], sys.argv[3]
files = sys.argv[4:]
ssh = paramiko.SSHClient(); ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(host, username='xilinx', password='xilinx', timeout=10)
ssh.exec_command('mkdir -p ' + pynq_dir)
sftp = ssh.open_sftp()
for f in files:
    sftp.put(os.path.join(scripts_dir, f), pynq_dir + '/' + f)
    print('  uploaded', f)
sftp.close(); ssh.close()
'@
Write-Host "Uploading REPL + shared modules to $Host_ ..." -ForegroundColor Cyan
$uploader | & $py - $Host_ $scripts $pynqDir @files
if ($LASTEXITCODE -ne 0) { throw "upload failed" }

# 2) Open the interactive REPL over ssh -t (PTY for python3 -i).
# NB: export XILINX_XRT/BOARD + ldconfig BEFORE python -- PYNQ device discovery needs
# them (without, Overlay() -> "No Devices Found"). Same env deploy_banding_test sets.
$goFlag = if ($Go) { 1 } else { 0 }
$env_setup = 'ldconfig 2>/dev/null; export XILINX_XRT=/usr; export BOARD=Pynq-Z2;'
$remote = "sudo bash -c '$env_setup cd $pynqDir && $boardPy -i camera_repl.py --go $goFlag'"
Write-Host "Opening camera REPL on $Host_ (type the SSH password: xilinx)." -ForegroundColor Cyan
Write-Host "  A menu opens: pick 1-4 by number. q quits the menu -> >>> prompt (cam.* API)." -ForegroundColor DarkGray
ssh -t "xilinx@$Host_" $remote
