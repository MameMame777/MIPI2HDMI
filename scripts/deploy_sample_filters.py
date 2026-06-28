"""Deploy + run sample_filters_capture.py on the PYNQ board, pull the stills.

Fully scripted, no manual operation: uploads the board-side deps + the capture
script, runs the gallery over SSH, streams its output, then pulls each
``<filter-name>.png`` / ``.npy`` from the board's jupyter ``_capture`` into
``<repo>/_capture/samples/`` with clean filenames (the names come from the
``SAMPLE_FILES=`` manifest the board script prints).

Usage (Windows):
    python scripts/deploy_sample_filters.py                       # all filters
    python scripts/deploy_sample_filters.py --only edges,sketch   # subset
    python scripts/deploy_sample_filters.py --no-upload --download 0   # attach/reuse
"""
import argparse
import os
import stat
import sys
import time

import paramiko

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PYNQ_DIR = '/home/xilinx/mipi2hdml'
PYTHON = '/usr/local/share/pynq-venv/bin/python3'
BOARD_CAP = '/home/xilinx/jupyter_notebooks/_capture'   # where cam.capture() writes

# board-side deps that sample_filters_capture.py (via camera_repl) imports
UPLOAD_FILES = [
    'pynq_bringup.py',
    'v65_capture.py',
    'full_init_steps.py',
    'frame_height_stability.py',
    'bitslip_lock.py',
    'camera_repl.py',
    'oneshot_capture.py',
    'sample_filters_capture.py',
]


def connect(host, user='xilinx', password='xilinx', timeout=10.0):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username=user, password=password,
                look_for_keys=False, allow_agent=False, timeout=timeout)
    return ssh


def reboot_and_wait(host, wait_max_s=240.0):
    """sudo reboot cycles the OV5640 power-state (the documented recovery from chip
    degradation: long_pkt drops to 0 after repeated re-locks). Wait for SSH back."""
    print(f'Rebooting {host} (OV5640 power-state reset)...')
    ssh = connect(host)
    try:
        ssh.exec_command('sudo reboot', timeout=10)
    except Exception:
        pass
    ssh.close()
    time.sleep(20)
    t0 = time.time()
    while time.time() - t0 < wait_max_s:
        try:
            connect(host, timeout=5).close()
            print(f'  board back after {time.time() - t0 + 20:.0f}s')
            return
        except Exception:
            time.sleep(5)
    print('ERROR: board did not come back after reboot')
    raise SystemExit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.2.99')
    ap.add_argument('--reboot', action='store_true',
                    help='sudo reboot first (OV5640 power-state reset; use after repeated '
                         'runs degrade the chip -> long_pkt 0 / black filters)')
    ap.add_argument('--download', type=int, default=1, help='1 = reprogram overlay')
    ap.add_argument('--hw', type=int, default=1, help='HW-lock FSM (cam.go hw=)')
    ap.add_argument('--val4800', default='0x14',
                    help='0x14 = healthy fs=30 constant-height stream (clean stills); '
                         '0x24 (no-LS) tiles')
    ap.add_argument('--settle', default='1.0', help='per-grab settle seconds')
    ap.add_argument('--grabs', default='6', help='grabs per filter (least-tiled kept)')
    ap.add_argument('--long-as-line', default='0', help='deliver no-LS longs as rows')
    ap.add_argument('--only', default='', help='comma list = capture only these names')
    ap.add_argument('--no-upload', action='store_true')
    ap.add_argument('--pull-dir', default=os.path.join('_capture', 'samples'),
                    help='local dir (relative to repo) for the pulled stills')
    ap.add_argument('--timeout', type=int, default=1200)
    args = ap.parse_args()

    if args.reboot:
        reboot_and_wait(args.host)

    print(f'Connecting to xilinx@{args.host} ...')
    ssh = connect(args.host)
    print('Connected.')
    try:
        if not args.no_upload:
            sftp = ssh.open_sftp()
            try:
                for fn in UPLOAD_FILES:
                    local = os.path.join(HERE, fn)
                    if not os.path.exists(local):
                        print(f'  WARNING: {fn} missing locally, skipping')
                        continue
                    print(f'  upload {fn} ({os.path.getsize(local) // 1024} kB)')
                    sftp.put(local, f'{PYNQ_DIR}/{fn}')
            finally:
                sftp.close()
            print()

        only = f'--only {args.only} ' if args.only else ''
        inner = (f'ldconfig 2>/dev/null; export XILINX_XRT=/usr; export BOARD=Pynq-Z2; '
                 f'cd {PYNQ_DIR} && {PYTHON} -u sample_filters_capture.py '
                 f'--download {args.download} --hw {args.hw} --val4800 {args.val4800} '
                 f'--settle {args.settle} --grabs {args.grabs} '
                 f'--long-as-line {getattr(args, "long_as_line")} {only}').strip()
        cmd = f'sudo bash -c "{inner}" 2>&1'
        print(f'Running: {cmd}')
        print('=' * 60)

        chan = ssh.get_transport().open_session()
        chan.get_pty()
        chan.exec_command(cmd)
        chan.setblocking(0)

        buf = ''
        deadline = time.time() + args.timeout
        while True:
            if chan.exit_status_ready():
                while chan.recv_ready():
                    s = chan.recv(4096).decode(errors='replace')
                    buf += s
                    print(s, end='', flush=True)
                break
            if chan.recv_ready():
                s = chan.recv(4096).decode(errors='replace')
                buf += s
                print(s, end='', flush=True)
            elif time.time() > deadline:
                print('\nERROR: timeout waiting for sample_filters_capture.py')
                chan.close()
                break
            else:
                time.sleep(0.2)
        rc = chan.recv_exit_status()
        print(f'\n[exit code: {rc}]')

        # ---- pull the stills (driven by the SAMPLE_FILES manifest) ----
        names = []
        for line in buf.splitlines():
            if line.strip().startswith('SAMPLE_FILES='):
                names = [x for x in line.split('=', 1)[1].strip().split(',') if x]
        dest = os.path.join(REPO, args.pull_dir)
        os.makedirs(dest, exist_ok=True)

        sftp = ssh.open_sftp()
        n = 0
        try:
            existing = {}
            for e in sftp.listdir_attr(BOARD_CAP):
                if stat.S_ISREG(e.st_mode):
                    existing[e.filename] = e
            if not names:                  # fallback: pull every png/npy stem present
                print('WARNING: no SAMPLE_FILES manifest; pulling all png/npy in _capture')
                names = sorted({f[:-4] for f in existing if f.endswith('.png')})
            for nm in names:
                for ext in ('.png', '.npy'):
                    fn = f'{nm}{ext}'
                    if fn in existing:
                        sftp.get(f'{BOARD_CAP}/{fn}', os.path.join(dest, fn))
                        n += 1
        finally:
            sftp.close()
        print(f'Pulled {n} files for {len(names)} filters -> {dest}')
    finally:
        ssh.close()


if __name__ == '__main__':
    main()
