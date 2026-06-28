"""Deploy + run a PYNQ-side test script, then pull its artifacts to _capture/.

Usage (Windows):
    python scripts/deploy_banding_test.py --host 192.168.2.99 --reboot --tests t1,t2,t5
    # user covers the lens, then:
    python scripts/deploy_banding_test.py --host 192.168.2.99 --no-upload \
        --download 0 --tests t3,t4
    # frame-height / PLL standardisation driver:
    python scripts/deploy_banding_test.py --host 192.168.2.99 --reboot \
        --script frame_height_stability.py --full-init 1 \
        --extra-args "--stages s0,s1,s2"
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

# Everything the PYNQ-side scripts import must be fresh on the board.
UPLOAD_FILES = [
    # shared modules (imported deps)
    'pynq_bringup.py',
    'v65_capture.py',
    'full_init_steps.py',
    'frame_height_stability.py',
    'bitslip_lock.py',
    'flicker_exposure_sweep.py',
    # end scripts (run via --script)
    'camera_hdmi_demo.py',
    'oneshot_capture.py',
    'zero_pynq_test.py',
    'vts_sweep.py',
    'vts_hdmi_ceiling.py',
    'focus_probe.py',
    'aec_probe.py',
    'pll_sweep.py',
    'pll30_idelay_lock.py',
    'settle_blank_sweep.py',
    'color_capture.py',
    'proc_slot_demo.py',
    'conv_kernel_demo.py',
    'dog_demo.py',
    'cascade_demo.py',
    'edge_demo.py',
    'hwlock_verify.py',
    'camera_repl.py',
    'banding_isolation.py',
    'frozen_pattern_test.py',
    'tpg_verify.py',
    'tpg_hdmi_demo.py',
]

# script -> artifact filename prefix on /home/xilinx (also the --script choices)
ARTIFACT_PREFIX = {
    'camera_hdmi_demo.py': 'hdmidemo_',
    'oneshot_capture.py': 'pic_',
    'zero_pynq_test.py': 'zeropynq_',
    'vts_sweep.py': 'vtssweep_',
    'vts_hdmi_ceiling.py': 'vtsceil_',
    'focus_probe.py': 'pic_focusbest_',
    'aec_probe.py': 'aecprobe_',
    'pll_sweep.py': 'pllsweep_',
    'pll30_idelay_lock.py': 'pll30_',
    'settle_blank_sweep.py': 'sbsweep_',
    'color_capture.py': 'picr_',
    'proc_slot_demo.py': 'procslot_',
    'conv_kernel_demo.py': 'convk_',
    'dog_demo.py': 'dog_',
    'cascade_demo.py': 'casc_',
    'edge_demo.py': 'edge_',
    'hwlock_verify.py': 'hwlock_',
    'camera_repl.py': 'repl_',
    'banding_isolation.py': 'banding_',
    'frame_height_stability.py': 'fhs_',
    'frozen_pattern_test.py': 'frozen_',
    'tpg_verify.py': 'tpg_',
    'tpg_hdmi_demo.py': 'tpghdmi_',
}


def connect(host: str, user: str = 'xilinx', password: str = 'xilinx',
            timeout: float = 10.0) -> paramiko.SSHClient:
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, username=user, password=password,
                look_for_keys=False, allow_agent=False, timeout=timeout)
    return ssh


def reboot_and_wait(host: str, wait_max_s: float = 240.0) -> None:
    print(f'Rebooting {host} (board power-state reset for OV5640) ...')
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
            ssh = connect(host, timeout=5)
            ssh.close()
            print(f'  board back after {time.time() - t0 + 20:.0f}s')
            return
        except Exception:
            time.sleep(5)
    print('ERROR: board did not come back after reboot')
    sys.exit(1)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='192.168.2.99')
    ap.add_argument('--reboot', action='store_true',
                    help='sudo reboot the board first (chip power-state reset)')
    ap.add_argument('--no-upload', action='store_true')
    ap.add_argument('--script', default='banding_isolation.py',
                    choices=sorted(ARTIFACT_PREFIX),
                    help='PYNQ-side script to run')
    ap.add_argument('--tests', default='t1,t2,t5',
                    help='banding_isolation.py only')
    ap.add_argument('--download', type=int, default=1,
                    help='passed through: 1=reprogram bitstream, 0=attach')
    ap.add_argument('--full-init', type=int, default=0)
    ap.add_argument('--extra-args', default='',
                    help='extra args appended to banding_isolation.py')
    ap.add_argument('--upload-bit', default='',
                    help='local .bit to push to /home/xilinx/mipi2hdml/'
                         'bd_wrapper.bit (its sibling .hwh is pushed too)')
    ap.add_argument('--pull-dir', default='_capture',
                    help='local directory (relative to repo) to pull artifacts '
                         'into (default _capture)')
    ap.add_argument('--timeout', type=int, default=900)
    args = ap.parse_args()

    if args.reboot:
        reboot_and_wait(args.host)

    print(f'Connecting to xilinx@{args.host} ...')
    ssh = connect(args.host)
    print('Connected.')

    try:
        if args.upload_bit:
            bit_local = args.upload_bit
            hwh_local = os.path.splitext(bit_local)[0] + '.hwh'
            if not os.path.exists(bit_local):
                print(f'ERROR: {bit_local} not found'); sys.exit(1)
            sftp = ssh.open_sftp()
            try:
                for src, dst in ((bit_local, f'{PYNQ_DIR}/bd_wrapper.bit'),
                                 (hwh_local, f'{PYNQ_DIR}/bd_wrapper.hwh')):
                    if os.path.exists(src):
                        mb = os.path.getsize(src) / 1e6
                        print(f'Uploading bitstream {os.path.basename(src)} '
                              f'({mb:.2f} MB) -> {dst}')
                        sftp.put(src, dst)
                    else:
                        print(f'WARNING: {src} not found, skipping')
            finally:
                sftp.close()
            print()

        if not args.no_upload:
            sftp = ssh.open_sftp()
            try:
                for fn in UPLOAD_FILES:
                    local = os.path.join(HERE, fn)
                    remote = f'{PYNQ_DIR}/{fn}'
                    size = os.path.getsize(local)
                    print(f'Uploading {fn} ({size // 1024} kB)')
                    sftp.put(local, remote)
            finally:
                sftp.close()
            print()

        tests_arg = (f'--tests {args.tests} '
                     if args.script == 'banding_isolation.py' else '')
        script_args = (f'{tests_arg}--download {args.download} '
                       f'--full-init {args.full_init} {args.extra_args}').strip()
        inner = (f'ldconfig 2>/dev/null; export XILINX_XRT=/usr; export BOARD=Pynq-Z2; '
                 f'cd {PYNQ_DIR} && {PYTHON} -u {args.script} {script_args}')
        cmd = f'sudo bash -c "{inner}" 2>&1'
        print(f'Running: {cmd}')
        print('=' * 60)

        chan = ssh.get_transport().open_session()
        chan.get_pty()
        chan.exec_command(cmd)
        chan.setblocking(0)

        deadline = time.time() + args.timeout
        while True:
            if chan.exit_status_ready():
                while chan.recv_ready():
                    print(chan.recv(4096).decode(errors='replace'), end='', flush=True)
                break
            if chan.recv_ready():
                print(chan.recv(4096).decode(errors='replace'), end='', flush=True)
            elif time.time() > deadline:
                print('\nERROR: timeout waiting for banding_isolation.py')
                chan.close()
                break
            else:
                time.sleep(0.2)
        rc = chan.recv_exit_status()
        print(f'\n[exit code: {rc}]')

        # ---- pull artifacts (npy/png) into the chosen local dir ----
        dest = (args.pull_dir if os.path.isabs(args.pull_dir)
                else os.path.join(REPO, args.pull_dir))
        os.makedirs(dest, exist_ok=True)
        prefix = ARTIFACT_PREFIX[args.script]
        sftp = ssh.open_sftp()
        n = 0
        try:
            for entry in sftp.listdir_attr('/home/xilinx'):
                name = entry.filename
                if name.startswith(prefix) and name.endswith(
                        ('.npy', '.png', '.raw', '.json')):
                    if not stat.S_ISREG(entry.st_mode):
                        continue
                    sftp.get(f'/home/xilinx/{name}', os.path.join(dest, name))
                    n += 1
        finally:
            sftp.close()
        print(f'Downloaded {n} artifacts to {dest}')

    finally:
        ssh.close()


if __name__ == '__main__':
    main()
