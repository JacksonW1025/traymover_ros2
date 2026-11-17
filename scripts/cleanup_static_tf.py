#!/usr/bin/env python3
"""Remove orphaned Traymover static TF publishers before navigation starts."""

import os
from pathlib import Path
import signal
import time


MANAGED_NODES = frozenset({
    'static_odom_to_camera_init', 'static_body_to_base_link',
    'option20_raw_origin', 'option20_base_height', 'option20_scan_origin',
    'stm32_base_height',
})


def find_publishers(proc=Path('/proc'), domain=None, uid=None):
    """Only this user's named static publishers in the selected ROS domain."""
    domain = str(int(os.environ.get('ROS_DOMAIN_ID', '0'))) if domain is None else str(int(domain))
    uid = os.getuid() if uid is None else uid
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            if entry.stat().st_uid != uid:
                continue
            args = entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
            if not args or Path(args[0]).name != 'static_transform_publisher':
                continue
            names = [arg.split(':=', 1)[1] for arg in args if arg.startswith('__node:=')]
            if len(names) != 1 or names[0] not in MANAGED_NODES:
                continue
            env = dict(item.split(b'=', 1) for item in entry.joinpath('environ').read_bytes().split(b'\0')
                       if b'=' in item)
            if str(int(env.get(b'ROS_DOMAIN_ID', b'0'))) != domain:
                continue
            # /proc stat's comm field can contain spaces and parentheses.
            fields = entry.joinpath('stat').read_text().rsplit(')', 1)[1].split()
            parent = int(fields[1])
            start = fields[19]
            parent_name = proc.joinpath(str(parent), 'comm').read_text().strip()
            orphan = parent == 1 or parent_name in ('systemd', 'init')
            found.append((int(entry.name), names[0], orphan, start))
        except (OSError, ValueError, IndexError, UnicodeError):
            # Processes can exit during inspection; never infer ownership on error.
            continue
    return found


def same_process(pid, start, proc=Path('/proc')):
    try:
        fields = proc.joinpath(str(pid), 'stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] == start and fields[0] != 'Z'
    except (OSError, IndexError):
        return False


def stop_orphan(pid, start):
    """Pin the process with pidfd where available; do not target a reused PID."""
    fd = None
    try:
        if hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'):
            fd = os.pidfd_open(pid)
        if not same_process(pid, start):
            return True

        def send(sig):
            if fd is not None:
                signal.pidfd_send_signal(fd, sig)
            elif same_process(pid, start):
                os.kill(pid, sig)

        send(signal.SIGTERM)
        deadline = time.monotonic() + 2.0
        while same_process(pid, start) and time.monotonic() < deadline:
            time.sleep(0.05)
        if same_process(pid, start):
            send(signal.SIGKILL)
            deadline = time.monotonic() + 1.0
            while same_process(pid, start) and time.monotonic() < deadline:
                time.sleep(0.05)
        return not same_process(pid, start)
    except ProcessLookupError:
        return True
    finally:
        if fd is not None:
            os.close(fd)


def main():
    publishers = find_publishers()
    active = [(pid, name) for pid, name, orphan, _ in publishers if not orphan]
    if active:
        print(f'[traymover] Navigation already owns static TF publishers: {active}. '
              'Finish stopping the previous navigation session before starting another.')
        return 1
    for pid, name, _, start in publishers:
        print(f'[traymover] Removing orphan static TF: {name} (PID {pid})', flush=True)
        if not stop_orphan(pid, start):
            print(f'[traymover] Could not stop stale TF publisher {pid}; aborting startup.')
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
