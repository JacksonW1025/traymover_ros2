#!/usr/bin/env python3
"""Stop Traymover launcher processes and verify that no live children remain."""

import os
from pathlib import Path
import re
import signal
import sys
import time


PROC = Path('/proc')
MANAGED = b'TRAYMOVER_MANAGED_PROCESS=1'
SHELLS = {'bash', 'sh', 'dash', 'zsh', 'xterm', 'gnome-terminal'}
INSPECTION_COMMANDS = {'awk', 'grep', 'pgrep', 'pkill', 'ps', 'rg'}


def identity(path):
    stat = (path / 'stat').read_text(errors='replace').rsplit(')', 1)[1].split()
    return int(stat[1]), stat[19], stat[0]


def protected_processes():
    protected = set()
    pid = os.getpid()
    while pid > 0 and pid not in protected:
        protected.add(pid)
        try:
            pid, _, _ = identity(PROC / str(pid))
        except OSError:
            break
    return protected


def find_processes(pattern, protected, tracked, marker):
    processes = {}
    targets = set()
    for path in PROC.iterdir():
        if not path.name.isdecimal() or int(path.name) in protected:
            continue
        pid = int(path.name)
        try:
            if path.stat().st_uid != os.getuid():
                continue
            parent, started, state = identity(path)
            if state == 'Z':
                continue
            argv = (path / 'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if not argv[0]:
            continue
        executable = os.path.basename(os.fsdecode(argv[0])).lstrip('-')
        command = os.fsdecode(b' '.join(argv)).strip()
        processes[pid] = (parent, started, executable, command)
        if executable in INSPECTION_COMMANDS:
            continue
        try:
            managed = marker in (path / 'environ').read_bytes().split(b'\0')
        except OSError:
            managed = False
        if executable in SHELLS:
            # An unrelated shell may mention a ROS command in its script body.
            # Legacy launcher shells have the banner or a named script argument.
            matches = ("echo '[traymover] " in command and pattern.search(command)) or any(
                pattern.search(os.fsdecode(arg)) for arg in argv[1:]
                if arg.endswith(b'.sh') and not any(c in arg for c in (b' ', b'\n')))
        else:
            matches = pattern.search(command)
        if managed or matches or (pid, started) in tracked:
            targets.add(pid)

    while True:
        children = {pid for pid, info in processes.items() if info[0] in targets}
        if children <= targets:
            break
        targets.update(children)
    tracked.update((pid, processes[pid][1]) for pid in targets)
    return {pid: processes[pid] for pid in targets}


def send(pid, started, sig):
    pidfd = None
    try:
        if hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'):
            pidfd = os.pidfd_open(pid)
        _, current, state = identity(PROC / str(pid))
        if current == started and state != 'Z':
            if pidfd is None:
                os.kill(pid, sig)
            else:
                signal.pidfd_send_signal(pidfd, sig)
    except (ProcessLookupError, FileNotFoundError):
        pass
    except PermissionError as exc:
        print(f'Cannot stop PID {pid}: {exc}', file=sys.stderr)
    finally:
        if pidfd is not None:
            os.close(pidfd)


def main(args, marker=MANAGED):
    if not args:
        print('Expected the Traymover launcher process patterns.', file=sys.stderr)
        return 2
    pattern = re.compile('|'.join(f'(?:{item})' for item in args))
    protected = protected_processes()
    tracked = set()

    def scan():
        return find_processes(pattern, protected, tracked, marker)

    targets = scan()
    initial_count = len(targets)
    # Let ROS launch shut down its own children, including the chassis stop frame.
    workers = {pid for pid, info in targets.items() if info[2] not in SHELLS}
    for pid in workers:
        parent, started, _, _ = targets[pid]
        if parent not in workers:
            send(pid, started, signal.SIGINT)

    for sig, delay in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0), (None, 1.0)):
        deadline = time.monotonic() + delay
        while targets and time.monotonic() < deadline:
            time.sleep(0.1)
            targets = scan()
        targets = scan()
        if sig is not None:
            for pid, (_, started, _, _) in targets.items():
                send(pid, started, sig)

    if targets:
        for pid, (_, _, _, command) in sorted(targets.items()):
            print(f'[traymover] Remaining PID {pid}: {command}', file=sys.stderr)
        return 1
    print(f'[traymover] Verified no live Traymover processes remain ({initial_count} before cleanup).')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
