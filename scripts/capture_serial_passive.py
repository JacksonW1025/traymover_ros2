#!/usr/bin/env python3
"""Capture raw serial RX as timestamped JSONL; never transmit serial bytes."""
import argparse
import fcntl
import json
import os
import select
import termios
import time
from datetime import datetime, timezone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    parser.add_argument('output')
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()
    baud = getattr(termios, f'B{args.baud}')
    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    old = termios.tcgetattr(fd)
    total = 0
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        attr = termios.tcgetattr(fd)
        attr[0] = attr[1] = attr[3] = 0
        attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[4] = attr[5] = baud
        attr[6][termios.VMIN] = 0
        attr[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attr)
        start = time.monotonic()
        with open(args.output, 'x') as out:
            out.write(json.dumps({'port': args.port, 'baud': args.baud,
                                  'utc': datetime.now(timezone.utc).isoformat(),
                                  'mode': 'passive_rx_no_tx'}) + '\n')
            while time.monotonic() - start < args.seconds:
                if select.select([fd], [], [], min(0.1, max(
                        0, args.seconds - (time.monotonic() - start))))[0]:
                    chunk = os.read(fd, 4096)
                    if not chunk:
                        raise RuntimeError('Serial device disconnected or returned EOF')
                    total += len(chunk)
                    out.write(json.dumps({'t': time.monotonic() - start,
                                          'hex': chunk.hex(' ')}) + '\n')
            out.write(json.dumps({'elapsed': time.monotonic() - start,
                                  'rx_bytes': total}) + '\n')
        print(f'{args.port}: {total} RX bytes -> {args.output}')
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSANOW, old)
        finally:
            os.close(fd)


if __name__ == '__main__':
    main()
