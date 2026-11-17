#!/usr/bin/env python3
"""Replay timestamped serial JSONL, checking framing before decoding candidates."""
import argparse
import collections
import csv
import json
from pathlib import Path


class FrameStream:
    """Handle split/coalesced reads and resynchronize after damaged frames."""

    def __init__(self):
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.invalid_candidates = 0

    def feed(self, chunk):
        self.buffer.extend(chunk)
        frames = []
        while len(self.buffer) >= 3:
            start = self.buffer.find(b'\x7f\x7f\x28')
            if start < 0:
                self.discarded_bytes += len(self.buffer) - 2
                del self.buffer[:-2]
                break
            self.discarded_bytes += start
            del self.buffer[:start]
            if len(self.buffer) < 40:
                break
            frame = bytes(self.buffer[:40])
            if frame[38:] != b'\r\n' or sum(frame[:38]) & 255:
                self.invalid_candidates += 1
                self.discarded_bytes += 1
                del self.buffer[0]
                continue
            frames.append(frame)
            del self.buffer[:40]
        return frames


def decode_candidates(frame):
    """Names/scales are hypotheses from protocol; raw fields stay available."""
    def signed_pair(offset):
        return int.from_bytes(frame[offset:offset + 2], 'big', signed=True)

    return dict(zip(
        ('left_raw', 'right_raw', 'vx_raw', 'wz_raw'),
        (signed_pair(offset) for offset in (11, 13, 29, 31))))


def analyze(path, csv_path=None):
    stream = FrameStream()
    types = collections.Counter()
    rows = []
    tx_count = 0
    rx_bytes = 0
    for line in Path(path).read_text().splitlines():
        record = json.loads(line)
        if 'hex' not in record:
            continue
        if record.get('direction') == 'tx':
            tx_count += 1
            continue
        chunk = bytes.fromhex(record['hex'])
        rx_bytes += len(chunk)
        for frame in stream.feed(chunk):
            types[f'{frame[3]:02x}/{frame[4]:02x}'] += 1
            if frame[3:5] == b'\x02\x60':
                row = {'t_rx_chunk_end': record['t'], **decode_candidates(frame)}
                row['vx_mps_document_scale'] = row['vx_raw'] / 1000
                row['wz_rads_document_scale'] = row['wz_raw'] / 1000
                row['frame_hex'] = frame.hex(' ')
                rows.append(row)
    if csv_path and rows:
        with open(csv_path, 'x', newline='') as out:
            writer = csv.DictWriter(out, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
    fields = {}
    for name in ('left_raw', 'right_raw', 'vx_raw', 'wz_raw'):
        values = [row[name] for row in rows]
        if values:
            fields[name] = {'min': min(values), 'max': max(values),
                            'nonzero': sum(value != 0 for value in values)}
    return {'file': str(path), 'tx_records': tx_count, 'rx_bytes': rx_bytes,
            'valid_frame_types_id_status': dict(types),
            'invalid_header_candidates': stream.invalid_candidates,
            'discarded_bytes': stream.discarded_bytes,
            'trailing_bytes': len(stream.buffer), 'motor_data': fields}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture')
    parser.add_argument('--csv', help='New output CSV path; never overwrite')
    args = parser.parse_args()
    print(json.dumps(analyze(args.capture, args.csv), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
