"""CLI regression: explicit PCD inputs may live outside the checkout."""
import csv
import json
from pathlib import Path
import struct
import subprocess
import sys


def test_external_pcd_input_writes_complete_results(tmp_path):
    pcd = tmp_path / 'external.pcd'
    header = (
        'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
        'COUNT 1 1 1\nWIDTH 8\nHEIGHT 1\nPOINTS 8\nDATA binary\n'
    )
    pcd.write_bytes(header.encode() + b''.join(
        struct.pack('<fff', float(i + 1), 0.5, 1.0) for i in range(8)))
    output = tmp_path / 'results'
    runner = Path(__file__).resolve().parents[1] / 'scripts/run_offline_experiments.py'
    result = subprocess.run(
        [sys.executable, str(runner), '--pcd', str(pcd), '--output-dir', str(output)],
        capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    metadata = json.loads((output / 'metadata.json').read_text())
    assert metadata['pcd_inputs'] == [str(pcd)]
    with (output / 'pcd_interference_stats.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]['pcd'] == str(pcd)
    assert rows[0]['original_points'] == '8'
    assert (output / 'gate_sweep_attempts.csv').is_file()
    assert (output / 'gate_sweep_summary.csv').is_file()
    assert 'not real vehicle' in (output / 'experiment_summary.md').read_text()
