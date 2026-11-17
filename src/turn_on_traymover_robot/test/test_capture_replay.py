"""Check offline analysis against actual split STM32 serial reads."""
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    'capture_analysis', ROOT / 'scripts/analyze_stm32_capture.py')
ANALYSIS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANALYSIS)
CAPTURES = ROOT / 'reference/serial_capture_20261117'


def test_real_capture_counts_and_motion():
    result = ANALYSIS.analyze(CAPTURES / 'usb0_data_query.jsonl')
    assert result['valid_frame_types_id_status'] == {'03/80': 1, '02/60': 1199}
    assert result['invalid_header_candidates'] == 0
    assert result['motor_data']['vx_raw']['min'] == -26
    assert result['motor_data']['vx_raw']['max'] == 26
    assert result['motor_data']['wz_raw']['min'] == -254
    assert result['motor_data']['wz_raw']['max'] == 156


def test_fragmentation_corruption_and_error_status():
    stream = ANALYSIS.FrameStream()
    frames = []
    for line in (CAPTURES / 'usb0_data_query.jsonl').read_text().splitlines():
        record = json.loads(line)
        if record.get('direction') == 'rx':
            frames.extend(stream.feed(bytes.fromhex(record['hex'])))
    error_frame, motor_frame = frames[:2]
    damaged = bytearray(motor_frame)
    damaged[29] ^= 1
    stream = ANALYSIS.FrameStream()
    assert stream.feed(b'noise\x7f') == []
    assert stream.feed(motor_frame[1:17]) == []
    assert stream.feed(motor_frame[17:] + damaged + error_frame) == [
        motor_frame, error_frame]
    assert stream.invalid_candidates == 1
    assert ANALYSIS.decode_candidates(motor_frame)['right_raw'] == -152


def test_imu_data_is_not_motor_feedback():
    result = ANALYSIS.analyze(CAPTURES / 'acm0_passive_115200.jsonl')
    assert result['valid_frame_types_id_status'] == {}
    assert result['motor_data'] == {}
