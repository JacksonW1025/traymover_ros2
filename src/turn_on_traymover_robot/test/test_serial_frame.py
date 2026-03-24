#!/usr/bin/env python3
"""
虚拟串口测试脚本

用法:
  1. 创建虚拟串口对:
     socat -d -d pty,raw,echo=0,link=/tmp/vserial0 pty,raw,echo=0,link=/tmp/vserial1

  2. 启动驱动节点 (使用 /tmp/vserial0):
     ros2 run turn_on_traymover_robot traymover_robot_node --ros-args -p usart_port_name:=/tmp/vserial0

  3. 运行本脚本读取 /tmp/vserial1:
     python3 test_serial_frame.py /tmp/vserial1

  4. 在另一个终端发送速度:
     ros2 run traymover_robot_keyboard traymover_keyboard
     或
     ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"
"""

import sys
import serial
import time


def parse_frame(data):
    """解析40字节帧并打印各字段"""
    if len(data) != 40:
        print(f"  [ERROR] Frame size: {len(data)}, expected 40")
        return False

    # 验证帧头帧尾
    if data[0] != 0x7F or data[1] != 0x7F:
        print(f"  [ERROR] Invalid header: {data[0]:02X} {data[1]:02X}")
        return False
    if data[38] != 0x0D or data[39] != 0x0A:
        print(f"  [ERROR] Invalid tail: {data[38]:02X} {data[39]:02X}")
        return False

    # 验证校验码
    total = sum(data[0:37]) & 0xFF
    expected_check = ((~total) + 0x01) & 0xFF
    if data[37] != expected_check:
        print(f"  [ERROR] Checksum mismatch: got 0x{data[37]:02X}, expected 0x{expected_check:02X}")
        return False

    # 解析速度
    vel_x = int.from_bytes(data[10:12], 'big', signed=True)
    vel_th = int.from_bytes(data[12:14], 'big', signed=True)

    # 解析其余字段
    serial_id = data[3]
    move_cmd = data[4]
    distance = int.from_bytes(data[5:7], 'big')
    sys_mode = data[7]
    left_scale = int.from_bytes(data[14:16], 'big')
    right_scale = int.from_bytes(data[16:18], 'big')
    wheel_dia = data[26]
    wheel_track = int.from_bytes(data[29:31], 'big')
    tick_ratio = data[31]
    jack_status = data[36]

    print(f"  Serial ID: 0x{serial_id:02X}  Move Cmd: 0x{move_cmd:02X}")
    print(f"  Vel_x: {vel_x} ({vel_x/1000.0:.3f} m/s)  Vel_th: {vel_th} ({vel_th/1000.0:.3f} rad/s)")
    print(f"  L/R Motor Scale: {left_scale}/{right_scale}")
    print(f"  Wheel: dia={wheel_dia}mm track={wheel_track}mm tick_ratio={tick_ratio}")
    print(f"  Jack: {jack_status}  Checksum: OK")
    return True


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_serial_frame.py <serial_port>")
        print("Example: python3 test_serial_frame.py /tmp/vserial1")
        sys.exit(1)

    port = sys.argv[1]
    print(f"Opening {port} for reading...")
    ser = serial.Serial(port, 115200, timeout=1.0)
    print(f"Listening for 40-byte frames on {port}...")
    print("Press Ctrl+C to stop.\n")

    frame_count = 0
    buffer = bytearray()

    try:
        while True:
            data = ser.read(40)
            if len(data) == 0:
                continue

            buffer.extend(data)

            # 寻找帧头 0x7F 0x7F
            while len(buffer) >= 40:
                # 查找帧头
                idx = -1
                for i in range(len(buffer) - 1):
                    if buffer[i] == 0x7F and buffer[i+1] == 0x7F:
                        idx = i
                        break

                if idx < 0:
                    # 没找到帧头，丢弃所有数据（保留最后一个字节）
                    buffer = buffer[-1:]
                    break

                if idx > 0:
                    # 丢弃帧头之前的数据
                    buffer = buffer[idx:]

                if len(buffer) < 40:
                    break

                # 提取一帧
                frame = bytes(buffer[:40])
                buffer = buffer[40:]
                frame_count += 1

                hex_str = ' '.join(f'{b:02X}' for b in frame)
                print(f"--- Frame #{frame_count} ---")
                print(f"  Raw: {hex_str}")
                parse_frame(frame)
                print()

    except KeyboardInterrupt:
        print(f"\nStopped. Total frames received: {frame_count}")
    finally:
        ser.close()


if __name__ == '__main__':
    main()
