# Traymover 串口通信协议完整推测文档

本文档基于以下参考资料推测而成：
1. `reference/ROS+To+Driver+串口通信协议解析.md` — 协议字段描述
2. `reference/example_ws/` — ROS1驱动实现源码（补充默认值、单位、编码细节）
3. 前进示例帧的逐字节验证

## 一、协议概览

| 项目 | 值 |
|------|-----|
| 帧长度 | 40 字节 (固定) |
| 帧头 | 0x7F 0x7F |
| 帧尾 | 0x0D 0x0A (\r\n) |
| 字节序 | 大端 (Big-Endian) |
| 校验 | 补码校验: (~sum(bytes[0:37]) + 0x01) & 0xFF |
| 波特率 | 115200 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控制 | 无 |

## 二、发送帧结构 (主控 → STM32)

### 2.1 完整字节表

| 字节 | 字段名 | 长度 | 类型 | 描述 | 默认值(前进示例) |
|------|--------|------|------|------|-----------------|
| 0-1 | Header | 2 | 固定 | 帧头 `0x7F 0x7F` | 0x7F 0x7F |
| 2 | Length | 1 | 固定 | 帧长度 `0x28` (40) | 0x28 |
| 3 | Serial ID Cmd | 1 | 命令 | 消息类型ID | 0x02 |
| 4 | Move Cmd | 1 | 数据 | 运动指令 | 0x00 |
| 5-6 | Distance | 2 | uint16 BE | 运动距离 | 0x0000 |
| 7 | System Mode | 1 | 数据 | 系统模式 | 0x00 |
| 8 | TOF Switch | 1 | 数据 | TOF传感器开关 | 0x00 |
| 9 | Reserved | 1 | 保留 | 保留(代码中用于reInitMotor) | 0x00 |
| **10-11** | **Vel_x** | **2** | **int16 BE** | **X轴线速度 = linear.x × 1000** | **0x0190 (400)** |
| **12-13** | **Vel_th** | **2** | **int16 BE** | **Z轴角速度 = angular.z × 1000** | **0x0000** |
| 14-15 | Left Motor Scale | 2 | uint16 BE | 左电机比例 | 0x0001 |
| 16-17 | Right Motor Scale | 2 | uint16 BE | 右电机比例 | 0x0001 |
| 18 | Update Motor Chip | 1 | 数据 | 电机芯片更新标志 | 0x00 |
| 19-20 | Motor Filter | 2 | uint16 BE | 电机滤波系数 | 0x0001 |
| 21 | Alarm LED | 1 | 数据 | 报警灯 (0=灭, 1=亮) | 0x01 |
| 22 | IR Switch | 1 | 数据 | 红外传感器开关 | 0x00 |
| 23 | Left IR Threshold | 1 | uint8 | 左红外参考值 | 0x14 (20) |
| 24 | Mid IR Threshold | 1 | uint8 | 中红外参考值 | 0x14 (20) |
| 25 | Right IR Threshold | 1 | uint8 | 右红外参考值 | 0x14 (20) |
| 26 | Wheel Diameter | 1 | uint8 | 轮径 = wheel_diameter_m × 1000 | 0xAA (170) |
| 27-28 | Gear Reduction (低16位) | 2 | uint16 BE | 减速比低位 | 见下文 |
| 29-30 | Wheel Track | 2 | uint16 BE | 轮距 = wheel_track_m × 1000 | 0x01C7 (455) |
| 31 | Tick Meter Ratio | 1 | uint8 | 里程计比例 | 0x0A (10) |
| 32 | Buzzer | 1 | 数据 | 蜂鸣器 (0=静, 1=响) | 0x00 |
| 33 | Arrival Signal | 1 | 数据 | 到达信号 (0=未到, 1=到达) | 0x00 |
| 34-35 | Gear Reduction (高16位) | 2 | uint16 BE | 减速比高位 | 见下文 |
| 36 | Jack Status | 1 | 数据 | 顶升 (0=无, 1=升, 2=降) | 0x00 |
| 37 | Checksum | 1 | 校验 | 补码校验 | 计算得出 |
| 38-39 | Tail | 2 | 固定 | 帧尾 `0x0D 0x0A` | 0x0D 0x0A |

### 2.2 Serial ID Cmd (字节3) 取值

| 值 | 名称 | 用途 |
|----|------|------|
| 0x00 | MSG_ID_NULL | 空消息 |
| 0x01 | MSG_ID_GET_BAUD | 查询波特率 |
| 0x02 | MSG_ID_GET_MOTOR_DATA | 电机数据帧（主要控制帧） |
| 0x03 | MSG_ID_GET_MOTOR_STATUS | 电机状态查询 |

### 2.3 Move Cmd (字节4) 取值

| 值 | 含义 |
|----|------|
| 0x00 | Forward (前进) |
| 0x01 | Backward (后退) |
| 0x02 | Turn Left (左转) |
| 0x03 | Turn Right (右转) |
| 0x04 | Stop (停止) |

### 2.4 Gear Reduction 编码

减速比为32位无符号整数，拆分存储在两个不相邻的位置：
- bytes[27-28]: 低16位 (convert_4data_array_结果的bytes[2]和bytes[3])
- bytes[34-35]: 高16位 (convert_4data_array_结果的bytes[0]和bytes[1])

以 gear_reduction = 5600 (0x000015E0) 为例:
- bytes[27] = 0x15, bytes[28] = 0xE0 (低16位 = 0x15E0)
- bytes[34] = 0x00, bytes[35] = 0x00 (高16位 = 0x0000)

## 三、校验算法

```
checksum = (~sum(frame[0:37])) + 0x01) & 0xFF

即: 将字节0到36的所有值相加(uint8自然溢出)，按位取反，再加1。
等价于对求和结果取补码(two's complement)。
```

### 验证 (前进示例帧)

```
帧: 7F 7F 28 02 00 00 00 00 00 00 01 90 00 00 00 01 00 01 00 00 00 01 00 14 14 14 AA 10 00 01 C7 0A 00 00 00 00 00 [7C] 0D 0A

sum(bytes[0:37]) = 0x384
0x384 & 0xFF = 0x84
~0x84 & 0xFF = 0x7B
0x7B + 0x01 = 0x7C ✓
```

## 四、速度编码

### 4.1 编码方式

ROS2 Twist消息中的速度(float, 单位m/s或rad/s)转换为int16大端序：

```python
vel_x_int16 = int(twist.linear.x * 1000)   # 线速度, 放大1000倍
vel_th_int16 = int(twist.angular.z * 1000)  # 角速度, 放大1000倍

# 大端序编码
frame[10] = (vel_x_int16 >> 8) & 0xFF      # 高字节
frame[11] = vel_x_int16 & 0xFF              # 低字节
frame[12] = (vel_th_int16 >> 8) & 0xFF
frame[13] = vel_th_int16 & 0xFF
```

### 4.2 控制模式

本驱动使用**速度控制模式**:
- Move Cmd (byte 4) 固定为 0x00
- Vel_x/Vel_th 携带实际速度值
- Distance (bytes 5-6) 为 0x0000

## 五、默认参数 (来自 driver_config_lungu.yaml)

| 参数 | 值 | 说明 |
|------|-----|------|
| wheel_diameter | 0.17 m | 轮径 |
| wheel_track | 0.455 m | 轮距(轮中心间距) |
| gear_reduction | 5600 | 编码器tick数/圈 |
| tick_meter_ratio | 10 | 里程计比例系数 |
| left_pwm_scale | 1.0 | 左电机比例 |
| right_pwm_scale | 1.0 | 右电机比例 |
| baud_rate | 115200 | 串口波特率 |

> 注意: 以上参数来自example_ws参考代码，实际新底盘的机械参数需要后续根据硬件实际情况调整。

## 六、预定义静态帧

### 6.1 查询波特率 (get_baud)
```
7F 7F 28 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 [checksum] 0D 0A
```

### 6.2 获取电机数据 (get_motor_data)
```
7F 7F 28 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 [checksum] 0D 0A
```

### 6.3 获取电机状态 (get_motor_status)
```
7F 7F 28 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 [checksum] 0D 0A
```

## 七、不确定因素

以下内容在文档和源码中信息不足，需要接入STM32后实测确认：

1. **速度单位**: Vel_x=400是否确实对应0.4m/s的实际运动速度
2. **发送频率**: 以多高频率发送控制帧最合适(暂定10Hz)
3. **响应帧**: STM32的回复帧格式(第一版暂不解析)
4. **Move Cmd行为**: 速度控制模式下Move Cmd=0x00是否为最优选择
5. **实际机械参数**: 新底盘的轮径、轮距、减速比等
