# Traymover 实机连接与联调步骤

本文档用于 `traymover_ros2` 在未接入导航算法前的基础实机联调，目标是先把以下链路逐步跑通：

- STM32 底盘串口控制与 `/odom`
- N300pro IMU 与 `/imu/data_raw`
- Leishen C32 Lidar 与 `/point_cloud_raw`
- `robot_localization` EKF 与 `/odom_combined`
- RViz 中的机器人姿态与点云显示

默认软件环境基于当前仓库中的以下配置：

- STM32 串口：`/dev/ttyUSB0`
- IMU 串口：`/dev/ttyCH343USB0`
- C32 雷达 IP：`192.168.1.200`
- 底盘参数：轮距 `0.445 m`，轮径 `0.20 m`

## 1. 联调前准备

1. 将驱动轮架空，确保机器人第一次联调时不会直接在地面运动。
2. 准备急停，确认底盘上电后能随时切断动力。
3. 主机安装并可正常使用 ROS 2 Humble。
4. 在主机上已编译当前工作区：

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select turn_on_traymover_robot traymover_robot_description
source install/setup.bash
```

## 2. 物理连接

### 2.1 STM32 底盘

1. 使用 USB 转串口线将 STM32 与主机连接。
2. 确认底盘串口接入后由系统识别为 `/dev/ttyUSB0` 或其它 `ttyUSB*` 设备。
3. 如果实际设备名不是 `/dev/ttyUSB0`，修改：
   [`traymover_robot.yaml`](/home/wheeltec/traymover_ros2/src/turn_on_traymover_robot/config/traymover_robot.yaml)

### 2.2 N300pro IMU

1. 使用 USB 线连接 N300pro 到主机。
2. 期望设备名为 `/dev/ttyCH343USB0`。
3. 如果实际设备名不同，可以在 launch 时覆盖，不必改源码。

### 2.3 Leishen C32 Lidar

1. 用网线将 C32 连接到主机网口或交换机。
2. 确认主机网卡与雷达处于同一网段。
3. 当前默认雷达 IP 为 `192.168.1.200`，定义在：
   [`traymover_param.yaml`](/home/wheeltec/traymover_ros2/src/turn_on_traymover_robot/config/traymover_param.yaml)

## 3. 识别设备

在接好线后，先检查主机是否真的看到了设备：

```bash
ls -l /dev/serial/by-id/
ls /dev/ttyUSB* /dev/ttyCH343USB* 2>/dev/null
ip -4 addr
```

通过标准：

- 能看到 STM32 对应串口
- 能看到 IMU 对应串口
- 能看到准备接雷达的网卡已经起来

## 4. 配置主机网口到雷达网段

将主机网口配置到 `192.168.1.x/24` 网段，例如：

```bash
sudo ip addr add 192.168.1.10/24 dev <你的网卡名>
sudo ip link set <你的网卡名> up
ping 192.168.1.200
```

通过标准：

- `ping 192.168.1.200` 能通

如果这里不通，先不要继续雷达联调，优先排查：

- 网口名写错
- 主机已有冲突 IP
- 雷达未上电
- 网线或交换机问题

## 5. 每个终端都先加载环境

所有后续终端都先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash
```

## 6. 单独联调 IMU

启动 IMU：

```bash
ros2 launch turn_on_traymover_robot traymover_imu.launch.py
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/data_raw --once
```

通过标准：

- `/imu/data_raw` 有频率
- 消息头 `frame_id` 为 `imu_link`
- 姿态四元数不是全零

如果 IMU 串口不是默认值，可直接覆盖：

```bash
ros2 launch turn_on_traymover_robot traymover_imu.launch.py imu_serial_port:=/dev/<你的IMU串口>
```

## 7. 单独联调 C32 点云

启动雷达：

```bash
ros2 launch turn_on_traymover_robot traymover_lidar.launch.py
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash
ros2 topic hz /point_cloud_raw
ros2 topic echo /point_cloud_raw --once
```

通过标准：

- `/point_cloud_raw` 存在
- 点云消息头 `frame_id` 为 `laser`
- 话题频率稳定

## 8. 单独联调 STM32 底盘与 `/odom`

先不启动 IMU，只看底盘回包和里程计：

```bash
ros2 launch turn_on_traymover_robot base_serial.launch.py use_imu:=false
```

另开终端观察：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash
ros2 topic hz /odom
ros2 topic echo /odom
```

通过标准：

- 终端中不再持续报 “No valid STM32 motor feedback frame received”
- `/odom` 有持续输出

如果实际 STM32 串口不是 `/dev/ttyUSB0`，先改：
[`traymover_robot.yaml`](/home/wheeltec/traymover_ros2/src/turn_on_traymover_robot/config/traymover_robot.yaml)

## 9. 架空状态下验证前进 odom 方向

保持轮子架空，发送很小的前进速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" -r 5
```

观察：

- 轮子是否向前转
- `/odom.pose.pose.position.x` 是否增加

停止命令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

如果轮子向前但 `x` 变负，修改：

- `left_encoder_sign`
- `right_encoder_sign`

配置文件位置：
[`traymover_robot.yaml`](/home/wheeltec/traymover_ros2/src/turn_on_traymover_robot/config/traymover_robot.yaml)

常见修正：

- 前进时 odom 后退：左右都改为 `-1`

## 10. 架空状态下验证原地旋转方向

发送原地转向速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.2}}" -r 5
```

观察：

- `/odom.twist.twist.angular.z` 是否为正
- yaw 是否按逆时针为正增长

如果前进方向正确但旋转方向反了，优先检查是否只有一侧编码器符号需要翻转。

## 11. 启动 EKF，验证 `/odom_combined`

当 `/odom` 已正常后，再带上 IMU 和 EKF：

```bash
ros2 launch turn_on_traymover_robot turn_on_traymover_robot.launch.py use_lidar:=false use_ekf:=true use_rviz:=false
```

另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash
ros2 topic hz /odom_combined
ros2 topic echo /odom_combined --once
ros2 run tf2_ros tf2_echo odom_combined base_footprint
```

通过标准：

- `/odom_combined` 存在
- `odom_combined -> base_footprint` TF 存在

## 12. 启动完整基础链路

当 IMU、雷达、底盘、EKF 都单项通过后，再启动完整基础 bringup：

```bash
ros2 launch turn_on_traymover_robot turn_on_traymover_robot.launch.py use_lidar:=true use_ekf:=true use_rviz:=true
```

此时应能同时看到：

- `/imu/data_raw`
- `/point_cloud_raw`
- `/odom`
- `/odom_combined`

## 13. RViz 纯展示链路

如果只是想看机器人姿态和点云，不需要底盘与 EKF，可直接启动：

```bash
ros2 launch turn_on_traymover_robot traymover_visualization.launch.py
```

该链路会带起：

- `traymover_imu.launch.py`
- `traymover_lidar.launch.py`
- `traymover_robot_description/display.launch.py`

## 14. 常见问题排查

### 14.1 没有 `/odom`

优先检查：

- STM32 串口名是否正确
- 波特率是否为 `115200`
- STM32 是否真的在回传 40 字节帧
- 回包长度字节是否为 `0x28`

### 14.2 `/odom` 方向反

优先修改：

- `left_encoder_sign`
- `right_encoder_sign`

### 14.3 有 `/odom` 但 RViz 不随 `odom` 走

默认 `publish_odom_tf: false`，这是刻意设置的，避免与 EKF 世界系 TF 混用。当前主要使用：

- `/odom` 作为原始轮式里程计
- `/odom_combined` 作为后续导航输入

### 14.4 雷达无点云

优先检查：

- 主机网卡 IP 是否在 `192.168.1.x`
- `ping 192.168.1.200` 是否可达
- 雷达是否上电
- 网线和交换机是否正常

## 15. 推荐的首次实机联调顺序

第一次上机建议严格按下面顺序：

1. 只识别设备，不启动任何节点
2. 单独跑 IMU
3. 单独跑 C32
4. 单独跑 STM32 `/odom`
5. 小速度测试前进与旋转方向
6. 启动 EKF 看 `/odom_combined`
7. 最后再启动完整 bringup

不要一开始就直接全链路启动，否则很难判断是串口、IMU、网络还是 TF 出了问题。
