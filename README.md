# Traymover ROS2

基于 ROS 2 Humble 的机场行李托盘搬运机器人软件栈。运行于 NVIDIA Jetson Orin NX (ARM64) 平台，通过 40 字节串口协议驱动 STM32 差速底盘，融合 N300Pro IMU 与 Leishen C32 32 线激光雷达，支持 EKF 里程计融合、GMapping / LIO-SAM 建图及 Nav2 自主导航。

## 硬件概览

| 组件 | 型号 | 接口 | 默认设备 |
|------|------|------|----------|
| 主控 | Jetson Orin NX | — | — |
| 底盘 | STM32 差速驱动 | USB-串口 (CH341) | `/dev/ttyCH341USB0` |
| IMU | HiPNUC N300Pro | USB-串口 (CH343) | `/dev/ttyCH343USB0` |
| 激光雷达 | Leishen C32 | 以太网 | `192.168.1.200` |
| 底盘参数 | 轮径 0.20 m，轮距 0.445 m | — | — |

## 目录结构

```
traymover_ros2/
├── src/
│   ├── turn_on_traymover_robot/          # 核心驱动与 bringup
│   │   ├── config/
│   │   │   ├── traymover_robot.yaml      #   底盘串口、编码器、里程计参数
│   │   │   ├── traymover_param.yaml      #   IMU 型号、雷达型号与 IP
│   │   │   ├── ekf.yaml                  #   robot_localization EKF 配置
│   │   │   └── imu.yaml                  #   IMU 滤波器参数
│   │   ├── launch/
│   │   │   ├── turn_on_traymover_robot.launch.py   # 主 bringup
│   │   │   ├── base_serial.launch.py               # 底盘 + IMU
│   │   │   ├── traymover_lidar.launch.py           # 雷达驱动
│   │   │   ├── traymover_imu.launch.py             # 单独 IMU
│   │   │   ├── traymover_ekf.launch.py             # EKF 融合
│   │   │   ├── traymover_gmapping.launch.py        # GMapping 建图
│   │   │   └── traymover_visualization.launch.py   # 纯可视化
│   │   └── turn_on_traymover_robot/
│   │       └── traymover_robot.py        #   STM32 串口节点 (40 字节协议)
│   │
│   ├── traymover_robot_description/      # URDF 模型与 RViz
│   │   ├── urdf/traymover.urdf.xacro
│   │   ├── scripts/imu_pose_broadcaster.py
│   │   ├── rviz/traymover.rviz
│   │   └── launch/                       #   description / rviz / display
│   │
│   ├── traymover_robot_keyboard/         # 键盘遥控
│   │   └── traymover_robot_keyboard/traymover_keyboard.py
│   │
│   ├── traymover_lidar_ros2/             # 雷达驱动包
│   │   ├── lslidar_ros2/                 #   Leishen C32/CX/X10 驱动
│   │   └── pointcloud_to_laserscan-humble/  # 点云 → LaserScan 转换
│   │
│   ├── traymover_robot_slam/             # SLAM 算法
│   │   ├── LIO-SAM-ROS2/                 #   LIO-SAM (激光-惯性 SLAM)
│   │   ├── slam_gmapping/                #   GMapping (2D 栅格 SLAM)
│   │   └── openslam_gmapping/            #   OpenSLAM 核心库
│   │
│   └── traymover_robot_nav2/             # Nav2 自主导航
│       ├── launch/                       #   导航启动、地图保存
│       ├── param/                        #   Nav2 参数 (AMCL, DWB, etc.)
│       └── map/                          #   保存的地图文件
│
├── reference/                            # 参考文档与原始资料
└── install/ build/ log/                  # colcon 构建产物
```

## 快速开始

### 环境准备

```bash
cd /home/wheeltec/traymover_ros2
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

> 局域网内有其他 ROS2 节点时，建议设置 `export ROS_LOCALHOST_ONLY=1`。

### 键盘遥控（最简验证）

```bash
# 终端 1：启动底盘
ros2 launch turn_on_traymover_robot base_serial.launch.py use_imu:=false

# 终端 2：键盘控制
ros2 run traymover_robot_keyboard traymover_keyboard
```

键盘按键：

| 按键 | 功能 |
|------|------|
| `i` / `,` | 前进 / 后退 |
| `j` / `l` | 左转 / 右转 |
| `u` / `o` | 左前 / 右前 |
| `m` / `.` | 左后 / 右后 |
| `k` / 空格 | 停止 |
| `q` / `z` | 整体加速 / 减速 10% |
| `w` / `x` | 线速度加 / 减 10% |
| `e` / `c` | 角速度加 / 减 10% |

## 功能命令

### 1. 单独联调传感器

```bash
# IMU
ros2 launch turn_on_traymover_robot traymover_imu.launch.py

# 雷达
ros2 launch turn_on_traymover_robot traymover_lidar.launch.py

# 底盘 (不带 IMU)
ros2 launch turn_on_traymover_robot base_serial.launch.py use_imu:=false
```

### 2. 完整 Bringup

```bash
# 底盘 + IMU + 雷达 + EKF + RViz
ros2 launch turn_on_traymover_robot turn_on_traymover_robot.launch.py \
    use_lidar:=true use_ekf:=true use_rviz:=true
```

主要参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_lidar` | `false` | 启动雷达驱动 |
| `use_ekf` | `false` | 启动 EKF 里程计融合 |
| `use_rviz` | `true` | 启动 RViz 可视化 |
| `use_imu` | `true` | 启动 N300Pro IMU |

### 3. GMapping 建图

```bash
# 启动建图（自动带起底盘 + IMU + 雷达 + EKF + GMapping）
ros2 launch turn_on_traymover_robot traymover_gmapping.launch.py

# 另开终端用键盘遥控探索环境
ros2 run traymover_robot_keyboard traymover_keyboard

# 保存地图
ros2 launch traymover_robot_nav2 save_map.launch.py
```

### 4. LIO-SAM 建图

```bash
# 先启动基础链路（不带 EKF，避免 TF 冲突）
ros2 launch turn_on_traymover_robot turn_on_traymover_robot.launch.py \
    use_lidar:=true use_ekf:=false use_rviz:=false

# 另开终端启动 LIO-SAM
ros2 launch lio_sam run.launch.py
```

### 5. Nav2 自主导航

```bash
# 需要先有建好的地图文件
ros2 launch traymover_robot_nav2 traymover_nav2.launch.py \
    map:=<地图yaml路径>
```

### 6. 纯可视化（不启动底盘）

```bash
ros2 launch turn_on_traymover_robot traymover_visualization.launch.py
```

## 话题与坐标系

### 关键话题

| 话题 | 类型 | 来源 | 说明 |
|------|------|------|------|
| `/cmd_vel` | `Twist` | 键盘 / Nav2 | 速度指令 |
| `/odom` | `Odometry` | traymover_robot_node | 轮式里程计 (20 Hz) |
| `/odom_combined` | `Odometry` | EKF | 融合里程计 (10 Hz) |
| `/imu/data_raw` | `Imu` | N300Pro 驱动 | IMU 原始数据 (100 Hz) |
| `/point_cloud_raw` | `PointCloud2` | lslidar 驱动 | 32 线点云 (~17 Hz) |
| `/scan` | `LaserScan` | pointcloud_to_laserscan | 2D 扫描 (用于 GMapping / Nav2) |
| `/map` | `OccupancyGrid` | SLAM 节点 | 栅格地图 |

### TF 坐标系树

```
map
 └── odom_combined (EKF) 或 odom (LIO-SAM)
      └── base_footprint
           └── base_link
                ├── left_wheel_link
                ├── right_wheel_link
                ├── sensor_mast_link
                │    └── laser
                ├── camera_mount_link
                │    └── camera_link
                └── imu_link
```

## 配置文件说明

| 文件 | 用途 |
|------|------|
| `config/traymover_robot.yaml` | 底盘串口、轮径、轮距、编码器方向、超时保护 |
| `config/traymover_param.yaml` | IMU 型号选择、雷达型号与 IP |
| `config/ekf.yaml` | EKF 传感器融合参数（输入源、噪声、协方差） |
| `config/imu.yaml` | IMU Madgwick 滤波器参数 |
| `LIO-SAM-ROS2/config/traymover_params.yaml` | LIO-SAM 外参标定、特征阈值、回环检测 |
| `traymover_robot_nav2/param/.../param_traymover_diff.yaml` | Nav2 导航参数（AMCL、规划器、控制器） |

## 实机联调

首次上机建议严格按以下顺序逐步验证，详细步骤见 [`reference/traymover_hardware_bringup.md`](reference/traymover_hardware_bringup.md)：

1. 识别设备（`ls /dev/ttyCH*`，`ping 192.168.1.200`）
2. 单独跑 IMU → 检查 `/imu/data_raw`
3. 单独跑雷达 → 检查 `/point_cloud_raw`
4. 单独跑底盘 → 检查 `/odom`
5. 架空轮子测试前进与旋转方向
6. 启动 EKF → 检查 `/odom_combined`
7. 全链路 bringup
