# Option 19 — 无地图、小范围 Guided Detour 测试

实现位于 `integration/nav-safety-reports-20260917` 工作树，基于已有 Option 18。
Option 19 测试局部 Detour 与 EStop-safe recovery；不是完整导航或物理制动认证。
全部默认值都是 **initial test values，未经实车安全验证**。

## 构建和启动

本次实际开发目录：

```bash
cd /tmp/traymover-integrated-20260917-2b46r7et/workspace
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash
colcon --log-base /tmp/traymover-guided-log build \
  --build-base /tmp/traymover-guided-build --install-base install --symlink-install \
  --packages-select traymover_robot_nav traymover_robot_safety turn_on_traymover_robot
source install/setup.bash
bash scripts/traymover.sh
```

选择 `19) Guided EStop-safe Detour Test (No Map / No Nav2)`。
启动选项只问 `Launch sensor visualization/RViz? [Y/n]`。
传感器与安全节点日志写入工作树的 `logs/`；引导器在当前终端前台运行。
可用时另开现有 manual EStop keyboard；RViz 使用现有 EStopPanel。
无图形终端时，引导器全程支持 `!` 然后 Enter 请求手动 EStop；`q`+Enter / Ctrl+C
停止当前测试。运动中输入会被消费，不能提前授权下一步。键盘/EStop 操作者应始终在机器人侧面。

不需要 PCD、PGM/YAML 地图、Navigation Goal、NDT 或 FAST-LIO prior map。
启动前先正常停止其他导航/遥控会话；若有竞争速度发布者，本测试的 preflight 会拒绝 ARM。
测试不会为了启动而自动终止其他会话。

## 软件链与共享逻辑

```text
LiDAR raw cloud -> /scan + /scan_detour_perimeter ----+
RealSense -> depthimage_to_laserscan -> /scan_realsense +-> shared Detour Core
IMU /imu/data_raw ----------------------------------+
RealSense depth -> automatic EStop -> driver states -+

foreground guided runner -> shared core -> /cmd_vel_detour_test
    -> Collision Monitor -> /cmd_vel -> driver EStop/watchdog -> chassis
```

`detour_core.py` 是原 Option 18 `detour_gate.py` 的核心，原导入路径保留兼容。
生产 supervisor 和 guided core 共享 Scene 障碍判断、左右空间候选、覆盖检查、WAIT、
clear confirmation、状态机 tick、EStop hold/discard、guard、转向控制、sensor freshness、
phase timeout 和恢复入口。`detour_obstacle.py` 共享时间戳、四元数和完整 3D scan-to-ray
变换，包括相机光轴深度约束。`detour_motion.py` 的 guided policy 只替换反馈/授权策略：
IMU yaw 转向、时间受限直行、显式逐动作授权和 session limits。

生产直行保留定位位移反馈；测试直行用时间反馈。二者共用同一状态机、Guard、超时和
中断路径；不能声称测试模式验证了生产定位位移精度。测试 observation 不伪造导航目标或
localization_healthy，不订阅 wheel odometry、map pose 或 navigation action。

LiDAR 使用完整周向投影，以保留生产旋转空间验证。未知/缺失 LiDAR rays 不当作自由空间。
缓存扫描依据采集时 IMU yaw 转到当前 heading，并扣除保守的命令运动余量；该余量不是实测距离。
固定 TF 使用 URDF 的 `base_link`，RViz 和测试 CM 的 fixed/odom frame 均为 `base_link`。
没有伪造 `odom`、`map` 或里程计。

只启动底盘、IMU、URDF/TF、LiDAR、RealSense、深度扫描、automatic EStop、CM 生命周期管理、
可选 RViz。CM 是 `nav2_collision_monitor` 软件包中的安全节点；不启动 Nav2 navigation stack、
map_server、planner_server、controller_server 或 bt_navigator。

测试 CM 保留 Option 18 的 stop/slow polygon 和 observation sources，仅改输入话题、无地图
frame 和 `stop_pub_timeout=1200 s`（覆盖默认 900 s 会话），使持续零输出可以做 freshness 检查。
安装的 Humble CM 对 stale source 本身不保证 fail-closed；共享 Core/ROS adapter 负责扫描超时
立即发零，driver watchdog 继续作为后备。

## 逐步使用示例

| 阶段 | 操作及结果 | 运动 |
|---|---|---|
| 0 Preflight | 自动检查 driver serial heartbeat、IMU、两种 scan、周向 scan、TF、EStop request/state/reason、driver auto 参数、CM ACTIVE/输出及速度发布者。任一 critical FAIL 禁止运动 | 无 |
| ARM | 清理平地和测试侧约 1 m 空间，操作者准备手动 EStop，输入严格大写 `ARM`；其他输入结束 | 无 |
| 1 Detection | 纸箱放在前方约 1.0–1.1 m，Enter；显示 nearest range、支持点数、confirmation frames、左右/中心数据；最多三次 | 无 |
| 2 WAIT/Clear | 留住纸箱，Enter，观察 NORMAL→WAITING；测试 WAIT=2 s，生产=5 s；移走纸箱后 Enter，观察 CLEARING→NORMAL | 无 |
| 3 Side | 再放纸箱并留出明显一侧空间，Enter；显示左右 clearance 和 LEFT/RIGHT/UNKNOWN；只有人工确认 `y` 才允许继续，最多重摆一次 | 无 |
| 4 TURN | 阅读方向/角度，站在占用范围外，Enter；IMU 控制转向后 ZERO→SETTLE→OBSERVE | 第一次运动，仅转向 |
| 5 STRAIGHT | 当前朝向重新扫描；Guard 不允许则停止/重摆/跳过；另一次 Enter 只执行极短直行，之后重新停止观察 | 独立低速直行 |
| 6 Guard | 用纸箱建立较近障碍，Enter，最多 0.5 s、0.02 m/s；Guard 可直接拒绝起步；若 Auto EStop 先介入记 WARN | 极短或零 |
| 7 Auto EStop | 清理前方后 Enter，以剩余 session budget 为准极慢前进；从侧面移入纸箱，禁止从机器人上方/下方伸手；超时自动停止 | 最多 5 s，通常因累计预算更短 |
| 8 Recovery | 移走纸箱后 Enter；EStop 清除、fresh settle/clear 后只到 READY，旧动作丢弃，显示当前重新选边 | 无 |
| 可选 | 全部核心检查 PASS 后可回答 `y`，基于当前扫描额外执行一次单独确认的 TURN；默认 N | 仅一个 TURN |
| 9 Report | Enter 打印 preliminary assessment，保存 JSON | 无 |

终端示例（数值只是交互格式示例，不是实车测量）：

```text
[Stage 1/8] FRONT OBSTACLE DETECTION
Robot motion: NONE.
Place a cardboard box approximately 1.0–1.1 m in front of the robot.
Press Enter when ready.
  nearest_range_m: 1.06
  supporting_points: 18
  confirmation_frames: 2
Front obstacle detection: PASS

[Stage 2/8] WAIT / CLEAR RECOVERY
Robot motion: NONE. Testing NORMAL -> WAITING.
TEST wait = 2.0 sec; PRODUCTION default = 5.0 sec.
Press Enter to observe WAITING.
WAIT transition: PASS
Now remove the obstacle. Robot motion remains disabled.
Press Enter after the area is clear.
Obstacle clear recovery: PASS
```

如果正中纸箱让 25° 方向仍被机器人宽度覆盖，选边可能 UNKNOWN；重新摆放纸箱或结束，
不能降低 Guard/Auto EStop 距离。若空间不足，停止后人工重新摆机器人，禁止自动倒车。

Stage 7 的 Guard 始终有效，可能先于 Auto EStop 阻止运动。程序允许继续观察静止 EStop
与释放，但没有真实“运动中被 Auto EStop 打断”的证据时，该项和完整旧动作丢弃验证保持
INCOMPLETE。这不会被改写成 PASS。无需人员站在运动机器人正前方。

## 初始限制

| 参数 | 默认 |
|---|---|
| trigger / guard / Auto EStop | 1.10 / 0.95 / 0.80 m，Auto 阈值未改 |
| test WAIT / production WAIT | 2 / 5 s |
| TURN | 25°，最大 0.15 rad/s，5° tolerance，硬超时 5 s |
| TURN assessment | ≤5° PASS，5–10° WARN，>10°/超时/故障 FAIL；阈值配置化 |
| STRAIGHT | 0.03 m/s，1.5 s 命令时长，硬上限 2 s |
| guard probe | 0.02 m/s，0.5 s |
| EStop probe | 0.02 m/s，最多 5 s，同时受剩余累计预算约束 |
| stopped settling | 至少 0.5 s，随后要求新鲜扫描 |
| scan / depth scan / IMU timeout | 1.0 / 1.0 / 0.5 s |
| session forward command | ≤5 s；不等于实测行驶时间/距离 |
| accumulated rotation | ≤90°（IMU yaw 变化与请求预算检查） |
| session duration | 900 s，等待用户也计时 |
| software control deadline | 0.25 s；调度超限停车，driver watchdog 仍是后备 |

每个 primitive 只有纯转向或纯向前，绝不同时转/走，绝不倒车。
0.03×1.5=0.045 m 仅是理论命令积分；日志记录 commanded motion duration，不记录“确实走了 4.5 cm”。
用户可在独立 YAML 中降低运动参数；高于代码保守上限的参数会被拒绝。

## 报告与判定

每次 runner 启动在 `logs/detour_test_<UTC timestamp>.json` 建立唯一记录；ARM 前验证可写。
记录 commit SHA、实际脚本 SHA256、完整参数、preflight freshness、各阶段、障碍范围/点数/选边、IMU delta/error、
命令时长/速度、Guard/EStop/timeout/fault、累计运动与中断原因，不默认写 rosbag。
`t_auto_request`、`t_auto_state`、`t_aggregate_estop`、`t_output_zero` 是本进程单调时钟收到消息
的时间；不能推导物理制动时间，也不能把当前 EStop 与早先的零输出配对。

- **FAIL**：critical preflight 缺失、运动中 sensor/IMU/CM/driver 故障、超时或异常、EStop 后非零、旧动作无授权恢复、TURN 无法受限等。
- **INCOMPLETE**：用户退出、跳过、无可确认安全侧、未实际打断活动 primitive、剩余测试未运行。它优先于 WARN，但低于 FAIL。
- **PASS WITH WARNING**：必需测试都完成，存在 Guard/EStop 竞争、可接受转角偏差、重摆后选边等 WARN。
- **PASS**：全部必需检查都有通过证据。

SIGINT/SIGTERM/exception/EOF 时尽最大努力重复发零后退出；不自动清除 EStop。
前台结束后 shell 只清理本 session 拥有的后台进程。强制进程杀死/系统故障不能依赖 Python
清理保证停车，既有 driver watchdog 保持原状。

最终终端和 JSON 始终包含以下边界；只完成的 PASS 项有对应证据：

This test provides preliminary validation of:

- local obstacle detection
- wait/clear state handling
- simple detour side selection
- low-speed turn primitive
- low-speed short forward primitive
- detour guard behavior
- Auto EStop interruption
- post-EStop re-evaluation

This test DOES NOT validate:

- global Nav2 replanning
- map-based detour
- full obstacle bypass
- original navigation goal recovery
- airport-scale operation
- physical stopping distance

还未验证 global costmap persistence、完整 Nav2 detour、机场真实环境及任何实车 stopping distance。
