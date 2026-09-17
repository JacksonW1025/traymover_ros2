# Option 18 软件交付与验证记录

开发分支：`integration/nav-safety-reports-20260917`。
开始时已 fetch 并检查远端；最新 HEAD 与给定基线一致：
`eca2c1e5f5d4ed6b5bf241d473ea02a2c716d124`。

修改位于此分支已有工作区：
`/tmp/traymover-integrated-20260917-2b46r7et/workspace`。
未创建 feature branch，未 merge/cherry-pick，也未读取或复制旧 detour 分支代码。
IDE 原工作区 `/home/wheeltec/traymover_ros2` 仍保留原分支和未提交修改；其 tracked
working/index diff 已与操作前快照逐字节比较一致。本记录与 Option 18/19 实现一同提交归档。

## 实现

- Option 18 默认 Auto EStop ON、Detour ON；Auto OFF 时强制 Detour OFF。
- 纯状态机负责等待、左右候选选择、TURN/STRAIGHT 分离、逐阶段停止和扫描刷新。
- EStop、guard、传感器或定位故障中断旧动作；恢复后从当前 pose 重新规划。
- 原 NavigateToPose action 被保存；暂停期间 Nav2 abort 后仅在安全交回时重发。
  用户取消、抢占和未完成的后端取消均撤销运动授权。
- 速度链为 Nav2 → Supervisor → Collision Monitor → driver aggregate EStop。
- Auto EStop 仅增加 reason 发布；既有 Bool 输出语句和调用流程未改。
- 复用当前 Option 16 launch 和底层导航模块，使用独立配置；不引入自定义消息。

集成审查中修复了四类实际问题：270° 导航扫描不能验证后方旋转空间；相机前移
0.45 m 导致底盘距离与 Auto EStop 深度距离不一致；Auto OFF 时 driver 状态无周期
心跳；短 EStop 脉冲及交回期间的新障碍可能导致恢复竞态。另补充了精确时间 TF
异步到达、缓存扫描随机器人运动变换和单独 map→odom 失效的检查。

## 文件清单

| 新增文件 | 用途 |
| --- | --- |
| `src/traymover_robot_nav/scripts/detour_gate.py` | 纯状态机、观测几何、离散动作 |
| `src/traymover_robot_nav/scripts/detour_supervisor.py` | ROS/TF/时间戳/速度边界 |
| `src/traymover_robot_nav/scripts/detour_goal_proxy.py` | 原导航目标保存、取消、重发 |
| `src/traymover_robot_nav/launch/traymover_nav_lidar_realsense_detour.launch.py` | 独立模式、单次相机启动、外围扫描 |
| `src/traymover_robot_nav/config/nav2_params_lidar_realsense_detour.yaml` | 可清除的全局动态障碍层 |
| `src/traymover_robot_nav/config/collision_monitor_lidar_realsense_detour.yaml` | 原 polygon，输入改为 detour 速度 |
| `src/traymover_robot_nav/test/test_detour_gate.py` | 51 项状态机/几何回归 |
| `src/traymover_robot_nav/test/test_detour_supervisor.py` | 15 项 ROS 边界回归 |
| `src/traymover_robot_nav/test/test_detour_goal_proxy.py` | 15 项真实 ROS action 测试 |
| `src/traymover_robot_nav/test/test_detour_config.py` | 24 项配置/菜单/launch 测试 |
| `src/traymover_robot_nav/test/test_detour_velocity_chain.py` | 真实 Collision Monitor 速度链测试 |
| `src/traymover_robot_safety/test/test_auto_estop_reason.py` | 7 项真实 Auto EStop 节点测试 |
| `docs/wait_detour.md` | 参数、操作方式、八阶段实车验证顺序 |
| `docs/wait_detour_verification.md` | 本记录 |

修改文件：`scripts/traymover.sh`、导航 `CMakeLists.txt`/`package.xml`、安全包
`CMakeLists.txt` 和 `src/auto_estop_node.cpp`。后者只有 22 行新增，无原有行删除或修改。

## 验证结果

2026-09-17 完成：

- 三包联合 `colcon build` 成功：`traymover_robot_nav`、`traymover_robot_safety`、
  `turn_on_traymover_robot`。
- 三包 `colcon test` 成功，`colcon test-result --verbose`：
  **190 tests, 0 errors, 0 failures, 0 skipped**。此统计包含 CTest 包装项；
  实际用例为导航 144 项、安全 21 项、driver 12 项。
- 新增 Python 文件 flake8（99 字符行限）通过；Python 编译检查、
  `bash -n scripts/traymover.sh` 和 `git diff --check` 通过。
- 真实 Collision Monitor 测试验证正常透明透传、下游 polygon 独立停车、障碍清除恢复、
  manual EStop 持续输出 zero。该测试对 Supervisor 感知和目标采用明确的测试替身；
  action 代理、传感器边界、完整绕行阶段分别由独立测试覆盖。
- 所有 ROS 动态测试均为合成数据与隔离测试端点/域；未启动底盘或相机。

生成目录：`/tmp/traymover-detour-build`、`/tmp/traymover-detour-install`、
`/tmp/traymover-detour-log`。详细命令见 [操作文档](wait_detour.md#inspection-and-software-verification)。
构建复用了当前已安装的未修改依赖，没有重建整个 workspace 的所有第三方包。
已有 driver 测试产生 `transforms3d` 的 NumPy 弃用 warning；无测试失败。

## 回归与限制

已确认 Option 10/15/16/17 的菜单 action 函数及既有 EStop 启动 helper 与基线一致。
driver EStop gate/watchdog、depth evaluator、阈值、原 Nav2/Collision Monitor 配置、
原 Option 16 launch 均未修改。

所有速度/距离均为初始调参值，**未做实车验证**。高位 LiDAR 与窄视场深度相机仍有低矮
障碍盲区；有限扫描覆盖不足时保持 zero/FAULT_HOLD/BLOCKED。候选动作以扫描几何验证，
未直接采样 global/local costmap 栅格；Nav2 仍消费动态 costmap，Collision Monitor 仍在下游。
本版本保存的是 NavigateToPose，未代理多目标 action，也不跨进程重启恢复目标。

必须按 [八阶段顺序](wait_detour.md#required-staged-hardware-validation) 验证：
架空/底盘禁用 → detour OFF 地面导航 → 只验证 WAITING → TURN-only →
0.03–0.05 m/s 短直行 → 0.08 m/s 完整绕行 → 中途 EStop 恢复 → 动态人员进入。
最后一阶段必须有人员可立即使用 physical/manual EStop。
