# Option 19 交付与验证记录

日期：2026-09-17。开发基线：`integration/nav-safety-reports-20260917`，
HEAD `eca2c1e5f5d4ed6b5bf241d473ea02a2c716d124`。

实现位于已有工作树 `/tmp/traymover-integrated-20260917-2b46r7et/workspace`，
在该目录当时未提交的 Option 18 之上完成。未切换或改写 IDE 原目录的 `260917` 分支。
本记录与 Option 18/19 实现一同提交归档。报告除 HEAD SHA 外还记录实际 detour/guided
Python 源码 SHA256，避免把开发基线 SHA 当作已包含后续实现的 commit。

## 本任务文件

| 文件（导航文件均在 `src/traymover_robot_nav/`） | 变化 |
|---|---|
| `scripts/traymover.sh`（workspace） | 增加菜单19，独立背景 sensor session、前台 runner、可选 RViz/手动键盘、退出清理 |
| `scripts/detour_core.py` | 从生产 gate 提取核心，以默认不变的 hook 支持 guided policy |
| `scripts/detour_gate.py` | 保留原生产导入路径，兼容转发共享 Core |
| `scripts/detour_motion.py` | GuidedConfig/Observation/Core；IMU/时间反馈、逐动作授权与 session limits |
| `scripts/detour_obstacle.py` | 提取生产 scan/quaternion/timestamp 转换函数 |
| `scripts/detour_supervisor.py` | 导入共享转换函数；生产逻辑及配置保持原行为 |
| `scripts/guided_detour_node.py` | 无地图 ROS 输入、freshness、TF、服务/速度发布者检查、软件事件时间与零输出 |
| `scripts/guided_detour_test.py` | ARM、逐阶段前台 UI、报告、异常和信号退出 |
| `launch/guided_detour_test.launch.py` | 独立无地图传感器及必需安全节点 |
| `config/guided_detour_test.yaml` | 更保守测试默认值 |
| `config/collision_monitor_guided_detour_test.yaml` | 原 polygon；测试输入、base_link fixed frame、持续零输出 |
| `rviz/guided_detour_test.rviz` | base_link 下 scan/深度scan/model/TF/EStopPanel |
| `test/test_detour_core.py` | 53 项 guided 核心测试 |
| `test/test_guided_detour_node.py` | 18 项 ROS adapter 测试 |
| `test/test_guided_detour_test.py` | 61 项报告和交互/中断测试 |
| `test/test_guided_detour_config.py` | 6 项配置、launch、shell 进程清理测试 |
| `test/test_guided_detour_velocity_chain.py` | 1 项真实 CM 无地图 DDS 集成测试 |
| `CMakeLists.txt` / `package.xml` | 注册测试，将已使用的 lifecycle_msgs 声明为 runtime dependency |
| `docs/guided_detour_test.md`（workspace） | 完整构建、启动、逐阶段使用、边界及参数说明 |
| `docs/guided_detour_verification.md`（workspace） | 本记录 |

已有 Option 18 的 safety 包 reason topic 改动仍保留；它们不是本次新增改动。
没有添加新的第三方安装依赖，也没有为测试修改 driver/EStop 阈值。

## 简化与复用

没有复制一个假绕行 FSM。生产与测试使用相同障碍几何、左右候选、WAIT、clear confirmation、
EStop hold/discard、Guard、转向控制、扫描 freshness、phase watchdog 和恢复状态机。
差异限制在真实反馈来源与动作授权：生产 pose displacement 对应 guided timed straight，
生产连续离散 phase 调度对应 guided 每次 Enter 的单 primitive。场景/安全判断不放宽。

原 gate 的 51 项回归与原 supervisor 的 15 项 ROS 回归在提取后通过。
实现前保存的 `/tmp/traymover-option19-baseline` 用于核对：Option10/15/16/17/18
启动函数、现有 EStop helper 逐字节不变；driver Python 与 Option18 launch/config 共16个
相关文件逐字节不变。

## 执行的验证

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash
colcon --log-base /tmp/traymover-guided-log build \
  --build-base /tmp/traymover-guided-build --install-base install --symlink-install \
  --packages-select traymover_robot_nav traymover_robot_safety turn_on_traymover_robot
source install/setup.bash
colcon --log-base /tmp/traymover-guided-log test \
  --build-base /tmp/traymover-guided-build --install-base install \
  --packages-select traymover_robot_nav traymover_robot_safety turn_on_traymover_robot
colcon test-result --test-result-base /tmp/traymover-guided-build --verbose
bash -n scripts/traymover.sh
git diff --check
```

结果：三个相关包构建成功；完整三包测试通过。最终补充短 Auto 脉冲来源回归后，
重新构建导航包并定向 `colcon test --ctest-args -R test_guided_detour_node` 通过。
实际测试用例 **316 项**：导航283、安全21、driver12；0失败/0跳过。
最终 `colcon test-result` 输出 **320 tests, 0 errors, 0 failures, 0 skipped**，
包含最后一次 CTest XML 中4个包装测试；不要把包装项当成额外独立用例。
本任务新增用例共139项。

新增/修改 Python 源码和测试 `flake8 --max-line-length=99` 通过；Python AST/编译检查通过；
Bash语法与 diff whitespace 检查通过。已有 driver 的 transforms3d/NumPy 弃用 warning
保持不变。构建复用主工作区已安装、未修改的依赖，未重建所有第三方包。

离线用例覆盖：preflight失败、front detection、WAIT/clear、左右选择、TURN成功/超时、IMU stale、
straight硬超时、Guard、自动/手动EStop、运动中sensor失败、信号/EOF/异常zero、EStop释放后旧动作
废弃、累计运动/会话时长、严格ARM、每阶段单独确认、历史或错误来源EStop不能误报PASS。

真实 CM 测试使用隔离DDS域和合成LaserScan/TF，无driver/相机/实体底盘：

- 只有 `base_link -> test sensor` TF，无 map/odom；直行和纯转向能通过速度链。
- LiDAR和RealSense分别触发原polygon停车。
- 零输出持续超过原0.5秒 stop_pub_timeout；生命周期inactive不输出速度。
- 已安装Humble CM会忽略stale source。其freshness fail-closed责任明确由guided adapter/Core承担，
  并由相应回归测试证明，不能谎称CM单独验证了这件事。

还在隔离 `ROS_DOMAIN_ID=230` 实际运行安装后的 foreground runner：不启动任何硬件，
缺少必需输入时输出PRECHECK FAILED、禁止ARM/运动、保存FAIL报告，累计命令时间/转角均为零。
结果：`/tmp/traymover-option19-installed-preflight/`。

详细验证日志：

- `/tmp/traymover-option19-build.txt`
- `/tmp/traymover-option19-colcon-test.txt`
- `/tmp/traymover-option19-final-boundary-test.txt`
- `/tmp/traymover-option19-installed-preflight.txt`

## 未验证

未启动实车运动；全部数值未经实车安全验证。未验证 Nav2 full detour、map-based replanning、
global costmap persistence、原始目标恢复、完整障碍绕过、airport real environment、physical
stopping distance。IMU yaw不是精确位置，timed forward不是实测距离，软件零输出时间不是物理制动时间。

Guard保持开启可能使Stage7无法得到活动Auto EStop中断证据；此时报告INCOMPLETE。
用户可正常结束后在更合适的场地重测，不应缩小0.80m Auto EStop阈值或屏蔽Guard来制造PASS。
