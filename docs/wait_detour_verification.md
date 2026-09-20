# Option 18 软件交付与验证记录

> 下文为 2026-09-17 的历史交付记录。当前选项 18 已改为直接复用选项 10
> 的菜单流程、launch 和基础参数，并补齐多点导航及 RViz `/goal_pose` 接管。
> 当前行为与配置格式见 [操作文档](wait_detour.md)。历史测试数量不代表当前版本。

## 2026-09-21：相机独立判障与直行小偏差锁停修复

最新 4TJQzz 日志两次直行退出均为 `straight phase unexpected pose displacement`，
当时 Auto EStop 为 FREE。

- `detour_core.py`：简单模式转向只看相机近障碍，不再把 LiDAR/perimeter 点
  纳入旋转否决。10 cm 横向或 5 cm 反向小偏差不再永久锁停简单直行，继续
  有界航向纠偏及实测 2.2 米距离控制；严格模式维持原检查。
- 手动/自动急停、相机断流、无有效位姿、严重航向偏差及动作超时保护保留。
- 更新 `test_simple_detour.py` 的相机独立判障与位姿小偏差回归。

按用户要求不运行全量测试，仅做针对性用例、语法/lint、构建与安装文件核对。
未启动底盘，实车效果待用户检查。

## 2026-09-21：固定右转 45°、前进 2.2 米、重发终点

用户明确改为固定流程，取消左右选择、净空裁短和四次转向上限。
最新 xn8YTW 记录表明，相机未判障碍时旧净空值 0.684 米仍立即阻断直行。

- `detour_core.py`：默认简单模式只选 RIGHT；实际转到 45° 后按位姿测量前进
  完整 2.2 米。相机确认新障碍时停止，从当前位姿重新执行完整一轮；迭代状态机
  不使用调用栈递归。单个缓存相机帧不能触发重复循环。
- 不再按 LiDAR 通道净空裁短或禁止直行；已观测到的 0.50 米车体旋转范围内
  障碍仍禁止转向。相机失效、位姿失效和真实急停保留停车权。
- 单纯传感器/急停中断保存未完成的转向角度或直行剩余量；新确认障碍才重置
  下一轮完整距离。前进完成后前方清晰则重发原目标，从新位置规划并继续导航。
- launch 默认 `detour_first_leg_m=2.2`、`straight_phase_timeout_sec=60.0`；
  当前 0.08 m/s 下名义直行需 27.5 秒。移除不再适用的转向次数预算参数。
- 更新 supervisor 的相机必需/LiDAR 可选策略、测试、操作文档；真实 Nav2 测试
  验证右转后位移约 (1.56,-1.56) 米时，仍以原终点从新位置重新规划。
- 修复等待定位时关闭节点的 TF 回调生命周期竞争：buffer/listener 保持到两个
  执行器结束后再释放，避免后台等待退出时销毁主执行器仍可能访问的局部 buffer。

固定模式、真实 Nav2 路径及相机/急停保护均用隔离域合成数据测试，未启动底盘。
构建及全包回归完成；修复关闭节点并发问题后，连续 3 次退出测试及全部受影响的
真实 Nav2/生命周期场景复测通过。当前注册 **34 组 / 507 个用例：0 失败、
0 错误、0 跳过**。已核对安装脚本/launch 与源文件一致；Python 语法/flake8、
Bash 语法及 diff 检查通过。

## 2026-09-21：中断直行的进度与时限修复

最新 `20260920T161327Z_option18.BkEv4y` 记录了 36 次进入直行、35 次保护中断，
没有一次前进完成或 RECOVER_TO_NAV。原实现每次恢复直行重置起点/距离/时钟。

- `detour_core.py`：简单模式增加独立的前进目标、已验证净前进量和首段起始时间。
  停止或换朝向后保留累计进度，新段从恢复位姿计量且只走剩余距离。初次净空
  裁剪后的目标不再延长；恢复段被再次裁短也不会提前宣告整个前进完成。
- 暂停期间推车/定位变化、转向漂移不记作前进；同一段往返抖动按净位移计算。
  保护与距离完成同时出现时先保存有效进度，但保持零速度，等安全后才交回。
- 总直行时限从首次进入直行算起，中断不重置；新目标或完成交回才清空记录。
  日志增加累计/目标/剩余距离及保护时净空、相机障碍判定。
- 新增 `test_detour_forward_progress.py` 的 15 项场景并注册 CMake；
  `test_simple_detour.py` 的位移断言使用与控制器一致的微米级浮点容差。

修改前先复现 8 项失败；修复后新增 15 项全部通过，覆盖 0.4+0.4 米、
连续保护、传感器故障/急停、暂停推移、最后 0.05 米、跨朝向、抖动及总超时。
全包验证：当前注册 **34 组 / 515 个用例，0 失败、0 错误、0 跳过**。
已重新构建并逐字节核对安装目录中的 core/supervisor/goal proxy 与源文件一致。
修改文件 flake8、Bash 语法及 diff 检查通过。未启动底盘或实车运动。

## 2026-09-21：逐次转向、实测前进、再重规划

依据 `20260920T154156Z_option18.bWfAcm` 中重复 TF 健康中断、转向后直行保护
反复退回规划的记录，并按用户补充“转向后还需要前进，之后再重规划”调整。

- 简单模式改为同方向固定角度转向（默认每次 45°），每次停稳后检查新相机帧；
  仍受阻则再转一步；确认中央视野和通行距离后低速前进，最多 0.8 米并按净空裁剪。
  按实际位姿位移完成前进，移除简单模式的自动转回/第二段直行。
- 每轮默认最多 4 个转向步骤，故障和急停不会清零预算；无可行方向时停止，
  需要新目标或重置，避免无限原地旋转。真实急停、扫描丢失及近距离保护保留。
- 完成实测前进后强制重规划：保留公共任务及剩余途经点，取消旧 Nav2 后端并等待
  终止结果，再重发原目标。重发期间不授权运动；新目标接受后仍等待新的速度指令。
  map 目标重发使用最新变换时间语义，不改变目标坐标。
- 修复低频 TF 的跳变误判：按相邻 TF 样本时间间隔比较位移/角度；重复读取同一
  时间戳不刷新比较基线。真实跳变、倒退时间戳、过期 TF 仍停车。
- 修改文件：`detour_core.py`、`detour_goal_proxy.py`、`detour_supervisor.py`、
  选项 18 launch 的转向预算参数，以及对应核心/动作/节点/真实 Nav2 回归测试。

独立实节点测试已确认：原公共目标不结束，后端重启后从改变后的 x/y 位置生成新路径；
核心测试积分验证实际转向和正向位移发生在重规划请求之前。
构建完成；全包回归及速度链测试替身接口更新后的复测通过。当前注册
**33 组 / 500 个用例：0 失败、0 错误、0 跳过**。修改的 Python 文件
语法/flake8、Bash 语法及 diff 检查通过。所有测试均未启动底盘。

## 2026-09-20：相机确认触发与正常导航防抖

依据最新 `20260920T152432Z_option18.YRWqSM`：除启动外的 6 次急停包含
4 次深度超时、1 次有效率过低、1 次近障碍；另有 15 次交回导航时因 LiDAR
通行条件不足重新进入 PLAN_DETOUR。这解释了无明确障碍时的反复停车/转向。

- 选项 18 简单模式：相机前方 1.00 米、左右各 0.35 米走廊，至少 8 个相邻
  支持点、连续 3 个新相机帧，才允许启动或重新进入绕行。故障/急停后清空旧
  确认计数，必须重新确认。LiDAR 只用于自主绕行动作的通行判断和保护。
- NORMAL 和被动恢复不再依赖 LiDAR 新鲜度；自主转向/直行仍要求新鲜 LiDAR
  和有效里程计。相机断流/无有效扫描、真实急停始终停车。
- 深度图与相机扫描超时 1.5 秒，自主绕行 LiDAR 超时 2.0 秒，TF 超时 1.5 秒。
  简单模式中尚未达到 Bool 停车确认的 LOW_VALID_DEPTH 原因不再提前绕过防抖。
- 仅选项 18 的 Auto EStop：近点面积至少 2%、至少 4 个近点网格、3 帧确认；
  有效比例门限 25%、严重不足 10%、低有效率 4 帧确认、恢复 12 帧。
  紧急停车距离 0.80 米和释放距离 1.00 米保留。其他菜单安全参数不变。
- 最终 Collision Monitor 保留原相机近距离 PolygonStop；移除独立 LiDAR
  制动和大范围 PolygonSlow，避免无相机障碍时单独减速。原手动急停保留。
- 修改文件：`detour_core.py`、`detour_supervisor.py`、选项 18 launch/碰撞配置、
  菜单默认值、CMake 测试注册和对应回归；新增 `test_option18_depth_policy.py`。

独立实节点测试验证：小面积/细线噪声不触发、持续障碍第 3 帧停车、12 帧清晰释放；
约 1 秒数据间断可容忍，超过 1.5 秒断流停车；低有效率确认和严重无效保护仍有效。
实际碰撞链验证 LiDAR 噪声/断流不影响正常导航、近相机障碍和相机断流仍停车。
构建和全包测试完成：当前注册 **33 组 / 473 个用例，0 失败、0 错误、0 跳过**。
真实 Auto EStop 节点的 3 项策略测试、实际 Collision Monitor 速度链、真实 Nav2
延迟定位/目标/路径/速度/取消测试均通过。修改 Python 文件语法与 flake8、
Bash 语法和 diff 检查通过。启动日志会记录实际触发距离、走廊宽度、帧数与超时。
本轮没有启动底盘或实车运动，移动时感知覆盖和停止距离仍需现场验证。

## 2026-09-20：等待人工初始定位与碰撞监控 TF 修复

针对 `20260920T150054Z_option18.kASuc8`：规划器在等待人工初始位姿的 TF，
旧管理器却消耗完 3 次服务重试并停用地图，最终导航器未激活、目标被拒收。

- `src/option18_lifecycle_manager.cpp`：地图服务器先激活；规划器激活前等待
  `map -> base_link`，该等待不消耗服务重试预算。定位完成自动继续导航激活；
  定位等待期间支持退出。TF 使用主执行器接收，启动等待在既有服务线程执行。
  真实服务故障仍有限重试，运行中的原生 bond 监控保留。
- `config/collision_monitor_lidar_realsense_detour.yaml`：仅选项 18 的固定参考帧
  改为 `base_link`，避免当前时间的 odom 外推错误使传感器输入被忽略。
  原停车区域、传感器来源和超时配置保留。采用局部快照，不补偿采集后车体运动；
  必需扫描超时仍由上游 supervisor 停车。
- `test_option18_navigation_smoke.py`：真实 Nav2 延迟 45 秒提供定位 TF，验证
  地图保持激活，随后目标被接受、生成实际路径/速度并可取消；另测等待中退出。
- `test_detour_velocity_chain.py`：真实 Collision Monitor 在 odom 延迟 0.4 秒时
  正常透传，LiDAR/RealSense 各自触发停车，扫描超时和手动急停使整链输出零。
- `CMakeLists.txt` / `package.xml`：已存在的 tf2_ros 依赖增加 C++ 构建链接；
  `test_detour_config.py` 和操作文档同步更新。选项 10 运行逻辑未改动。

旧版本已分别复现延迟定位启动失败和碰撞监控 TF 外推失败。
完成全包回归，并修正等待期间关闭节点的线程退出异常后复测所有受影响的管理器场景。
当前注册 **32 组 / 434 个用例：0 失败、0 错误、0 跳过**。
真实 Nav2 的立即定位、延迟 45 秒定位、等待中退出三场景通过（53.085 秒）；
生命周期异常恢复三场景通过；正常/延迟 odom 的实际碰撞链两场景通过。
构建、修改 Python 文件 flake8、Bash 语法及 diff 检查通过。
本轮验证使用独立 ROS 域和合成数据，没有启动底盘或实车运动。

## 2026-09-20：实测故障修复与简化绕行（当前版本）

依据 `logs/nav/20260920T143415Z_option18.GthwDV` 的真实运行记录修正。

- 目标代理：将两种 Nav2 action 的五个服务/话题端点逐一重映射，修复
  公共 action 冲突和私有后端不可用；RViz 目标记录接受与最终执行结果。
- 导航基础：默认关闭额外动态 costmap 层，直接使用选项 10 的完整参数文件。
  地图、定位、规划器、控制器、行为树和菜单公共流程保持一致。
- 简化绕行：`simple_detour=true`，默认 1.10 米触发、等待 1.0 秒，选左右
  45° 方向，短直行、转回、再直行，交回原导航目标。保留 0.95 米直行保护、
  急停、传感器新鲜度、绕行时当前里程计、位移/时间限制和取消授权。
  不再要求全周每个角度都有有限回波；转向仍检查已观察到的近距离障碍。
- 感知：正常导航不再等待扫描时刻的 odom TF；接受 base_link 局部扫描，
  有历史 odom 时补偿，否则保留原采集时间进行超时检查。深度扫描扩为 320 行。
  可选周边扫描异常只记录并丢弃，不向简单模式注入无谓的零速度脉冲。
- 启动：新增 `src/option18_lifecycle_manager.cpp`，复用已安装 Nav2 管理器，
  对初次启动做有界状态核对/重试，使用 bond formed 回调规避状态轮询竞争；
  保留运行期间原生 bond 故障监控。仅选项 18 的导航和碰撞监控采用此管理器。
- 修改核心文件：选项 18/公共导航/碰撞监控 launch，`detour_core.py`、
  `detour_supervisor.py`、`detour_goal_proxy.py`、菜单、CMake/package 声明与回归测试。

验证结果：

- `colcon build --packages-select traymover_robot_nav --symlink-install` 通过。
- `colcon test --packages-select traymover_robot_nav --ctest-args --output-on-failure` 通过。
- 当前注册 **32 组 / 431 个 pytest 用例，0 失败、0 错误、0 跳过**。
  统计排除 build 目录中遗留的旧测试 XML 和 CTest 包装计数。
- 新增真实 Nav2 隔离测试：正确私有 action 端点、实际节点激活、公共目标接受、
  实际路径与正向导航速度输出、取消；全程使用合成地图/TF/里程计，无底盘。
- 新增生命周期测试：慢配置、成功但响应丢失、已有部分节点配置、失败重试有界、
  bond 丢失会停用其余节点。左右完整绕行均用实际输出积分验证位姿和交回。
- 修改的 Python 文件语法与 flake8（99 字符）通过；Bash 语法及 diff 检查通过。

未进行实车运动验证。简化模式不保证未观测区域不存在障碍；无采集时刻 odom 的
扫描不做运动补偿，但仍保留采集年龄限制。此模式按用户指定用于封闭测试环境。
新管理器修复初次启动；外部手动 STARTUP/RESUME 及自动重连仍沿用上游实现。

## 2026-09-20：首次选项 10 基础共用改造（历史）

当前工作区：`/home/wheeltec/traymover_ros2`。

- `scripts/traymover.sh`：18 调用 `action_start_nav 18`，共用地图、定位、日志和启动流程。
- `launch/traymover_nav.launch.py`：增加默认关闭的扫描开关，保持选项 10 默认行为。
- `launch/traymover_nav_lidar_realsense_detour.launch.py`：直接包含选项 10 launch，
  叠加感知、任务代理、Collision Monitor；重映射仅在 Nav2 启动范围内生效。
- `config/nav2_params_lidar_realsense_detour.yaml`：删除复制的导航设置，仅保留障碍层；
  启动时叠加至 `nav2_params.yaml`，后续基础参数修改自动生效。
- `scripts/detour_goal_proxy.py`：统一接管单目标、多点目标和 RViz `/goal_pose`，
  绕行后保留未完成路径，跨任务类型取消/替换共用同一运动授权。
- 新增菜单行为与真实 launch 展开测试，补充多点/话题目标测试，
  更新配置和走廊规划测试以读取运行时合成参数；CMake 注册新测试。

验证：

- `colcon build --packages-select traymover_robot_nav --symlink-install` 通过。
- 完整 `colcon test` 后，修正两组旧测试对独立配置格式的假设，定向复测
  `test_nav_clearance_*` 和新 `test_option18_launch_graph` 通过。
- 当前 CTest 注册的 **28 组 / 390 个 pytest 用例：0 失败、0 错误、0 跳过**。
  此数字按当前注册名单汇总 XML，排除 build 中遗留的旧测试结果。
- 所有修改的 Python 文件 flake8（99 字符行限）、语法检查，
  `bash -n scripts/traymover.sh`、`git diff --check` 通过。
- 已安装选项 18 launch 的 `--show-args` 解析通过；launch 展开测试禁止进程启动，
  动态测试使用合成 ROS 数据，未启动底盘或相机，未做实车绕行验证。

## 2026-09-17 历史记录

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
