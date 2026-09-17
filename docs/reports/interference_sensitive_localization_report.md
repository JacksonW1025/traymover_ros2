# Traymover 干扰因素敏感的定位算法技术报告

**（工程核对版 · 基于 Jetson 实机源码；实车 A/B/C/D/E 量化待安全窗口补测）**

---

## 摘要

针对 Traymover 机场行李搬运机器人在室内高动态场景下的全局定位需求，本文系统梳理 **FAST-LIO + NDT_OMP** 双阶段定位架构对多类干扰因素的敏感机理，并给出已在工程中落地的鲁棒化对策。干扰因素涵盖：激光雷达原始测量噪声、地图鬼影与结构污染、几何退化场景、动态遮挡与后方设备反射、运动工况突变（U 型转弯）、以及边缘平台算力与时序约束。

本文不研发新型定位算法，核心贡献为：**（1）** 建立「干扰类型 → 失效模式 → 工程参数/链路对策」映射表；**（2）** 说明定位链路与姊妹课题「点云干扰噪声分层滤除」的上游—下游协同关系；**（3）** 在 Jetson 实机工作区核对 `lidar_localization_ros2`、FAST-LIO、`ndt_omp_ros2` 与回归测试，明确 NDT 对齐结果被收敛、fitness、pose jump、map→odom jump 与平滑门限共同约束。

当前正文主体依据 `traymover_ros2` 实机工作区中的 launch、YAML、Python 预处理、`lidar_localization_ros2` C++ 实现与单元测试断言撰写。2026-06-10 核对结果显示：源码级行为和 `test_initialpose_is_only_ndt_guess_until_valid_alignment` 已通过验证；但本机未保留按 A/B/C/D/E 标记的当前实现实车 NDT aligned/fitness 日志，因此第 6 章只记录可验证结论和未采集原因，不伪造 NDT fitness/漂移指标。

**关键词**：NDT 定位；FAST-LIO；干扰敏感性；map→odom；机场室内导航；ROS 2 Humble

---

## 1 项目背景与定位需求

### 1.1 作业场景与硬件软件栈

| 类别 | 配置 |
|------|------|
| 场景 | 机场候机大厅、行李转运通道；人员流动频繁、遮挡与反光并存 |
| 激光雷达 | 雷神 Leishen C32 全景 3D 雷达，话题 `/point_cloud_raw` |
| IMU | HiPNUC N300Pro，`/imu/data_raw` |
| 计算平台 | NVIDIA Jetson Orin NX |
| 软件栈 | ROS 2 Humble；FAST-LIO 里程计；NDT_OMP 全局校正；Nav2 二维导航 |

### 1.2 定位工程目标

1. 在预建 FAST-LIO PCD 地图上提供稳定的 `map → odom` 全局校正；
2. 与 FAST-LIO 高频里程计协同，避免开阔场地纯 NDT 位姿冻结或 yaw 振荡；
3. 对常见干扰具备可解释、可调试、可参数化的工程鲁棒性；
4. 不修改 FAST-LIO/NDT 核心源码，通过配置、launch 与旁路预处理实现适配。

### 1.3 与姊妹课题分工

| 课题 | 作用 | 与定位关系 |
|------|------|-----------|
| 点云干扰噪声分层滤除 | 建图/离线净化 PCD | **上游**：提升 NDT 地图与实时点云质量 |
| 多模态动态目标感知（设计阶段） | 在线语义避障 | **并行**：不反馈 FAST-LIO/NDT |
| **本报告** | 定位对干扰的敏感性与对策 | **下游消费**：地图 + 实时点云 + 运动先验 |

---

## 2 定位系统架构

### 2.1 双阶段定位 rationale

纯 NDT 对预建 PCD 配准在开阔机场空间易失效：NDT 收敛盆地约 `ndt_resolution/2`（本系统 0.5 m 体素 → 约 0.25 m），无运动先验时 `init_guess = previous_pose`，机器人运动超过该尺度即出现**位姿冻结**或 **yaw 振荡**。

**对策**：FAST-LIO 以 IMU+LiDAR 紧耦合提供约 20 Hz 的 `odom → body`；NDT 仅以较低频率校正 FAST-LIO 相对预建地图的慢漂移。

### 2.2 运行时 TF 与话题链

```
map ──(NDT, ~0.5 Hz 校正)──> odom ──(static)──> camera_init ──(FAST-LIO ~20Hz)──> body ──(static +0.12m z)──> base_link
```

| 话题/TF | 发布者 | 说明 |
|---------|--------|------|
| `/point_cloud_raw` | C32 驱动 | 原始点云 |
| `/point_cloud_localization` | `pointcloud_nav_preprocessor` | 剔除后方禁区后的定位输入 |
| `/odom` | FAST-LIO（remap） | 唯一里程计发布者 |
| `/pcl_pose` | `lidar_localization_ros2` | 仅 NDT 重对齐时发布 |
| `map → odom` | `lidar_localization_ros2` | 须 NDT 验证通过后才发布 |

启动入口：`scripts/traymover.sh` 选项 `10`/`15` → `traymover_nav.launch.py` → `lidar_localization.launch.py`。

### 2.3 NDT 节点行为约束（单元测试 + 源码摘录）

实机核对对象：

| 文件 | 作用 |
|------|------|
| `src/traymover_robot_slam/lidar_localization_ros2/src/lidar_localization_component.cpp` | NDT_OMP 节点参数、状态机、对齐、门限、TF 发布 |
| `src/traymover_robot_slam/lidar_localization_ros2/include/lidar_localization/lidar_localization_component.hpp` | NDT 状态变量、jump gate、平滑与 TF rebroadcast 成员 |
| `src/traymover_robot_nav/test/test_nav_simple_config.py` | 配置与 NDT 行为回归断言 |

源码行为核对如下，完整摘录见 `docs/reports/localization_codex_supplement.md`：

| 行为 | 源码位置 | 干扰敏感性说明 |
|------|----------|----------------|
| 参数入口 | `lidar_localization_component.cpp:98-137`、`:310-389` | score、range、far point boost、jump gate、平滑和 map→odom 二级门限都在参数层暴露，便于按地图鬼影、几何退化、U 型转弯和 TF stale 分别调参。 |
| NDT_OMP 初始化 | `lidar_localization_component.cpp:441-486` | NDT 分辨率、步长、迭代次数、线程数和体素大小决定收敛盆地、算力占用与几何约束保留程度。 |
| `/initialpose` 仅作 guess | `lidar_localization_component.cpp:489-518` | 初值到达后清空 `have_good_pose_`、置 `ndt_bootstrap_pending_`，避免人工初值误差直接写入 `map→odom`。 |
| odom 先验 | `lidar_localization_component.cpp:549-600` | `use_odom:true` 使用 FAST-LIO `/odom` 位姿差推进 init guess，降低开阔区域纯 NDT 冻结风险。 |
| 对齐节流与等待 | `lidar_localization_component.cpp:647-699` | `ndt_align_interval_s_` 控制重配准频率；失败时保持 last good TF 或继续等待首个有效 NDT。 |
| 源点云预处理 | `lidar_localization_component.cpp:729-752` | 体素滤波、距离裁剪和远点复制直接影响长墙 yaw 约束和远距噪声权重。 |
| 收敛/fitness gate | `lidar_localization_component.cpp:774-797` | 未收敛或 fitness 超过 `score_threshold_` 即拒绝，防止地图鬼影/动态遮挡造成的错误匹配进入 TF。 |
| pose jump gate | `lidar_localization_component.cpp:827-843` | 已有好姿态后，NDT 结果相对 init guess 的平移/yaw 超限即拒绝，针对重复墙面和急转后的误锁。 |
| map→odom jump gate | `lidar_localization_component.cpp:845-885` | 运行时可用 `max_map_odom_update_translation/rotation` 限制单次全局 TF 校正，保护 Nav2/局部规划器。 |
| 平滑和发布 | `lidar_localization_component.cpp:888-946` | 通过插值/SLERP 平滑 accepted map→odom；只有 accepted 结果才发布 `/pcl_pose` 和 `ndt_aligned_scan`。 |

单元测试 `test_initialpose_is_only_ndt_guess_until_valid_alignment` 对上述关键行为建立了文本级回归约束：`ndt_bootstrap_pending_` 必须存在；源码不得包含旧的 `Seeded map`/`from initialpose` 播种路径；`NDT map→odom jump` 必须发生在 `ndt_aligned_scan` 发布之前。2026-06-10 执行 `colcon test --packages-select traymover_robot_nav --pytest-args -k test_initialpose_is_only_ndt_guess_until_valid_alignment --event-handlers console_direct+`，该包 4 个 pytest 入口全部通过；`colcon test-result --verbose` 汇总为 42 tests、0 errors、0 failures、0 skipped。

---

## 3 干扰因素分类与敏感性分析

### 3.1 干扰—失效—对策总表

| 干扰类型 | 典型来源 | 定位失效模式 | 敏感环节 | 工程对策（已实现） |
|----------|----------|--------------|----------|-------------------|
| **I1 传感器噪声** | 超量程、近距离无效反射 | 配准 score 升高、偶发拒配 | NDT 源点云 | C32 距离有效性过滤；`scan_min/max_range` |
| **I2 地图鬼影** | 行人/操作员写入 PCD | 虚假结构吸引配准，yaw/横向偏置 | NDT 目标地图 | 姊妹滤除链路 `pcd_clean.py` 等；导航前选净化 PCD |
| **I3 几何退化** | 长廊、对称矩形、特征匮乏 | 多解/摇摆，yaw 不稳定 | NDT 几何约束 | 保持 **360°** 扫描；`far_point_boost` 强化远墙 yaw 约束 |
| **I4 后方动态遮挡** | 尾随人员、车体后方设备 | 后方点云污染配准 | NDT 源点云 | `pointcloud_nav_preprocessor` 后方矩形禁区剔除 |
| **I5 开阔场地运动** | 大位移无结构锚点 | 位姿冻结、TF stale | init_guess | `use_odom: true`，FAST-LIO 积分预测位姿 |
| **I6 运动突变** | U 型转弯、大角速度 | 合法 NDT 校正被误拒 | pose jump gate | `max_pose_jump_rotation: 0.45` rad |
| **I7 6DOF 欠约束** | 窄室内 z/roll/pitch 不可观 | 模型倾斜、fitness 暴涨 | NDT 输出 | 曾试 `lock_planar`，因首帧 z 错误已 **关闭** |
| **I8 算力/时序** | Jetson CPU 竞争 | TF 过期、Nav2 abort | 对齐频率 | `ndt_align_interval_s: 2.0`；TF 每帧重广播 |
| **I9 控制器冲击** | map→odom 阶跃 | 路径跟踪抖动 | 校正应用 | `map_odom_smoothing: 0.30` |
| **I10 评分门限** | 室内可用匹配 0.7–0.9 | 过度拒配 → stale TF | score 判定 | `score_threshold: 1.0`（由更严门限放宽） |

### 3.2 分干扰详细机理

#### I1 传感器层干扰

C32 驱动配置（`traymover_param.yaml`）保留完整 360° 输出，禁扫角设为 0。NDT 侧 `scan_min_range: 0.5`、`scan_max_range: 120.0`：既保留远墙结构，又避免单次对齐算力过大饿死 Nav2/TF 回调。

#### I2 地图质量干扰

若 PCD 残留行人鬼影或路径 speckle：

- NDT 将虚假静态结构当作配准锚点 → 横向/yaw 系统性偏差；
- `pcd2pgm.py` 投影后 Nav2 全局路径绕虚假障碍 → 间接表现为「定位/nav 不顺」。

**上游治理**：建图在线滤除 `fastlio_online_map_filter.py`、离线 `pcd_clean.py`、多会话 `pcd_intersect.py`（详见姊妹滤除报告）。

#### I3 几何退化

注释明确指出：若驱动层屏蔽后半圈扫描，走廊/矩形房间等场景 **NDT 找不到唯一最优** → 车体左右摇摆。因此：

- **定位**：必须 360°；
- **避障**：`pointcloud_to_laserscan` 将 `/scan` 限为前方约 180°，costmap 不看后方。

#### I4 后方动态干扰

`pointcloud_nav_preprocessor.py` 在车体坐标系划定后方禁区：

- x ∈ [-1.25, -0.35] m，|y| ≤ 0.90 m，z ∈ [-0.50, 2.00] m
- 定位支路 `/point_cloud_localization` 剔除该区域内点；导航支路 `/point_cloud_nav` 保留全云（供其他模块使用）。

#### I5–I6 运动与先验

`use_odom: true`：`lidar_localization` 在 NDT 周期间积分 FAST_LIO twist，使 `init_guess` 跟随机器人运动——修复开阔场地「车动位姿不动」。

`max_pose_jump_translation: 0.35` m、`max_pose_jump_rotation: 0.45` rad：旧门限 0.25 rad 在 U 型转弯时拒绝合法校正，导致 FAST_LIO 漂移累积。

#### I7 平面锁失败教训

`lock_planar: true` 曾将首帧错误 z=0.274 m 锁死（URDF 为 0.12 m），后续 fitness 达 15–20。**结论**：地面机器人窄室内仍保持 6DOF 自由，由 jump gate 防大漂移。

#### I8–I10 时序与门限

- 1 Hz NDT 曾导致 TF rebroadcast 不足；现 `ndt_align_interval_s: 2.0` s，且每帧重广播末次良好位姿；
- `score_threshold: 1.0`：接受室内 0.7–0.9 的可用匹配，拒绝数米级失败（score > 1.0）；
- `map_odom_smoothing: 0.30`：Accepted 校正渐变施加，减轻控制器冲击。

### 3.3 远距点加权策略

`far_point_boost_min_range: 15.0`、`far_point_boost_factor: 2`：体素滤波后对远距点保守复制，增强长直墙对 **yaw** 与横向约束，同时避免稀疏远距噪声主导（注释强调 intentionally conservative）。

---

## 4 关键参数与配置文件索引

### 4.1 `localization.yaml` 核心参数

| 参数 | 当前值 | 干扰关联 |
|------|--------|----------|
| `registration_method` | NDT_OMP | — |
| `score_threshold` | 1.0 | I10 |
| `ndt_resolution` | 0.5 | 收敛盆地尺度 |
| `voxel_leaf_size` | 0.15 | 算力 vs 结构保留 |
| `use_odom` | true | I5 |
| `use_imu` | false | 避免与 FAST-LIO 去畸变重复 |
| `enable_map_odom_tf` | true | 发布 map→odom 而非 map→base_link |
| `lock_planar` | false | I7 |
| `max_pose_jump_translation` | 0.35 m | I6 |
| `max_pose_jump_rotation` | 0.45 rad | I6 |
| `ndt_align_interval_s` | 2.0 s | I8 |
| `map_odom_smoothing` | 0.30 | I9 |

### 4.2 Launch 层可覆盖参数

`lidar_localization.launch.py` 支持运行时：

- `pcd_path`：切换先验地图；
- `cloud_topic`：默认 `/point_cloud_raw`（若走预处理需改为 `/point_cloud_localization`）；
- `max_map_odom_update_translation/rotation`：选项 14 三维导航可设更严门限（0.30 m / 0.12 rad）。

### 4.3 选项 10 vs 14 定位门限差异

| 模式 | map→odom 单次更新上限 | 说明 |
|------|----------------------|------|
| 选项 10（2D Nav2） | 默认 -1（禁用 launch 层额外门限） | 依赖 YAML 内 jump gate |
| 选项 14（3D 实验） | 0.30 m / 0.12 rad | CMU 规划器对 TF 更敏感 |

---

## 5 定位健康检查与调试流程

### 5.1 标准操作流程（选项 10）

1. `bash scripts/traymover.sh` → 选择 `10`，选定净化后 PCD；
2. 等待 FAST-LIO IMU 初始化 + NDT lifecycle 激活（约 8–12 s）；
3. RViz `2D Pose Estimate` 设置初值；
4. 观察 RobotModel 与点云地图对齐后发送 `2D Goal Pose`。

### 5.2 `check_localization.sh` 预期指标

| 检查项 | 预期 |
|--------|------|
| `/odom` 发布者数量 | 恰好 1（FAST_LIO） |
| `/odom` 频率 | ~20 Hz |
| `/pcl_pose` 频率 | ~1 / `ndt_align_interval_s` |
| 静止 15 s 后 map→odom 平移漂移 | < 0.05 m/min |
| 日志 | 无持续 `didn't converge`、异常 `Fitness` 暴涨 |

### 5.3 RViz 诊断话题

- `NDTAlignedScan`：配准后扫描与地图叠合目视检查；
- 关注日志关键词：`NDT`、`Fitness`、`Planar lock`、`jump`、`didn't converge`。

---

## 6 实验验证

### 6.1 已有验证（本地/仓库可复现）

| 验证类型 | 状态 | 说明 |
|----------|------|------|
| 配置回归测试 | ✅ | `test_nav_simple_config.py` 约束 YAML/launch 参数合理性 |
| 定位健康脚本 | ✅ | `check_localization.sh` 现场快速体检 |
| 上游滤除合成实验 | ✅ | 姊妹报告：改善 PCD 质量（间接利于 NDT） |

### 6.2 实机源码与测试核对（2026-06-10）

#### 6.2.1 NDT 源码级行为核对

- [x] 摘录 `lidar_localization_component.cpp` 中 score/fitness 判定逻辑，见 `localization_codex_supplement.md` §2.8。
- [x] 摘录 `map→odom` 平滑与 jump 拒绝代码段，见 `localization_codex_supplement.md` §2.9–§2.11。
- [x] 确认 `ndt_bootstrap_pending_` 状态机与测试断言一致，见 `localization_codex_supplement.md` §2.4、§2.6。
- [x] 确认三类核心包存在：`lidar_localization_ros2`、FAST-LIO、`ndt_omp_ros2`。

#### 6.2.2 A/B/C/D/E 干扰对照实验状态

任务要求的现场实验组如下；原始 CSV 位于 `docs/reports/localization_experiment_metrics.csv`。由于当前机器未运行定位栈，且 ROS 日志中没有按 A/B/C/D/E 标记、也没有当前实现 `NDT aligned: fitness=...` 格式的对齐统计，以下实验没有写入伪数值。

| 实验组 | 变量 | 状态 | 原因 |
|--------|------|------|------|
| A | 净化 PCD + 默认参数 | 未采集 | 无当前实现的 `NDT aligned`/fitness 日志，不能从旧日志推断基线。 |
| B | 未净化 PCD | 未采集 | 未找到按 B 组标记的原始 PCD 定位实验日志；姊妹滤除报告的合成实验不等同于定位实验。 |
| C | `use_odom:false` 短距安全区 | 略 | 该组会削弱运动先验，需现场安全区、急停和看护；本次未启动车辆测试。 |
| D | `score_threshold:0.5` | 未采集 | 未找到严门限运行日志，不能用默认 `score_threshold:1.0` 日志外推。 |
| E | U 型转弯 | 未采集 | 未找到转弯前后 30 s 的 accepted/rejected NDT 统计与 jump 日志。 |

#### 6.2.3 量化结果表

| 实验组 | fitness 中位数 | fitness p90 | 拒配比例 | 1 min 静止漂移 (m) | 导航 abort 次数 | 数据状态 |
|--------|----------------|-------------|----------|-------------------|----------------|----------|
| A | NA | NA | NA | NA | NA | 未采集 |
| B | NA | NA | NA | NA | NA | 未采集 |
| C | NA | NA | NA | NA | NA | 安全原因略 |
| D | NA | NA | NA | NA | NA | 未采集 |
| E | NA | NA | NA | NA | NA | 未采集 |

### 6.3 实验结论与数据缺口

1. **已验证结论**：当前实现不再允许 `/initialpose` 直接播种 `map→odom`；首次和后续 TF 发布都必须经过 NDT 收敛、fitness 与 jump gate。该结论由源码和 `test_initialpose_is_only_ndt_guess_until_valid_alignment` 支撑。
2. **已验证结论**：`ndt_aligned_scan` 只在 accepted NDT 结果之后发布，因此 RViz 叠合良好代表“已通过门限的诊断输出”，不是所有配准尝试的可视化。
3. **未验证问题**：鬼影地图使 fitness/漂移恶化多少、`use_odom:false` 多远开始失效、`score_threshold:0.5` 是否导致 stale TF、U 型转弯时 0.45 rad yaw jump gate 是否足够，均需要新的现场日志回答。
4. **数据要求**：后续补测必须保留 `NDT aligned:`、`Fitness ... > ...`、`NDT jump`、`NDT map→odom jump` 和 Nav2 goal 结果日志，并按 A/B/C/D/E 标记实验组；否则不能计算 `fitness_median`、`fitness_p90`、拒配比例和漂移。

---

## 7 结论与后续工作

### 7.1 主要结论

1. Traymover 定位采用 **FAST-LIO 高频里程计 + NDT 低频全局校正**，本质上是「运动先验降低 NDT 对短时干扰与大开环运动的敏感性」；
2. 干扰敏感性主要集中在 **地图质量、几何退化、后方动态点、评分门限与算力时序** 五类，已通过分层滤除（上游）、360° 扫描策略、后方禁区预处理、odom 融合、jump gate 与平滑校正（下游）形成工程闭环；
3. 实机工作区含完整 **配置、集成层与 NDT C++ 节点实现**；源码行为和回归测试已补全，实车 A/B/C/D/E 量化仍需按安全流程另行采集。

### 7.2 后续优化方向

1. 在线监测 NDT fitness/score 滑动统计，自适应 `score_threshold`；
2. 将 `/point_cloud_localization` 预处理参数与场景类型（通道/大厅）建立预设表；
3. 完成第 6.2 节 A/B/C/D/E 对照实验，形成可计算的 fitness、拒配率、map→odom 漂移与 abort 指标；
4. 评估是否将 `lidar_localization_ros2`、`FAST_LIO`、`ndt_omp_ros2` 纳入版本管理或 `.gitmodules`。

---

## 参考文献

[1] Xu W, Cai Y, He T, et al. FAST-LIO: Fast, Robust LiDAR-Inertial Odometry and Mapping. IEEE RA-L, 2021.
[2] Biber P, Straßer W. The Normal Distributions Transform: A New Approach to Laser Scan Matching. IROS, 2003.
[3] （姊妹课题）Traymover 点云干扰噪声分层滤除工程方案与验证报告，2026.
[4] Traymover ROS2 README 与 `localization.yaml` 工程注释，2026.

---

## 附录 A 文件索引

| 路径 | 作用 |
|------|------|
| `src/traymover_robot_nav/config/localization.yaml` | NDT 全参数与现场注释 |
| `src/traymover_robot_nav/launch/lidar_localization.launch.py` | 双阶段定位 launch |
| `src/traymover_robot_nav/scripts/pointcloud_nav_preprocessor.py` | 定位/导航点云分叉 |
| `src/turn_on_traymover_robot/config/traymover_param.yaml` | C32 360° 策略 |
| `scripts/check_localization.sh` | 定位健康体检 |
| `src/traymover_robot_nav/test/test_nav_simple_config.py` | 配置与 NDT 行为回归 |
| `docs/CODEX_TASK_localization_report_supplement.md` | 远程 Codex 补全任务书 |
| `docs/reports/localization_codex_supplement.md` | NDT 源码摘录与日志排除说明 |
| `docs/reports/localization_experiment_metrics.csv` | A/B/C/D/E 实验数据状态表 |
